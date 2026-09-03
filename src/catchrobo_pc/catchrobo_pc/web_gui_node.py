"""
web_gui_node.py
FastAPI + WebSocket サーバーとROS2を橋渡しするPC側GUIノード

- ブラウザ ↔ WebSocket ↔ ROS2 のブリッジ
- http://localhost:8080 でブラウザGUIを提供
- ブラウザからの操作コマンドをROS2トピックへPublish
- ROS2からのフィードバックをWebSocketでブラウザへ配信
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

import asyncio
import threading
import json
import os
import time
import struct
import signal
import socket
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn
import serial
import serial.tools.list_ports

from ament_index_python.packages import get_package_prefix

# static ファイルのパス (開発時は実体ディレクトリを優先、インストール先 lib/catchrobo_pc/static/ もフォールバック)
_LOCAL_STATIC = Path(__file__).resolve().parent / 'static'
if _LOCAL_STATIC.exists():
    STATIC_DIR = _LOCAL_STATIC
else:
    STATIC_DIR = Path(get_package_prefix('catchrobo_pc')) / 'lib' / 'catchrobo_pc' / 'static'

# 外部コントローラ入力の安全範囲 (GUIスライダー範囲と同じ)
RM1_MIN = -20.0
RM1_MAX = 85.0
RM2_MIN = -20.0
RM2_MAX = 110.0
LM1_MIN = -20.0
LM1_MAX = 70.0
LM2_MIN = -15.0
LM2_MAX = 90.0


class WebGuiNode(Node):
    def __init__(self):
        super().__init__('web_gui_node')

        # WebSocket接続中のクライアント一覧
        self._ws_clients: list[WebSocket] = []
        self._ws_lock = asyncio.Lock()

        # ROS2トピック用イベントループ（FastAPIとは別）
        self._ros_loop = asyncio.new_event_loop()

        # 外部コントローラ連携の状態
        self._external_ctrl_enabled = False
        self._external_lock = threading.Lock()
        self._external_serial_port = ''
        self._external_last_rx = 0.0
        self._external_packet_count = 0
        self._external_mdd1_deg = [0.0, 0.0, 0.0, 0.0]
        self._external_mdd1_lsw = [0, 0, 0, 0]
        self._external_mdd2_deg = [0.0, 0.0, 0.0, 0.0]
        self._external_mdd2_lsw = [0, 0, 0, 0]
        self._external_stop_event = threading.Event()
        self._external_last_warn_ts = 0.0
        self._external_no_data_err_count = 0
        self._motor_targets_cache = [0.0] * 6
        self._sv1_valves_cache = 0
        self._sv2_valves_cache = 0
        self._servo1_targets_cache = [90] * 6
        self._servo1_ch6_last_update = 0.0

        # ─── ROS2 パブリッシャー ──────────────────────
        self.pub_motor_cmd  = self.create_publisher(Float32MultiArray, '/catchrobo/motor_cmd',  10)
        self.pub_motor_mode = self.create_publisher(String,            '/catchrobo/motor_mode', 10)
        self.pub_module_cmd = self.create_publisher(String,            '/catchrobo/module_cmd', 10)
        self.pub_set_ports  = self.create_publisher(String,            '/catchrobo/set_ports',  10)
        self.pub_external_mdd = self.create_publisher(String,          '/catchrobo/external_mdd_status', 10)

        # ─── ROS2 サブスクライバー ─────────────────────
        self.create_subscription(String,            '/catchrobo/can_status',       self._on_can_status,       10)
        self.create_subscription(String,            '/catchrobo/serial_status',    self._on_serial_status,    10)
        self.create_subscription(Float32MultiArray, '/catchrobo/motor_fb',         self._on_motor_fb,         10)
        self.create_subscription(String,            '/catchrobo/available_ports',  self._on_available_ports,  10)

        # ─── FastAPI アプリ ────────────────────────────
        self.app = FastAPI(title='Catchrobo 2026 GUI')

        # 外部コントローラ監視スレッド起動
        self._external_thread = threading.Thread(target=self._external_serial_worker, daemon=True)
        self._external_thread.start()
        # WebUI向け状態配信 (5Hz)
        self.create_timer(0.2, self._publish_external_status)

        # static ファイル配信
        if STATIC_DIR.exists():
            self.app.mount('/static', StaticFiles(directory=str(STATIC_DIR)), name='static')

        # ルート: index.html
        @self.app.get('/')
        async def root():
            return FileResponse(str(STATIC_DIR / 'index.html'))

        # スマホ用簡易UI: mobile.html
        @self.app.get('/mobile')
        async def mobile():
            return FileResponse(str(STATIC_DIR / 'mobile.html'))

        # REST API: ネットワーク情報（IPアドレス・接続URL）の取得
        @self.app.get('/api/network_info')
        async def network_info():
            return self._get_network_info()

        # WebSocket エンドポイント
        @self.app.websocket('/ws')
        async def ws_endpoint(ws: WebSocket):
            await ws.accept()
            async with self._ws_lock:
                self._ws_clients.append(ws)
            self.get_logger().info('WebSocket クライアント接続')
            try:
                while True:
                    raw = await ws.receive_text()
                    await self._handle_ws_message(raw)
            except WebSocketDisconnect:
                pass
            finally:
                async with self._ws_lock:
                    self._ws_clients.remove(ws)
                self.get_logger().info('WebSocket クライアント切断')

        self.get_logger().info('WebGUIノード 起動完了  http://localhost:8080')

    # ─────────────────────────────────────────────────
    # WebSocket メッセージ処理
    # ─────────────────────────────────────────────────

    async def _handle_ws_message(self, raw: str):
        """ブラウザからのメッセージを対応するROS2トピックへ転送する"""
        try:
            data = json.loads(raw)
            cmd = data.get('cmd', '')

            if cmd == 'motor_cmd':
                # ロボマスモーター目標値 (6軸 degree値)
                targets = data.get('targets', [0.0] * 6)
                msg = Float32MultiArray()
                msg.data = [float(t) for t in targets[:6]]
                self.pub_motor_cmd.publish(msg)

            elif cmd == 'motor_mode':
                # 制御モード切り替え
                msg = String()
                msg.data = json.dumps({'mode': int(data.get('mode', 0))})
                self.pub_motor_mode.publish(msg)

            elif cmd == 'module_cmd':
                # モジュール操作コマンド (MDD/Solenoid)
                payload = data.get('payload', {})
                # GUIからのMDD1の目標値送信時にM1, M2, M3を反転してCANに送る
                if payload.get('type') == 'mdd' and payload.get('name') == 'MDD1' and payload.get('action') == 'set_target':
                    t = payload.get('targets', [])
                    if len(t) >= 3:
                        t[0] = -t[0]
                        t[1] = -t[1]
                        t[2] = -t[2]
                
                msg = String()
                msg.data = json.dumps(payload)
                self.pub_module_cmd.publish(msg)

            elif cmd == 'set_ports':
                # ポート設定 → NUCへ送信
                msg = String()
                msg.data = json.dumps({
                    'can_port':    data.get('can_port', ''),
                    'serial_port': data.get('serial_port', ''),
                })
                self.pub_set_ports.publish(msg)

            elif cmd == 'ext_ctrl_mode':
                enabled = bool(data.get('enabled', False))
                with self._external_lock:
                    self._external_ctrl_enabled = enabled
                self.get_logger().info(f'外部コントローラモード: {"ON" if enabled else "OFF"}')
                self._publish_external_status()

            elif cmd == 'ext_ctrl_get_status':
                self._publish_external_status()

            elif cmd == 'get_network_info':
                info = self._get_network_info()
                self._broadcast_sync({'type': 'network_info', 'data': info})

            if cmd == 'motor_cmd':
                self._motor_targets_cache = [float(t) for t in targets[:6]]

            if cmd == 'module_cmd':
                payload = data.get('payload', {})
                if payload.get('type') == 'solenoid':
                    if payload.get('name') == 'SV_1' and payload.get('action') == 'set_valves':
                        self._sv1_valves_cache = int(payload.get('valves', 0))
                    elif payload.get('name') == 'SV_2' and payload.get('action') == 'set_valves':
                        self._sv2_valves_cache = int(payload.get('valves', 0))
                elif payload.get('type') == 'servo' and payload.get('name') == 'Servo1' and payload.get('action') == 'set_target':
                    self._servo1_targets_cache = [int(v) for v in payload.get('targets', [90] * 6)[:6]]

        except Exception as e:
            self.get_logger().error(f'WS メッセージ処理エラー: {e}')

    # ─────────────────────────────────────────────────
    # ROS2 サブスクライバー → WebSocket ブロードキャスト
    # ─────────────────────────────────────────────────

    def _on_can_status(self, msg: String):
        data = json.loads(msg.data)
        try:
            mdd1 = data.get('modules', {}).get('MDD1')
            if mdd1 and 'enc_deg' in mdd1:
                deg = mdd1['enc_deg']
                if len(deg) >= 3:
                    deg[0] = -deg[0]
                    deg[1] = -deg[1]
                    deg[2] = -deg[2]
        except Exception:
            pass
        self._broadcast_sync({'type': 'can_status', 'data': data})

    def _on_serial_status(self, msg: String):
        self._broadcast_sync({'type': 'serial_status', 'data': json.loads(msg.data)})

    def _on_motor_fb(self, msg: Float32MultiArray):
        self._broadcast_sync({'type': 'motor_fb', 'data': list(msg.data)})

    def _on_available_ports(self, msg: String):
        self._broadcast_sync({'type': 'available_ports', 'data': json.loads(msg.data)})

    def _publish_external_status(self):
        now = time.time()
        with self._external_lock:
            age = (now - self._external_last_rx) if self._external_last_rx > 0 else 999.0
            payload = {
                'type': 'external_controller',
                'data': {
                    'enabled': self._external_ctrl_enabled,
                    'port': self._external_serial_port,
                    'online': self._external_last_rx > 0 and age < 0.5,
                    'last_rx_age': age,
                    'packet_count': self._external_packet_count,
                    'mdd1_deg': list(self._external_mdd1_deg),
                    'mdd1_lsw': list(self._external_mdd1_lsw),
                    'mdd2_deg': list(self._external_mdd2_deg),
                    'mdd2_lsw': list(self._external_mdd2_lsw),
                },
            }
        self._broadcast_sync(payload)

    def _get_network_info(self) -> dict:
        """自PCのIPアドレス一覧とアクセス用URL情報を取得する"""
        ips = []
        port = 8080

        # 1. アクティブなプライマリIPの取得テスト
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('10.255.255.255', 1))
            primary_ip = s.getsockname()[0]
            s.close()
            if primary_ip and not primary_ip.startswith('127.'):
                ips.append(primary_ip)
        except Exception:
            pass

        # 2. hostname経由での追加IP取得
        try:
            hostname = socket.gethostname()
            for ip in socket.gethostbyname_ex(hostname)[2]:
                if not ip.startswith('127.') and ip not in ips:
                    ips.append(ip)
        except Exception:
            pass

        if not ips:
            ips = ['127.0.0.1']

        interfaces = []
        for idx, ip in enumerate(ips):
            label = 'メインネットワーク' if idx == 0 and ip != '127.0.0.1' else f'ネットワーク ({ip})'
            interfaces.append({
                'name': label,
                'ip': ip,
                'url': f'http://{ip}:{port}',
                'mobile_url': f'http://{ip}:{port}/mobile',
            })

        return {
            'port': port,
            'interfaces': interfaces
        }

    def _find_external_port(self) -> str:
        candidates = []
        for p in serial.tools.list_ports.comports():
            dev = p.device or ''
            if dev.startswith('/dev/ttyACM'):
                candidates.append(dev)
        candidates.sort()
        return candidates[0] if candidates else ''

    def _external_serial_worker(self):
        pkt_h1 = 0xAA
        pkt_h2 = 0x55
        dev_ids = (0x01, 0x02)
        data_len_expected = 12
        buf = bytearray()
        port = None

        while not self._external_stop_event.is_set():
            try:
                if port is None:
                    dev = self._find_external_port()
                    if not dev:
                        with self._external_lock:
                            self._external_serial_port = ''
                        time.sleep(0.5)
                        continue

                    # 制御ノード側がポート所有者になるため排他オープンする
                    try:
                        port = serial.Serial(dev, 115200, timeout=0.1, exclusive=True)
                    except TypeError:
                        # 古いpyserial互換: exclusive未対応なら通常オープン
                        port = serial.Serial(dev, 115200, timeout=0.1)
                    with self._external_lock:
                        self._external_serial_port = dev
                    self.get_logger().info(f'外部コントローラ接続: {dev}')

                chunk = port.read(256)
                if not chunk:
                    continue
                self._external_no_data_err_count = 0

                buf.extend(chunk)
                while True:
                    if len(buf) < 5:
                        break
                    if buf[0] != pkt_h1 or buf[1] != pkt_h2:
                        buf.pop(0)
                        continue

                    dev_id = buf[2]
                    data_len = buf[3]
                    total = 5 + data_len
                    if len(buf) < total:
                        break

                    data = bytes(buf[4:4 + data_len])
                    cs_recv = buf[4 + data_len]

                    cs = dev_id ^ data_len
                    for b in data:
                        cs ^= b
                    cs &= 0xFF

                    if cs == cs_recv and data_len == data_len_expected and dev_id in dev_ids:
                        deg = [struct.unpack_from('<h', data, i * 2)[0] / 10.0 for i in range(4)]
                        if dev_id == 0x01:
                            # M1は機械的向き反転に伴い、受信時の符号反転を解除
                            deg[1] = -deg[1]
                            deg[2] = -deg[2]
                        elif dev_id == 0x02:
                            # M1は機械的向き反転に伴い、受信時に符号反転
                            deg[0] = -deg[0]
                        lsw = [int(v) for v in data[8:12]]
                        self._publish_external_mdd_packet(dev_id, deg, lsw)
                        if dev_id == 0x01:
                            self._on_external_mdd1(deg, lsw)
                        elif dev_id == 0x02:
                            self._on_external_mdd2(deg, lsw)

                    del buf[:total]

            except serial.SerialException as e:
                # 複数アクセス時に発生しやすい例外。短時間でのwarn連打を抑制する。
                msg = str(e)
                now = time.time()
                if 'returned no data' in msg:
                    self._external_no_data_err_count += 1
                    if self._external_no_data_err_count < 5:
                        time.sleep(0.1)
                        continue

                if (now - self._external_last_warn_ts) > 2.0:
                    self.get_logger().warn(f'外部コントローラ通信エラー: {msg}')
                    self._external_last_warn_ts = now

                if port is not None:
                    try:
                        port.close()
                    except Exception:
                        pass
                port = None
                buf.clear()
                with self._external_lock:
                    self._external_serial_port = ''
                time.sleep(0.5)
            except Exception as e:
                now = time.time()
                if (now - self._external_last_warn_ts) > 2.0:
                    self.get_logger().warn(f'外部コントローラ通信エラー: {e}')
                    self._external_last_warn_ts = now
                if port is not None:
                    try:
                        port.close()
                    except Exception:
                        pass
                port = None
                buf.clear()
                with self._external_lock:
                    self._external_serial_port = ''
                time.sleep(0.5)

        if port is not None:
            try:
                port.close()
            except Exception:
                pass

    def _on_external_mdd1(self, deg: list[float], lsw: list[int]):
        now = time.time()
        with self._external_lock:
            self._external_last_rx = now
            self._external_packet_count += 1
            self._external_mdd1_deg = deg[:4]
            self._external_mdd1_lsw = lsw[:4]
            enabled = self._external_ctrl_enabled

        # モードON時のみ、受信値から実機コマンドへ変換
        if enabled:
            self._update_external_control()

    def _on_external_mdd2(self, deg: list[float], lsw: list[int]):
        now = time.time()
        with self._external_lock:
            self._external_last_rx = now
            self._external_packet_count += 1
            self._external_mdd2_deg = deg[:4]
            self._external_mdd2_lsw = lsw[:4]
            enabled = self._external_ctrl_enabled

        # モードON時のみ、受信値から実機コマンドへ変換
        if enabled:
            self._update_external_control()

    def _publish_external_mdd_packet(self, dev_id: int, deg: list[float], lsw: list[int]):
        msg = String()
        msg.data = json.dumps({
            'device_id': int(dev_id),
            'name': 'MDD1' if int(dev_id) == 0x01 else 'MDD2',
            'port': self._external_serial_port,
            'deg': [float(v) for v in deg[:4]],
            'lsw': [int(v) for v in lsw[:4]],
            'packet_count': int(self._external_packet_count),
            'stamp': time.time(),
        })
        self.pub_external_mdd.publish(msg)

    def _update_external_control(self):
        # 必要なデータをスレッドセーフにコピー
        with self._external_lock:
            mdd1_deg = list(self._external_mdd1_deg)
            mdd1_lsw = list(self._external_mdd1_lsw)
            mdd2_deg = list(self._external_mdd2_deg)
            mdd2_lsw = list(self._external_mdd2_lsw)

        targets = list(self._motor_targets_cache)

        # MDD1に基づくマッピング
        if len(mdd1_deg) >= 4:
            # M1 -> RM2, M2 -> RM1
            targets[0] = max(RM1_MIN, min(RM1_MAX, float(mdd1_deg[1])))
            targets[1] = max(RM2_MIN, min(RM2_MAX, float(mdd1_deg[0])))

            # SW2 (lsw[1]) と SW3 (lsw[2]) による SM1 (targets[4]) の制御
            # スイッチ入力は反転している（ONのとき0、OFFのとき1）ため、lsw == 0 を ON と判定する
            sw2_on = (mdd1_lsw[1] == 0)
            sw3_on = (mdd1_lsw[2] == 0)

            # SW2がONのとき電流指令値+2000(2.0A)、SW3がONのとき-2000(-2.0A)、それ以外は停止(0)
            if sw2_on and not sw3_on:
                targets[4] = 2000.0
            elif sw3_on and not sw2_on:
                targets[4] = -2000.0
            else:
                targets[4] = 0.0

        # MDD2に基づくマッピング
        if len(mdd2_deg) >= 4:
            # MDD2 M2 -> LM1 (左右の動作対称性に合わせて符号反転)
            targets[2] = max(LM1_MIN, min(LM1_MAX, -float(mdd2_deg[1])))
            # MDD2 M1 -> LM2 (左右の動作対称性に合わせて符号反転、可動限界: -15〜90)
            targets[3] = max(LM2_MIN, min(LM2_MAX, -float(mdd2_deg[0])))

        self._motor_targets_cache = targets

        motor_msg = Float32MultiArray()
        motor_msg.data = targets[:6]
        self.pub_motor_cmd.publish(motor_msg)

        # --- ソレノイドバルブ制御 ---

        next_sv1_valves = self._sv1_valves_cache

        # MDD1に基づくSV_1制御
        if len(mdd1_deg) >= 4:
            # M3 が 45deg 超なら SV_1 の CH4(bit3) を ON
            v4_want_on = float(mdd1_deg[2]) > 45.0
            
            # SW1 (lsw[0]) が ON のとき SV_1 の CH3 (bit2) を ON。
            # スイッチは反転している（ONのとき0、OFFのとき1）ため、lsw[0] == 0 のとき物理スイッチONと判定する。
            sw1_want_on = (mdd1_lsw[0] == 0)

            # bit 3 (CH4) の更新
            if v4_want_on:
                next_sv1_valves |= 0x08
            else:
                next_sv1_valves &= ~0x08

            # bit 2 (CH3) の更新
            if sw1_want_on:
                next_sv1_valves |= 0x04
            else:
                next_sv1_valves &= ~0x04

        # MDD2に基づくSV_1制御
        # MDD2のスイッチ1（lsw[0]）がONのとき SV_1 (0x300) の ch1,ch2 を OFF
        # MDD2のスイッチ1がOFFのとき SV_1 (0x300) の ch1,ch2 を ON
        if len(mdd2_lsw) >= 1:
            mdd2_sw1_on = (mdd2_lsw[0] == 0)
            if mdd2_sw1_on:
                next_sv1_valves &= ~0x03 # OFF
            else:
                next_sv1_valves |= 0x03  # ON

        if next_sv1_valves != self._sv1_valves_cache:
            self._sv1_valves_cache = next_sv1_valves
            module_msg = String()
            module_msg.data = json.dumps({
                'type': 'solenoid',
                'name': 'SV_1',
                'action': 'set_valves',
                'valves': int(self._sv1_valves_cache),
            })
            self.pub_module_cmd.publish(module_msg)

        # --- サーボモータ (Servo1) 制御 ---
        servo_updated = False
        next_servo_targets = list(self._servo1_targets_cache)

        # MDD1に基づくサーボ制御
        if len(mdd1_deg) >= 4:
            # M4 (deg[3]) -> Servo1 ch1 (targets[0])
            # ギヤ比10:1による減速 (エンコーダが10倍加速して測定されているため10で割る)
            # 0度のとき90度、0~180度にクランプ
            try:
                servo1_ch1 = int(round((float(mdd1_deg[3]) / 10.0) + 90.0))
                servo1_ch1 = max(0, min(180, servo1_ch1))
                if next_servo_targets[0] != servo1_ch1:
                    next_servo_targets[0] = servo1_ch1
                    servo_updated = True
            except (ValueError, TypeError) as e:
                self.get_logger().error(f'サーボch1目標値計算エラー: {e}')

        # MDD2に基づくサーボ制御
        if len(mdd2_deg) >= 4:
            # MDD2のM4の符号を反転し＋90したものをServo1のch3に代入（0〜180まで）
            # M4はギヤ比10:1を考慮して10で割る
            try:
                servo1_ch3 = int(round(-(float(mdd2_deg[3]) / 10.0) + 90.0))
                servo1_ch3 = max(0, min(180, servo1_ch3))
                if next_servo_targets[2] != servo1_ch3:
                    next_servo_targets[2] = servo1_ch3
                    servo_updated = True
            except (ValueError, TypeError) as e:
                self.get_logger().error(f'サーボch3目標値計算エラー: {e}')

            # MDD2のM3に＋90したものをServo1のch4に代入（85〜140まで）
            # 表示仕様に合わせて符号を反転して適用
            try:
                servo1_ch4 = int(round(-float(mdd2_deg[2]) + 90.0))
                servo1_ch4 = max(85, min(140, servo1_ch4))
                if next_servo_targets[3] != servo1_ch4:
                    next_servo_targets[3] = servo1_ch4
                    servo_updated = True
            except (ValueError, TypeError) as e:
                self.get_logger().error(f'サーボch4目標値計算エラー: {e}')

            # MDD2のスイッチ2がONのとき Servo1 ch2 が 40度、OFFのとき 70度
            try:
                if len(mdd2_lsw) >= 2:
                    mdd2_sw2_on = (mdd2_lsw[1] == 0)
                    servo1_ch2 = 40 if mdd2_sw2_on else 70
                    if next_servo_targets[1] != servo1_ch2:
                        next_servo_targets[1] = servo1_ch2
                        servo_updated = True
            except (ValueError, TypeError, IndexError) as e:
                pass

            # MDD2のスイッチ1がONのとき Servo1 ch5 が 180度、OFFのとき 90度
            try:
                if len(mdd2_lsw) >= 1:
                    mdd2_sw1_on = (mdd2_lsw[0] == 0)
                    servo1_ch5 = 180 if mdd2_sw1_on else 90
                    if next_servo_targets[4] != servo1_ch5:
                        next_servo_targets[4] = servo1_ch5
                        servo_updated = True
            except (ValueError, TypeError, IndexError) as e:
                pass

            # MDD2のSW3/SW4によるServo1 ch6の連続角度制御 (0〜180度)
            try:
                if len(mdd2_lsw) >= 4:
                    mdd2_sw3_on = (mdd2_lsw[2] == 0)
                    mdd2_sw4_on = (mdd2_lsw[3] == 0)
                    
                    now = time.time()
                    if (mdd2_sw3_on or mdd2_sw4_on) and (now - self._servo1_ch6_last_update > 0.05):
                        if mdd2_sw3_on and not mdd2_sw4_on:
                            servo1_ch6 = min(180, next_servo_targets[5] + 1)
                            if next_servo_targets[5] != servo1_ch6:
                                next_servo_targets[5] = servo1_ch6
                                servo_updated = True
                                self._servo1_ch6_last_update = now
                        elif mdd2_sw4_on and not mdd2_sw3_on:
                            servo1_ch6 = max(0, next_servo_targets[5] - 1)
                            if next_servo_targets[5] != servo1_ch6:
                                next_servo_targets[5] = servo1_ch6
                                servo_updated = True
                                self._servo1_ch6_last_update = now
            except (ValueError, TypeError, IndexError) as e:
                self.get_logger().error(f'Servo1 ch6制御エラー: {e}')

        # サーボの更新があればまとめてパブリッシュ
        if servo_updated:
            self._servo1_targets_cache = next_servo_targets
            servo_msg = String()
            servo_msg.data = json.dumps({
                'type': 'servo',
                'name': 'Servo1',
                'action': 'set_target',
                'targets': list(self._servo1_targets_cache)
            })
            self.pub_module_cmd.publish(servo_msg)

        self._publish_external_status()

    def _broadcast_sync(self, payload: dict):
        """ROS2コールバック（同期）からWebSocketブロードキャストをスケジュールする"""
        raw = json.dumps(payload)
        # FastAPIのイベントループへタスクを投入
        if hasattr(self, '_fastapi_loop') and self._fastapi_loop.is_running():
            asyncio.run_coroutine_threadsafe(self._broadcast(raw), self._fastapi_loop)

    async def _broadcast(self, raw: str):
        """全WebSocketクライアントへメッセージを送信する"""
        async with self._ws_lock:
            dead = []
            for ws in self._ws_clients:
                try:
                    await ws.send_text(raw)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self._ws_clients.remove(ws)


def main(args=None):
    rclpy.init(args=args)
    node = WebGuiNode()

    # ROS2 spinを別スレッドで動かす
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(node),
        daemon=True
    )
    ros_thread.start()

    # FastAPIのイベントループをノードに登録（ブロードキャスト用）
    loop = asyncio.new_event_loop()
    node._fastapi_loop = loop

    config = uvicorn.Config(
        node.app,
        host='0.0.0.0',
        port=8080,
        loop='asyncio',
        log_level='warning',
    )
    server = uvicorn.Server(config)

    def _graceful_shutdown(*_args):
        node._external_stop_event.set()
        server.should_exit = True

    signal.signal(signal.SIGINT, _graceful_shutdown)
    signal.signal(signal.SIGTERM, _graceful_shutdown)

    try:
        loop.run_until_complete(server.serve())
    except KeyboardInterrupt:
        pass
    finally:
        node._external_stop_event.set()
        server.should_exit = True
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
