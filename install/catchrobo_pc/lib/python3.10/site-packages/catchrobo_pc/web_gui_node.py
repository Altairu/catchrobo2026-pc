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
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn
import serial
import serial.tools.list_ports

from ament_index_python.packages import get_package_prefix

# static ファイルのパス (インストール先 lib/catchrobo_pc/static/)
STATIC_DIR = Path(get_package_prefix('catchrobo_pc')) / 'lib' / 'catchrobo_pc' / 'static'

# 外部コントローラ入力の安全範囲 (GUIスライダー範囲と同じ)
RM1_MIN = -20.0
RM1_MAX = 60.0
RM2_MIN = -15.0
RM2_MAX = 90.0


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
        self._external_stop_event = threading.Event()
        self._external_last_warn_ts = 0.0
        self._external_no_data_err_count = 0
        self._motor_targets_cache = [0.0] * 6
        self._sv2_valves_cache = 0

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
                msg = String()
                payload = data.get('payload', {})
                msg.data = json.dumps(payload)
                self.get_logger().info(f'WS module_cmd 受信: payload={payload}')
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

            if cmd == 'motor_cmd':
                self._motor_targets_cache = [float(t) for t in targets[:6]]

            if cmd == 'module_cmd':
                payload = data.get('payload', {})
                if payload.get('type') == 'solenoid' and payload.get('name') == 'SV_2' and payload.get('action') == 'set_valves':
                    self._sv2_valves_cache = int(payload.get('valves', 0))

        except Exception as e:
            self.get_logger().error(f'WS メッセージ処理エラー: {e}')

    # ─────────────────────────────────────────────────
    # ROS2 サブスクライバー → WebSocket ブロードキャスト
    # ─────────────────────────────────────────────────

    def _on_can_status(self, msg: String):
        self._broadcast_sync({'type': 'can_status', 'data': json.loads(msg.data)})

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
                },
            }
        self._broadcast_sync(payload)

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
                        lsw = [int(v) for v in data[8:12]]
                        self._publish_external_mdd_packet(dev_id, deg, lsw)
                        if dev_id == 0x01:
                            self._on_external_mdd1(deg, lsw)

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
            self._apply_external_control(deg)

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

    def _apply_external_control(self, deg: list[float]):
        # M1 -> RM2, M2 -> RM1
        targets = list(self._motor_targets_cache)
        targets[0] = max(RM1_MIN, min(RM1_MAX, float(deg[1])))
        targets[1] = max(RM2_MIN, min(RM2_MAX, float(deg[0])))
        self._motor_targets_cache = targets

        motor_msg = Float32MultiArray()
        motor_msg.data = targets[:6]
        self.pub_motor_cmd.publish(motor_msg)

        # M3 が 45deg 超なら SV_2 の V6(bit5) を ON
        want_on = float(deg[2]) > 45.0
        current_on = (self._sv2_valves_cache & 0x20) != 0
        if want_on != current_on:
            if want_on:
                self._sv2_valves_cache |= 0x20
            else:
                self._sv2_valves_cache &= ~0x20

            module_msg = String()
            module_msg.data = json.dumps({
                'type': 'solenoid',
                'name': 'SV_2',
                'action': 'set_valves',
                'valves': int(self._sv2_valves_cache),
            })
            self.pub_module_cmd.publish(module_msg)

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
