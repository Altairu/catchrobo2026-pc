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
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn


# static ファイルのパス (パッケージ内 static/ ディレクトリ)
STATIC_DIR = Path(__file__).parent / 'static'


class WebGuiNode(Node):
    def __init__(self):
        super().__init__('web_gui_node')

        # WebSocket接続中のクライアント一覧
        self._ws_clients: list[WebSocket] = []
        self._ws_lock = asyncio.Lock()

        # ROS2トピック用イベントループ（FastAPIとは別）
        self._ros_loop = asyncio.new_event_loop()

        # ─── ROS2 パブリッシャー ──────────────────────
        self.pub_motor_cmd  = self.create_publisher(Float32MultiArray, '/catchrobo/motor_cmd',  10)
        self.pub_motor_mode = self.create_publisher(String,            '/catchrobo/motor_mode', 10)
        self.pub_module_cmd = self.create_publisher(String,            '/catchrobo/module_cmd', 10)
        self.pub_set_ports  = self.create_publisher(String,            '/catchrobo/set_ports',  10)

        # ─── ROS2 サブスクライバー ─────────────────────
        self.create_subscription(String,            '/catchrobo/can_status',       self._on_can_status,       10)
        self.create_subscription(String,            '/catchrobo/serial_status',    self._on_serial_status,    10)
        self.create_subscription(Float32MultiArray, '/catchrobo/motor_fb',         self._on_motor_fb,         10)
        self.create_subscription(String,            '/catchrobo/available_ports',  self._on_available_ports,  10)

        # ─── FastAPI アプリ ────────────────────────────
        self.app = FastAPI(title='Catchrobo 2026 GUI')

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
                # ロボマスモーター目標値 (5軸 degree値)
                targets = data.get('targets', [0.0] * 5)
                msg = Float32MultiArray()
                msg.data = [float(t) for t in targets[:5]]
                self.pub_motor_cmd.publish(msg)

            elif cmd == 'motor_mode':
                # 制御モード切り替え
                msg = String()
                msg.data = json.dumps({'mode': int(data.get('mode', 0))})
                self.pub_motor_mode.publish(msg)

            elif cmd == 'module_cmd':
                # モジュール操作コマンド (MDD/Solenoid)
                msg = String()
                msg.data = json.dumps(data.get('payload', {}))
                self.pub_module_cmd.publish(msg)

            elif cmd == 'set_ports':
                # ポート設定 → NUCへ送信
                msg = String()
                msg.data = json.dumps({
                    'can_port':    data.get('can_port', ''),
                    'serial_port': data.get('serial_port', ''),
                })
                self.pub_set_ports.publish(msg)

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

    try:
        loop.run_until_complete(server.serve())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
