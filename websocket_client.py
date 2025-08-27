import threading
import time
import socket
import random
from typing import Callable, Optional

import json
import requests
import websocket


__all__ = ["WebSocketRadarClient"]


class WebSocketRadarClient:
    """Simple client for the HMASS WebSocket API."""

    def __init__(
        self,
        login_url: str,
        ws_url: str,
        username: str,
        password: str,
        message_handler: Optional[Callable[[str], None]] = None,
        reconnect_delay: int = 5,
        ping_interval: int = 25,
        ping_timeout: int = 60,
    ) -> None:
        self.login_url = login_url
        self.ws_url = ws_url
        self.username = username
        self.password = password
        self.message_handler = message_handler
        self.session = requests.Session()
        self.ws = None
        self.thread = None
        self.ping_thread = None
        self.watchdog_thread = None
        self.stop_event = threading.Event()
        self.reconnect_delay = reconnect_delay
        self.max_reconnect_delay = 60
        self.current_delay = reconnect_delay
        self.ping_interval = ping_interval
        self.ping_timeout = ping_timeout
        self.last_message_time = time.time()

    # ---------------------------
    # 1. Login with requests
    # ---------------------------
    def login(self) -> str:
        """Authenticate and return a Cookie header for the WebSocket."""
        response = self.session.post(
            self.login_url,
            data={"username": self.username, "password": self.password},
        )
        if response.status_code != 200:
            raise RuntimeError("Login failed")
        cookies = self.session.cookies.get_dict()
        return "; ".join(f"{k}={v}" for k, v in cookies.items())

    # ---------------------------
    # 2. Connect to WebSocket
    # ---------------------------
    def connect(self):
        """Start connection thread with automatic reconnection."""
        self.stop_event.clear()
        if not (self.thread and self.thread.is_alive()):
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
        if not (self.ping_thread and self.ping_thread.is_alive()):
            self.ping_thread = threading.Thread(target=self._ping_loop, daemon=True)
            self.ping_thread.start()
        if not (self.watchdog_thread and self.watchdog_thread.is_alive()):
            self.watchdog_thread = threading.Thread(
                target=self._watchdog, daemon=True
            )
            self.watchdog_thread.start()

    def _create_ws_app(self, cookie_header: str) -> websocket.WebSocketApp:
        return websocket.WebSocketApp(
            self.ws_url,
            header={"Cookie": cookie_header},
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
        )

    def _run(self):
        while not self.stop_event.is_set():
            try:
                cookie_header = self.login()
                self.ws = self._create_ws_app(cookie_header)
                self.ws.run_forever(
                    ping_interval=0,
                    ping_timeout=None,
                    sockopt=((socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1),),
                )
                self.current_delay = self.reconnect_delay
            except Exception as e:
                print("❌ Error WebSocket:", e)
            if self.stop_event.is_set():
                break
            jitter = random.uniform(0, self.current_delay)
            wait_time = self.current_delay + jitter
            print(f"🔄 Reintentando conexión en {wait_time:.1f} segundos...")
            if self.stop_event.wait(wait_time):
                break
            self.current_delay = min(self.max_reconnect_delay, self.current_delay * 2)

    def _ping_loop(self):
        while not self.stop_event.is_set():
            if self.ws and self.ws.sock and self.ws.sock.connected:
                try:
                    self.ws.send("ping")
                except Exception:
                    pass
            if self.stop_event.wait(self.ping_interval):
                break

    def _watchdog(self):
        while not self.stop_event.is_set():
            if time.time() - self.last_message_time > self.ping_timeout:
                if self.ws:
                    try:
                        self.ws.close()
                    except Exception:
                        pass
            if self.stop_event.wait(self.ping_interval):
                break

    # WebSocket callbacks
    def on_message(self, ws, message):
        """Handle incoming WebSocket messages."""
        self.last_message_time = time.time()
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            data = None

        if (
            isinstance(data, list)
            and len(data) >= 2
            and data[0] == "API.Targets.UPD.Main"
            and isinstance(data[1], list)
        ):
            for target in data[1]:
                tail = target.get("tail", [])
                if not tail:
                    continue
                point = tail[-1]
                simplified = {
                    "id": target.get("uid"),
                    "lat": point.get("lat"),
                    "lon": point.get("lng"),
                    "altitude": point.get("alt", 0),
                    "speed": target.get("speed", 0),
                    "accuracy": 0,
                    "course": 0,
                    "timestamp": int(time.time()),
                }
                print(json.dumps(simplified))

        if self.message_handler:
            self.message_handler(message)

    def on_error(self, ws, error):
        print("❌ Error:", error)

    def on_close(self, ws, close_status_code, close_msg):
        print("🔒 Conexión cerrada")

    def on_open(self, ws):
        print("🔓 Conexión WebSocket abierta")
        self.last_message_time = time.time()
        self.current_delay = self.reconnect_delay

    def close(self):
        self.stop_event.set()
        if self.ws:
            try:
                self.ws.close()
            except Exception:
                pass
        if self.thread and self.thread.is_alive():
            self.thread.join()
        self.thread = None
        if self.ping_thread and self.ping_thread.is_alive():
            self.ping_thread.join()
        self.ping_thread = None
        if self.watchdog_thread and self.watchdog_thread.is_alive():
            self.watchdog_thread.join()
        self.watchdog_thread = None

