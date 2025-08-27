import threading
import time
from typing import Callable, Optional

import requests
import websocket


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
        ping_interval: int = 5,
        ping_timeout: int = 2,
    ) -> None:
        self.login_url = login_url
        self.ws_url = ws_url
        self.username = username
        self.password = password
        self.message_handler = message_handler
        self.session = requests.Session()
        self.ws = None
        self.thread = None
        self.stop_event = threading.Event()
        self.reconnect_delay = reconnect_delay  # seconds
        self.ping_interval = ping_interval
        self.ping_timeout = ping_timeout

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
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

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
                    ping_interval=self.ping_interval,
                    ping_timeout=self.ping_timeout,
                )
            except Exception as e:
                print("❌ Error WebSocket:", e)
            if not self.stop_event.is_set():
                print(f"🔄 Reintentando conexión en {self.reconnect_delay} segundos...")
                if self.stop_event.wait(self.reconnect_delay):
                    break

    # WebSocket callbacks
    def on_message(self, ws, message):
        print("\U0001f4e9 Mensaje recibido:")
        print(message)
        if self.message_handler:
            self.message_handler(message)

    def on_error(self, ws, error):
        print("\u274c Error:", error)

    def on_close(self, ws, close_status_code, close_msg):
        print("\ud83d\udd12 Conexi\u00f3n cerrada")

    def on_open(self, ws):
        print("\ud83d\udd13 Conexi\u00f3n WebSocket abierta")

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

