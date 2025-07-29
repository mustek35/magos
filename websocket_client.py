import threading
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
    ) -> None:
        self.login_url = login_url
        self.ws_url = ws_url
        self.username = username
        self.password = password
        self.message_handler = message_handler
        self.session = requests.Session()
        self.ws = None
        self.thread = None

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
        cookie_header = self.login()
        self.ws = websocket.WebSocketApp(
            self.ws_url,
            header={"Cookie": cookie_header},
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
        )
        self.thread = threading.Thread(target=self.ws.run_forever)
        self.thread.daemon = True
        self.thread.start()

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
        if self.ws:
            self.ws.close()
        if self.thread and self.thread.is_alive():
            self.thread.join()

