# magos_ws_failover.py
import json
import time
import threading
import random
import socket
import ssl
import requests
import websocket  # websocket-client

# --- ajustes recomendados ---
PING_PERIOD = 25        # cada cuánto envío ping manual (s)
STALL_TIMEOUT = 60      # si no llega ni msg ni pong en X s, fuerzo reconnect
RECONNECT_BASE = 3      # backoff base (s)
RECONNECT_MAX = 30      # backoff máx (s)

def _now():
    return time.monotonic()

class RobustWSClient:
    """
    WebSocket robusto con:
    - login por cookie
    - ping manual (websocket-client ping desactivado)
    - watchdog de inactividad
    - backoff exponencial con jitter
    - SO_KEEPALIVE
    """
    def __init__(self, name, login_url, ws_url, username, password,
                 message_handler=None):
        self.name = name
        self.login_url = login_url
        self.ws_url = ws_url
        self.username = username
        self.password = password
        self.message_handler = message_handler

        self.session = requests.Session()
        self.ws = None
        self.thread = None
        self.stop_event = threading.Event()

        self._last_rx = _now()
        self._last_pong = _now()
        self._connected_event = threading.Event()

        self._heartbeat_thread = None

    # ---------- autenticación ----------
    def login(self) -> str:
        r = self.session.post(self.login_url, data={
            "username": self.username,
            "password": self.password
        }, timeout=10)
        if r.status_code != 200:
            raise RuntimeError(f"[{self.name}] Login failed: {r.status_code}")
        cookies = self.session.cookies.get_dict()
        return "; ".join(f"{k}={v}" for k, v in cookies.items())

    # ---------- ciclo principal ----------
    def start(self):
        self.stop_event.clear()
        if not (self.thread and self.thread.is_alive()):
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def stop(self):
        self.stop_event.set()
        try:
            if self.ws:
                self.ws.close()
        except Exception:
            pass
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=3)
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=3)
        self.thread = None
        self._heartbeat_thread = None

    def _make_ws(self, cookie_header: str):
        # on_ping/on_pong ayudan a diagnosticar (algunos servers envían pings)
        return websocket.WebSocketApp(
            self.ws_url,
            header=[f"Cookie: {cookie_header}"],
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
            on_ping=self._on_ping,
            on_pong=self._on_pong
        )

    def _run(self):
        backoff = RECONNECT_BASE
        while not self.stop_event.is_set():
            try:
                cookie_header = self.login()
                self.ws = self._make_ws(cookie_header)

                # Opciones de socket: SO_KEEPALIVE
                sockopt = [
                    (socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1),
                ]
                # TLS: normalmente certificado válido; si es self-signed:
                sslopt = {"cert_reqs": ssl.CERT_REQUIRED}

                self._connected_event.clear()
                self._last_rx = _now()
                self._last_pong = _now()

                # Desactivamos ping interno del lib (ping_interval=0)
                # y usaremos ping manual en _heartbeat_loop
                t = threading.Thread(target=self._heartbeat_loop, daemon=True)
                self._heartbeat_thread = t
                t.start()

                print(f"🔓 [{self.name}] Abriendo WebSocket…")
                self.ws.run_forever(
                    ping_interval=0,
                    ping_timeout=None,
                    sockopt=sockopt,
                    sslopt=sslopt,
                    skip_utf8_validation=True,
                    ping_payload="hb"  # por si envías pings manuales
                )
            except Exception as e:
                print(f"❌ [{self.name}] Error run_forever: {e}")

            # Si salimos, cerramos hb thread
            if self._heartbeat_thread and self._heartbeat_thread.is_alive():
                # hará break al detectar stop_event o ws None
                self._heartbeat_thread.join(timeout=1)

            if self.stop_event.is_set():
                break

            # backoff con jitter
            wait = min(backoff, RECONNECT_MAX)
            wait += random.uniform(0, 1.0)
            print(f"🔄 [{self.name}] Reintentando en {wait:.1f}s…")
            self.stop_event.wait(wait)
            backoff = min(backoff * 2, RECONNECT_MAX)

    # ---------- heartbeat + watchdog ----------
    def _heartbeat_loop(self):
        while not self.stop_event.is_set() and self.ws:
            # Enviamos ping manual
            try:
                if self.ws and self.ws.sock and self.ws.sock.connected:
                    # si el server no responde pongs, igual este ping ayuda a mantener el túnel
                    self.ws.sock.ping("hb")
            except Exception as e:
                print(f"⚠️ [{self.name}] Error ping manual: {e}")

            # Watchdog: sin mensajes ni pongs por mucho tiempo → cierro para forzar reconnect
            since_rx = _now() - max(self._last_rx, self._last_pong)
            if since_rx > STALL_TIMEOUT:
                print(f"⏱️ [{self.name}] Stall {since_rx:.1f}s → reconectar")
                try:
                    if self.ws:
                        self.ws.close()
                except Exception:
                    pass
                return  # sale y deja que _run haga backoff/reconnect

            if self.stop_event.wait(PING_PERIOD):
                break

    # ---------- callbacks ----------
    def _on_open(self, ws):
        self._connected_event.set()
        self._last_rx = _now()
        print(f"✅ [{self.name}] Conexión abierta")

    def _on_message(self, ws, message):
        self._last_rx = _now()
        # Procesa sólo lo que te interesa (Targets.UPD)
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            data = None

        if (isinstance(data, list) and len(data) >= 2
            and data[0] == "API.Targets.UPD.Main" and isinstance(data[1], list)):
            for target in data[1]:
                tail = target.get("tail") or []
                if not tail:
                    continue
                pt = tail[-1]
                simplified = {
                    "id": target.get("uid"),
                    "lat": pt.get("lat"),
                    "lon": pt.get("lng"),
                    "altitude": pt.get("alt", 0),
                    "speed": target.get("speed", 0),
                    "accuracy": 0,
                    "course": 0,
                    "timestamp": int(time.time()),
                }
                print(json.dumps(simplified))

        if self.message_handler:
            self.message_handler(message)

    def _on_error(self, ws, error):
        print(f"❌ [{self.name}] Error: {error}")

    def _on_close(self, ws, code, reason):
        print(f"🔒 [{self.name}] Cerrado (code={code}, reason={reason})")

    def _on_ping(self, ws, message):
        # algunos servidores envían ping; el cliente responde pong automáticamente
        # igual lo registramos
        self._last_rx = _now()

    def _on_pong(self, ws, message):
        self._last_pong = _now()


class DualConnectionCoordinator:
    """
    Maneja dos conexiones: primary y standby.
    - Sólo la primaria emite datos al pipeline.
    - Si la primaria cae/stallea, standby toma liderazgo automáticamente.
    """
    def __init__(self, login_url, ws_url, username, password, handler=None):
        self.leader_lock = threading.Lock()
        self.leader = "primary"  # o "standby"

        def leader_wrapper(tag):
            def _wrap(msg):
                with self.leader_lock:
                    if self.leader == tag and handler:
                        handler(msg)
            return _wrap

        self.primary = RobustWSClient(
            "primary", login_url, ws_url, username, password,
            message_handler=leader_wrapper("primary")
        )
        self.standby = RobustWSClient(
            "standby", login_url, ws_url, username, password,
            message_handler=leader_wrapper("standby")
        )

        # hilos que vigilan salud básica para decidir liderazgo
        self._arbiter_stop = threading.Event()
        self._arbiter_thread = threading.Thread(target=self._arbiter_loop,
                                                daemon=True)

    def start(self):
        self.primary.start()
        # espera breve para dar prioridad a primaria
        time.sleep(1.0)
        self.standby.start()
        self._arbiter_stop.clear()
        if not self._arbiter_thread.is_alive():
            self._arbiter_thread = threading.Thread(target=self._arbiter_loop, daemon=True)
            self._arbiter_thread.start()

    def stop(self):
        self._arbiter_stop.set()
        self.primary.stop()
        self.standby.stop()

    def _arbiter_loop(self):
        """
        Toma decisiones simples:
        - si primary no recibe nada en STALL_TIMEOUT/2 → switch a standby
        - si primary se recupera → vuelve a primary
        """
        while not self._arbiter_stop.is_set():
            p_ok = (_now() - max(self.primary._last_rx, self.primary._last_pong) < STALL_TIMEOUT/2)
            s_ok = (_now() - max(self.standby._last_rx, self.standby._last_pong) < STALL_TIMEOUT/2)

            with self.leader_lock:
                if self.leader == "primary":
                    if not p_ok and s_ok:
                        print("👑 Failover → standby")
                        self.leader = "standby"
                else:
                    if p_ok:
                        print("👑 Failback → primary")
                        self.leader = "primary"

            if self._arbiter_stop.wait(5):
                break
