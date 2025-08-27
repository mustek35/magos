"""Process API.Targets.UPD events and send consolidated positions to Traccar.

This script connects to the MAGOS radar WebSocket API using ``WebSocketRadarClient``
and listens only to ``API.Targets.UPD.Main`` messages. For each target it keeps a
persistent tail of latitude/longitude points per ``uid`` without duplicating the
last known coordinate. New points are written to a CSV file and forwarded to a
Traccar server.

The reconnection parameters of the WebSocket client are tuned for a quick
recovery to avoid losing data.
"""

from __future__ import annotations

import csv
import json
import os
import time
from collections import defaultdict
from typing import Dict, List

import requests

from websocket_client import WebSocketRadarClient

# ---------------------------------------------------------------------------
# Configuration placeholders
# ---------------------------------------------------------------------------
LOGIN_URL = "https://example.com/login"
WS_URL = "wss://example.com/ws"
USERNAME = "user"
PASSWORD = "pass"

TRACCAR_URL = "https://traccar.example.com/api/positions"
AUTH = ("traccar_user", "traccar_pass")
DEVICE_MAP: Dict[str, int] = {
    # "uid": deviceId
}

CSV_FILE = "detections.csv"

# ---------------------------------------------------------------------------
# Helpers for tail accumulation and output
# ---------------------------------------------------------------------------
tails: Dict[str, List[Dict[str, float]]] = defaultdict(list)

def registrar_deteccion(uid: str, target: dict, point: dict) -> None:
    """Append a new detection row to the CSV file."""
    file_exists = os.path.exists(CSV_FILE)
    with open(CSV_FILE, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(
                [
                    "timestamp",
                    "uid",
                    "lat",
                    "lng",
                    "alt",
                    "speed",
                    "distance",
                    "class",
                    "conf",
                ]
            )
        writer.writerow(
            [
                time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                uid,
                point.get("lat"),
                point.get("lng"),
                point.get("alt"),
                target.get("speed"),
                target.get("distance"),
                target.get("class"),
                target.get("conf"),
            ]
        )

def enviar_a_traccar(uid: str, target: dict, point: dict) -> None:
    """Send a position update to Traccar."""
    device_id = DEVICE_MAP.get(uid)
    if device_id is None:
        return
    data = {
        "deviceId": device_id,
        "latitude": point["lat"],
        "longitude": point["lng"],
        "altitude": point["alt"],
        "speed": target.get("speed"),
        "attributes": {
            "distance": target.get("distance"),
            "conf": target.get("conf"),
        },
        "deviceTime": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    try:
        requests.post(TRACCAR_URL, json=data, auth=AUTH, timeout=5)
    except requests.RequestException as exc:
        print("Error enviando a Traccar:", exc)

def handle_targets_update(targets: List[dict]) -> None:
    """Process a list of target dictionaries."""
    for t in targets:
        uid = t.get("uid")
        if not uid:
            continue
        new_tail = t.get("tail", [])
        last_tail = tails[uid]
        for p in new_tail:
            if not last_tail or (
                p.get("lat"), p.get("lng")
            ) != (
                last_tail[-1].get("lat"),
                last_tail[-1].get("lng"),
            ):
                last_tail.append(p)
                registrar_deteccion(uid, t, p)
                enviar_a_traccar(uid, t, p)

def handle_message(message: str) -> None:
    """WebSocket message handler."""
    try:
        data = json.loads(message)
    except json.JSONDecodeError:
        return
    if (
        isinstance(data, list)
        and len(data) >= 2
        and data[0] == "API.Targets.UPD.Main"
        and isinstance(data[1], list)
    ):
        handle_targets_update(data[1])

# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------

def main() -> None:
    client = WebSocketRadarClient(
        LOGIN_URL,
        WS_URL,
        USERNAME,
        PASSWORD,
        message_handler=handle_message,
        reconnect_delay=5,
        ping_interval=5,
        ping_timeout=2,
    )
    client.connect()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        client.close()

if __name__ == "__main__":
    main()
