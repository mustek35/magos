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
import math
import os
import time
from collections import defaultdict
from typing import Any, Dict, List

import requests

from websocket_client import WebSocketRadarClient

# ---------------------------------------------------------------------------
# Configuration placeholders
# ---------------------------------------------------------------------------
LOGIN_URL = "https://example.com/login"
WS_URL = "wss://example.com/ws"
USERNAME = "user"
PASSWORD = "pass"

TRACCAR_URL = "http://179.57.170.61:5055"
DEVICE_MAP: Dict[str, int] = {
    # "uid": device_id
}

CSV_FILE = "detections.csv"
ZONE_CSV_FILE = "zone_alarms.csv"

# Map zone center names to device IDs and optional coordinates
ZONE_MAP: Dict[str, Dict[str, Any]] = {
    # "Huelmo": {"device_id": 123, "lat": -41.0, "lng": -73.0},
}

# ---------------------------------------------------------------------------
# Helpers for tail accumulation and output
# ---------------------------------------------------------------------------
tails: Dict[str, List[Dict[str, float]]] = defaultdict(list)


def registrar_deteccion(uid: str, target: dict, point: dict, heading: float) -> None:
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
                    "heading",
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
                heading,
            ]
        )


def registrar_alarma(name: str, alarm: dict) -> None:
    """Append a zone alarm row to its CSV file."""
    file_exists = os.path.exists(ZONE_CSV_FILE)
    with open(ZONE_CSV_FILE, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["timestamp", "name", "alarmClass", "isAlarmed"])
        writer.writerow(
            [
                time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                name,
                alarm.get("alarmClass"),
                alarm.get("isAlarmed"),
            ]
        )

def calc_heading(p1: Dict[str, float], p2: Dict[str, float]) -> float:
    """Return bearing in degrees from p1 to p2."""
    lat1, lon1 = math.radians(p1["lat"]), math.radians(p1["lng"])
    lat2, lon2 = math.radians(p2["lat"]), math.radians(p2["lng"])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def enviar_a_traccar(uid: str, target: dict, point: dict, heading: float) -> None:
    """Send a position update to Traccar via query parameters."""
    device_id = DEVICE_MAP.get(uid)
    if device_id is None:
        return
    params = {
        "id": device_id,
        "lat": point.get("lat"),
        "lon": point.get("lng"),
        "altitude": point.get("alt", 0),
        "accuracy": target.get("accuracy", 0),
        "speed": target.get("speed", 0),
        "course": heading,
        "voltage": target.get("voltage", 0),
        "batteryLevel": target.get("batteryLevel", 0),
        "timestamp": int(time.time() * 1000),
        "satellites": target.get("satellites", 0),
    }
    try:
        requests.get(TRACCAR_URL, params=params, timeout=5)
    except requests.RequestException as exc:
        print("Error enviando a Traccar:", exc)


def enviar_alarma_traccar(name: str, alarm: dict) -> None:
    """Send a zone alarm to Traccar if configured."""
    parts = name.split("_")
    center = parts[1] if len(parts) > 1 else name
    info = ZONE_MAP.get(center)
    if not info:
        return
    params = {
        "id": info.get("device_id"),
        "lat": info.get("lat", 0),
        "lon": info.get("lng", 0),
        "altitude": 0,
        "accuracy": 0,
        "speed": 0,
        "course": 0,
        "voltage": 0,
        "batteryLevel": 0,
        "timestamp": int(time.time() * 1000),
        "satellites": 0,
        "alarm": alarm.get("alarmClass", 0),
    }
    try:
        requests.get(TRACCAR_URL, params=params, timeout=5)
    except requests.RequestException as exc:
        print("Error enviando alarma a Traccar:", exc)

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
                heading = calc_heading(last_tail[-1], p) if last_tail else 0.0
                last_tail.append(p)
                registrar_deteccion(uid, t, p, heading)
                enviar_a_traccar(uid, t, p, heading)


def handle_zone_alarm(alarm: dict) -> None:
    """Process zone alarm messages."""
    if not isinstance(alarm, dict) or not alarm.get("isAlarmed"):
        return
    name = alarm.get("name", "")
    registrar_alarma(name, alarm)
    enviar_alarma_traccar(name, alarm)

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
    elif (
        isinstance(data, list)
        and len(data) >= 2
        and data[0] == "API.Zone.UPD.Alarm"
        and isinstance(data[1], dict)
    ):
        handle_zone_alarm(data[1])

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
        reconnect_delay=1,
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
