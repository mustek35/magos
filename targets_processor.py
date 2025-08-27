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

TRACCAR_HTTP_URL = "http://179.57.170.61:5055"
DEVICE_MAP: Dict[str, str] = {
    # "uid": "device_id"
}

# Optional mapping for zone alarms: zone name -> {device_id?, lat, lon, alt}
# If ``device_id`` is omitted, the script uses the center name extracted from the
# alarm (e.g., ``Alarm_Huelmo_M`` -> ``Huelmo``) as the Traccar device id.
ZONE_MAP: Dict[str, Dict[str, float | str]] = {
    # Example: "Alarm_Huelmo_M": {
    #     "lat": -41.6659,
    #     "lon": -73.0527,
    #     "alt": 0,
    #     # optional: "device_id": "Huelmo",
    # }
}

CSV_FILE = "detections.csv"
ALARM_FILE = "alarms.csv"


def obtener_nombre_centro(nombre: str) -> str | None:
    """Extrae el nombre del centro desde el identificador de alarma."""
    partes = nombre.split("_")
    if len(partes) >= 2:
        return partes[1]
    return None

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


def registrar_alarma(alarm: dict) -> None:
    """Store zone alarm information in a CSV file."""
    file_exists = os.path.exists(ALARM_FILE)
    with open(ALARM_FILE, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["timestamp", "uid", "name", "alarmClass", "isAlarmed"])
        writer.writerow(
            [
                time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                alarm.get("uid"),
                alarm.get("name"),
                alarm.get("alarmClass"),
                alarm.get("isAlarmed"),
            ]
        )

def enviar_a_traccar(
    device_id: str,
    lat: float,
    lon: float,
    alt: float = 0.0,
    speed: float = 0.0,
    accuracy: float = 10.0,
    course: float = 0.0,
    voltage: float | None = None,
    battery_level: float | None = None,
    satellites: int | None = None,
) -> None:
    """Send a position update to Traccar using the HTTP protocol."""
    params = {
        "id": device_id,
        "lat": lat,
        "lon": lon,
        "altitude": alt,
        "speed": speed,
        "accuracy": accuracy,
        "course": course,
        "timestamp": int(time.time()),
    }
    if voltage is not None:
        params["voltage"] = voltage
    if battery_level is not None:
        params["batteryLevel"] = f"{battery_level:.2f}"
    if satellites is not None:
        params["satellites"] = satellites
    try:
        requests.get(TRACCAR_HTTP_URL, params=params, timeout=5)
    except requests.RequestException as exc:
        print("Error enviando a Traccar:", exc)


def enviar_target_a_traccar(uid: str, target: dict, point: dict) -> None:
    """Send a target position to Traccar if a device mapping exists."""
    device_id = DEVICE_MAP.get(uid)
    if not device_id:
        return
    enviar_a_traccar(
        device_id,
        point.get("lat"),
        point.get("lng"),
        point.get("alt", 0.0),
        speed=target.get("speed", 0.0),
    )

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
                enviar_target_a_traccar(uid, t, p)


def handle_zone_alarm(alarm: dict) -> None:
    """Process zone alarm messages."""
    registrar_alarma(alarm)
    name = alarm.get("name")
    info = ZONE_MAP.get(name, {})
    if alarm.get("isAlarmed") and name:
        device_id = info.get("device_id") or obtener_nombre_centro(name)
        lat = info.get("lat")
        lon = info.get("lon")
        alt = info.get("alt", 0.0)
        if device_id and lat is not None and lon is not None:
            enviar_a_traccar(device_id, lat, lon, alt)

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
