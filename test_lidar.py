#!/usr/bin/env python3
"""Sample MTF-01P range/flow telemetry from ArduPilot and save it as CSV.

This script never arms the vehicle, changes flight mode, drives a servo, or
runs a motor test. It only asks the flight controller to stream existing
sensor messages for the current MAVLink connection.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

MESSAGE_TYPES = (
    "HEARTBEAT",
    "RANGEFINDER",
    "DISTANCE_SENSOR",
    "OPTICAL_FLOW",
    "OPTICAL_FLOW_RAD",
)
CSV_FIELDS = (
    "timestamp_utc",
    "elapsed_s",
    "message",
    "distance_m",
    "voltage_v",
    "quality",
    "flow_x",
    "flow_y",
    "ground_distance_m",
)


def _find_device(requested: str | None) -> Path:
    if requested:
        path = Path(requested)
        if not path.exists():
            raise SystemExit(f"Serial device does not exist: {path}")
        return path

    candidates = [
        *sorted(Path("/dev/serial/by-id").glob("*ArduPilot*")),
        *sorted(Path("/dev").glob("ttyACM*")),
        Path("/dev/serial0"),
    ]
    for path in candidates:
        if path.exists():
            return path
    raise SystemExit("No ArduPilot serial device found. Use --device /dev/...")


def _is_armed(heartbeat: Any) -> bool:
    return bool(heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def _request_sensor_messages(connection: Any) -> None:
    message_ids = (
        mavlink.MAVLINK_MSG_ID_RANGEFINDER,
        mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD,
    )
    for message_id in message_ids:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            100_000,
            0,
            0,
            0,
            0,
            0,
        )


def _row(message: Any, started: float) -> dict[str, str | float | int]:
    message_type = message.get_type()
    row: dict[str, str | float | int] = {field: "" for field in CSV_FIELDS}
    row.update(
        timestamp_utc=datetime.now(timezone.utc).isoformat(),
        elapsed_s=round(time.monotonic() - started, 3),
        message=message_type,
    )

    if message_type == "RANGEFINDER":
        row["distance_m"] = float(message.distance)
        row["voltage_v"] = float(message.voltage)
    elif message_type == "DISTANCE_SENSOR":
        row["distance_m"] = float(message.current_distance) / 100.0
    elif message_type == "OPTICAL_FLOW":
        row["quality"] = int(message.quality)
        row["flow_x"] = int(message.flow_x)
        row["flow_y"] = int(message.flow_y)
        row["ground_distance_m"] = float(message.ground_distance)
    elif message_type == "OPTICAL_FLOW_RAD":
        row["quality"] = int(message.quality)
        row["flow_x"] = float(message.integrated_x)
        row["flow_y"] = float(message.integrated_y)
        row["ground_distance_m"] = float(message.distance)
    return row


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return Path("artifacts") / f"mtf-01p-{stamp}.csv"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read MTF-01P telemetry for a few seconds and save CSV output."
    )
    parser.add_argument("--device", help="MAVLink serial device (auto-detected)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be greater than zero")

    device = _find_device(args.device)
    output = args.output or _default_output()
    output.parent.mkdir(parents=True, exist_ok=True)

    print(f"Connecting to {device} at {args.baud} baud ...")
    connection = mavutil.mavlink_connection(str(device), baud=args.baud)
    heartbeat = connection.wait_heartbeat(timeout=args.timeout)
    if heartbeat is None:
        connection.close()
        raise SystemExit("No ArduPilot heartbeat received.")
    if _is_armed(heartbeat):
        connection.close()
        raise SystemExit("Vehicle is ARMED; aborting without requesting sensor data.")

    print("Vehicle is DISARMED. Requesting sensor telemetry only.")
    print("The drone battery must power the MTF-01P; this script cannot power it.")
    _request_sensor_messages(connection)

    rows: list[dict[str, str | float | int]] = []
    distances: list[float] = []
    qualities: list[int] = []
    started = time.monotonic()
    became_armed = False

    try:
        while time.monotonic() - started < args.duration:
            message = connection.recv_match(
                type=list(MESSAGE_TYPES), blocking=True, timeout=0.5
            )
            if message is None:
                continue
            if message.get_type() == "HEARTBEAT":
                if _is_armed(message):
                    became_armed = True
                    print("Vehicle became ARMED; stopping capture immediately.")
                    break
                continue

            row = _row(message, started)
            rows.append(row)
            if row["distance_m"] != "":
                distances.append(float(row["distance_m"]))
            if row["quality"] != "":
                qualities.append(int(row["quality"]))
    finally:
        connection.close()

    with output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved {len(rows)} sensor messages to {output}")
    positive_distances = [distance for distance in distances if distance > 0]
    if positive_distances:
        print(
            "Rangefinder: working, "
            f"samples={len(positive_distances)}, "
            f"median={statistics.median(positive_distances):.2f} m, "
            f"min={min(positive_distances):.2f} m, "
            f"max={max(positive_distances):.2f} m"
        )
    elif distances:
        print("Rangefinder messages arrived, but all distances were zero.")
    else:
        print("No rangefinder messages arrived.")

    if qualities:
        print(
            f"Optical flow: samples={len(qualities)}, "
            f"median quality={statistics.median(qualities):.0f}/255, "
            f"max={max(qualities)}/255"
        )
    else:
        print("No optical-flow messages arrived.")

    if became_armed:
        return 3
    return 0 if positive_distances else 2


if __name__ == "__main__":
    raise SystemExit(main())
