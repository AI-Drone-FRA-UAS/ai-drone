"""Sample MTF-01P range/flow telemetry from ArduPilot and save it as CSV.

This script never arms the vehicle, changes flight mode, drives a servo, or
runs a motor test. It only asks the flight controller to stream existing
sensor messages for the current MAVLink connection.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import tempfile
import time
from collections import defaultdict
from collections.abc import Mapping, Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.mavlink.devices import find_serial_device
from ai_drone.mavlink.safety import heartbeat_is_armed, is_vehicle_message
from ai_drone.recording import request_message_intervals

MESSAGE_TYPES = (
    "HEARTBEAT",
    "RANGEFINDER",
    "DISTANCE_SENSOR",
    "OPTICAL_FLOW",
    "OPTICAL_FLOW_RAD",
)
SENSOR_RATES_HZ: Mapping[int, float] = {
    mavlink.MAVLINK_MSG_ID_RANGEFINDER: 10.0,
    mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: 10.0,
    mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW: 10.0,
    mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: 10.0,
}
CSV_FIELDS = (
    "timestamp_utc",
    "elapsed_s",
    "message",
    "sensor_id",
    "sensor_type",
    "sensor_type_name",
    "orientation",
    "orientation_name",
    "distance_m",
    "distance_valid",
    "min_distance_m",
    "max_distance_m",
    "covariance_cm2",
    "voltage_v",
    "quality",
    "flow_x",
    "flow_y",
    "ground_distance_m",
)

CsvValue = str | float | int
CsvRow = dict[str, CsvValue]


def _find_device(requested: str | None) -> Path:
    try:
        return find_serial_device(
            requested,
            include_pi_uart=True,
            missing_message="No ArduPilot serial device found. Use --device /dev/...",
        )
    except FileNotFoundError as error:
        raise SystemExit(str(error)) from error


def _request_sensor_messages(connection: Any) -> None:
    request_message_intervals(connection, SENSOR_RATES_HZ)


def _enum_name(group: str, value: int) -> str:
    entry = mavlink.enums.get(group, {}).get(value)
    return entry.name if entry is not None else f"UNKNOWN_{value}"


def _finite_float(value: Any) -> float | str:
    converted = float(value)
    return converted if math.isfinite(converted) else ""


def _row(message: Any, started: float) -> CsvRow:
    message_type = message.get_type()
    row: CsvRow = {field: "" for field in CSV_FIELDS}
    row.update(
        timestamp_utc=datetime.now(UTC).isoformat(),
        elapsed_s=round(time.monotonic() - started, 3),
        message=message_type,
    )

    if message_type == "RANGEFINDER":
        distance = _finite_float(message.distance)
        row["distance_m"] = distance
        row["distance_valid"] = int(isinstance(distance, float) and distance > 0)
        row["voltage_v"] = _finite_float(message.voltage)
    elif message_type == "DISTANCE_SENSOR":
        current_cm = int(message.current_distance)
        minimum_cm = int(message.min_distance)
        maximum_cm = int(message.max_distance)
        sensor_type = int(message.type)
        orientation = int(message.orientation)
        row.update(
            sensor_id=int(message.id),
            sensor_type=sensor_type,
            sensor_type_name=_enum_name("MAV_DISTANCE_SENSOR", sensor_type),
            orientation=orientation,
            orientation_name=_enum_name("MAV_SENSOR_ORIENTATION", orientation),
            distance_m=current_cm / 100.0,
            distance_valid=int(
                current_cm > 0
                and minimum_cm <= current_cm <= maximum_cm
                and maximum_cm >= minimum_cm
            ),
            min_distance_m=minimum_cm / 100.0,
            max_distance_m=maximum_cm / 100.0,
            covariance_cm2=int(message.covariance),
        )
    elif message_type == "OPTICAL_FLOW":
        row["quality"] = int(message.quality)
        row["flow_x"] = int(message.flow_x)
        row["flow_y"] = int(message.flow_y)
        row["ground_distance_m"] = _finite_float(message.ground_distance)
    elif message_type == "OPTICAL_FLOW_RAD":
        row["quality"] = int(message.quality)
        row["flow_x"] = _finite_float(message.integrated_x)
        row["flow_y"] = _finite_float(message.integrated_y)
        row["ground_distance_m"] = _finite_float(message.distance)
    return row


def _distance_stream_label(row: Mapping[str, CsvValue]) -> str:
    if row["message"] == "DISTANCE_SENSOR":
        return (
            f"DISTANCE_SENSOR id={row['sensor_id']}, "
            f"type={row['sensor_type_name']}, "
            f"orientation={row['orientation_name']}"
        )
    return "RANGEFINDER legacy (sensor ID and orientation unavailable)"


def _distance_groups(
    rows: Sequence[Mapping[str, CsvValue]],
) -> tuple[dict[str, list[float]], int]:
    groups: dict[str, list[float]] = defaultdict(list)
    range_message_count = 0
    for row in rows:
        if row["message"] not in {"RANGEFINDER", "DISTANCE_SENSOR"}:
            continue
        range_message_count += 1
        distance = row["distance_m"]
        if row["distance_valid"] == 1 and isinstance(distance, float):
            groups[_distance_stream_label(row)].append(distance)
    return dict(groups), range_message_count


def _print_sensor_summary(rows: Sequence[Mapping[str, CsvValue]]) -> bool:
    distance_groups, range_message_count = _distance_groups(rows)
    if distance_groups:
        for label, distances in sorted(distance_groups.items()):
            print(
                f"{label}: working, samples={len(distances)}, "
                f"median={statistics.median(distances):.2f} m, "
                f"min={min(distances):.2f} m, max={max(distances):.2f} m"
            )
    elif range_message_count:
        print("Rangefinder messages arrived, but none contained a valid distance.")
    else:
        print("No rangefinder messages arrived.")

    qualities = [
        int(row["quality"])
        for row in rows
        if row["quality"] != ""
        and row["message"] in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}
    ]
    if qualities:
        print(
            f"Optical flow: samples={len(qualities)}, "
            f"median quality={statistics.median(qualities):.0f}/255, "
            f"max={max(qualities)}/255"
        )
    else:
        print("No optical-flow messages arrived.")
    return bool(distance_groups)


def _write_csv(output: Path, rows: Sequence[Mapping[str, CsvValue]]) -> None:
    """Atomically replace ``output`` with a complete sensor CSV."""

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
            encoding="utf-8",
            newline="",
            delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
            handle.flush()
            os.fsync(handle.fileno())
        temporary_path.replace(output)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return Path("artifacts") / f"mtf-01p-{stamp}.csv"


def _capture_rows(
    connection: Any,
    *,
    duration: float,
    timeout: float,
) -> tuple[list[CsvRow], bool]:
    heartbeat = connection.wait_heartbeat(timeout=timeout)
    if heartbeat is None:
        raise SystemExit("No ArduPilot heartbeat received.")
    if heartbeat_is_armed(heartbeat):
        raise SystemExit("Vehicle is ARMED; aborting without requesting sensor data.")

    print("Vehicle is DISARMED. Requesting sensor telemetry only.")
    print("The drone battery must power the MTF-01P; this script cannot power it.")
    _request_sensor_messages(connection)

    rows: list[CsvRow] = []
    started = time.monotonic()
    became_armed = False
    while time.monotonic() - started < duration:
        message = connection.recv_match(
            type=list(MESSAGE_TYPES), blocking=True, timeout=0.5
        )
        if message is None:
            continue
        if not is_vehicle_message(
            message,
            system_id=int(connection.target_system),
            component_id=int(connection.target_component),
        ):
            continue
        if message.get_type() == "HEARTBEAT":
            if heartbeat_is_armed(message):
                became_armed = True
                print("Vehicle became ARMED; stopping capture immediately.")
                break
            continue
        rows.append(_row(message, started))
    return rows, became_armed


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read MTF-01P telemetry for a few seconds and save CSV output."
    )
    parser.add_argument("--device", help="MAVLink serial device (auto-detected)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(arguments)

    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and greater than zero")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and greater than zero")
    if args.baud <= 0:
        parser.error("--baud must be greater than zero")

    device = _find_device(args.device)
    output = args.output or _default_output()

    print(f"Connecting to {device} at {args.baud} baud ...")
    connection = mavutil.mavlink_connection(str(device), baud=args.baud)
    try:
        rows, became_armed = _capture_rows(
            connection,
            duration=args.duration,
            timeout=args.timeout,
        )
    finally:
        connection.close()

    _write_csv(output, rows)

    print(f"Saved {len(rows)} sensor messages to {output}")
    valid_distances = _print_sensor_summary(rows)

    if became_armed:
        return 3
    return 0 if valid_distances else 2


if __name__ == "__main__":
    raise SystemExit(main())
