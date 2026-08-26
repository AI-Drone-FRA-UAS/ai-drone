"""Shared helpers for synchronized drone sensor recordings."""

from __future__ import annotations

import json
import math
import statistics
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import UTC, datetime
from itertools import pairwise
from pathlib import Path
from typing import Any, TextIO

from pymavlink.dialects.v10 import ardupilotmega as mavlink

# Rates are deliberately bounded for the verified 115200-baud Pi/FC link.
# Unsupported messages are harmless: ArduPilot rejects or ignores the request.
TELEMETRY_RATES_HZ: Mapping[str, float] = {
    "ATTITUDE": 20.0,
    "DISTANCE_SENSOR": 20.0,
    "OPTICAL_FLOW": 20.0,
    "OPTICAL_FLOW_RAD": 20.0,
    "LOCAL_POSITION_NED": 10.0,
    "RANGEFINDER": 10.0,
    "RAW_IMU": 10.0,
    "AHRS2": 5.0,
    "BATTERY_STATUS": 2.0,
    "EKF_STATUS_REPORT": 5.0,
    "GLOBAL_POSITION_INT": 5.0,
    "GPS_RAW_INT": 5.0,
    "HIGHRES_IMU": 5.0,
    "RC_CHANNELS": 5.0,
    "SCALED_IMU2": 5.0,
    "SCALED_IMU3": 5.0,
    "SCALED_PRESSURE": 5.0,
    "SERVO_OUTPUT_RAW": 5.0,
    "SYS_STATUS": 5.0,
    "VFR_HUD": 5.0,
    "VIBRATION": 5.0,
    "HOME_POSITION": 1.0,
    "SYSTEM_TIME": 1.0,
}


@dataclass(frozen=True)
class RecordingPaths:
    """Files produced by one all-sensor recording."""

    root: Path
    video: Path
    video_timestamps: Path
    camera_events: Path
    telemetry_tlog: Path
    telemetry_events: Path
    actuation_events: Path
    first_frame: Path
    last_frame: Path
    manifest: Path


def create_recording_paths(
    output: Path | None = None,
    *,
    now: datetime | None = None,
) -> RecordingPaths:
    """Create a unique output directory and return its standard file paths."""
    if output is None:
        current = now or datetime.now().astimezone()
        base = Path("artifacts") / "sensor-recordings"
        candidate = base / current.strftime("%Y%m%d-%H%M%S")
    else:
        candidate = output

    suffix = 0
    while True:
        root = (
            candidate
            if suffix == 0
            else candidate.with_name(f"{candidate.name}-{suffix}")
        )
        try:
            # An exclusive mkdir is the reservation. An exists()/mkdir(exist_ok=True)
            # pair lets concurrent recorders select and write into the same dataset.
            root.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            suffix += 1
        else:
            break
    return RecordingPaths(
        root=root,
        video=root / "camera.h264",
        video_timestamps=root / "camera.pts",
        camera_events=root / "camera.jsonl",
        telemetry_tlog=root / "telemetry.tlog",
        telemetry_events=root / "telemetry.jsonl",
        actuation_events=root / "servo.jsonl",
        first_frame=root / "first-frame.jpg",
        last_frame=root / "last-frame.jpg",
        manifest=root / "manifest.json",
    )


def json_safe(value: Any) -> Any:
    """Convert MAVLink and camera metadata values to JSON-compatible objects."""
    if value is None or isinstance(value, str | int | bool):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, bytes | bytearray | memoryview):
        return bytes(value).hex()
    if isinstance(value, Mapping):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, tuple | list):
        return [json_safe(item) for item in value]
    if hasattr(value, "tolist"):
        return json_safe(value.tolist())
    if hasattr(value, "item"):
        return json_safe(value.item())
    return str(value)


def telemetry_record(message: Any, *, elapsed_s: float) -> dict[str, Any]:
    """Build a timestamped JSON record from any received MAVLink message."""
    return {
        "timestamp_utc": datetime.now(UTC).isoformat(),
        "elapsed_s": round(elapsed_s, 6),
        "message": message.get_type(),
        "source_system": message.get_srcSystem(),
        "source_component": message.get_srcComponent(),
        "fields": json_safe(message.to_dict()),
    }


def write_json_line(handle: TextIO, record: Mapping[str, Any]) -> None:
    """Write one compact, standards-compliant JSON Lines record."""
    handle.write(
        json.dumps(
            json_safe(record),
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    )
    handle.write("\n")


def request_message_intervals(
    connection: Any,
    rates_hz: Mapping[int, float],
) -> None:
    """Request bounded MAVLink streams without issuing actuator commands."""

    for message_id, rate_hz in rates_hz.items():
        if message_id < 0:
            raise ValueError("MAVLink message IDs must be non-negative")
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("MAVLink message rates must be finite and positive")
        interval_us = round(1_000_000 / rate_hz)
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )


def request_telemetry_messages(connection: Any) -> list[str]:
    """Request bounded sensor streams without issuing any actuator command."""
    requested: list[str] = []
    rates_by_id: dict[int, float] = {}
    for name, rate_hz in TELEMETRY_RATES_HZ.items():
        message_id = getattr(mavlink, f"MAVLINK_MSG_ID_{name}", None)
        if message_id is None:
            continue
        rates_by_id[int(message_id)] = rate_hz
        requested.append(name)
    request_message_intervals(connection, rates_by_id)
    return requested


def video_timestamp_summary(path: Path) -> dict[str, int | float]:
    """Summarize Picamera2's millisecond PTS sidecar without decoding video."""
    if not path.exists():
        return {
            "encoded_frames": 0,
            "encoded_span_s": 0.0,
            "encoded_duration_s": 0.0,
        }
    timestamps = [
        float(line)
        for line in path.read_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    span_s = (timestamps[-1] - timestamps[0]) / 1000.0 if len(timestamps) > 1 else 0.0
    frame_period_ms = (
        statistics.median(
            current - previous for previous, current in pairwise(timestamps)
        )
        if len(timestamps) > 1
        else 0.0
    )
    return {
        "encoded_frames": len(timestamps),
        "encoded_span_s": round(span_s, 6),
        "encoded_duration_s": round(span_s + frame_period_ms / 1000.0, 6),
    }
