"""Shared helpers for synchronized, disarmed drone sensor recordings."""

from __future__ import annotations

import json
import statistics
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from itertools import pairwise
from pathlib import Path
from typing import Any

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

HARDWARE_TOPOLOGY: Mapping[str, Mapping[str, str]] = {
    "camera": {
        "sensor": "Raspberry Pi AI Camera (Sony IMX500)",
        "connected_to": "Raspberry Pi Zero 2 W",
        "interface": "CSI/libcamera",
    },
    "downward_range_and_flow": {
        "sensor": "MicoAir MTF-01P",
        "connected_to": "FlywooF745 flight controller",
        "interface": "FC UART5 / ArduPilot SERIAL5, MAVLink1 at 115200",
        "pi_path": "Forwarded as processed MAVLink telemetry over FC UART4",
    },
    "flight_controller_to_pi": {
        "sensor": "ArduPilot telemetry and fused state",
        "connected_to": "Raspberry Pi GPIO UART",
        "interface": "FC UART4 <-> Pi /dev/serial0 (/dev/ttyAMA0), 115200",
    },
    "forward_range_planned": {
        "sensor": "MicoAir MT-15",
        "connected_to": "Not connected yet; planned FlywooF745 UART7",
        "interface": "FC T7/R7, 5V, GND; 3.3V UART signaling at 115200",
    },
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

    root = candidate
    suffix = 1
    while root.exists() and any(root.iterdir()):
        root = candidate.with_name(f"{candidate.name}-{suffix}")
        suffix += 1
    root.mkdir(parents=True, exist_ok=True)
    return RecordingPaths(
        root=root,
        video=root / "camera.h264",
        video_timestamps=root / "camera.pts",
        camera_events=root / "camera.jsonl",
        telemetry_tlog=root / "telemetry.tlog",
        telemetry_events=root / "telemetry.jsonl",
        first_frame=root / "first-frame.jpg",
        last_frame=root / "last-frame.jpg",
        manifest=root / "manifest.json",
    )


def json_safe(value: Any) -> Any:
    """Convert MAVLink and camera metadata values to JSON-compatible objects."""
    if value is None or isinstance(value, str | int | float | bool):
        return value
    if isinstance(value, bytes | bytearray | memoryview):
        return bytes(value).hex()
    if isinstance(value, Mapping):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, tuple | list):
        return [json_safe(item) for item in value]
    if hasattr(value, "item"):
        return json_safe(value.item())
    return str(value)


def telemetry_record(message: Any, *, elapsed_s: float) -> dict[str, Any]:
    """Build a timestamped JSON record from any received MAVLink message."""
    return {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "elapsed_s": round(elapsed_s, 6),
        "message": message.get_type(),
        "source_system": message.get_srcSystem(),
        "source_component": message.get_srcComponent(),
        "fields": json_safe(message.to_dict()),
    }


def write_json_line(handle: Any, record: Mapping[str, Any]) -> None:
    """Write one compact JSON Lines record."""
    handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")))
    handle.write("\n")


def request_telemetry_messages(connection: Any) -> list[str]:
    """Request bounded sensor streams without issuing any actuator command."""
    requested: list[str] = []
    for name, rate_hz in TELEMETRY_RATES_HZ.items():
        message_id = getattr(mavlink, f"MAVLINK_MSG_ID_{name}", None)
        if message_id is None:
            continue
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
        requested.append(name)
    return requested


def is_armed_vehicle_heartbeat(message: Any, *, vehicle_system: int) -> bool:
    """Return whether a heartbeat from the connected vehicle reports armed."""
    if message.get_type() != "HEARTBEAT":
        return False
    if message.get_srcSystem() != vehicle_system:
        return False
    return bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


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
