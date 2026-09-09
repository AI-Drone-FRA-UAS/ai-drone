"""Offline CSV and chart data from a recording; never opens a drone connection."""

from __future__ import annotations

import csv
import json
import math
from collections import Counter
from collections.abc import Iterator
from contextlib import ExitStack
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

COMMON = ["elapsed_s", "timestamp_utc", "message", "source_system", "source_component"]
CSV_FIELDS = {
    "ranges.csv": [
        *COMMON,
        "direction",
        "sensor_id",
        "orientation",
        "distance_m",
        "minimum_m",
        "maximum_m",
        "signal_quality",
        "valid",
        "invalid_reason",
    ],
    "optical_flow.csv": [
        *COMMON,
        "sensor_id",
        "quality",
        "valid",
        "invalid_reason",
        "flow_x_deprecated_rad_s",
        "flow_y_deprecated_rad_s",
        "velocity_x_m_s",
        "velocity_y_m_s",
        "rate_x_rad_s",
        "rate_y_rad_s",
        "integration_time_us",
        "integrated_x_rad",
        "integrated_y_rad",
        "distance_m",
        "distance_valid",
    ],
    "imu.csv": [
        *COMMON,
        "sensor_id",
        "acceleration_x_m_s2",
        "acceleration_y_m_s2",
        "acceleration_z_m_s2",
        "gyro_x_rad_s",
        "gyro_y_rad_s",
        "gyro_z_rad_s",
        "mag_x_mgauss",
        "mag_y_mgauss",
        "mag_z_mgauss",
        "xacc_raw",
        "yacc_raw",
        "zacc_raw",
        "xgyro_raw",
        "ygyro_raw",
        "zgyro_raw",
        "xmag_raw",
        "ymag_raw",
        "zmag_raw",
        "temperature_c",
    ],
    "motion.csv": [
        *COMMON,
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "north_m",
        "east_m",
        "down_m",
        "velocity_north_m_s",
        "velocity_east_m_s",
        "velocity_down_m_s",
    ],
    "environment.csv": [
        *COMMON,
        "pressure_hpa",
        "temperature_c",
        "voltage_v",
        "current_a",
        "remaining_percent",
    ],
    "battery.csv": [
        *COMMON,
        "battery_id",
        "voltage_v",
        "voltages_mv",
        "voltages_ext_mv",
        "current_a",
        "remaining_percent",
        "temperature_c",
        "consumed_mah",
        "consumed_j",
        "time_remaining_s",
        "charge_state",
        "fault_bitmask",
    ],
    "events.csv": [
        *COMMON,
        "armed",
        "custom_mode",
        "ekf_flags",
        "sensors_present",
        "sensors_enabled",
        "sensors_healthy",
        "severity",
        "text",
    ],
    "camera.csv": [
        "elapsed_s",
        "timestamp_utc",
        "frame",
        "sensor_timestamp_ns",
        "exposure_us",
        "frame_duration_us",
        "analogue_gain",
        "lux",
        "focus_fom",
        "lens_position",
        "tag_ids",
    ],
}


def number(value: Any) -> float | None:
    """Reject unavailable/nonfinite fields rather than plotting them as zero."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    try:
        return float(value) if math.isfinite(value) else None
    except OverflowError:
        return None


def scaled(fields: dict[str, Any], name: str, factor: float = 1) -> float | None:
    value = number(fields.get(name))
    return None if value is None else number(value * factor)


def _integer(value: Any, maximum: int, minimum: int = 0) -> int | None:
    """Validate integer wire fields without treating JSON booleans as IDs."""
    if type(value) is int and minimum <= value <= maximum:
        return value
    return None


def _records(path: Path, warnings: Counter[str]) -> Iterator[dict[str, Any]]:
    try:
        handle = path.open("rb")
    except OSError:
        warnings[
            f"Missing or unreadable {path.name}; this part of the recording is unavailable."
        ] += 1
        return
    # Decode each binary line inside the try block so a corrupt UTF-8 row cannot
    # prevent review of the subsequent intact records in an interrupted file.
    with handle:
        for line in handle:
            try:
                row = json.loads(line)
            except (ValueError, UnicodeError):
                warnings[
                    f"Unreadable rows in {path.name} were skipped; raw data is preserved."
                ] += 1
                continue
            if (
                not isinstance(row, dict)
                or number(row.get("elapsed_s")) is None
                or row["elapsed_s"] < 0
            ):
                warnings[
                    f"Rows without a valid elapsed time in {path.name} were skipped."
                ] += 1
                continue
            yield row


@dataclass
class Series:
    key: str
    label: str
    panel: str
    unit: str
    bucket_s: float
    points: dict[int, list[list[Any]]] = field(default_factory=dict)
    samples: int = 0
    valid_samples: int = 0
    minimum: float | None = None
    maximum: float | None = None

    def add(self, elapsed: float, value: float | None, valid: bool = True) -> None:
        self.samples += 1
        valid = valid and value is not None
        if valid:
            self.valid_samples += 1
            assert value is not None
            self.minimum = value if self.minimum is None else min(self.minimum, value)
            self.maximum = value if self.maximum is None else max(self.maximum, value)
        point = [elapsed, value, valid]
        bucket = int(elapsed / self.bucket_s)
        previous = self.points.get(bucket, [])
        candidates = [*previous, point]
        finite = [p for p in candidates if p[2]]
        invalid = [p for p in candidates if not p[2]]
        selected = [candidates[0], point]
        if finite:
            selected += [
                min(finite, key=lambda p: p[1]),
                max(finite, key=lambda p: p[1]),
            ]
        if invalid:
            selected.append(invalid[0])
        self.points[bucket] = sorted(
            {tuple(p): p for p in selected}.values(), key=lambda p: p[0]
        )

    def chart(self) -> dict[str, Any]:
        points = sorted(
            (p for bucket in self.points.values() for p in bucket), key=lambda p: p[0]
        )
        return {
            "id": self.key,
            "label": self.label,
            "panel": self.panel,
            "unit": self.unit,
            "points": points,
            "gap_s": 2.0,
        }

    def summary(self) -> dict[str, Any]:
        return {
            "label": self.label,
            "samples": self.samples,
            "valid_samples": self.valid_samples,
            "min": self.minimum,
            "max": self.maximum,
            "unit": self.unit,
        }


class Export:
    """Stream complete CSVs while retaining a bounded min/max chart envelope."""

    def __init__(self, output: Path, duration: float, stack: ExitStack) -> None:
        self.bucket_s = max(duration / 800, 0.025)
        self.series: dict[str, Series] = {}
        self.writers = {}
        self.rows: Counter[str] = Counter()
        for name, fields in CSV_FIELDS.items():
            handle = stack.enter_context(
                (output / name).open("w", newline="", encoding="utf-8")
            )
            writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            self.writers[name] = writer

    def csv(self, name: str, row: dict[str, Any]) -> None:
        # Keep spreadsheet text inert without changing numeric negative values.
        safe = {
            key: "'" + value
            if isinstance(value, str) and value.startswith(("=", "+", "-", "@"))
            else value
            for key, value in row.items()
        }
        self.writers[name].writerow(safe)
        self.rows[name] += 1

    def point(
        self,
        key: str,
        label: str,
        panel: str,
        unit: str,
        elapsed: float,
        value: float | None,
        valid: bool = True,
    ) -> None:
        if key not in self.series:
            self.series[key] = Series(key, label, panel, unit, self.bucket_s)
        self.series[key].add(elapsed, value, valid)


def _range(export: Export, base: dict[str, Any], f: dict[str, Any]) -> None:
    orientation = _integer(f.get("orientation"), 255)
    direction = (
        {0: "forward", 25: "downward"}.get(orientation, f"orientation_{orientation}")
        if orientation is not None
        else "unknown"
    )
    distance, minimum, maximum = (
        scaled(f, key, 0.01)
        for key in ("current_distance", "min_distance", "max_distance")
    )
    quality = _integer(f.get("signal_quality", 0), 100)
    reason = ""
    if distance is None or distance <= 0:
        reason = "missing_or_nonpositive_distance"
    elif (
        minimum is None
        or maximum is None
        or minimum < 0
        or maximum <= 0
        or minimum > maximum
    ):
        reason = "missing_or_invalid_limits"
    elif not minimum <= distance <= maximum:
        reason = "outside_configured_range"
    if quality is None or quality == 1:
        reason = "invalid_signal_quality"
    if orientation is None:
        reason = "invalid_orientation"
    valid = not reason
    export.csv(
        "ranges.csv",
        {
            **base,
            "direction": direction,
            "sensor_id": f.get("id"),
            "orientation": f.get("orientation"),
            "distance_m": distance,
            "minimum_m": minimum,
            "maximum_m": maximum,
            "signal_quality": f.get("signal_quality", 0),
            "valid": valid,
            "invalid_reason": reason,
        },
    )
    if orientation in (0, 25):
        label = "Forward MT-15" if orientation == 0 else "Downward MTF-01P"
        export.point(
            f"range_{direction}",
            label,
            "Distance",
            "m",
            base["elapsed_s"],
            distance,
            valid,
        )


def _flow(export: Export, base: dict[str, Any], f: dict[str, Any]) -> None:
    quality = _integer(f.get("quality"), 255)
    valid = quality is not None and quality > 0
    reason = "" if valid else "invalid_quality"
    row = {**base, "sensor_id": f.get("sensor_id"), "quality": number(f.get("quality"))}
    if base["message"] == "OPTICAL_FLOW":
        row.update(
            flow_x_deprecated_rad_s=number(f.get("flow_x")),
            flow_y_deprecated_rad_s=number(f.get("flow_y")),
            velocity_x_m_s=number(f.get("flow_comp_m_x")),
            velocity_y_m_s=number(f.get("flow_comp_m_y")),
            rate_x_rad_s=number(f.get("flow_rate_x")),
            rate_y_rad_s=number(f.get("flow_rate_y")),
            distance_m=number(f.get("ground_distance")),
        )
    else:
        row.update(
            integration_time_us=number(f.get("integration_time_us")),
            integrated_x_rad=number(f.get("integrated_x")),
            integrated_y_rad=number(f.get("integrated_y")),
            distance_m=number(f.get("distance")),
        )
        if row["integration_time_us"] is None or row["integration_time_us"] <= 0:
            reason = reason or "missing_or_nonpositive_integration_time"
    distance = row["distance_m"]
    # Unknown flow distance does not invalidate the independent flow sample.
    row["distance_valid"] = distance is not None and (
        distance > 0 if base["message"] == "OPTICAL_FLOW" else distance >= 0
    )
    row.update(valid=not reason, invalid_reason=reason)
    export.csv("optical_flow.csv", row)
    suffix = "flow" if base["message"] == "OPTICAL_FLOW" else "flow_rad"
    export.point(
        f"{suffix}_quality",
        f"{base['message']} quality",
        "Optical flow quality",
        "0-255",
        base["elapsed_s"],
        quality,
        valid,
    )
    for axis in ("x", "y"):
        if base["message"] == "OPTICAL_FLOW":
            export.point(
                f"flow_velocity_{axis}",
                f"Compensated flow {axis}",
                "Optical flow velocity",
                "m/s",
                base["elapsed_s"],
                row[f"velocity_{axis}_m_s"],
                valid,
            )
            if f"flow_rate_{axis}" in f:
                export.point(
                    f"flow_rate_{axis}",
                    f"Flow rate {axis}",
                    "Optical flow angular rate",
                    "rad/s",
                    base["elapsed_s"],
                    row[f"rate_{axis}_rad_s"],
                    valid,
                )
        else:
            export.point(
                f"flow_integrated_{axis}",
                f"Integrated flow {axis}",
                "Integrated optical flow",
                "rad",
                base["elapsed_s"],
                row[f"integrated_{axis}_rad"],
                not reason,
            )


def _imu(export: Export, base: dict[str, Any], f: dict[str, Any]) -> None:
    highres = base["message"] == "HIGHRES_IMU"
    raw = base["message"] == "RAW_IMU"
    sensor = f.get("id", {"SCALED_IMU2": 1, "SCALED_IMU3": 2}.get(base["message"], 0))
    row = {**base, "sensor_id": sensor}
    temperature = number(f.get("temperature"))
    row["temperature_c"] = (
        temperature if highres else (temperature / 100 if temperature else None)
    )
    for axis in ("x", "y", "z"):
        if raw:
            # RAW_IMU has no calibrated units in the MAVLink schema. Preserve
            # its raw values; do not infer a scale from a stationary sample.
            for kind, panel in (
                ("acc", "Raw accelerometer"),
                ("gyro", "Raw gyroscope"),
                ("mag", "Raw magnetometer"),
            ):
                name = f"{axis}{kind}_raw"
                row[name] = number(f.get(f"{axis}{kind}"))
                export.point(
                    f"RAW_IMU_{sensor}_{kind}_{axis}",
                    f"RAW_IMU {sensor} {axis}",
                    panel,
                    "raw",
                    base["elapsed_s"],
                    row[name],
                )
            continue
        row[f"acceleration_{axis}_m_s2"] = scaled(
            f, f"{axis}acc", 1 if highres else 0.00980665
        )
        row[f"gyro_{axis}_rad_s"] = scaled(f, f"{axis}gyro", 1 if highres else 0.001)
        row[f"mag_{axis}_mgauss"] = scaled(f, f"{axis}mag", 1000 if highres else 1)
        for kind, field_name, panel, unit in (
            ("acceleration", f"acceleration_{axis}_m_s2", "Accelerometer", "m/s²"),
            ("gyro", f"gyro_{axis}_rad_s", "Gyroscope", "rad/s"),
            ("mag", f"mag_{axis}_mgauss", "Magnetometer", "mG"),
        ):
            export.point(
                f"{base['message']}_{sensor}_{kind}_{axis}",
                f"{base['message']} {axis}",
                panel,
                unit,
                base["elapsed_s"],
                row[field_name],
            )
    export.csv("imu.csv", row)


def _motion(export: Export, base: dict[str, Any], f: dict[str, Any]) -> None:
    row = dict(base)
    if base["message"] == "ATTITUDE":
        for axis in ("roll", "pitch", "yaw"):
            value = scaled(f, axis, 180 / math.pi)
            row[f"{axis}_deg"] = value
            export.point(axis, axis.title(), "Attitude", "°", base["elapsed_s"], value)
    else:
        for axis, field_name in (("north", "x"), ("east", "y"), ("down", "z")):
            row[f"{axis}_m"] = number(f.get(field_name))
            row[f"velocity_{axis}_m_s"] = number(f.get("v" + field_name))
            export.point(
                f"position_{axis}",
                f"EKF {axis}",
                "Estimated local position",
                "m",
                base["elapsed_s"],
                row[f"{axis}_m"],
            )
    export.csv("motion.csv", row)


def _environment(export: Export, base: dict[str, Any], f: dict[str, Any]) -> None:
    row = dict(base)
    if base["message"] == "SYS_STATUS":
        voltage, current, remaining = (
            number(f.get(n))
            for n in ("voltage_battery", "current_battery", "battery_remaining")
        )
        row.update(
            voltage_v=voltage / 1000
            if voltage is not None and 0 <= voltage < 65535
            else None,
            current_a=current / 100 if current is not None and current != -1 else None,
            remaining_percent=remaining
            if remaining is not None and 0 <= remaining <= 100
            else None,
        )
        export.point(
            "battery_voltage",
            "Battery voltage",
            "Battery",
            "V",
            base["elapsed_s"],
            row["voltage_v"],
        )
        export.point(
            "battery_current",
            "Battery current",
            "Battery current",
            "A",
            base["elapsed_s"],
            row["current_a"],
        )
        export.csv(
            "events.csv",
            {
                **base,
                "sensors_present": f.get("onboard_control_sensors_present"),
                "sensors_enabled": f.get("onboard_control_sensors_enabled"),
                "sensors_healthy": f.get("onboard_control_sensors_health"),
            },
        )
    else:
        row.update(
            pressure_hpa=number(f.get("press_abs")),
            temperature_c=scaled(f, "temperature", 0.01),
        )
        export.point(
            "pressure",
            "Pressure",
            "Barometer",
            "hPa",
            base["elapsed_s"],
            row["pressure_hpa"],
        )
        export.point(
            "temperature",
            "Barometer temperature",
            "Temperature",
            "°C",
            base["elapsed_s"],
            row["temperature_c"],
        )
    export.csv("environment.csv", row)


def _battery(export: Export, base: dict[str, Any], f: dict[str, Any]) -> None:
    """Keep per-battery details separate from the SYS_STATUS battery estimate."""
    cells, extra = f.get("voltages"), f.get("voltages_ext", [])
    total = None
    if (
        isinstance(cells, list)
        and 1 <= len(cells) <= 10
        and isinstance(extra, list)
        and len(extra) <= 4
    ):
        values = [_integer(value, 65535) for value in cells]
        extensions = [_integer(value, 65535) for value in extra]
        if all(value is not None for value in values + extensions):
            # Cell 0 can instead contain the complete pack voltage. Both
            # forms sum correctly; the two arrays use different sentinels.
            measured = [
                value for value in values if value is not None and value != 65535
            ]
            measured += [
                value for value in extensions if value is not None and value != 0
            ]
            if measured:
                total = sum(measured) / 1000
    current = number(f.get("current_battery"))
    temperature = number(f.get("temperature"))
    remaining = number(f.get("battery_remaining"))
    consumed = number(f.get("current_consumed"))
    energy = number(f.get("energy_consumed"))
    time_remaining = number(f.get("time_remaining"))
    export.csv(
        "battery.csv",
        {
            **base,
            "battery_id": f.get("id"),
            "voltage_v": total,
            "voltages_mv": json.dumps(cells),
            "voltages_ext_mv": json.dumps(extra),
            "current_a": current / 100
            if current is not None and current != -1
            else None,
            "remaining_percent": remaining
            if remaining is not None and 0 <= remaining <= 100
            else None,
            "temperature_c": temperature / 100
            if temperature is not None and temperature != 32767
            else None,
            "consumed_mah": consumed
            if consumed is not None and consumed >= 0
            else None,
            "consumed_j": energy * 100 if energy is not None and energy >= 0 else None,
            "time_remaining_s": time_remaining
            if time_remaining is not None and time_remaining > 0
            else None,
            "charge_state": f.get("charge_state"),
            "fault_bitmask": f.get("fault_bitmask"),
        },
    )


def _event(export: Export, base: dict[str, Any], f: dict[str, Any]) -> None:
    row = dict(base)
    if base["message"] == "HEARTBEAT":
        mode = number(f.get("base_mode"))
        armed = bool(int(mode) & 128) if mode is not None else None
        row.update(armed=armed, custom_mode=f.get("custom_mode"))
        export.point(
            "armed",
            "Armed state",
            "Vehicle state",
            "0 / 1",
            base["elapsed_s"],
            float(armed) if armed is not None else None,
        )
    elif base["message"] == "EKF_STATUS_REPORT":
        row["ekf_flags"] = f.get("flags")
    else:
        row.update(severity=f.get("severity"), text=f.get("text"))
    export.csv("events.csv", row)


def _camera(export: Export, row: dict[str, Any]) -> None:
    raw_metadata, raw_tags = row.get("metadata"), row.get("tags")
    metadata = raw_metadata if isinstance(raw_metadata, dict) else {}
    tags = (
        [t for t in raw_tags if isinstance(t, dict)]
        if isinstance(raw_tags, list)
        else []
    )
    values = {
        "elapsed_s": row["elapsed_s"],
        "timestamp_utc": row.get("timestamp_utc"),
        "frame": row.get("frame"),
        "sensor_timestamp_ns": metadata.get("SensorTimestamp"),
        "exposure_us": metadata.get("ExposureTime"),
        "frame_duration_us": metadata.get("FrameDuration"),
        "analogue_gain": metadata.get("AnalogueGain"),
        "lux": metadata.get("Lux"),
        "focus_fom": metadata.get("FocusFoM"),
        "lens_position": metadata.get("LensPosition"),
        "tag_ids": json.dumps([t.get("id") for t in tags if isinstance(t, dict)]),
    }
    export.csv("camera.csv", values)
    export.point(
        "camera_frame",
        "Analyzed camera frame",
        "Camera timeline",
        "frame",
        row["elapsed_s"],
        number(row.get("frame")),
    )
    export.point(
        "camera_tags",
        "Detected AprilTags",
        "AprilTags",
        "count",
        row["elapsed_s"],
        float(len(tags)),
    )


_HANDLERS = {
    "DISTANCE_SENSOR": _range,
    "OPTICAL_FLOW": _flow,
    "OPTICAL_FLOW_RAD": _flow,
    "RAW_IMU": _imu,
    "SCALED_IMU": _imu,
    "SCALED_IMU2": _imu,
    "SCALED_IMU3": _imu,
    "HIGHRES_IMU": _imu,
    "ATTITUDE": _motion,
    "LOCAL_POSITION_NED": _motion,
    "SYS_STATUS": _environment,
    "SCALED_PRESSURE": _environment,
    "BATTERY_STATUS": _battery,
    "HEARTBEAT": _event,
    "EKF_STATUS_REPORT": _event,
    "STATUSTEXT": _event,
}


def export_recording(
    recording: Path, output: Path, *, system: int = 1, component: int = 1
) -> dict[str, Any]:
    """Write full CSVs and return chart data for the selected FC's recording."""
    warnings: Counter[str] = Counter()
    manifest: dict[str, Any] = {}
    try:
        loaded = json.loads((recording / "manifest.json").read_text())
        if not isinstance(loaded, dict):
            raise ValueError("manifest must be an object")
        manifest = loaded
    except (OSError, ValueError):
        warnings[
            "No readable completion manifest: the recording may be interrupted or still running."
        ] += 1
    duration = number(manifest.get("actual_duration_s"))
    if duration is None or duration <= 0:
        # A streaming first pass bounds chart memory even for interrupted captures.
        duration = max(
            (
                row["elapsed_s"]
                for name in ("telemetry.jsonl", "camera.jsonl")
                for row in _records(recording / name, Counter())
            ),
            default=0,
        )
    output.mkdir(parents=True, exist_ok=True)
    messages: Counter[str] = Counter()
    other_sources = outside_window = 0
    with ExitStack() as stack:
        export = Export(output, duration, stack)
        for row in _records(recording / "telemetry.jsonl", warnings):
            if row["elapsed_s"] > duration:
                outside_window += 1
                continue
            if (
                _integer(row.get("source_system"), 255, 1),
                _integer(row.get("source_component"), 255, 1),
            ) != (system, component):
                other_sources += 1
                continue
            name, fields = row.get("message"), row.get("fields")
            if not isinstance(name, str) or not isinstance(fields, dict):
                warnings[
                    "Telemetry rows with an invalid message or fields object were skipped."
                ] += 1
                continue
            messages[name] += 1
            handler = _HANDLERS.get(name)
            if handler:
                handler(export, {key: row.get(key) for key in COMMON}, fields)
        for row in _records(recording / "camera.jsonl", warnings):
            if row["elapsed_s"] <= duration:
                _camera(export, row)
    if other_sources:
        warnings[
            f"Excluded {other_sources} telemetry messages from other MAVLink sources; raw logs retain them."
        ] += 1
    if outside_window:
        warnings[
            f"Excluded {outside_window} telemetry messages outside the recorded capture interval."
        ] += 1
    if not messages.get("HEARTBEAT"):
        warnings[
            "No selected-FC heartbeat was recorded; vehicle state is unavailable."
        ] += 1
    if not messages.get("DISTANCE_SENSOR"):
        warnings[
            "No orientation-labelled range readings were recorded; legacy RANGEFINDER is retained only in the raw log."
        ] += 1
    if messages.get("OPTICAL_FLOW") and messages.get("OPTICAL_FLOW_RAD"):
        warnings[
            "Both optical-flow formats are present; keep their samples separate rather than adding their rates."
        ] += 1
    if messages.get("RAW_IMU"):
        warnings[
            "RAW_IMU fields are shown in raw units because their MAVLink definition does not specify calibration scales."
        ] += 1
    for key in ("range_forward", "range_downward"):
        series = export.series.get(key)
        if series and series.valid_samples < series.samples:
            warnings[
                f"{series.label}: {series.samples - series.valid_samples} readings are invalid or outside FC-configured limits; CSV retains their reported distances."
            ] += 1
    warnings[
        "Times use the recording's elapsed clock. Local position is an EKF estimate; optical flow is not a map or ground-truth path."
    ] += 1
    warnings[
        "Charts retain a bounded min/max envelope; CSVs retain every selected sample. Gaps and invalid readings are not filled in."
    ] += 1
    return {
        "title": recording.name,
        "started_utc": manifest.get("started_utc"),
        "duration_s": duration,
        "completed": manifest.get("completed") is True,
        "error": manifest.get("error"),
        "warnings": [
            message + (f" ({count} rows)" if count > 1 else "")
            for message, count in warnings.items()
        ],
        "vehicle": {"system": system, "component": component},
        "messages": dict(sorted(messages.items())),
        "series": [series.chart() for series in export.series.values()],
        "summary": [
            series.summary()
            for series in export.series.values()
            if series.key
            in (
                "range_forward",
                "range_downward",
                "flow_quality",
                "flow_rad_quality",
                "battery_voltage",
                "camera_frame",
            )
        ],
        "csv_rows": dict(export.rows),
        "manifest": manifest,
    }
