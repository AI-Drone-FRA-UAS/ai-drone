"""Bounded, disarmed FC bench checks without a flight-controller lifecycle."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight import controller as flight_config
from ai_drone.mavlink.connection import open_ardupilot_connection
from ai_drone.mavlink.devices import (
    STABLE_FLIGHT_CONTROLLER_DEVICE,
    resolve_mavlink_endpoint,
)
from ai_drone.mavlink.ownership import require_available_serial
from ai_drone.mavlink.parameters import decode_parameter_name
from ai_drone.mavlink.safety import (
    heartbeat_is_armed,
    is_vehicle_message,
    require_ardupilot_heartbeat,
    require_fresh_disarmed_heartbeat,
)
from ai_drone.platform import is_raspberry_pi
from ai_drone.recording import json_safe, request_message_intervals

RATES_HZ = {
    "SYS_STATUS": 2.0,
    "ATTITUDE": 5.0,
    "RAW_IMU": 5.0,
    "SCALED_PRESSURE": 2.0,
    "DISTANCE_SENSOR": 5.0,
    "OPTICAL_FLOW": 5.0,
    "EKF_STATUS_REPORT": 2.0,
    "RC_CHANNELS": 2.0,
}
HEALTH_BITS = {
    "gyro": mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
    "accelerometer": mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL,
    "compass": mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
    "barometer": mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
    "optical_flow": mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW,
    "rangefinder": mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION,
    "battery": mavlink.MAV_SYS_STATUS_SENSOR_BATTERY,
}
FRESHNESS_S = 2.5


def _resolve_check_endpoint(requested: str | None) -> str:
    if requested:
        return resolve_mavlink_endpoint(requested)
    if STABLE_FLIGHT_CONTROLLER_DEVICE.exists():
        return str(STABLE_FLIGHT_CONTROLLER_DEVICE)
    if is_raspberry_pi() and Path("/dev/serial0").exists():
        return "/dev/serial0"
    raise FileNotFoundError(
        "Project FC USB device or Pi UART not found; select another endpoint explicitly with --device"
    )


def _finite(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    return float(value) if math.isfinite(value) else None


@dataclass
class Observations:
    """Only messages from the exact project FC contribute to this report."""

    latest: dict[str, dict[str, Any]] = field(default_factory=dict)
    seen: dict[str, float] = field(default_factory=dict)
    counts: Counter[str] = field(default_factory=Counter)
    parameters: dict[str, float] = field(default_factory=dict)
    status_text: list[str] = field(default_factory=list)
    prearm_result: int | None = None
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def observe(self, message: Any) -> None:
        if message is None or not is_vehicle_message(
            message, system_id=1, component_id=1
        ):
            return
        kind = message.get_type()
        if kind == "BAD_DATA":
            return
        if kind == "HEARTBEAT" and heartbeat_is_armed(message):
            raise RuntimeError("FC reported ARMED; bench check aborted")
        self.counts[kind] += 1
        fields = message.to_dict()
        key = (
            f"DISTANCE_SENSOR:{fields.get('orientation')}"
            if kind == "DISTANCE_SENSOR"
            else kind
        )
        self.latest[key] = fields
        self.seen[key] = time.monotonic()
        if (
            kind == "COMMAND_ACK"
            and fields.get("command") == mavlink.MAV_CMD_RUN_PREARM_CHECKS
        ):
            self.prearm_result = int(fields["result"])
        if kind == "PARAM_VALUE":
            value = _finite(fields.get("param_value"))
            if value is not None:
                self.parameters[decode_parameter_name(fields["param_id"])] = value
        if kind == "STATUSTEXT":
            text = fields.get("text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            text = str(text).split("\0", 1)[0]
            if text and text not in self.status_text:
                self.status_text.append(text)

    def fresh(self, key: str, now: float) -> dict[str, Any] | None:
        age = now - self.seen.get(key, -math.inf)
        if key not in self.latest or not 0 <= age <= FRESHNESS_S:
            self.errors.append(f"{key}: missing or stale FC telemetry")
            return None
        return self.latest[key]


def expected_parameters() -> dict[str, float]:
    """Use the active flight gate; this full-airframe check also requires MT-15."""
    return {
        **flight_config.REQUIRED_NOGPS_LOITER_PARAMETERS,
        **flight_config.FORWARD_RANGEFINDER_PARAMETERS,
        "ARMING_SKIPCHK": 0.0,
        "RNGFND2_TYPE": 10.0,
    }


def _command(connection: Any, command: int, parameter: float = 0.0) -> None:
    if command not in (
        mavlink.MAV_CMD_REQUEST_MESSAGE,
        mavlink.MAV_CMD_RUN_PREARM_CHECKS,
    ):
        raise ValueError("command is outside the bench-check allowlist")
    connection.mav.command_long_send(1, 1, command, 0, parameter, 0, 0, 0, 0, 0, 0)


def _read_parameters(
    connection: Any, observed: Observations, *, timeout: float
) -> None:
    required = expected_parameters()
    deadline = time.monotonic() + timeout
    retry_at = time.monotonic() + timeout / 2
    retried = False

    def request_missing() -> None:
        for name in required.keys() - observed.parameters.keys():
            connection.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)

    request_missing()
    while required.keys() - observed.parameters.keys():
        now = time.monotonic()
        if now >= deadline:
            break
        if not retried and now >= retry_at:
            request_missing()
            retried = True
        observed.observe(
            connection.recv_match(blocking=True, timeout=min(0.25, deadline - now))
        )
    for name, expected in required.items():
        actual = observed.parameters.get(name)
        if actual is None:
            observed.errors.append(f"{name}: parameter unavailable")
        elif not math.isclose(
            actual,
            expected,
            rel_tol=0.0,
            abs_tol=flight_config.PARAMETER_ABS_TOLERANCE,
        ):
            observed.errors.append(f"{name}={actual:g}; expected {expected:g}")


def _firmware(observed: Observations) -> dict[str, Any]:
    data = observed.latest.get("AUTOPILOT_VERSION", {})
    packed = data.get("flight_sw_version", 0)
    version = tuple((int(packed) >> shift) & 255 for shift in (24, 16, 8))
    custom = data.get("flight_custom_version", b"")
    identity = custom.encode("ascii") if isinstance(custom, str) else bytes(custom)
    matches = (
        version == flight_config.EXPECTED_FIRMWARE_VERSION
        and identity == flight_config.EXPECTED_FIRMWARE_COMMIT
    )
    if not matches:
        observed.errors.append(
            "Firmware version/identity is missing or differs from the active reviewed gate"
        )
    return {
        "version": ".".join(map(str, version)),
        "identity": identity.decode("ascii", errors="replace"),
        "expected_version": ".".join(map(str, flight_config.EXPECTED_FIRMWARE_VERSION)),
        "expected_identity": flight_config.EXPECTED_FIRMWARE_COMMIT.decode("ascii"),
        "matches": matches,
    }


def _health(observed: Observations, data: dict[str, Any]) -> dict[str, Any]:
    masks = {
        name: int(data.get(f"onboard_control_sensors_{name}", 0))
        for name in ("present", "enabled", "health")
    }
    result = {}
    for name, bit in HEALTH_BITS.items():
        values = {label: bool(value & bit) for label, value in masks.items()}
        result[name] = values
        if not all(values.values()):
            observed.errors.append(
                f"{name}: FC does not report present, enabled and healthy"
            )
    bit = mavlink.MAV_SYS_STATUS_PREARM_CHECK
    result["prearm"] = {label: bool(value & bit) for label, value in masks.items()}
    if masks["present"] & bit and not masks["health"] & bit:
        observed.errors.append(
            "FC reports pre-arm checks not healthy; inspect status text"
        )
    return result


def _range(observed: Observations, orientation: int, now: float) -> dict[str, Any]:
    data = observed.fresh(f"DISTANCE_SENSOR:{orientation}", now) or {}
    expected_id = 0 if orientation == 25 else 1
    if data and data.get("id") != expected_id:
        observed.errors.append(
            f"Range orientation {orientation}: expected FC instance ID {expected_id}"
        )
    distance = _finite(data.get("current_distance"))
    minimum, maximum = (
        _finite(data.get("min_distance")),
        _finite(data.get("max_distance")),
    )
    valid = (
        distance is not None
        and minimum is not None
        and maximum is not None
        and 0 < distance < 65535
        and minimum <= distance <= maximum
        and data.get("signal_quality", 0) != 1
    )
    if distance is None or not 0 < distance < 65535:
        observed.errors.append(
            f"Range orientation {orientation}: no finite usable distance"
        )
    elif not valid:
        observed.warnings.append(
            f"Range orientation {orientation}: received but outside valid bounds/quality; bench geometry can cause this"
        )
    return {
        "received": bool(data),
        "orientation": orientation,
        "id": data.get("id"),
        "distance_m": distance / 100 if distance is not None else None,
        "within_reported_bounds": bool(valid),
        "signal_quality": data.get("signal_quality"),
    }


def _sensor_summary(observed: Observations, *, min_battery: float) -> dict[str, Any]:
    now = time.monotonic()
    observed.fresh("HEARTBEAT", now)
    system = observed.fresh("SYS_STATUS", now) or {}
    voltage = _finite(system.get("voltage_battery"))
    battery = voltage / 1000 if voltage is not None and 0 < voltage < 65535 else None
    if battery is None or battery < min_battery:
        observed.errors.append(
            f"Battery voltage {battery!r} V is missing or below {min_battery:g} V"
        )
    imu = observed.fresh("RAW_IMU", now) or {}
    axes = {
        name: [_finite(imu.get(f"{axis}{name}")) for axis in "xyz"]
        for name in ("acc", "gyro", "mag")
    }
    for name, values in axes.items():
        if any(value is None for value in values) or (
            name != "gyro" and not any(values)
        ):
            observed.errors.append(f"RAW_IMU {name}: missing/invalid vector")
    attitude = observed.fresh("ATTITUDE", now) or {}
    angles = {name: _finite(attitude.get(name)) for name in ("roll", "pitch", "yaw")}
    if any(value is None for value in angles.values()):
        observed.errors.append("ATTITUDE: missing/invalid angles")
    pressure = _finite((observed.fresh("SCALED_PRESSURE", now) or {}).get("press_abs"))
    if pressure is None or pressure <= 0:
        observed.errors.append("Barometer: missing/invalid pressure")
    flow = observed.fresh("OPTICAL_FLOW", now) or {}
    quality = _finite(flow.get("quality"))
    if quality is None or not 0 <= quality <= 255:
        observed.errors.append("Optical flow: missing/invalid quality")
    elif quality == 0:
        observed.warnings.append(
            "Optical flow quality is zero; receipt does not establish usable motion sensing"
        )
    velocity = {
        name: _finite(flow.get(name)) for name in ("flow_comp_m_x", "flow_comp_m_y")
    }
    if any(value is None for value in velocity.values()):
        observed.errors.append("Optical flow: missing/invalid compensated velocity")
    channels = (observed.fresh("RC_CHANNELS", now) or {}).get("chancount")
    if channels != 0:
        observed.errors.append(
            f"RC channel count is {channels!r}; reviewed topology requires zero"
        )
    ekf = observed.fresh("EKF_STATUS_REPORT", now) or {}
    flags = int(ekf.get("flags", 0))
    if not flags & mavlink.EKF_ATTITUDE:
        observed.errors.append("EKF attitude estimate is not reported valid")
    relative = bool(flags & mavlink.EKF_POS_HORIZ_REL)
    constant = bool(flags & mavlink.EKF_CONST_POS_MODE)
    if not relative or constant:
        observed.warnings.append(
            "Relative EKF aiding is not established; a stationary low bench check cannot prove optical-flow fusion"
        )
    return {
        "battery_v": battery,
        "minimum_battery_v": min_battery,
        "fc_health": _health(observed, system),
        "downward_range": _range(observed, 25, now),
        "forward_range": _range(observed, 0, now),
        "optical_flow": {
            "quality": quality,
            **velocity,
        },
        "imu": {
            "acceleration_mg": axes["acc"],
            "gyro_mrad_s": axes["gyro"],
            "compass_mgauss": axes["mag"],
        },
        "attitude_rad": angles,
        "pressure_hpa": pressure,
        "rc_channels": channels,
        "ekf": {
            "flags": flags,
            "relative_position": relative,
            "constant_position_mode": constant,
            "flow_fusion": "Not proven by EKF_STATUS_REPORT alone",
        },
    }


def check_connection(
    connection: Any,
    observed: Observations,
    *,
    duration: float,
    timeout: float,
    parameter_timeout: float,
    prearm: bool,
    min_battery: float,
) -> dict[str, Any]:
    """Read one selected disarmed FC; caller owns the serial descriptor only."""
    first = require_ardupilot_heartbeat(
        connection, system_id=1, component_id=1, timeout=timeout
    )
    observed.observe(first)
    observed.observe(
        require_fresh_disarmed_heartbeat(
            connection, system_id=1, component_id=1, timeout=timeout
        )
    )
    connection.target_system = connection.target_component = 1
    _command(
        connection,
        mavlink.MAV_CMD_REQUEST_MESSAGE,
        mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
    )
    _read_parameters(connection, observed, timeout=parameter_timeout)
    observed.observe(
        require_fresh_disarmed_heartbeat(
            connection, system_id=1, component_id=1, timeout=timeout
        )
    )
    request_message_intervals(
        connection,
        {
            getattr(mavlink, f"MAVLINK_MSG_ID_{name}"): rate
            for name, rate in RATES_HZ.items()
        },
    )
    _command(
        connection,
        mavlink.MAV_CMD_REQUEST_MESSAGE,
        mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
    )
    if prearm:
        _command(connection, mavlink.MAV_CMD_RUN_PREARM_CHECKS)
    deadline = time.monotonic() + duration
    while (remaining := deadline - time.monotonic()) > 0:
        observed.observe(
            connection.recv_match(blocking=True, timeout=min(remaining, 0.25))
        )
    observed.observe(
        require_fresh_disarmed_heartbeat(
            connection, system_id=1, component_id=1, timeout=timeout
        )
    )
    sensors = _sensor_summary(observed, min_battery=min_battery)
    if prearm and observed.prearm_result != mavlink.MAV_RESULT_ACCEPTED:
        observed.errors.append(
            f"Pre-arm diagnostic request was not accepted (result={observed.prearm_result!r})"
        )
    for text in observed.status_text:
        if "prearm:" in text.lower():
            observed.errors.append(f"FC status: {text}")
    return {"firmware": _firmware(observed), "sensors": sensors}


def _print_report(report: dict[str, Any]) -> None:
    print("Bench check: " + ("PASS" if report["passed"] else "FAIL"))
    print(
        "Disarmed sensor/configuration check; not flight clearance or a full ROMFS feature verification."
    )
    if firmware := report.get("firmware"):
        print(
            f"Firmware: {firmware['version']} {firmware['identity']} (expected {firmware['expected_version']} {firmware['expected_identity']})"
        )
    if sensors := report.get("sensors"):
        print(
            f"Battery: {sensors['battery_v']} V; RC channels: {sensors['rc_channels']}"
        )
        for name in (
            "downward_range",
            "forward_range",
            "optical_flow",
            "imu",
            "attitude_rad",
            "pressure_hpa",
            "ekf",
            "fc_health",
        ):
            print(f"{name}: {json.dumps(sensors[name], ensure_ascii=True)}")
    for name in ("errors", "warnings", "status_text"):
        for text in report[name]:
            print(f"{name}: {json.dumps(text, ensure_ascii=True)}")
    print(
        "Camera and servo feedback are outside this check; use drone-walk for camera recording."
    )


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--timeout", type=float, default=5.0, help="Per-heartbeat wait, seconds"
    )
    parser.add_argument(
        "--parameter-timeout",
        type=float,
        default=15.0,
        help="Total parameter phase deadline, seconds",
    )
    parser.add_argument("--min-battery", type=float, default=14.4)
    parser.add_argument(
        "--prearm",
        action="store_true",
        help="Request non-arming pre-arm checks and capture FC status text",
    )
    parser.add_argument("--json", action="store_true", help="Print the report as JSON")
    args = parser.parse_args(arguments)
    for name, maximum in (
        ("duration", 60),
        ("timeout", 15),
        ("parameter_timeout", 30),
        ("min_battery", 60),
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0 < value <= maximum:
            parser.error(
                f"--{name.replace('_', '-')} must be finite, positive and at most {maximum}"
            )
    if not 0 < args.baud <= 3_000_000:
        parser.error("--baud must be between 1 and 3000000")
    observed = Observations()
    connection = None
    report: dict[str, Any] = {
        "started_utc": datetime.now(UTC).isoformat(),
        "source": {"system": 1, "component": 1},
    }
    started = time.monotonic()
    try:
        endpoint = _resolve_check_endpoint(args.device)
        report["source"] = {
            "system": 1,
            "component": 1,
            "endpoint": endpoint,
            "baud": args.baud,
        }
        require_available_serial(endpoint, on_pi=is_raspberry_pi())
        connection = open_ardupilot_connection(
            endpoint,
            baud=args.baud,
            source_system=255,
            source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        report.update(
            check_connection(
                connection,
                observed,
                duration=args.duration,
                timeout=args.timeout,
                parameter_timeout=args.parameter_timeout,
                prearm=args.prearm,
                min_battery=args.min_battery,
            )
        )
    except (OSError, RuntimeError, TimeoutError, ValueError, TypeError) as error:
        observed.errors.append(str(error))
    except KeyboardInterrupt:
        observed.errors.append("Operator interrupted the bench check")
    finally:
        if connection is not None:
            try:
                connection.close()
            except Exception as error:
                observed.errors.append(f"Closing connection failed: {error}")
    report.update(
        passed=not observed.errors,
        elapsed_s=round(time.monotonic() - started, 3),
        errors=list(dict.fromkeys(observed.errors)),
        warnings=list(dict.fromkeys(observed.warnings)),
        status_text=observed.status_text,
        messages=dict(observed.counts),
        parameters=observed.parameters,
        prearm_requested=args.prearm,
        prearm_command_result=observed.prearm_result,
        limitation="Bench check only. Firmware identity is not compiled-feature proof; flash acceptance verifies ROMFS and artifact hashes. No camera or actuator test.",
    )
    report = json_safe(report)
    if args.json:
        print(json.dumps(report, indent=2, allow_nan=False))
    else:
        _print_report(report)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
