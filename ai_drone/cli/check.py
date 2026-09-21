"""Bounded, disarmed FC bench checks without a flight-controller lifecycle."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from dataclasses import dataclass, field, replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Literal

from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.cli.harness import connection_scope
from ai_drone.flight import params as flight_config
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
from ai_drone.mavlink.shared import received_monotonic
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
    if is_raspberry_pi():
        from ai_drone.settings import load_settings

        shared = Path(load_settings().runtime.socket)
        if shared.is_socket():
            return f"unix:{shared}"
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


@dataclass(frozen=True)
class Diagnostic:
    severity: Literal["error", "warning"]
    message: str

    def to_dict(self) -> dict[str, str]:
        return {"severity": self.severity, "message": self.message}


@dataclass(frozen=True)
class CheckAnalysis:
    firmware: dict[str, Any]
    sensors: dict[str, Any]
    diagnostics: tuple[Diagnostic, ...]

    def to_dict(self) -> dict[str, Any]:
        return {"firmware": dict(self.firmware), "sensors": dict(self.sensors)}


@dataclass(frozen=True)
class ObservationKey:
    system: int
    component: int
    message: str
    sensor_id: int | None = None
    orientation: int | None = None
    parameter_name: str | None = None
    command: int | None = None


def _wire_id(value: Any) -> int:
    if type(value) is not int or not 0 <= value <= 255:
        raise ValueError("invalid sensor identity")
    return value


def observation_key(
    kind: str, fields: dict[str, Any] | None = None, *, source: tuple[int, int] = (1, 1)
) -> ObservationKey:
    fields = fields or {}
    sensor = orientation = command = None
    parameter = None
    if kind in {"DISTANCE_SENSOR", "RAW_IMU", "HIGHRES_IMU", "BATTERY_STATUS"}:
        sensor = _wire_id(fields.get("id", 0))
    elif kind in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}:
        sensor = _wire_id(fields.get("sensor_id", 0))
    elif kind in {"SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"}:
        sensor = {"SCALED_IMU": 0, "SCALED_IMU2": 1, "SCALED_IMU3": 2}[kind]
    if kind == "DISTANCE_SENSOR":
        orientation = _wire_id(fields.get("orientation"))
    if kind == "PARAM_VALUE":
        value = fields.get("param_id")
        if not isinstance(value, str | bytes):
            raise ValueError("invalid parameter identity")
        parameter = decode_parameter_name(value)
        if not parameter:
            raise ValueError("empty parameter identity")
    if kind == "COMMAND_ACK":
        command = fields.get("command")
        if type(command) is not int or not 0 <= command <= 65535:
            raise ValueError("invalid command identity")
    return ObservationKey(*source, kind, sensor, orientation, parameter, command)


@dataclass
class Observations:
    """Only messages from the exact project FC contribute to this report."""

    latest: dict[ObservationKey, dict[str, Any]] = field(default_factory=dict)
    seen: dict[ObservationKey, float] = field(default_factory=dict)
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
        try:
            key = observation_key(
                kind,
                fields,
                source=(message.get_srcSystem(), message.get_srcComponent()),
            )
        except (ValueError, UnicodeError):
            self.warnings.append(f"{kind}: invalid observation identity ignored")
            return
        now = time.monotonic()
        received = received_monotonic(message, default=now)
        if received > now or received < self.seen.get(key, -math.inf):
            return
        self.latest[key] = fields
        self.seen[key] = received
        self._observe_diagnostics(kind, fields)

    def _observe_diagnostics(self, kind: str, fields: dict[str, Any]) -> None:
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

    def fresh(
        self,
        kind: str,
        now: float,
        *,
        sensor_id: int | None = None,
        orientation: int | None = None,
    ) -> dict[str, Any] | None:
        fields = {}
        if sensor_id is not None:
            fields.update(id=sensor_id, sensor_id=sensor_id)
        if orientation is not None:
            fields["orientation"] = orientation
        key = observation_key(kind, fields)
        age = now - self.seen.get(key, -math.inf)
        if key not in self.latest or not 0 <= age <= FRESHNESS_S:
            label = f"{kind}:{orientation}" if orientation is not None else kind
            self.errors.append(f"{label}: missing or stale FC telemetry")
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
    data = observed.latest.get(observation_key("AUTOPILOT_VERSION"), {})
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
    expected_id = 0 if orientation == 25 else 1
    data = (
        observed.fresh(
            "DISTANCE_SENSOR", now, sensor_id=expected_id, orientation=orientation
        )
        or {}
    )
    if not data and any(
        key.message == "DISTANCE_SENSOR"
        and key.orientation == orientation
        and key.sensor_id != expected_id
        for key in observed.latest
    ):
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


def _imu_axes(observed: Observations, now: float) -> dict[str, list[float | None]]:
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
    return axes


def _flow_summary(observed: Observations, now: float) -> dict[str, float | None]:
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
    return {"quality": quality, **velocity}


def _sensor_summary(
    observed: Observations, *, min_battery: float, now: float
) -> dict[str, Any]:
    observed.fresh("HEARTBEAT", now)
    system = observed.fresh("SYS_STATUS", now) or {}
    voltage = _finite(system.get("voltage_battery"))
    battery = voltage / 1000 if voltage is not None and 0 < voltage < 65535 else None
    if battery is None or battery < min_battery:
        observed.errors.append(
            f"Battery voltage {battery!r} V is missing or below {min_battery:g} V"
        )
    axes = _imu_axes(observed, now)
    attitude = observed.fresh("ATTITUDE", now) or {}
    angles = {name: _finite(attitude.get(name)) for name in ("roll", "pitch", "yaw")}
    if any(value is None for value in angles.values()):
        observed.errors.append("ATTITUDE: missing/invalid angles")
    pressure = _finite((observed.fresh("SCALED_PRESSURE", now) or {}).get("press_abs"))
    if pressure is None or pressure <= 0:
        observed.errors.append("Barometer: missing/invalid pressure")
    flow = _flow_summary(observed, now)
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
        "optical_flow": flow,
        "imu": {
            "acceleration_raw": axes["acc"],
            "gyro_raw": axes["gyro"],
            "compass_raw": axes["mag"],
            "units": "raw; MAVLink RAW_IMU does not specify calibration scales",
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


def analyze_observations(
    observed: Observations, *, now: float, min_battery: float, prearm: bool = False
) -> CheckAnalysis:
    """Analyze a fixed observation without I/O, clock reads or caller mutation."""
    # Working diagnostics belong to this invocation. Copy every mutable collection
    # so historical observations remain reusable for alternate analysis times.
    local = replace(
        observed,
        latest={key: dict(value) for key, value in observed.latest.items()},
        seen=dict(observed.seen),
        counts=observed.counts.copy(),
        parameters=dict(observed.parameters),
        status_text=list(observed.status_text),
        errors=[],
        warnings=[],
    )
    sensors = _sensor_summary(local, min_battery=min_battery, now=now)
    if prearm and local.prearm_result != mavlink.MAV_RESULT_ACCEPTED:
        local.errors.append(
            f"Pre-arm diagnostic request was not accepted (result={local.prearm_result!r})"
        )
    for text in local.status_text:
        if "prearm:" in text.lower():
            local.errors.append(f"FC status: {text}")
    firmware = _firmware(local)
    diagnostics = tuple(
        Diagnostic("error", message) for message in local.errors
    ) + tuple(Diagnostic("warning", message) for message in local.warnings)
    return CheckAnalysis(firmware=firmware, sensors=sensors, diagnostics=diagnostics)


def check_connection(
    connection: Any,
    observed: Observations,
    *,
    duration: float,
    timeout: float,
    parameter_timeout: float,
    prearm: bool,
    min_battery: float,
) -> CheckAnalysis:
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
    return analyze_observations(
        observed, now=time.monotonic(), min_battery=min_battery, prearm=prearm
    )


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
        "Camera and servo feedback are outside this check; use drone walk for camera recording."
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
        with connection_scope(
            connection,
            close_error=lambda error: observed.errors.append(
                f"Closing connection failed: {error}"
            ),
        ):
            analysis = check_connection(
                connection,
                observed,
                duration=args.duration,
                timeout=args.timeout,
                parameter_timeout=args.parameter_timeout,
                prearm=args.prearm,
                min_battery=args.min_battery,
            )
            report.update(analysis.to_dict())
            for diagnostic in analysis.diagnostics:
                collection = (
                    observed.errors
                    if diagnostic.severity == "error"
                    else observed.warnings
                )
                collection.append(diagnostic.message)
    except (OSError, RuntimeError, TimeoutError, ValueError, TypeError) as error:
        observed.errors.append(str(error))
    except KeyboardInterrupt:
        observed.errors.append("Operator interrupted the bench check")
    report.update(
        passed=not observed.errors,
        elapsed_s=round(time.monotonic() - started, 3),
        errors=list(dict.fromkeys(observed.errors)),
        warnings=list(dict.fromkeys(observed.warnings)),
        diagnostics=[
            Diagnostic(severity, message).to_dict()
            for severity, messages in (
                ("error", observed.errors),
                ("warning", observed.warnings),
            )
            for message in dict.fromkeys(messages)
        ],
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
