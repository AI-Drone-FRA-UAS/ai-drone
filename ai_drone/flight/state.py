"""Typed flight observations and pure vehicle-state updates."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Generic, Literal, TypeVar, assert_never

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.mavlink.safety import distance_sensor_valid, heartbeat_is_armed, is_fresh

T = TypeVar("T")
NAVIGATION_SAMPLE_MAX_AGE_S = 1.0
BATTERY_SAMPLE_MAX_AGE_S = 2.0
MAX_SAMPLE_AGE_MS = 1_000
FUTURE_TOLERANCE_MS = 250


@dataclass(frozen=True)
class Sample(Generic[T]):
    value: T
    received_at: float


def fresh(sample: Sample[T] | None, now: float, max_age: float) -> T | None:
    return (
        sample.value
        if sample is not None and is_fresh(sample.received_at, now, max_age)
        else None
    )


@dataclass(frozen=True)
class Heartbeat:
    armed: bool
    mode: str | None


@dataclass(frozen=True)
class Firmware:
    version: int
    commit: bytes


@dataclass(frozen=True)
class BootClock:
    milliseconds: int
    received_at: float


@dataclass(frozen=True)
class AltitudeAlignment:
    offset: float
    local: Sample[float]


@dataclass(frozen=True)
class ObservationClock:
    kind: str
    received_at: float
    boot_ms: int | None = None


@dataclass(frozen=True)
class VehicleState:
    heartbeat: Sample[Heartbeat] | None = None
    altitude: Sample[float] | None = None
    local_altitude: Sample[float] | None = None
    yaw: Sample[float] | None = None
    battery: Sample[float] | None = None
    ekf_flags: Sample[int] | None = None
    flow_quality: Sample[int] | None = None
    rc_channels: Sample[int] | None = None
    firmware: Sample[Firmware] | None = None
    boot: BootClock | None = None
    alignment: AltitudeAlignment | None = None
    clocks: tuple[ObservationClock, ...] = ()


@dataclass(frozen=True)
class TimedReading:
    kind: Literal["altitude", "local_altitude", "yaw", "rc_channels"]
    value: float | int | None
    boot_ms: object


@dataclass(frozen=True)
class Reading:
    kind: Literal["battery", "ekf_flags", "flow_quality"]
    value: float | int | None


Payload = Heartbeat | Firmware | TimedReading | Reading


@dataclass(frozen=True)
class Observation:
    system: int
    component: int
    received_at: float
    payload: Payload


def _battery_voltage(message: Any) -> float | None:
    millivolts = float(message.voltage_battery)
    healthy = all(
        int(getattr(message, field, 0)) & mavlink.MAV_SYS_STATUS_SENSOR_BATTERY
        for field in (
            "onboard_control_sensors_present",
            "onboard_control_sensors_enabled",
            "onboard_control_sensors_health",
        )
    )
    return millivolts / 1_000.0 if healthy and 0.0 < millivolts < 65_535.0 else None


def _pose_reading(message: Any, kind: str) -> TimedReading | None:
    boot = getattr(message, "time_boot_ms", None)
    if kind == "DISTANCE_SENSOR":
        if int(message.orientation) != mavlink.MAV_SENSOR_ROTATION_PITCH_270:
            return None
        identifier = getattr(message, "id", 0)
        # The reviewed RNGFND1 backend is emitted as FC instance 0. Forward
        # RNGFND2 and another downward instance cannot supply the flight datum.
        if isinstance(identifier, bool) or identifier != 0:
            return None
        value = (
            int(message.current_distance) / 100.0
            if distance_sensor_valid(message)
            else None
        )
        return TimedReading("altitude", value, boot)
    if kind == "RC_CHANNELS":
        count = int(message.chancount)
        return TimedReading("rc_channels", count if 0 <= count <= 18 else None, boot)
    value = float(message.yaw) if kind == "ATTITUDE" else -float(message.z)
    field = "yaw" if kind == "ATTITUDE" else "local_altitude"
    return TimedReading(field, value if math.isfinite(value) else None, boot)


def decode(
    message: Any, *, received: float, fallback_mode: object = None
) -> Observation | None:
    """Decode a MAVLink packet once; transport owns its original receipt time."""
    kind = message.get_type()
    payload: Payload | None
    if kind == "HEARTBEAT":
        mode = (
            mavutil.mode_string_v10(message)
            if isinstance(getattr(message, "custom_mode", None), int)
            else fallback_mode
        )
        payload = Heartbeat(
            heartbeat_is_armed(message), mode if isinstance(mode, str) else None
        )
    elif kind == "AUTOPILOT_VERSION":
        payload = Firmware(
            int(message.flight_sw_version), bytes(message.flight_custom_version)
        )
    elif kind in {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR", "RC_CHANNELS"}:
        payload = _pose_reading(message, kind)
    elif kind == "SYS_STATUS":
        payload = Reading("battery", _battery_voltage(message))
    elif kind == "EKF_STATUS_REPORT":
        payload = Reading("ekf_flags", int(message.flags))
    elif kind in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}:
        payload = Reading("flow_quality", int(message.quality))
    else:
        return None
    return (
        None
        if payload is None
        else Observation(
            message.get_srcSystem(), message.get_srcComponent(), received, payload
        )
    )


def advance_boot(
    clock: BootClock | None, raw: object, received: float
) -> BootClock | None:
    if isinstance(raw, bool) or not isinstance(raw, int) or not 0 < raw < 2**32:
        return None
    if clock is None:
        return BootClock(raw, received)
    elapsed_ms = round((received - clock.received_at) * 1_000)
    delta = (raw - clock.milliseconds + 2**31) % 2**32 - 2**31
    if not -MAX_SAMPLE_AGE_MS <= delta - elapsed_ms <= FUTURE_TOLERANCE_MS:
        return None
    return BootClock(raw, received) if delta > 0 else clock


def align_altitude(state: VehicleState, now: float) -> VehicleState:
    local = fresh(state.local_altitude, now, NAVIGATION_SAMPLE_MAX_AGE_S)
    distance = fresh(state.altitude, now, NAVIGATION_SAMPLE_MAX_AGE_S)
    if local is None or distance is None or state.local_altitude is None:
        return state
    offset = distance - local if state.alignment is None else state.alignment.offset
    return replace(
        state,
        alignment=AltitudeAlignment(
            offset, Sample(local + offset, state.local_altitude.received_at)
        ),
    )


def _reading(state: VehicleState, payload: Reading, received: float) -> VehicleState:
    match payload.kind:
        case "battery":
            return replace(
                state,
                battery=None
                if payload.value is None
                else Sample(float(payload.value), received),
            )
        case "ekf_flags":
            return replace(
                state,
                ekf_flags=None
                if payload.value is None
                else Sample(int(payload.value), received),
            )
        case "flow_quality":
            return replace(
                state,
                flow_quality=None
                if payload.value is None
                else Sample(int(payload.value), received),
            )
        case _ as impossible:
            assert_never(impossible)


def _advance_observation(
    state: VehicleState, kind: str, received: float, *, boot_ms: int | None = None
) -> VehicleState | None:
    previous = next((clock for clock in state.clocks if clock.kind == kind), None)
    if previous is not None:
        if received < previous.received_at:
            return None
        if boot_ms is not None and previous.boot_ms is not None:
            delta = (boot_ms - previous.boot_ms + 2**31) % 2**32 - 2**31
            if delta <= 0:
                return None
    clock = ObservationClock(kind, received, boot_ms)
    return replace(
        state,
        clocks=(*(item for item in state.clocks if item.kind != kind), clock),
    )


def _invalidate_timed(state: VehicleState, kind: str) -> VehicleState:
    match kind:
        case "altitude":
            return replace(state, altitude=None)
        case "local_altitude":
            return replace(state, local_altitude=None)
        case "yaw":
            return replace(state, yaw=None)
        case "rc_channels":
            return replace(state, rc_channels=None)
        case _:
            raise ValueError(f"unknown timed reading {kind!r}")


def _timed_reading(
    state: VehicleState, payload: TimedReading, received: float
) -> VehicleState:
    boot = advance_boot(state.boot, payload.boot_ms, received)
    if boot is None:
        return state
    assert isinstance(payload.boot_ms, int)  # advance_boot validated representation
    accepted = _advance_observation(
        state, payload.kind, received, boot_ms=payload.boot_ms
    )
    if accepted is None:
        return state
    state = replace(accepted, boot=boot)
    if payload.value is None:
        return _invalidate_timed(state, payload.kind)
    match payload.kind:
        case "altitude":
            return align_altitude(
                replace(state, altitude=Sample(float(payload.value), received)),
                received,
            )
        case "local_altitude":
            return align_altitude(
                replace(state, local_altitude=Sample(float(payload.value), received)),
                received,
            )
        case "yaw":
            return replace(state, yaw=Sample(float(payload.value), received))
        case "rc_channels":
            return replace(state, rc_channels=Sample(int(payload.value), received))
        case _ as impossible:
            assert_never(impossible)


def observe(
    state: VehicleState,
    observation: Observation,
    *,
    now: float,
    target_system: int = 1,
    target_component: int = 1,
    heartbeat_max_age: float = 2.5,
) -> VehicleState:
    if (observation.system, observation.component) != (target_system, target_component):
        return state
    received = observation.received_at
    if not math.isfinite(received) or not math.isfinite(now) or received > now:
        return state
    match observation.payload:
        case Heartbeat() as heartbeat:
            if not is_fresh(received, now, heartbeat_max_age) or (
                state.heartbeat is not None and received < state.heartbeat.received_at
            ):
                return state
            if heartbeat.mode is None and state.heartbeat is not None:
                heartbeat = replace(heartbeat, mode=state.heartbeat.value.mode)
            return replace(
                state,
                heartbeat=Sample(heartbeat, received),
                alignment=state.alignment if heartbeat.armed else None,
            )
        case Firmware() as firmware:
            accepted = _advance_observation(state, "firmware", received)
            return (
                state
                if accepted is None
                else replace(accepted, firmware=Sample(firmware, received))
            )
        case Reading() as payload:
            accepted = _advance_observation(state, payload.kind, received)
            return state if accepted is None else _reading(accepted, payload, received)
        case TimedReading() as payload:
            return _timed_reading(state, payload, received)
        case _ as impossible:
            assert_never(impossible)
