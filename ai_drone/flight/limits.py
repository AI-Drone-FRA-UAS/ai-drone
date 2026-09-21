"""Pure readiness, envelope and progress decisions for the flight shell."""

from __future__ import annotations

from dataclasses import dataclass

from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight.state import Sample, fresh
from ai_drone.validation import finite_in_range

MAX_PHYSICAL_ALTITUDE_M = 0.8
TAKEOFF_OVERSHOOT_RESERVE_M = 0.05


@dataclass(frozen=True)
class FlightLimits:
    ceiling_m: float = MAX_PHYSICAL_ALTITUDE_M
    min_battery_voltage: float = 0.0

    def __post_init__(self) -> None:
        finite_in_range(
            self.ceiling_m, "max_altitude", minimum=0.1, maximum=MAX_PHYSICAL_ALTITUDE_M
        )
        finite_in_range(
            self.min_battery_voltage, "min_battery_voltage", minimum=0.0, maximum=60.0
        )


@dataclass(frozen=True)
class FlightEvidence:
    altitude: float | None
    aligned_local_altitude: float | None
    navigation_healthy: bool
    no_rc_input: bool
    battery_voltage: float | None
    battery_fresh: bool
    altitude_fresh: bool
    heartbeat_fresh: bool


def violation(
    limits: FlightLimits,
    evidence: FlightEvidence,
    *,
    in_loiter: bool,
    require_flight_telemetry: bool,
) -> str | None:
    """Preserve ordered guards; stale high readings still conservatively guard height."""
    altitudes = (evidence.altitude, evidence.aligned_local_altitude)
    if any(value is not None and value > limits.ceiling_m for value in altitudes):
        measured = max(value for value in altitudes if value is not None)
        return f"altitude {measured:.2f} m exceeds {limits.ceiling_m:.2f} m"
    if in_loiter and not evidence.navigation_healthy:
        return (
            "Loiter navigation became unhealthy (optical flow or relative EKF position)"
        )
    if not evidence.no_rc_input:
        return "autonomous-flight receiver topology changed or RC_CHANNELS became stale"
    if limits.min_battery_voltage > 0.0:
        if not evidence.battery_fresh or evidence.battery_voltage is None:
            return "battery telemetry became stale during flight"
        if evidence.battery_voltage < limits.min_battery_voltage:
            return f"battery {evidence.battery_voltage:.2f} V is below {limits.min_battery_voltage:.2f} V"
    if require_flight_telemetry:
        if not evidence.altitude_fresh:
            return "altitude became stale during flight"
        if not evidence.heartbeat_fresh:
            return "heartbeat became stale during flight"
    return None


def relative_position_ready(
    sample: Sample[int] | None, now: float, max_age: float
) -> bool:
    flags = fresh(sample, now, max_age)
    required = mavlink.EKF_VELOCITY_HORIZ | mavlink.EKF_POS_HORIZ_REL
    return (
        flags is not None
        and flags & required == required
        and not flags & mavlink.EKF_CONST_POS_MODE
    )


def takeoff_ceiling_violation(
    target: float, ground_reference: float | None, ceiling: float
) -> str | None:
    if ground_reference is None:
        return "takeoff requires a fresh ground reference"
    ground = ground_reference
    if ground + target + TAKEOFF_OVERSHOOT_RESERVE_M > ceiling:
        return f"takeoff target {target:.2f} m above ground reference {ground:.2f} m including {TAKEOFF_OVERSHOOT_RESERVE_M:.2f} m overshoot reserve exceeds maximum altitude {ceiling:.2f} m"
    return None


def takeoff_reached(
    sample: Sample[float] | None, *, started: float, ground: float | None, target: float
) -> bool:
    return (
        sample is not None
        and sample.received_at >= started
        and ground is not None
        and sample.value - ground >= target * 0.9
    )
