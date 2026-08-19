"""The controller contract and the in-flight safety guards built on it.

These guards are what stop the aircraft when telemetry says something is wrong.
They are deliberately independent of any particular mission: hover, velocity
tests, and future tag-search flights all need the same battery, ceiling, and
staleness checks.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class FlightController(Protocol):
    """What a mission needs from a controller in order to be flown safely.

    Every member is required.  ``altitude_is_fresh`` and ``heartbeat_is_fresh``
    are part of the contract rather than optional extras: without them a stale
    telemetry stream cannot be distinguished from a healthy one, and the guards
    below would silently degrade to no-ops.
    """

    battery_voltage: float | None
    current_altitude: float | None
    max_altitude: float
    is_flying: bool
    is_armed: bool

    def update_telemetry(self) -> None: ...
    def emergency_stop(self) -> None: ...
    def altitude_is_fresh(self) -> bool: ...
    def heartbeat_is_fresh(self) -> bool: ...
    def send_velocity_body(
        self, vx: float, vy: float, vz: float, yaw_rate_deg: float = 0.0
    ) -> None: ...


class FlightGuardError(RuntimeError):
    """Raised when a guard has stopped the aircraft."""


def check_safety_guardrails(drone: FlightController, min_battery_v: float) -> None:
    """Stop the aircraft and raise if any bounded flight condition is violated.

    Each failing check calls ``emergency_stop`` before raising, so the caller
    cannot continue commanding a vehicle that has already been told to stop.
    """

    voltage = drone.battery_voltage
    if voltage is not None and 0.0 < voltage < min_battery_v:
        drone.emergency_stop()
        raise FlightGuardError("flight stopped by battery guard")

    altitude = drone.current_altitude
    if altitude is not None and altitude > drone.max_altitude:
        drone.emergency_stop()
        raise FlightGuardError("flight stopped by altitude guard")

    if not drone.is_flying:
        return

    if not drone.altitude_is_fresh():
        drone.emergency_stop()
        raise FlightGuardError("flight stopped because altitude is stale")
    if not drone.heartbeat_is_fresh():
        drone.emergency_stop()
        raise FlightGuardError("flight stopped because heartbeat is stale")
