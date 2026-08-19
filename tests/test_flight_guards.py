"""Offline tests for the shared in-flight safety guards."""

from __future__ import annotations

import pytest

from ai_drone.flight.guards import (
    FlightController,
    FlightGuardError,
    check_safety_guardrails,
)


class _Drone:
    """Minimal controller stand-in that records whether it was stopped."""

    def __init__(self, **overrides: object) -> None:
        self.battery_voltage: float | None = 16.0
        self.current_altitude: float | None = 0.3
        self.max_altitude = 0.8
        self.is_flying = True
        self.is_armed = True
        self.stopped = False
        self._altitude_fresh = True
        self._heartbeat_fresh = True
        for name, value in overrides.items():
            setattr(self, name, value)

    def update_telemetry(self) -> None:
        return

    def emergency_stop(self) -> None:
        self.stopped = True

    def altitude_is_fresh(self) -> bool:
        return self._altitude_fresh

    def heartbeat_is_fresh(self) -> bool:
        return self._heartbeat_fresh

    def send_velocity_body(
        self, vx: float, vy: float, vz: float, yaw_rate_deg: float = 0.0
    ) -> None:
        return


def test_stand_in_satisfies_the_controller_contract() -> None:
    assert isinstance(_Drone(), FlightController)


def test_healthy_telemetry_passes() -> None:
    drone = _Drone()
    check_safety_guardrails(drone, 14.4)
    assert not drone.stopped


@pytest.mark.parametrize(
    ("overrides", "expected"),
    [
        ({"battery_voltage": 14.0}, "battery"),
        ({"current_altitude": 1.5}, "altitude guard"),
        ({"_altitude_fresh": False}, "altitude is stale"),
        ({"_heartbeat_fresh": False}, "heartbeat is stale"),
    ],
)
def test_each_guard_stops_the_aircraft(overrides: dict, expected: str) -> None:
    drone = _Drone(**overrides)
    with pytest.raises(FlightGuardError, match=expected):
        check_safety_guardrails(drone, 14.4)
    assert drone.stopped, "the aircraft must be stopped before the guard raises"


def test_unknown_battery_is_not_treated_as_empty() -> None:
    drone = _Drone(battery_voltage=None)
    check_safety_guardrails(drone, 14.4)
    assert not drone.stopped


def test_staleness_is_only_checked_while_flying() -> None:
    drone = _Drone(is_flying=False, _altitude_fresh=False, _heartbeat_fresh=False)
    check_safety_guardrails(drone, 14.4)
    assert not drone.stopped
