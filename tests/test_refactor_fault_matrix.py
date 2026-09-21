"""Deterministic whole-operation failure paths; all command endpoints are mocks."""

from dataclasses import replace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight.controller import DroneController, FlightSafetyError
from ai_drone.flight.phase import Armed, Flight, Landing, cleanup
from ai_drone.flight.state import Heartbeat, Sample, VehicleState


@pytest.fixture
def operation(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(
        "ai_drone.flight.controller.time.sleep",
        lambda seconds: clock.__setitem__(0, round(clock[0] + seconds, 6)),
    )
    monkeypatch.setattr(
        "ai_drone.flight.controller.open_ardupilot_connection",
        lambda *_args, **_kwargs: pytest.fail("Fault matrix must not open a transport"),
    )
    monkeypatch.setattr(
        DroneController, "find_device", staticmethod(lambda value: value)
    )
    drone = DroneController(device="mock:refactor-fault-matrix")
    drone.connection = MagicMock()
    drone.connection.recv_match.return_value = None
    drone.phase = Armed()
    drone.state = VehicleState(
        heartbeat=Sample(Heartbeat(True, "GUIDED_NOGPS"), clock[0]),
        altitude=Sample(0.05, clock[0]),
        yaw=Sample(0.0, clock[0]),
        rc_channels=Sample(0, clock[0]),
        flow_quality=Sample(100, clock[0]),
    )
    monkeypatch.setattr(drone, "wait_for_altitude", lambda **_kwargs: 0.05)
    return drone, clock


@pytest.mark.parametrize(
    "trajectory", ["no_liftoff", "stalled_climb", "unexpected_descent"]
)
def test_unsuccessful_takeoff_bounds_wait_and_keeps_land_obligation(
    operation, monkeypatch, trajectory
):
    drone, clock = operation

    def telemetry():
        elapsed = clock[0] - 100.0
        height = {
            "no_liftoff": 0.05,
            "stalled_climb": min(0.25, 0.05 + elapsed),
            "unexpected_descent": max(0.05, 0.3 - elapsed),
        }[trajectory]
        drone.state = replace(
            drone.state,
            altitude=Sample(height, clock[0]),
            heartbeat=Sample(Heartbeat(True, "GUIDED_NOGPS"), clock[0]),
            yaw=Sample(0.0, clock[0]),
            rc_channels=Sample(0, clock[0]),
        )

    monkeypatch.setattr(drone, "update_telemetry", telemetry)
    with pytest.raises(TimeoutError, match="takeoff altitude was not reached"):
        drone.takeoff(0.5, timeout=1.0)
    assert clock[0] == 101.0
    assert drone.connection.mav.set_attitude_target_send.call_count == 20
    drone.connection.mav.set_mode_send.assert_called_once_with(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    )
    drone.connection.arducopter_disarm.assert_not_called()
    assert isinstance(drone.phase, Landing)
    assert cleanup(drone.phase) == "land"


def test_navigation_health_flicker_never_accumulates_into_loiter_readiness(
    operation, monkeypatch
):
    drone, clock = operation
    drone.phase = Flight("awaiting_loiter", 0.05)

    def telemetry():
        drone.state = replace(
            drone.state,
            altitude=Sample(0.5, clock[0]),
            heartbeat=Sample(Heartbeat(True, "GUIDED_NOGPS"), clock[0]),
            yaw=Sample(0.0, clock[0]),
        )

    monkeypatch.setattr(drone, "update_telemetry", telemetry)
    monkeypatch.setattr(
        drone, "navigation_is_healthy", lambda: not 100.4 <= clock[0] < 100.6
    )
    with pytest.raises(FlightSafetyError, match="never established stable"):
        drone.wait_for_relative_position(timeout=1.0, stable_for=0.5)
    sends = drone.connection.mav.set_attitude_target_send.call_args_list
    assert len(sends) == 20
    assert all(call.args[-1] == 0.5 for call in sends)
    drone.connection.mav.set_mode_send.assert_called_once_with(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    )
    drone.connection.arducopter_disarm.assert_not_called()


def test_recovered_inputs_do_not_resume_altitude_hold_after_a_failed_mission(operation):
    drone, clock = operation
    drone.phase = Flight("holding_altitude", 0.05)
    drone.state = replace(drone.state, altitude=Sample(0.5, 90.0))
    with pytest.raises(FlightSafetyError, match="altitude became stale"):
        drone.hold_altitude(0.2)
    attempts = len(drone.command_attempts)
    drone.state = replace(drone.state, altitude=Sample(0.5, clock[0]))
    with pytest.raises(FlightSafetyError, match="requires a controller takeoff"):
        drone.hold_altitude(0.2)
    assert len(drone.command_attempts) == attempts
    drone.connection.mav.set_attitude_target_send.assert_not_called()
    drone.connection.arducopter_disarm.assert_not_called()
    assert cleanup(drone.phase) == "land"


@pytest.mark.parametrize("method", ["takeoff", "enter_loiter", "hold_loiter"])
def test_landing_latch_blocks_public_mission_reentry(operation, monkeypatch, method):
    drone, clock = operation
    previous = Landing(Flight("loitering", 0.05))
    drone.phase = previous
    mode = "LOITER" if method == "hold_loiter" else "GUIDED_NOGPS"
    drone.state = replace(
        drone.state, heartbeat=Sample(Heartbeat(True, mode), clock[0])
    )
    monkeypatch.setattr(drone, "wait_for_relative_position", lambda **_kwargs: None)
    monkeypatch.setattr(drone, "navigation_is_healthy", lambda: True)
    monkeypatch.setattr(drone, "set_mode", lambda _mode: None)
    monkeypatch.setattr(
        drone,
        "update_telemetry",
        lambda: setattr(
            drone, "state", replace(drone.state, altitude=Sample(0.55, clock[0]))
        ),
    )
    with pytest.raises(FlightSafetyError, match=r"[Ll][Aa][Nn][Dd]|cleanup"):
        if method == "takeoff":
            drone.takeoff(0.5, timeout=1.0)
        elif method == "enter_loiter":
            drone.enter_loiter(timeout=0.5, stable_for=0.1)
        else:
            drone.hold_loiter(0.2)
    assert drone.phase is previous
    assert not drone.command_attempts
    drone.connection.mav.set_attitude_target_send.assert_not_called()
    drone.connection.arducopter_disarm.assert_not_called()
