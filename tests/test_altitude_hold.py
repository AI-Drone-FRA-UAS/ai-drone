from contextlib import contextmanager
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.cli import control
from ai_drone.flight.controller import (
    DroneController,
    FlightSafetyError,
    HumanControlTaken,
)
from ai_drone.flight.phase import Flight, Human
from ai_drone.flight.state import Heartbeat, Sample, VehicleState


@pytest.fixture
def hold_controller(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(
        "ai_drone.flight.controller.time.sleep",
        lambda seconds: clock.__setitem__(0, round(clock[0] + seconds, 6)),
    )
    drone = DroneController(device="tcp:127.0.0.1:5760")
    drone.phase = Flight("holding_altitude", 0.05, 0.55, 100.0)
    drone.state = VehicleState(
        heartbeat=Sample(Heartbeat(True, "GUIDED_NOGPS"), 100.0),
        altitude=Sample(0.52, 100.0),
        yaw=Sample(0.0, 100.0),
        rc_channels=Sample(0, 100.0),
    )
    drone.connection = MagicMock()
    drone.connection.recv_match.return_value = None
    return drone


def test_vertical_hold_sends_twenty_hz_neutral_climb_with_no_loiter_claim(
    hold_controller,
):
    drone = hold_controller
    drone.hold_altitude(0.2)
    sends = drone.connection.mav.set_attitude_target_send.call_args_list
    assert len(sends) == 4
    assert all(call.args[-1] == 0.5 for call in sends)
    assert drone.flight_mode == "GUIDED_NOGPS"
    assert drone.phase == Flight("holding_altitude", 0.05, 0.55, 100.0)
    # The operation explicitly needs no relative horizontal aiding in flight.
    assert not drone.navigation_is_healthy()
    drone.connection.mav.set_mode_send.assert_not_called()


@pytest.mark.parametrize(
    "duration", [0, -1, float("nan"), float("inf"), 10.001, 30.001]
)
def test_vertical_duration_is_bounded_before_effects(hold_controller, duration):
    with pytest.raises(ValueError):
        hold_controller.hold_altitude(duration)
    assert hold_controller.connection.mock_calls == []


def test_loiter_duration_is_bounded_before_effects(hold_controller):
    with pytest.raises(ValueError):
        hold_controller.hold_loiter(30.001)
    assert hold_controller.connection.mock_calls == []


@pytest.mark.parametrize("field", ["altitude", "yaw", "heartbeat", "rc_channels"])
def test_vertical_required_input_loss_requests_land_without_climb(
    hold_controller, field
):
    drone = hold_controller
    sample = getattr(drone.state, field)
    drone.state = replace(drone.state, **{field: replace(sample, received_at=90.0)})
    with pytest.raises(FlightSafetyError):
        drone.hold_altitude(0.2)
    drone.connection.mav.set_mode_send.assert_called_once_with(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    )
    drone.connection.mav.set_attitude_target_send.assert_not_called()
    drone.connection.arducopter_disarm.assert_not_called()


def test_vertical_write_failure_latches_land_and_preserves_original_error(
    hold_controller,
):
    drone = hold_controller
    drone.connection.mav.set_attitude_target_send.side_effect = OSError(
        "setpoint dropped"
    )
    with pytest.raises(OSError, match="setpoint dropped"):
        drone.hold_altitude(0.2)
    assert drone._landing_commanded
    drone.connection.mav.set_mode_send.assert_called_once()


def test_vertical_mode_loss_lands_and_does_not_resume_guided(
    hold_controller, monkeypatch
):
    drone = hold_controller

    def update():
        drone.state = replace(
            drone.state, heartbeat=Sample(Heartbeat(True, "ALT_HOLD"), 100.0)
        )

    monkeypatch.setattr(drone, "update_telemetry", update)
    with pytest.raises(FlightSafetyError, match="left GUIDED_NOGPS"):
        drone.hold_altitude(0.2)
    drone.connection.mav.set_mode_send.assert_called_once_with(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    )
    drone.connection.mav.set_attitude_target_send.assert_not_called()


def test_vertical_human_takeover_never_lands(hold_controller, monkeypatch):
    drone = hold_controller
    monkeypatch.setattr(
        drone, "update_telemetry", lambda: setattr(drone, "phase", Human(True))
    )
    with pytest.raises(HumanControlTaken):
        drone.hold_altitude(0.2)
    drone.connection.mav.set_mode_send.assert_not_called()
    drone.connection.mav.set_attitude_target_send.assert_not_called()


def test_altitude_hold_cli_records_datums_and_uses_dedicated_operation(monkeypatch):
    calls = []
    drone = SimpleNamespace(
        _ground_reference=0.05,
        current_altitude=0.52,
        flight_mode="GUIDED_NOGPS",
        takeoff=lambda target: calls.append(("takeoff", target)),
        hold_altitude=lambda duration: calls.append(("vertical", duration)),
        land=lambda: calls.append(("land",)),
    )
    record = SimpleNamespace(event=lambda name, **fields: calls.append((name, fields)))

    @contextmanager
    def session(_args):
        yield drone, record

    @contextmanager
    def operator(_args):
        yield lambda: True, lambda: False

    monkeypatch.setattr(control, "_flight_session", session)
    monkeypatch.setattr(control, "_operator_link", operator)
    monkeypatch.setattr(control, "is_raspberry_pi", lambda: False)
    assert (
        control.main(
            [
                "altitude-hold",
                "--duration",
                "10",
                "--confirm-flight",
                control.FLIGHT_CONFIRMATION,
            ]
        )
        == 0
    )
    assert (
        calls.index(("takeoff", 0.5))
        < calls.index(("vertical", 10.0))
        < calls.index(("land",))
    )
    metadata = next(fields for name, fields in calls if name == "altitude_hold_started")
    assert metadata["floor_target_m"] == pytest.approx(0.55)
    assert metadata["entry_altitude_m"] == 0.52
    assert metadata["horizontal_position_hold"] is False


@pytest.mark.parametrize("operation", ["altitude-hold", "hover", "takeoff"])
def test_flight_cli_rejects_unbounded_duration_before_session(monkeypatch, operation):
    monkeypatch.setattr(
        control, "_flight_session", lambda _args: pytest.fail("session opened")
    )
    assert (
        control.main(
            [
                operation,
                "--duration",
                "31",
                "--confirm-flight",
                control.FLIGHT_CONFIRMATION,
            ]
        )
        == 1
    )


def test_extended_vertical_experiment_requires_isolation_and_literal_loopback(
    monkeypatch,
):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")
    local = DroneController(device="tcp:127.0.0.1:5760")
    assert local.altitude_hold_limit_s == 30.0
    remote = DroneController(device="tcp:192.168.1.10:5760")
    assert remote.altitude_hold_limit_s == 10.0
    monkeypatch.delenv("AI_DRONE_ISOLATED_SITL")
    assert DroneController(device="tcp:127.0.0.1:5760").altitude_hold_limit_s == 10.0


def test_normal_vertical_cli_rejects_over_ten_seconds_before_hardware_access(
    monkeypatch,
):
    monkeypatch.delenv("AI_DRONE_ISOLATED_SITL", raising=False)
    monkeypatch.setattr(
        control, "_flight_session", lambda _args: pytest.fail("session opened")
    )
    assert (
        control.main(
            [
                "altitude-hold",
                "--duration",
                "10.01",
                "--confirm-flight",
                control.FLIGHT_CONFIRMATION,
            ]
        )
        == 1
    )
