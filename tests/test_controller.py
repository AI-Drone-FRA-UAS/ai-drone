from __future__ import annotations

import math
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone import DroneController, FlightSafetyError
from ai_drone.cli import control


def _message(message_type: str, **fields):
    return SimpleNamespace(
        get_type=lambda: message_type,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
        **fields,
    )


def test_passive_context_never_controls_an_already_armed_vehicle() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    controller.is_armed = True

    controller.__exit__(None, None, None)

    assert controller.connection is None
    connection.mav.set_mode_send.assert_not_called()
    connection.arducopter_disarm.assert_not_called()
    connection.close.assert_called_once()


def test_downward_live_packet_updates_altitude_and_forward_sensor_is_ignored() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    connection.recv_match.side_effect = [
        _message(
            "DISTANCE_SENSOR",
            orientation=mavlink.MAV_SENSOR_ROTATION_PITCH_270,
            time_boot_ms=1_000,
            current_distance=42,
            min_distance=2,
            max_distance=1200,
            signal_quality=0,
        ),
        _message(
            "DISTANCE_SENSOR",
            orientation=mavlink.MAV_SENSOR_ROTATION_NONE,
            time_boot_ms=1_050,
            current_distance=300,
            min_distance=20,
            max_distance=800,
            signal_quality=80,
        ),
        _message(
            "DISTANCE_SENSOR",
            orientation=mavlink.MAV_SENSOR_ROTATION_PITCH_270,
            time_boot_ms=1_100,
            current_distance=47,
            min_distance=2,
            max_distance=1200,
            signal_quality=0,
        ),
        None,
    ]

    controller.update_telemetry()

    assert controller.current_altitude == 0.47


def test_stale_and_future_downward_samples_are_rejected() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    base = dict(
        orientation=mavlink.MAV_SENSOR_ROTATION_PITCH_270,
        min_distance=2,
        max_distance=1200,
        signal_quality=0,
    )
    controller._process_message(
        _message("DISTANCE_SENSOR", time_boot_ms=10_000, current_distance=40, **base),
        100.0,
    )
    controller._process_message(
        _message("DISTANCE_SENSOR", time_boot_ms=8_000, current_distance=90, **base),
        100.1,
    )
    controller._process_message(
        _message("DISTANCE_SENSOR", time_boot_ms=15_000, current_distance=95, **base),
        100.2,
    )

    assert controller.current_altitude == 0.4


def test_arming_checks_must_be_exactly_all(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter", lambda *_args: 0.0
    )

    with pytest.raises(FlightSafetyError, match="ARMING_CHECK=0"):
        controller.verify_arming_checks()


@pytest.mark.parametrize(
    ("parameters", "message"),
    [
        ({"LOG_BACKEND_TYPE": 2.0, "LOG_BITMASK": 1.0}, "onboard"),
        ({"LOG_BACKEND_TYPE": 1.0, "LOG_BITMASK": 0.0}, "LOG_BITMASK"),
    ],
)
def test_flight_requires_onboard_logging(monkeypatch, parameters, message) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter",
        lambda _connection, name: parameters[name],
    )

    with pytest.raises(FlightSafetyError, match=message):
        controller.verify_onboard_logging()


def test_land_timeout_never_force_disarms(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    controller.is_armed = True
    monkeypatch.setattr(controller, "set_mode", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(controller, "update_telemetry", lambda: None)
    times = iter([0.0, 0.0, 2.0])
    monkeypatch.setattr(
        "ai_drone.flight.controller.time.monotonic", lambda: next(times)
    )

    with pytest.raises(TimeoutError, match="LAND remains commanded"):
        controller.land(timeout=1.0)

    connection.arducopter_disarm.assert_not_called()


def test_velocity_command_is_bounded_and_uses_body_ned() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    controller.is_armed = controller.is_flying = True
    controller._flight_started_by_controller = True
    controller.send_velocity_body(0.5, 0.0, -0.1, 15.0)
    values = connection.mav.set_position_target_local_ned_send.call_args.args
    assert values[3] == mavlink.MAV_FRAME_BODY_NED
    assert values[4] == 0x05C7
    assert values[8:11] == (0.5, 0.0, -0.1)
    assert math.isclose(values[15], math.radians(15.0))
    with pytest.raises(ValueError):
        controller.send_velocity_body(float("nan"), 0.0, 0.0)


def test_flight_confirmation_is_checked_before_device_access(monkeypatch) -> None:
    monkeypatch.setattr(
        control,
        "_controller",
        lambda _args: pytest.fail("must not access MAVLink before confirmation"),
    )
    assert control.main(["hover", "--confirm-flight", "yes"]) == 1


@pytest.fixture
def armless(monkeypatch: pytest.MonkeyPatch) -> DroneController:
    """A controller wired past the pre-arm gates, ready to send the arm command.

    The gates themselves are covered by their own tests; these cases are about
    what happens once the arm command has gone out.
    """

    monkeypatch.setattr(DroneController, "verify_arming_checks", lambda self: None)
    monkeypatch.setattr(DroneController, "verify_onboard_logging", lambda self: None)
    monkeypatch.setattr(
        DroneController, "wait_for_altitude", lambda self, timeout=3.0: 0.05
    )
    monkeypatch.setattr(
        DroneController, "set_mode", lambda self, name, timeout=5.0: None
    )
    monkeypatch.setattr(
        DroneController, "_fresh_disarmed", lambda self, timeout=2.5: None
    )
    monkeypatch.setattr(
        DroneController, "altitude_is_fresh", lambda self, max_age=1.0: True
    )
    return DroneController(device="udp:127.0.0.1:14550")


def test_a_refused_arm_reports_what_the_vehicle_said(armless) -> None:
    controller = armless
    connection = MagicMock()
    controller.connection = connection
    connection.recv_match.side_effect = lambda *a, **k: _message(
        "STATUSTEXT", text="PreArm: Need Position Estimate"
    )

    with pytest.raises(FlightSafetyError, match="Need Position Estimate"):
        controller.arm(timeout=1.0)


def test_a_rejected_arm_command_is_reported_with_its_result_code(armless) -> None:
    controller = armless
    connection = MagicMock()
    controller.connection = connection
    connection.recv_match.side_effect = lambda *a, **k: _message(
        "COMMAND_ACK",
        command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        result=mavlink.MAV_RESULT_DENIED,
    )

    with pytest.raises(FlightSafetyError, match="MAV_RESULT"):
        controller.arm(timeout=1.0)


def test_a_silent_refusal_still_fails_rather_than_appearing_to_arm(armless) -> None:
    controller = armless
    connection = MagicMock()
    controller.connection = connection
    connection.recv_match.side_effect = lambda *a, **k: None

    with pytest.raises(TimeoutError, match="gave no reason"):
        controller.arm(timeout=1.0)
    assert controller.is_armed is False


def test_a_confirmed_arm_is_owned_by_the_controller(armless) -> None:
    controller = armless
    connection = MagicMock()
    controller.connection = connection
    # arm() first drains telemetry to reject an already-armed vehicle, so the
    # armed heartbeat must only appear after the command has been sent.
    seen = {"count": 0}

    def recv_match(*args, **kwargs):
        seen["count"] += 1
        if seen["count"] == 1:
            return None
        return _message("HEARTBEAT", base_mode=mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    connection.recv_match.side_effect = recv_match

    controller.arm(timeout=1.0)

    assert controller.is_armed is True
    assert controller._armed_by_controller is True


def test_a_climb_command_is_a_bounded_climb_rate_not_a_throttle(armless) -> None:
    # With GUID_OPTIONS bit 3 clear the thrust field is a climb rate, and 0.5
    # holds altitude. Commanding raw motor power is never what this sends.
    controller = armless
    controller.connection = MagicMock()
    controller.is_armed = True

    controller.send_level_climb(0.0)
    thrust = controller.connection.mav.set_attitude_target_send.call_args.args[8]
    assert thrust == controller.NEUTRAL_THRUST


@pytest.mark.parametrize("climb", [-1.0, -0.4, 0.0, 0.4, 1.0])
def test_every_climb_command_stays_inside_the_thrust_bounds(armless, climb) -> None:
    controller = armless
    controller.connection = MagicMock()
    controller.is_armed = True

    controller.send_level_climb(climb)

    thrust = controller.connection.mav.set_attitude_target_send.call_args.args[8]
    assert controller.MIN_THRUST <= thrust <= controller.MAX_THRUST


def test_a_climb_command_is_refused_on_a_disarmed_vehicle(armless) -> None:
    controller = armless
    controller.connection = MagicMock()
    controller.is_armed = False

    with pytest.raises(FlightSafetyError, match="armed"):
        controller.send_level_climb(0.5)
    controller.connection.mav.set_attitude_target_send.assert_not_called()


@pytest.mark.parametrize("climb", [1.5, -1.5, math.nan, math.inf])
def test_an_out_of_range_climb_command_is_rejected(armless, climb) -> None:
    controller = armless
    controller.connection = MagicMock()
    controller.is_armed = True

    with pytest.raises(ValueError):
        controller.send_level_climb(climb)
    controller.connection.mav.set_attitude_target_send.assert_not_called()


def test_a_position_free_takeoff_will_not_exceed_the_altitude_ceiling(armless) -> None:
    controller = armless
    controller.connection = MagicMock()
    controller.max_altitude = 0.5

    with pytest.raises(ValueError):
        controller.takeoff_without_position(0.9)


def test_a_position_free_takeoff_arms_in_a_mode_that_needs_no_position(
    armless, monkeypatch
) -> None:
    controller = armless
    controller.connection = MagicMock()
    modes: list[str] = []
    monkeypatch.setattr(
        DroneController,
        "arm",
        lambda self, timeout=10.0, mode="GUIDED": modes.append(mode),
    )
    monkeypatch.setattr(
        DroneController, "wait_for_altitude", lambda self, timeout=3.0: None
    )

    with pytest.raises(FlightSafetyError):
        controller.takeoff_without_position(0.3)

    assert modes == ["GUIDED_NOGPS"]


def test_the_controller_does_not_hang_up_on_an_airborne_vehicle(armless) -> None:
    """The defect this pins caused a real flyaway on 2026-08-20.

    A guard requested LAND, the request was never confirmed, and the context
    manager closed the connection anyway. The aircraft kept flying with nobody
    talking to it until its battery was physically pulled.
    """

    controller = armless
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    controller.connection = connection
    controller._flight_started_by_controller = True
    controller._landing_commanded = True  # a guard already asked once
    controller.is_armed = True
    controller.is_flying = True

    # The vehicle never disarms, so ensure_landed must exhaust its budget
    # rather than the controller quietly giving up.
    controller.ensure_landed = lambda timeout=60.0, retry_every=1.0: False  # type: ignore[method-assign]

    controller.__exit__(None, None, None)

    connection.close.assert_called_once()


def test_a_landing_is_retried_until_the_vehicle_confirms_disarm(armless) -> None:
    controller = armless
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    controller.connection = connection
    controller.is_armed = True

    calls = {"n": 0}

    def update_telemetry(max_messages: int = 50) -> None:
        calls["n"] += 1
        if calls["n"] >= 3:
            controller.is_armed = False

    controller.update_telemetry = update_telemetry  # type: ignore[method-assign]

    assert controller.ensure_landed(timeout=10.0, retry_every=0.1) is True
    assert controller.is_flying is False
    # LAND was requested, not assumed.
    assert connection.mav.set_mode_send.call_count >= 1


def test_a_vehicle_that_never_disarms_is_reported_not_silently_accepted(
    armless,
) -> None:
    controller = armless
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    controller.connection = connection
    controller.is_armed = True
    controller.update_telemetry = lambda max_messages=50: None  # type: ignore[method-assign]

    assert controller.ensure_landed(timeout=1.0, retry_every=0.2) is False


def test_ensure_landed_keeps_landing_through_an_altitude_complaint(armless) -> None:
    # update_telemetry raises above the ceiling. While we are already landing
    # that is not news, and it must not abort the landing.
    controller = armless
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    controller.connection = connection
    controller.is_armed = True
    state = {"n": 0}

    def update_telemetry(max_messages: int = 50) -> None:
        state["n"] += 1
        if state["n"] == 1:
            raise FlightSafetyError("altitude exceeds ceiling")
        controller.is_armed = False

    controller.update_telemetry = update_telemetry  # type: ignore[method-assign]

    assert controller.ensure_landed(timeout=5.0, retry_every=0.2) is True


def test_a_climb_faster_than_commanded_aborts_the_takeoff(armless) -> None:
    """If the thrust field does not mean what we think, the aircraft says so.

    The rehearsal simulator encodes our *assumption* about the thrust field, so
    it cannot catch us being wrong about it. Measuring the actual climb can.
    """

    controller = armless
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9, "GUIDED_NOGPS": 20}
    controller.connection = connection
    controller.is_armed = True
    controller._armed_by_controller = True
    controller.max_altitude = 5.0
    controller.wait_for_altitude = lambda timeout=3.0: 0.02  # type: ignore[method-assign]
    controller.altitude_is_fresh = lambda max_age=1.0: True  # type: ignore[method-assign]
    controller.heartbeat_is_fresh = lambda max_age=2.5: True  # type: ignore[method-assign]

    # Rockets upward at 2 m/s: far past MAX_MEASURED_CLIMB_MS, far short of
    # the 4 m target that would end the climb normally.
    start = time.monotonic()

    def update_telemetry(max_messages: int = 50) -> None:
        controller.current_altitude = (time.monotonic() - start) * 2.0
        controller.last_telemetry_time = time.monotonic()

    controller.update_telemetry = update_telemetry  # type: ignore[method-assign]

    with pytest.raises(FlightSafetyError, match="faster than"):
        controller.takeoff_without_position(4.0, climb=0.2, timeout=10.0)

    # LAND was requested as part of the abort.
    connection.mav.set_mode_send.assert_called()
