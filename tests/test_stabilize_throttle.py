"""The throttle stick means a different thing in every mode it is sent to.

STABILIZE reads it as motor thrust and ALT_HOLD reads it as a climb rate, so
the *same* PWM value is a gentle hold in one mode and roughly twice hover
thrust in the other.  These tests pin the places where that distinction is
enforced, because getting it wrong is a flyaway rather than a bug report.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from ai_drone import DroneController, FlightSafetyError
from ai_drone.flight.controller import ThrottleCalibration

# The live values captured from the aircraft on 2026-08-20, including the
# hover throttle ArduPilot learned for itself during the flight.
CALIBRATION = ThrottleCalibration(
    minimum_pwm=988,
    maximum_pwm=2011,
    roll_trim_pwm=1501,
    pitch_trim_pwm=1500,
    yaw_trim_pwm=1500,
    hover=0.263,
    deadzone=40,
)


def _armed_controller(mode: str) -> tuple[DroneController, MagicMock]:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    controller._throttle_calibration = CALIBRATION
    controller.is_armed = True
    controller.flight_mode = mode
    return controller, connection


def _sent_throttle(connection: MagicMock) -> int:
    return connection.mav.rc_channels_override_send.call_args.args[4]


def test_hover_throttle_is_built_from_the_vehicles_own_learned_value() -> None:
    # 26.3 % of a 988-2011 stick.  If this ever becomes a hardcoded PWM value
    # it stops tracking what the aircraft learned about itself.
    assert CALIBRATION.pwm_for(CALIBRATION.hover) == 1257


def test_stabilize_throttle_is_capped_however_much_is_asked_for() -> None:
    controller, connection = _armed_controller("STABILIZE")

    controller.command_stabilize_throttle(0.9)

    ceiling = CALIBRATION.pwm_for(
        CALIBRATION.hover + DroneController.MAX_THROTTLE_ABOVE_HOVER
    )
    assert _sent_throttle(connection) == ceiling
    # Well under half stick: centring it in STABILIZE is the mistake this cap
    # exists to make impossible.
    assert ceiling < CALIBRATION.middle_pwm


def test_alt_hold_centre_is_refused_while_the_vehicle_reports_stabilize() -> None:
    controller, connection = _armed_controller("STABILIZE")

    with pytest.raises(FlightSafetyError, match="STABILIZE"):
        controller.command_alt_hold_climb(0.0)

    connection.mav.rc_channels_override_send.assert_not_called()


def test_stabilize_throttle_is_refused_once_the_vehicle_reports_alt_hold() -> None:
    controller, connection = _armed_controller("ALT_HOLD")

    with pytest.raises(FlightSafetyError, match="ALT_HOLD"):
        controller.command_stabilize_throttle(0.06)

    connection.mav.rc_channels_override_send.assert_not_called()


def test_alt_hold_hold_centres_the_stick_and_climb_clears_the_deadzone() -> None:
    controller, connection = _armed_controller("ALT_HOLD")

    controller.command_alt_hold_climb(0.0)
    assert _sent_throttle(connection) == CALIBRATION.middle_pwm

    controller.command_alt_hold_climb(0.5)
    assert (
        _sent_throttle(connection) > CALIBRATION.middle_pwm + CALIBRATION.deadzone_pwm
    )

    controller.command_alt_hold_climb(-0.5)
    assert (
        _sent_throttle(connection) < CALIBRATION.middle_pwm - CALIBRATION.deadzone_pwm
    )


def test_an_abort_leaves_a_throttle_that_descends_in_every_mode() -> None:
    controller, connection = _armed_controller("STABILIZE")
    controller.command_stabilize_throttle(0.06)

    controller.abort()

    left = _sent_throttle(connection)
    # Below hover, so it descends in STABILIZE; below the ALT_HOLD deadzone,
    # so it descends there too; ignored in LAND.  An abort cannot assume its
    # own mode change was accepted, so the value has to be safe in all three.
    assert left < CALIBRATION.pwm_for(CALIBRATION.hover)
    assert left < CALIBRATION.middle_pwm - CALIBRATION.deadzone_pwm


def test_the_override_is_never_released_on_an_armed_vehicle() -> None:
    controller, connection = _armed_controller("STABILIZE")
    controller.command_stabilize_throttle(0.0)
    connection.mav.rc_channels_override_send.reset_mock()

    with pytest.raises(FlightSafetyError, match="armed"):
        controller.clear_rc_override()

    connection.mav.rc_channels_override_send.assert_not_called()


def test_releasing_a_disarmed_override_hands_every_channel_back() -> None:
    controller, connection = _armed_controller("STABILIZE")
    controller.command_stabilize_throttle(0.0)
    controller.is_armed = False

    controller.clear_rc_override()

    assert connection.mav.rc_channels_override_send.call_args.args[2:6] == (0, 0, 0, 0)
    assert controller._rc_override is None


def test_the_override_is_refreshed_before_ardupilot_lets_it_lapse() -> None:
    controller, connection = _armed_controller("STABILIZE")
    controller.command_stabilize_throttle(0.0)
    connection.mav.rc_channels_override_send.reset_mock()

    controller._pump_rc_override()
    connection.mav.rc_channels_override_send.assert_not_called()

    # An override this old is one ArduPilot is about to hand back to a
    # receiver that does not exist on this airframe.
    controller._rc_override_sent -= DroneController.RC_OVERRIDE_REFRESH_S
    controller._pump_rc_override()
    connection.mav.rc_channels_override_send.assert_called_once()


def test_the_refresh_interval_stays_well_inside_rc_override_time() -> None:
    # ArduPilot's RC_OVERRIDE_TIME is 3 s on this vehicle.
    assert DroneController.RC_OVERRIDE_REFRESH_S <= 1.0


def test_ramping_starts_below_hover_and_ends_at_the_commanded_climb() -> None:
    controller, _ = _armed_controller("STABILIZE")

    assert controller._ramped_throttle(0.0, 0.06) < 0.0
    assert controller._ramped_throttle(
        DroneController.STABILIZE_RAMP_SECONDS, 0.06
    ) == pytest.approx(0.06)


def test_a_climb_beyond_the_maximum_throttle_is_refused_outright() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()

    with pytest.raises(ValueError, match="climb"):
        controller.climb_in_stabilize(0.4, climb=0.5)


def _reading_controller() -> DroneController:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    controller.connection.target_system = 1
    controller.connection.target_component = 1
    return controller


def _patch_parameters(monkeypatch, values: dict[str, float]) -> None:
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter",
        lambda connection, name, **kwargs: values[name],
    )


HEALTHY = {
    "RCMAP_ROLL": 1.0,
    "RCMAP_PITCH": 2.0,
    "RCMAP_THROTTLE": 3.0,
    "RCMAP_YAW": 4.0,
    "RC1_TRIM": 1501.0,
    "RC2_TRIM": 1500.0,
    "RC3_MIN": 988.0,
    "RC3_MAX": 2011.0,
    "RC4_TRIM": 1500.0,
    "MOT_THST_HOVER": 0.263,
    "THR_DZ": 40.0,
}


def test_the_calibration_is_read_from_the_vehicle(monkeypatch) -> None:
    _patch_parameters(monkeypatch, HEALTHY)

    calibration = _reading_controller().read_throttle_calibration()

    assert calibration == CALIBRATION


def test_a_vehicle_that_maps_its_sticks_elsewhere_is_refused(monkeypatch) -> None:
    # This class sends the first four override channels positionally, so a
    # different RCMAP would put roll on the throttle.
    _patch_parameters(monkeypatch, {**HEALTHY, "RCMAP_THROTTLE": 4.0})

    with pytest.raises(FlightSafetyError, match="RCMAP_THROTTLE"):
        _reading_controller().read_throttle_calibration()


def test_an_uncalibrated_throttle_channel_is_refused(monkeypatch) -> None:
    _patch_parameters(monkeypatch, {**HEALTHY, "RC3_MAX": 1000.0})

    with pytest.raises(FlightSafetyError, match="not calibrated"):
        _reading_controller().read_throttle_calibration()


def test_an_unlearned_hover_throttle_is_refused(monkeypatch) -> None:
    # A hover value this high would build a STABILIZE throttle from a number
    # the aircraft never demonstrated.
    _patch_parameters(monkeypatch, {**HEALTHY, "MOT_THST_HOVER": 0.95})

    with pytest.raises(FlightSafetyError, match="MOT_THST_HOVER"):
        _reading_controller().read_throttle_calibration()
