"""Stopping an aircraft that never left the ground must not go through LAND.

On 2026-08-21 a STABILIZE climb never lifted the aircraft, the climb timed
out, and the abort requested LAND.  The vehicle's EKF was reporting -10000 m
and a 38 m/s descent, LAND is an altitude-controlled mode, and its altitude
controller answered that estimate with full throttle in a single log sample.
The aircraft was destroyed one second later.

These tests pin the two halves of the fix: a flight is only a flight once the
aircraft has actually left the ground, and stopping one that has not is a
disarm rather than a mode change.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock

from ai_drone import DroneController
from ai_drone.mavlink.preflight import (
    EKF_POS_VERT_ABS,
    Snapshot,
    assess,
)

MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "GUIDED_NOGPS": 20, "LAND": 9}


def _grounded_controller() -> tuple[DroneController, MagicMock]:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    connection.mode_mapping.return_value = dict(MODES)
    controller.connection = connection
    controller.is_armed = True
    controller._armed_by_controller = True
    # The rangefinder reading the aircraft actually produced for the whole of
    # both runs: 2 cm, fresh, against a 2 cm reference taken at arming.
    controller.current_altitude = 0.02
    controller._ground_reference = 0.02
    controller.last_telemetry_time = time.monotonic()
    return controller, connection


def _requested_land(connection: MagicMock) -> bool:
    return any(
        call.args[-1] == MODES["LAND"]
        for call in connection.mav.set_mode_send.call_args_list
    )


def test_stopping_a_grounded_aircraft_disarms_instead_of_landing() -> None:
    controller, connection = _grounded_controller()

    controller.emergency_stop()

    connection.arducopter_disarm.assert_called_once()
    assert not _requested_land(connection)


def test_stopping_an_airborne_aircraft_still_requests_land() -> None:
    controller, connection = _grounded_controller()
    # The one thing that makes this a flight: the controller saw it lift off.
    controller._flight_started_by_controller = True
    controller.current_altitude = 0.40

    controller.emergency_stop()

    assert _requested_land(connection)
    connection.arducopter_disarm.assert_not_called()


def test_a_stale_rangefinder_is_not_evidence_of_being_on_the_ground() -> None:
    controller, connection = _grounded_controller()
    controller.last_telemetry_time = time.monotonic() - 30.0

    assert not controller.never_left_the_ground()
    controller.emergency_stop()
    # Nothing can vouch for the altitude, so this must not force a disarm.
    assert _requested_land(connection)


def test_a_rangefinder_above_the_liftoff_margin_is_not_the_ground() -> None:
    controller, _ = _grounded_controller()
    controller.current_altitude = 0.02 + DroneController.LIFTOFF_MARGIN_M + 0.01

    assert not controller.never_left_the_ground()


def test_the_diverged_vertical_rate_from_the_accident_is_rejected() -> None:
    controller, _ = _grounded_controller()

    controller.local_position_climb = -38.0
    assert not controller.vertical_estimate_is_sane()

    # The value the bench showed while the aircraft stood still, before
    # EK3_SRC1_POSZ was moved off the rangefinder.
    controller.local_position_climb = -17.78
    assert not controller.vertical_estimate_is_sane()

    controller.local_position_climb = 0.01
    assert controller.vertical_estimate_is_sane()


def test_a_missing_vertical_rate_is_not_treated_as_a_reason_to_refuse() -> None:
    controller, _ = _grounded_controller()
    controller.local_position_climb = None

    # The absence of a number is not evidence against stopping the aircraft.
    assert controller.vertical_estimate_is_sane()


def _vertical_check(snapshot: Snapshot) -> tuple[bool | None, str]:
    check = next(c for c in assess(snapshot) if c.name == "vertical_position")
    return check.passed, check.detail


def test_preflight_fails_the_altitude_the_flag_claimed_was_fine() -> None:
    passed, detail = _vertical_check(
        Snapshot(
            ekf_flags=EKF_POS_VERT_ABS,
            local_position=True,
            vertical_altitude_m=-10000.0,
            vertical_climb_ms=-38.0,
        )
    )
    assert passed is False
    assert "EK3_SRC1_POSZ" in detail


def test_preflight_fails_a_stationary_aircraft_that_reports_falling() -> None:
    passed, detail = _vertical_check(
        Snapshot(
            ekf_flags=EKF_POS_VERT_ABS,
            armed=False,
            local_position=True,
            vertical_altitude_m=0.05,
            vertical_climb_ms=-17.78,
        )
    )
    assert passed is False
    assert "diverged" in detail


def test_preflight_passes_the_estimate_the_bench_produced_after_the_fix() -> None:
    passed, _ = _vertical_check(
        Snapshot(
            ekf_flags=EKF_POS_VERT_ABS,
            armed=False,
            local_position=True,
            vertical_altitude_m=0.05,
            vertical_climb_ms=0.01,
        )
    )
    assert passed is True


def test_preflight_will_not_pass_on_the_flag_alone() -> None:
    passed, detail = _vertical_check(
        Snapshot(ekf_flags=EKF_POS_VERT_ABS, local_position=False)
    )
    assert passed is None
    assert "flag alone is not enough" in detail


def test_the_escape_sets_a_descending_throttle_before_leaving_land() -> None:
    from ai_drone.flight.controller import ThrottleCalibration

    controller, connection = _grounded_controller()
    calibration = ThrottleCalibration(
        minimum_pwm=988,
        maximum_pwm=2011,
        roll_trim_pwm=1501,
        pitch_trim_pwm=1500,
        yaw_trim_pwm=1500,
        hover=0.263,
        deadzone=40,
    )
    controller._throttle_calibration = calibration
    controller._rc_override = (1501, 1500, 1500, 1500)
    # The state the escape actually runs in: LAND has been asked for already.
    controller._landing_commanded = True

    assert controller._escape_to_manual_descent(3.0, 0.3)

    # Throttle first, mode second.  The other order hands STABILIZE whatever
    # stick happens to be standing for as long as the mode change takes.
    throttle_at = connection.mav.rc_channels_override_send.call_args.args[4]
    assert throttle_at < calibration.pwm_for(calibration.hover)
    assert connection.mav.set_mode_send.call_args.args[-1] == MODES["STABILIZE"]
    # Clearing this would re-arm the ceiling check, whose answer to an
    # over-height aircraft is to command the LAND we just escaped.
    assert controller._landing_commanded


def test_the_escape_is_refused_when_there_is_no_throttle_to_escape_to() -> None:
    controller, connection = _grounded_controller()
    controller._throttle_calibration = None

    assert not controller._escape_to_manual_descent(3.0, 0.3)
    assert not connection.mav.set_mode_send.called


def test_one_low_sample_is_not_a_landing() -> None:
    controller, _ = _grounded_controller()
    controller._flight_started_by_controller = True

    # An aircraft crossing over an obstacle produces exactly one of these.
    assert not controller.settled_on_the_ground(for_seconds=0.5)
    controller.current_altitude = 2.0
    controller.last_telemetry_time = time.monotonic()
    assert not controller.settled_on_the_ground(for_seconds=0.5)


def test_a_held_ground_reading_is_a_landing() -> None:
    controller, _ = _grounded_controller()
    controller._flight_started_by_controller = True

    assert not controller.settled_on_the_ground(for_seconds=0.0)
    controller.last_telemetry_time = time.monotonic()
    assert controller.settled_on_the_ground(for_seconds=0.0)
