"""The route up that leaves the altitude loop where it belongs.

GUIDED is refused by this vehicle -- its own pre-arm verdict is `Need Position
Estimate`, because EKF3 starts optical-flow navigation only after it detects a
takeoff -- and STABILIZE puts raw motor thrust on the stick through a mapping
two flights failed to pin down. ALT_HOLD needs no position estimate and reads
the stick as a climb rate ArduPilot bounds by PILOT_SPEED_UP.

These pin the properties that make that safe rather than merely convenient.
"""

from __future__ import annotations

import contextlib
import time
from unittest.mock import MagicMock

import pytest

from ai_drone import DroneController, FlightSafetyError
from ai_drone.flight.controller import ThrottleCalibration

CALIBRATION = ThrottleCalibration(
    minimum_pwm=988,
    maximum_pwm=2011,
    roll_trim_pwm=1501,
    pitch_trim_pwm=1500,
    yaw_trim_pwm=1500,
    hover=0.263,
    deadzone=40,
)


def _controller(
    monkeypatch, altitudes: list[float]
) -> tuple[DroneController, MagicMock]:
    controller = DroneController(device="udp:127.0.0.1:14550", max_altitude=0.5)
    connection = MagicMock()
    connection.recv_match.return_value = None
    connection.mode_mapping.return_value = {"ALT_HOLD": 2, "LAND": 9, "STABILIZE": 0}
    controller.connection = connection
    controller.flight_mode = "ALT_HOLD"

    monkeypatch.setattr(controller, "read_throttle_calibration", lambda: CALIBRATION)

    def fake_arm(mode: str = "GUIDED", **_kwargs) -> None:
        controller.is_armed = True
        controller._armed_by_controller = True
        controller.flight_mode = mode

    monkeypatch.setattr(controller, "arm", fake_arm)

    def fake_wait_for_altitude(timeout: float = 3.0) -> float:
        controller.current_altitude = 0.02
        controller.last_telemetry_time = time.monotonic()
        return 0.02

    monkeypatch.setattr(controller, "wait_for_altitude", fake_wait_for_altitude)
    monkeypatch.setattr(controller, "heartbeat_is_fresh", lambda *_a, **_k: True)
    monkeypatch.setattr(controller, "altitude_is_fresh", lambda *_a, **_k: True)

    feed = iter(altitudes)

    def fake_update(max_messages: int = 50) -> None:
        with contextlib.suppress(StopIteration):
            controller.current_altitude = next(feed)
        controller.last_telemetry_time = time.monotonic()

    monkeypatch.setattr(controller, "update_telemetry", fake_update)
    return controller, connection


def _throttles(connection: MagicMock) -> list[int]:
    return [
        call.args[4] for call in connection.mav.rc_channels_override_send.call_args_list
    ]


def test_the_throttle_is_at_its_minimum_before_the_vehicle_is_armed(
    monkeypatch,
) -> None:
    armed_at: list[int] = []
    controller, connection = _controller(monkeypatch, [0.02, 0.10, 0.31])

    original = controller.arm

    def recording_arm(mode: str = "GUIDED", **kwargs) -> None:
        armed_at.append(len(_throttles(connection)))
        # By keyword: arm() takes `timeout` first, and passing the mode
        # positionally would silently bind it there.
        original(mode=mode, **kwargs)

    monkeypatch.setattr(controller, "arm", recording_arm)
    controller.climb_in_alt_hold(0.3)

    # An override that only starts after arming leaves a window in which the
    # vehicle has no throttle source at all.
    assert armed_at and armed_at[0] >= 1
    assert _throttles(connection)[0] == CALIBRATION.minimum_pwm


def test_the_climb_stick_is_above_the_deadzone_and_below_full(monkeypatch) -> None:
    controller, connection = _controller(monkeypatch, [0.02, 0.10, 0.31])

    controller.climb_in_alt_hold(0.3, climb=0.5)

    climbing = _throttles(connection)[1:]
    assert climbing, "no climb stick was ever sent"
    for pwm in climbing:
        assert pwm > CALIBRATION.middle_pwm + CALIBRATION.deadzone_pwm
        assert pwm < CALIBRATION.maximum_pwm


def test_a_climb_faster_than_pilot_speed_up_stops_the_flight(monkeypatch) -> None:
    # PILOT_SPEED_UP is 0.25 m/s on this aircraft.  Ten times that means the
    # stick is not being read as a climb rate at all.
    controller, _ = _controller(
        monkeypatch, [0.02, 0.10, 0.20, 0.30, 0.40, 0.45, 0.48, 0.49]
    )

    with pytest.raises(FlightSafetyError, match="not being read as a climb rate"):
        controller.climb_in_alt_hold(0.5)


def test_a_climb_that_never_lifts_is_not_treated_as_a_flight(monkeypatch) -> None:
    controller, connection = _controller(monkeypatch, [0.02] * 400)

    with pytest.raises(TimeoutError, match="did not reach the target altitude"):
        controller.climb_in_alt_hold(0.3, timeout=1.0)

    # The 2026-08-21 shape: still on the floor, so this ends with a disarm and
    # never hands the aircraft to an altitude controller.
    assert not controller.is_flying
    assert not controller._flight_started_by_controller
    connection.arducopter_disarm.assert_called()
    assert not connection.mav.set_mode_send.called


def test_leaving_the_ground_is_what_makes_it_a_flight(monkeypatch) -> None:
    controller, _ = _controller(monkeypatch, [0.02, 0.02, 0.20, 0.31])

    controller.climb_in_alt_hold(0.3)

    assert controller.is_flying
    assert controller._flight_started_by_controller
