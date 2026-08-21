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
    # The settle gate has its own tests below; waiting out its real window in
    # every climb test would buy nothing and cost seconds apiece.
    monkeypatch.setattr(
        controller, "wait_for_vertical_estimate_to_settle", lambda **_kwargs: 0.0
    )

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


def test_the_climb_stick_never_descends_and_never_reaches_full(monkeypatch) -> None:
    # The stick used to be pinned at the full requested rate from the first
    # sample.  It is ramped now, so the opening sticks sit at or just above
    # centre; what still has to hold is that a climb never asks for a descent
    # and never asks for everything the channel has.
    controller, connection = _controller(monkeypatch, [0.02, 0.10, 0.31])

    controller.climb_in_alt_hold(0.3, climb=0.5)

    climbing = _throttles(connection)[1:]
    assert climbing, "no climb stick was ever sent"
    for pwm in climbing:
        assert pwm >= CALIBRATION.middle_pwm
        assert pwm < CALIBRATION.maximum_pwm


def test_a_climb_faster_than_pilot_speed_up_stops_the_flight(monkeypatch) -> None:
    # PILOT_SPEED_UP is 0.25 m/s on this aircraft.  The 2026-08-21 runaway was
    # about 3.8 m/s; a real liftoff transient is around 0.5 m/s and must not
    # trip this.
    controller, _ = _controller(
        monkeypatch, [0.02, 0.20, 0.40, 0.45, 0.48, 0.49, 0.49, 0.49]
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


def _grounded_with_references() -> tuple[DroneController, MagicMock]:
    controller = DroneController(device="udp:127.0.0.1:14550", max_altitude=0.5)
    connection = MagicMock()
    connection.recv_match.return_value = None
    connection.mode_mapping.return_value = {"ALT_HOLD": 2, "LAND": 9, "STABILIZE": 0}
    controller.connection = connection
    controller.is_armed = True
    controller._armed_by_controller = True
    controller.current_altitude = 0.02
    controller._ground_reference = 0.02
    controller.local_position_altitude = 0.05
    controller._ekf_reference = 0.05
    controller.last_telemetry_time = time.monotonic()
    return controller, connection


def test_the_afternoon_of_2026_08_21_is_a_disagreement() -> None:
    """The EKF ran to 4 m in seven seconds while the rangefinder held 0.02 m."""

    controller, _ = _grounded_with_references()

    assert controller.altitude_sources_agree()

    controller.local_position_altitude = 0.05 + 0.30
    assert controller.altitude_sources_agree(), "0.30 m is inside the tolerance"

    controller.local_position_altitude = 0.05 + 4.00
    assert not controller.altitude_sources_agree()


def test_a_constant_offset_between_the_two_is_not_a_disagreement() -> None:
    # The two are measured from different origins, so only the *change* since
    # arming is comparable.  A fixed offset must not stop a healthy flight.
    controller, _ = _grounded_with_references()
    controller._ekf_reference = 12.0
    controller.local_position_altitude = 12.0 + 0.28
    controller.current_altitude = 0.02 + 0.30

    assert controller.altitude_sources_agree()


def test_a_missing_estimate_is_not_treated_as_a_disagreement() -> None:
    controller, _ = _grounded_with_references()
    controller.local_position_altitude = None

    # Stopping a flight for want of a message would be its own hazard.
    assert controller.altitude_sources_agree()


def test_the_disagreement_stops_the_flight_through_update_telemetry() -> None:
    controller, connection = _grounded_with_references()
    controller._flight_started_by_controller = True
    controller.is_flying = True
    controller.local_position_altitude = 0.05 + 4.00

    with pytest.raises(FlightSafetyError, match="the EKF believes it has climbed"):
        controller.update_telemetry()

    # And it stopped the aircraft rather than merely complaining.
    assert controller._landing_commanded
    assert connection.arducopter_disarm.called or connection.mav.set_mode_send.called


class TestTheClimbIsShapedNotJustRequested:
    """The 2026-08-21 evening flight put the whole requested climb rate on the
    stick from the first sample and kept it there for 2.9 s while the aircraft
    sat at 0.02 m.  It broke free with a thrust surplus, went 0.02 m to 0.57 m
    in 0.77 s, and tripped its own 0.50 m ceiling.  These pin the shaping that
    replaced that: ask for less at the start, and ask for less near the end.
    """

    @staticmethod
    def _controller() -> DroneController:
        controller = DroneController(device="udp:127.0.0.1:14550")
        controller._ground_reference = 0.0
        return controller

    def test_the_first_request_is_zero_not_the_full_rate(self) -> None:
        controller = self._controller()

        assert controller._shaped_climb(0.0, 0.02, 0.30, 0.5) == 0.0

    def test_the_request_grows_over_the_ramp_and_stops_at_the_asked_rate(self) -> None:
        controller = self._controller()
        ramp = DroneController.ALT_HOLD_RAMP_SECONDS

        # Far below the target, so the ramp is the only thing binding.
        rising = [controller._shaped_climb(t, 0.0, 10.0, 0.5) for t in (0.0, 1.0, 2.0)]
        assert rising == sorted(rising)
        assert rising[0] < rising[-1]
        assert controller._shaped_climb(ramp, 0.0, 10.0, 0.5) == pytest.approx(0.5)
        # The ramp is a limit, never a multiplier that overshoots the request.
        assert controller._shaped_climb(ramp * 10, 0.0, 10.0, 0.5) == pytest.approx(0.5)

    def test_the_request_fades_to_zero_as_the_target_arrives(self) -> None:
        controller = self._controller()
        # Long past the ramp, so only the approach taper binds.
        late = DroneController.ALT_HOLD_RAMP_SECONDS * 10

        approaching = [
            controller._shaped_climb(late, alt, 0.30, 0.5) for alt in (0.10, 0.20, 0.25)
        ]
        assert approaching == sorted(approaching, reverse=True)
        assert controller._shaped_climb(late, 0.30, 0.30, 0.5) == 0.0
        # Above the target the request cannot turn back into a climb.
        assert controller._shaped_climb(late, 0.40, 0.30, 0.5) == 0.0

    def test_the_shaped_request_never_exceeds_what_was_asked_for(self) -> None:
        controller = self._controller()

        for elapsed in (0.0, 0.5, 3.0, 30.0):
            for altitude in (0.0, 0.1, 0.29, 0.3, 1.0):
                shaped = controller._shaped_climb(elapsed, altitude, 0.30, 0.5)
                assert 0.0 <= shaped <= 0.5

    def test_a_missing_altitude_still_ramps_rather_than_jumping_to_full(self) -> None:
        controller = self._controller()

        assert controller._shaped_climb(0.0, None, 0.30, 0.5) == 0.0

    def test_the_taper_leaves_room_to_stop_before_the_usual_ceiling(self) -> None:
        # The flown geometry: 0.30 m target under a 0.50 m ceiling.  ArduPilot
        # decelerates at PILOT_ACCEL_Z, 2.5 m/s^2 by default.  The request has
        # to be small enough over the last stretch that the braking distance
        # fits in the 0.20 m that is left.
        controller = self._controller()
        late = DroneController.ALT_HOLD_RAMP_SECONDS * 10

        at_target_minus_5cm = controller._shaped_climb(late, 0.25, 0.30, 0.5)
        braking_distance = at_target_minus_5cm**2 / (2 * 2.5)
        assert braking_distance < 0.20


class TestTheVerticalEstimateHasToSettleBeforeArming:
    """The evening flight of 2026-08-21 armed on top of a ringing estimate.

    The aircraft had been picked up shortly before.  Disarmed, with the motors
    stopped and the rangefinder pinned at 0.02 m, it reported -1.89 m/s; by the
    time it armed it was reporting -2.40 m/s.  ALT_HOLD's altitude controller
    acts on that number, so ArduPilot commanded 92 % throttle against a 27.7 %
    hover to arrest a fall that was not happening, and the aircraft left the
    ground with the whole surplus.  Untouched for thirty seconds, the same
    aircraft reports -0.01 m/s.
    """

    @staticmethod
    def _settling(monkeypatch, rates: list[float]) -> DroneController:
        controller = DroneController(device="udp:127.0.0.1:14550")
        controller.connection = MagicMock()
        monkeypatch.setattr(DroneController, "SETTLE_WINDOW_S", 0.2)
        feed = iter(rates)

        def fake_update(max_messages: int = 50) -> None:
            with contextlib.suppress(StopIteration):
                controller.local_position_climb = next(feed)

        monkeypatch.setattr(controller, "update_telemetry", fake_update)
        return controller

    def test_a_still_aircraft_settles_and_is_allowed_to_arm(self, monkeypatch) -> None:
        controller = self._settling(monkeypatch, [-0.01] * 40)

        assert controller.wait_for_vertical_estimate_to_settle(timeout=5.0) == -0.01

    def test_the_rate_the_evening_flight_armed_on_is_refused(self, monkeypatch) -> None:
        controller = self._settling(monkeypatch, [-1.89, -2.11, -2.40] * 20)

        with pytest.raises(FlightSafetyError, match="never settled"):
            controller.wait_for_vertical_estimate_to_settle(timeout=1.0)

    def test_a_rate_that_swings_through_zero_does_not_count_as_settled(
        self, monkeypatch
    ) -> None:
        # Seven seconds after touchdown the same aircraft passed through zero
        # on its way out to +1.27 m/s.  A single sample near zero is not a
        # settled estimate, which is why the window exists.
        controller = self._settling(monkeypatch, [-0.43, -0.21, 0.0, 0.31, 0.78, 1.16])

        with pytest.raises(FlightSafetyError, match="never settled"):
            controller.wait_for_vertical_estimate_to_settle(timeout=1.0)

    def test_settling_is_refused_outright_once_the_vehicle_is_armed(self) -> None:
        controller = DroneController(device="udp:127.0.0.1:14550")
        controller.connection = MagicMock()
        controller.is_armed = True

        with pytest.raises(FlightSafetyError, match="before arming"):
            controller.wait_for_vertical_estimate_to_settle()

    def test_a_climb_on_an_unsettled_estimate_never_arms(self, monkeypatch) -> None:
        controller, _connection = _controller(monkeypatch, [0.02, 0.10, 0.31])
        armed: list[bool] = []

        def refuse(**_kwargs) -> float:
            raise FlightSafetyError("the vertical estimate never settled")

        monkeypatch.setattr(controller, "wait_for_vertical_estimate_to_settle", refuse)
        monkeypatch.setattr(controller, "arm", lambda *_a, **_k: armed.append(True))

        with pytest.raises(FlightSafetyError, match="never settled"):
            controller.climb_in_alt_hold(0.3, climb=0.5)

        assert armed == [], "the vehicle armed on an estimate that had not settled"


class TestTheClimbRateHasToMatchTheRangefinder:
    """Both flights of 2026-08-21 failed on the vehicle's own climb rate.

    Once the motors turn, this airframe's accelerometer reads 0.85 m/s^2 too
    much and EKF3 integrates it. The first flight reported -2.22 m/s within a
    second of arming and ArduPilot answered with 92 % throttle; the second
    reached +4.27 m/s and ArduPilot held the throttle down and never lifted.
    The rangefinder said 0.00 m/s throughout both. That contradiction is
    visible in seconds, where the height error it integrates into is not.
    """

    @staticmethod
    def _on_the_ground(believed: float) -> DroneController:
        controller = DroneController(device="udp:127.0.0.1:14550")
        controller.current_altitude = 0.02
        controller.local_position_climb = believed
        return controller

    def test_a_healthy_hover_is_not_a_disagreement(self) -> None:
        controller = self._on_the_ground(0.01)

        assert controller.climb_sources_disagree_for(0.0) is None
        controller.current_altitude = 0.02
        assert controller.climb_sources_disagree_for(1.0) is None

    def test_a_real_climb_both_sources_see_is_not_a_disagreement(self) -> None:
        controller = self._on_the_ground(0.50)
        controller.current_altitude = 0.10
        controller.climb_sources_disagree_for(0.0)
        # Half a metre per second, and the rangefinder rises to match.
        controller.current_altitude = 0.60
        assert controller.climb_sources_disagree_for(1.0) is None

    def test_the_evening_of_2026_08_21_is_caught_within_the_window(self) -> None:
        # +4.27 m/s reported, 0.020 m on the rangefinder for 359 samples.
        controller = self._on_the_ground(4.27)
        controller.climb_sources_disagree_for(0.0)
        assert controller.climb_sources_disagree_for(0.5) == 0.0
        held = controller.climb_sources_disagree_for(1.5)
        assert held is not None and held >= DroneController.CLIMB_DISAGREEMENT_WINDOW_S

    def test_the_overshoot_flight_is_caught_the_same_way(self) -> None:
        # -2.22 m/s reported a second after arming, aircraft on the floor.
        controller = self._on_the_ground(-2.22)
        controller.climb_sources_disagree_for(0.0)
        controller.climb_sources_disagree_for(0.5)
        held = controller.climb_sources_disagree_for(1.5)
        assert held is not None and held >= DroneController.CLIMB_DISAGREEMENT_WINDOW_S

    def test_a_brief_disagreement_clears_instead_of_accumulating(self) -> None:
        # Liftoff transients are real; only a sustained contradiction counts.
        controller = self._on_the_ground(3.0)
        controller.climb_sources_disagree_for(0.0)
        assert controller.climb_sources_disagree_for(0.5) == 0.0

        controller.local_position_climb = 0.0
        assert controller.climb_sources_disagree_for(1.0) is None
        assert controller._climb_disagreement_since is None

    def test_nothing_is_claimed_before_a_rangefinder_measurement_exists(self) -> None:
        controller = DroneController(device="udp:127.0.0.1:14550")
        controller.local_position_climb = 4.27

        assert controller.climb_sources_disagree_for(0.0) is None

    def test_the_disagreement_stops_the_flight_through_update_telemetry(
        self, monkeypatch
    ) -> None:
        controller = DroneController(device="udp:127.0.0.1:14550")
        connection = MagicMock()
        connection.recv_match.return_value = None
        controller.connection = connection
        controller.current_altitude = 0.02
        controller.local_position_climb = 4.27
        controller._ground_reference = 0.02
        stopped: list[bool] = []
        monkeypatch.setattr(
            controller, "emergency_stop", lambda *_a, **_k: stopped.append(True)
        )

        controller.update_telemetry()
        controller._climb_disagreement_since = (
            time.monotonic() - DroneController.CLIMB_DISAGREEMENT_WINDOW_S - 1.0
        )
        controller._climb_reference = (time.monotonic() - 1.0, 0.02)

        with pytest.raises(FlightSafetyError, match="rangefinder measures"):
            controller.update_telemetry()

        assert stopped == [True]
