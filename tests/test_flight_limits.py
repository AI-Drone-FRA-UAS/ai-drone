from dataclasses import replace

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight.limits import (
    FlightEvidence,
    FlightLimits,
    relative_position_ready,
    takeoff_ceiling_violation,
    takeoff_reached,
    violation,
)
from ai_drone.flight.state import Sample

GOOD = FlightEvidence(0.5, 0.5, True, True, 16.0, True, True, True)


@pytest.mark.parametrize(
    "change,expected",
    [
        ({"altitude": 0.81}, "altitude 0.81"),
        ({"aligned_local_altitude": 0.81}, "altitude 0.81"),
        ({"navigation_healthy": False}, "navigation became unhealthy"),
        ({"no_rc_input": False}, "receiver topology"),
        ({"battery_fresh": False}, "battery telemetry"),
        ({"battery_voltage": 14.0}, "battery 14.00"),
        ({"altitude_fresh": False}, "altitude became stale"),
        ({"heartbeat_fresh": False}, "heartbeat became stale"),
    ],
)
def test_flight_violations_keep_their_original_reasons(change, expected):
    reason = violation(
        FlightLimits(min_battery_voltage=14.4),
        replace(GOOD, **change),
        in_loiter=True,
        require_flight_telemetry=True,
    )
    assert reason is not None and expected in reason


def test_inclusive_floor_ceiling_and_conservative_local_range_are_independent():
    assert (
        violation(
            FlightLimits(),
            replace(GOOD, altitude=0.8, aligned_local_altitude=0.8),
            in_loiter=True,
            require_flight_telemetry=True,
        )
        is None
    )
    assert takeoff_ceiling_violation(0.5, 0.3, 0.8) is None
    assert takeoff_ceiling_violation(0.5, 0.31, 0.8) is not None


def test_navigation_is_operation_specific_but_not_ignored_in_loiter():
    evidence = replace(GOOD, navigation_healthy=False)
    assert (
        violation(
            FlightLimits(), evidence, in_loiter=False, require_flight_telemetry=True
        )
        is None
    )
    assert (
        violation(
            FlightLimits(), evidence, in_loiter=True, require_flight_telemetry=True
        )
        is not None
    )


@pytest.mark.parametrize(
    "received,expected",
    [(99.0, True), (98.999, False), (100.001, False), (float("nan"), False)],
)
def test_relative_aiding_requires_inclusive_finite_receipt_age(received, expected):
    flags = mavlink.EKF_VELOCITY_HORIZ | mavlink.EKF_POS_HORIZ_REL
    assert relative_position_ready(Sample(flags, received), 100.0, 1.0) is expected
    assert not relative_position_ready(
        Sample(flags | mavlink.EKF_CONST_POS_MODE, received), 100.0, 1.0
    )


def test_takeoff_progress_requires_post_start_range_in_the_pad_datum():
    assert not takeoff_reached(
        Sample(0.55, 99.0), started=100.0, ground=0.1, target=0.5
    )
    assert takeoff_reached(Sample(0.55, 100.0), started=100.0, ground=0.1, target=0.5)
    assert not takeoff_reached(
        Sample(0.54, 100.0), started=100.0, ground=0.1, target=0.5
    )
