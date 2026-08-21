"""A stationary aircraft measures one g, so anything else is the sensor.

These carry the numbers from 2026-08-21 rather than invented ones: with the
motors stopped the vehicle measured 9.79 m/s^2 and with them turning at idle
spin it measured 10.65 m/s^2, and that difference is why it reported climbing
at 4.27 m/s while its rangefinder sat at 0.02 m.
"""

from __future__ import annotations

import pytest

from ai_drone.mavlink.accel_bias import (
    SIGNIFICANT_BIAS_MS2,
    STANDARD_GRAVITY,
    AccelBias,
    magnitude,
)

# RAW_IMU reports milligravities.  The evening of 2026-08-21, near enough.
STOPPED = (-31.0, -4.0, -998.0)
TURNING = (-35.0, 0.0, -1085.0)


def test_a_level_stationary_aircraft_measures_one_g() -> None:
    assert magnitude(0.0, 0.0, -1000.0) == pytest.approx(STANDARD_GRAVITY)


def test_orientation_does_not_change_the_magnitude() -> None:
    # The whole point of using the magnitude: a tilted bench is not a fault.
    on_its_side = magnitude(-1000.0, 0.0, 0.0)
    assert on_its_side == pytest.approx(STANDARD_GRAVITY)


def test_the_evening_of_2026_08_21_is_a_significant_bias() -> None:
    bias = AccelBias.empty()
    for _ in range(20):
        bias.observe(*STOPPED, moving=False)
        bias.observe(*TURNING, moving=True)

    assert bias.still_mean == pytest.approx(9.79, abs=0.02)
    assert bias.turning_mean == pytest.approx(10.65, abs=0.02)
    assert bias.bias == pytest.approx(0.85, abs=0.02)
    assert bias.is_significant


def test_a_healthy_aircraft_is_not_reported_as_biased() -> None:
    bias = AccelBias.empty()
    for _ in range(20):
        bias.observe(0.0, 0.0, -1000.0, moving=False)
        bias.observe(0.0, 0.0, -1004.0, moving=True)

    assert not bias.is_significant
    assert "Below the threshold" in bias.describe()


def test_a_bias_in_either_direction_counts() -> None:
    # 2026-08-21 shifted the estimate upward and the aircraft would not lift;
    # a shift the other way is the overshoot.  Both are the same fault.
    low = AccelBias.empty()
    for _ in range(10):
        low.observe(0.0, 0.0, -1000.0, moving=False)
        low.observe(0.0, 0.0, -900.0, moving=True)

    assert low.bias is not None and low.bias < -SIGNIFICANT_BIAS_MS2
    assert low.is_significant


def test_nothing_is_claimed_before_both_halves_are_measured() -> None:
    bias = AccelBias.empty()
    assert bias.bias is None
    assert not bias.is_significant
    assert "no accelerometer reading" in bias.describe()

    bias.observe(*STOPPED, moving=False)
    assert bias.bias is None
    assert "under power" in bias.describe()


def test_a_non_finite_reading_is_discarded_rather_than_averaged_in() -> None:
    bias = AccelBias.empty()
    bias.observe(*STOPPED, moving=False)
    bias.observe(float("nan"), 0.0, 0.0, moving=False)
    bias.observe(float("inf"), 0.0, 0.0, moving=False)

    assert len(bias.still) == 1


def test_the_description_names_the_mechanical_causes_worth_checking() -> None:
    bias = AccelBias.empty()
    for _ in range(10):
        bias.observe(*STOPPED, moving=False)
        bias.observe(*TURNING, moving=True)

    described = bias.describe()
    assert "propeller" in described
    assert "hard-mounted" in described
