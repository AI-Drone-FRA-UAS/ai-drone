from types import SimpleNamespace

import pytest

from ai_drone.mavlink.safety import distance_sensor_valid, is_fresh
from ai_drone.validation import json_int, json_number


@pytest.mark.parametrize("value", [True, False, "1", None, 1.0])
def test_json_integer_rejects_coercions(value):
    with pytest.raises(ValueError, match="integer"):
        json_int(value, "schema")


@pytest.mark.parametrize(
    "value", [True, "1", None, float("nan"), float("inf"), 10**400]
)
def test_json_number_rejects_invalid_or_overflow(value):
    with pytest.raises(ValueError, match="finite number"):
        json_number(value, "reading")


@pytest.mark.parametrize(
    "at,expected",
    [
        (8.0, True),
        (7.999, False),
        (10.0, True),
        (10.001, False),
        (None, False),
        (True, False),
        (float("nan"), False),
        (10**400, False),
    ],
)
def test_freshness_boundary(at, expected):
    assert is_fresh(at, 10.0, 2.0) is expected


@pytest.mark.parametrize(
    "quality,expected", [(0, True), (1, False), (2, True), (255, True), (256, False)]
)
def test_range_unspecified_bounds_are_explicit_policy(quality, expected):
    message = SimpleNamespace(
        current_distance=50, min_distance=0, max_distance=0, signal_quality=quality
    )
    assert distance_sensor_valid(message) is expected
    assert not distance_sensor_valid(message, require_bounds=True)


@pytest.mark.parametrize("distance", [0, -1, 65536, True, 1.5])
def test_range_rejects_invalid_wire_values(distance):
    assert not distance_sensor_valid(
        SimpleNamespace(current_distance=distance, min_distance=0, max_distance=100)
    )
