"""Offline tests for the canonical finite-number validators."""

from __future__ import annotations

import math

import pytest

from ai_drone.validation import finite_in_range, positive_finite


@pytest.mark.parametrize("value", [0.1, 1.0, 9.9, 10.0])
def test_finite_in_range_accepts_bounds_inclusively(value: float) -> None:
    assert finite_in_range(value, "--x", minimum=0.1, maximum=10.0) == value


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, 0.09, 10.01, -1.0])
def test_finite_in_range_rejects_non_finite_and_out_of_range(value: float) -> None:
    with pytest.raises(ValueError, match="--x must be finite"):
        finite_in_range(value, "--x", minimum=0.1, maximum=10.0)


@pytest.mark.parametrize("value", [0.0, -1.0, math.nan, math.inf])
def test_positive_finite_rejects_non_positive_and_non_finite(value: float) -> None:
    with pytest.raises(ValueError, match="--y must be a positive finite number"):
        positive_finite(value, "--y")


def test_positive_finite_accepts_small_positive_values() -> None:
    assert positive_finite(1e-9, "--y") == pytest.approx(1e-9)
