"""The canonical finite-number validators used at CLI and file boundaries.

Flight code turns operator input and telemetry into commands that move an
aircraft, so a NaN or an out-of-range value must be rejected at the boundary
rather than clamped silently.  This is the only implementation of that check;
do not add a second one.
"""

from __future__ import annotations

import math


def finite_in_range(
    value: float, name: str, *, minimum: float, maximum: float
) -> float:
    """Return ``value`` as a float, or raise if it is not finite and in range."""

    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(
            f"{name} must be finite and between {minimum:g} and {maximum:g}"
        )
    return number


def positive_finite(value: float, name: str) -> float:
    """Return ``value`` as a float, or raise if it is not finite and positive."""

    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return number
