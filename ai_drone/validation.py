"""The canonical finite-number validators used at CLI and file boundaries.

Flight code turns operator input and telemetry into commands that move an
aircraft, so a NaN or an out-of-range value must be rejected at the boundary
rather than clamped silently.  This is the only implementation of that check;
do not add a second one.
"""

from __future__ import annotations

import math


def json_int(value: object, name: str) -> int:
    """Parse a JSON integer without coercing booleans or numeric strings."""
    if type(value) is not int:
        raise ValueError(f"{name} must be an integer")
    return value


def json_number(value: object, name: str) -> float:
    """Parse a finite JSON number, including integers but never booleans."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except OverflowError:
        raise ValueError(f"{name} must be a finite number") from None
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def bounded(value: float, name: str, *, minimum: float, maximum: float) -> float:
    """Check a parsed number; CLI coercion remains in finite_in_range."""
    return finite_in_range(value, name, minimum=minimum, maximum=maximum)


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
