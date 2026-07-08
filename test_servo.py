#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`ai_drone.cli.servo`."""

from __future__ import annotations

from ai_drone.cli.servo import (
    WIRING_DIAGRAM,
    _is_raspberry_pi,
    _parser,
    _pulse_us,
    _target_value_from_input,
    main,
)

__all__ = [
    "WIRING_DIAGRAM",
    "_is_raspberry_pi",
    "_parser",
    "_pulse_us",
    "_target_value_from_input",
    "main",
]


if __name__ == "__main__":
    raise SystemExit(main())
