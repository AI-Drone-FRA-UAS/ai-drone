#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`ai_drone.cli.fly_and_land`."""

from __future__ import annotations

from ai_drone.cli.fly_and_land import (
    BAUD,
    MAX_ALTITUDE,
    PORT,
    TAKEOFF_ALT,
    emergency_land,
    get_relative_alt,
    main,
    request_position_stream,
    set_mode,
)

__all__ = [
    "BAUD",
    "MAX_ALTITUDE",
    "PORT",
    "TAKEOFF_ALT",
    "emergency_land",
    "get_relative_alt",
    "main",
    "request_position_stream",
    "set_mode",
]


if __name__ == "__main__":
    main()
