#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`ai_drone.cli.lidar`."""

from __future__ import annotations

from ai_drone.cli.lidar import (
    CSV_FIELDS,
    MESSAGE_TYPES,
    _default_output,
    _find_device,
    _is_armed,
    _request_sensor_messages,
    _row,
    main,
    mavlink,
    mavutil,
    time,
)

__all__ = [
    "CSV_FIELDS",
    "MESSAGE_TYPES",
    "_default_output",
    "_find_device",
    "_is_armed",
    "_request_sensor_messages",
    "_row",
    "main",
    "mavlink",
    "mavutil",
    "time",
]


if __name__ == "__main__":
    raise SystemExit(main())
