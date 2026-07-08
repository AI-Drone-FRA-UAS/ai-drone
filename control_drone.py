#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`ai_drone.cli.control`."""

from __future__ import annotations

from ai_drone.cli.control import (
    cmd_follow,
    cmd_hover,
    cmd_status,
    cmd_velocity_test,
    logger,
    main,
)

__all__ = [
    "cmd_follow",
    "cmd_hover",
    "cmd_status",
    "cmd_velocity_test",
    "logger",
    "main",
]


if __name__ == "__main__":
    raise SystemExit(main())
