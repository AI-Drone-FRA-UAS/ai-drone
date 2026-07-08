#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`ai_drone.cli.picam`."""

from __future__ import annotations

from ai_drone.cli.picam import Path, _is_raspberry_pi, main

__all__ = ["Path", "_is_raspberry_pi", "main"]


if __name__ == "__main__":
    main()
