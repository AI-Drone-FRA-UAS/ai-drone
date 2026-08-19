"""Reusable argument parsers for the project's command-line tools."""

from __future__ import annotations

import argparse


def parse_even_resolution(value: str) -> tuple[int, int]:
    """Parse ``WIDTHxHEIGHT`` and require positive, even dimensions."""

    try:
        width_text, height_text = value.casefold().split("x", 1)
        width, height = int(width_text), int(height_text)
    except (AttributeError, TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError(
            "resolution must look like 1280x960"
        ) from error
    if width <= 0 or height <= 0 or width % 2 or height % 2:
        raise argparse.ArgumentTypeError(
            "resolution dimensions must be positive even integers"
        )
    return width, height
