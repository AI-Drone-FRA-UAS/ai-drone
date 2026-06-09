#!/usr/bin/env python3
"""Start the IMX500 NanoDet stream on the Raspberry Pi."""

from __future__ import annotations

import argparse
from pathlib import Path


def _is_raspberry_pi() -> bool:
    try:
        model = Path("/proc/device-tree/model").read_text()
    except (FileNotFoundError, PermissionError):
        return False
    return "raspberry pi" in model.lower()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run NanoDet on the IMX500 and serve annotated MJPEG."
    )
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--confidence", type=float, default=0.40)
    parser.add_argument("--threshold", type=float, default=2.0)
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=0)
    parser.add_argument("--altitude", type=float, default=0.0)
    parser.add_argument("--fov", type=float, default=66.0)
    args = parser.parse_args()

    if not _is_raspberry_pi():
        raise SystemExit(
            "test_picam.py is Pi-only. Run './deploy.sh --picam' from the laptop "
            "or 'uv run test_picam.py' on the Pi."
        )

    from ai_drone.nearest_person import run_nearest_person

    run_nearest_person(
        confidence=args.confidence,
        output="stream",
        port=args.port,
        distance_threshold=args.threshold,
        rotate=args.rotate,
        altitude=args.altitude,
        fov=args.fov,
    )


if __name__ == "__main__":
    main()
