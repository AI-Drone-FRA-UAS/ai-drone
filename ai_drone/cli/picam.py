"""Start the IMX500 NanoDet stream on the Raspberry Pi."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

from ai_drone.nearest_person import NearestPersonConfig, run_nearest_person
from ai_drone.platform import is_raspberry_pi


def main(arguments: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Run NanoDet on the IMX500 and serve annotated MJPEG."
    )
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--confidence", type=float, default=0.40)
    parser.add_argument("--threshold", type=float, default=2.0)
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=0)
    parser.add_argument("--altitude", type=float, default=0.0)
    parser.add_argument("--fov", type=float, default=66.0)
    parser.add_argument(
        "--regions-file",
        type=Path,
        help="explicit camera-specific metre-per-pixel calibration regions",
    )
    parser.add_argument(
        "--confirm-nadir-geometry",
        help="required as NADIR_CALIBRATED when --altitude is nonzero",
    )
    args = parser.parse_args(arguments)

    try:
        NearestPersonConfig(
            regions_file=args.regions_file,
            confidence=args.confidence,
            output="stream",
            port=args.port,
            distance_threshold=args.threshold,
            rotate=args.rotate,
            altitude_m=args.altitude,
            hfov_deg=args.fov,
        )
    except ValueError as error:
        parser.error(str(error))
    if args.altitude > 0 and args.confirm_nadir_geometry != "NADIR_CALIBRATED":
        parser.error(
            "--altitude requires --confirm-nadir-geometry NADIR_CALIBRATED; "
            "the current loose forward-facing camera does not qualify"
        )

    if not is_raspberry_pi():
        raise SystemExit(
            "drone-picam is Pi-only. Run 'uv run drone-deploy --picam' "
            "from the laptop or 'uv run drone-picam' on the Pi."
        )

    run_nearest_person(
        regions_file=args.regions_file,
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
