"""Compare AprilTag camera range against the downward rangefinder.

From the repository root, after `drone record` has produced a dataset:
    uv run python scripts/tag_range_check.py RECORDING --tag-size 0.28
    uv run python scripts/tag_range_check.py RECORDING --tag-size 0.28 --reference-m 1.5
    uv run python scripts/tag_range_check.py RECORDING --tag-size 0.28 --solve-tag-size

Reads `camera.jsonl` tag corners and `telemetry.jsonl` DISTANCE_SENSOR samples,
both written by the passive recorder. Nothing is sent to the drone.

Camera range uses the pinhole relation Z = fx * S / p, where S is the black
square's edge length and p its apparent width in analysis pixels. That needs no
lens calibration, only a focal length: pass `--calibration`, or `--fx`, or rely
on the IMX500's nominal horizontal field of view.

`--reference-m` supplies a tape-measured distance and is the better ground truth
for a stationary bench check. Without it the downward rangefinder is used, which
ArduPilot reports only inside RNGFND1_MIN..RNGFND1_MAX; samples outside those
wire bounds are counted separately and excluded.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import pathlib
import statistics
from typing import Any, NamedTuple

import numpy

DOWNWARD = 25  # MAV_SENSOR_ROTATION_PITCH_270
NOMINAL_HFOV_DEG = 66.3  # Raspberry Pi AI Camera (IMX500) standard lens


class Observation(NamedTuple):
    seconds: float
    tag_id: int
    width_px: float
    margin: float
    hamming: int
    exposure_us: float | None
    focus_fom: float | None
    pose_distance_m: float | None


class Range(NamedTuple):
    seconds: float
    metres: float
    in_bounds: bool
    source: str


def _timestamp(record: dict[str, Any]) -> float | None:
    text = record.get("timestamp_utc")
    if not isinstance(text, str):
        return None
    try:
        return dt.datetime.fromisoformat(text).timestamp()
    except ValueError:
        return None


def _apparent_width_px(corners: list[list[float]]) -> float | None:
    """Return the side of the square with the quadrilateral's projected area.

    Using the area rather than a mean edge length keeps the estimate stable
    when the tag is viewed off-axis, where the near edge grows as the far edge
    shrinks.
    """
    points = numpy.asarray(corners, dtype=float)
    if points.shape != (4, 2) or not numpy.isfinite(points).all():
        return None
    x, y = points[:, 0], points[:, 1]
    area = 0.5 * abs(numpy.dot(x, numpy.roll(y, -1)) - numpy.dot(y, numpy.roll(x, -1)))
    return math.sqrt(area) if area > 0 else None


def read_camera(path: pathlib.Path, tag_id: int | None) -> list[Observation]:
    observations: list[Observation] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        seconds = _timestamp(record)
        metadata = record.get("metadata") or {}
        for tag in record.get("tags") or []:
            if tag_id is not None and tag.get("id") != tag_id:
                continue
            width = _apparent_width_px(tag.get("corners_px") or [])
            if width is None or seconds is None:
                continue
            observations.append(
                Observation(
                    seconds=seconds,
                    tag_id=int(tag["id"]),
                    width_px=width,
                    margin=float(tag.get("decision_margin") or 0.0),
                    hamming=int(tag.get("hamming") or 0),
                    exposure_us=metadata.get("ExposureTime"),
                    focus_fom=metadata.get("FocusFoM"),
                    pose_distance_m=tag.get("distance_m"),
                )
            )
    return observations


def _distance_sensor(record: dict[str, Any], orientation: int) -> Range | None:
    fields = record.get("fields") or {}
    if int(fields.get("orientation", -1)) != orientation:
        return None
    seconds = _timestamp(record)
    current = fields.get("current_distance")
    if seconds is None or not isinstance(current, int | float) or current <= 0:
        return None
    low, high = fields.get("min_distance") or 0, fields.get("max_distance") or 0
    return Range(
        seconds=seconds,
        metres=current / 100.0,
        in_bounds=(low == 0 or current >= low) and (high == 0 or current <= high),
        source="DISTANCE_SENSOR",
    )


def _legacy_rangefinder(record: dict[str, Any]) -> Range | None:
    """RANGEFINDER carries no wire bounds, so it survives a low RNGFND1_MAX."""
    fields = record.get("fields") or {}
    seconds = _timestamp(record)
    distance = fields.get("distance")
    if seconds is None or not isinstance(distance, int | float) or distance <= 0:
        return None
    return Range(
        seconds=seconds, metres=float(distance), in_bounds=True, source="RANGEFINDER"
    )


def read_ranges(path: pathlib.Path, orientation: int) -> list[Range]:
    ranges: list[Range] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if '"DISTANCE_SENSOR"' not in line and '"RANGEFINDER"' not in line:
            continue
        record = json.loads(line)
        message = record.get("message")
        if message == "DISTANCE_SENSOR":
            item = _distance_sensor(record, orientation)
        elif message == "RANGEFINDER":
            item = _legacy_rangefinder(record)
        else:
            continue
        if item is not None:
            ranges.append(item)
    return ranges


def nearest(ranges: list[Range], seconds: float, tolerance: float) -> Range | None:
    if not ranges:
        return None
    best = min(ranges, key=lambda item: abs(item.seconds - seconds))
    return best if abs(best.seconds - seconds) <= tolerance else None


def analysis_width(recording: pathlib.Path, override: int | None) -> tuple[int, str]:
    """Resolve the detector's frame width; fx is meaningless without it."""
    if override is not None:
        return override, "--analysis-width"
    manifest = recording / "manifest.json"
    if manifest.is_file():
        try:
            camera = json.loads(manifest.read_text(encoding="utf-8"))["camera"]
            width, _ = camera["analysis_resolution"]
            return int(width), "manifest.json"
        except (KeyError, ValueError, TypeError):
            pass
    return 640, "default"


def focal_length_px(args: argparse.Namespace, width: int) -> tuple[float, str]:
    if args.fx is not None:
        return args.fx, "--fx"
    if args.calibration is not None:
        from ai_drone.vision.apriltags import CameraCalibration

        calibration = CameraCalibration.load(args.calibration)
        matrix, _ = calibration.arrays_for(width, args.analysis_height)
        return float(matrix[0][0]), str(args.calibration)
    nominal = (width / 2.0) / math.tan(math.radians(args.hfov_deg / 2.0))
    return nominal, f"nominal {args.hfov_deg} deg HFOV"


def _spread(label: str, values: list[float], unit: str = "") -> str:
    if not values:
        return f"  {label}: none"
    mean = statistics.fmean(values)
    deviation = statistics.stdev(values) if len(values) > 1 else 0.0
    return (
        f"  {label}: mean {mean:.3f}{unit}  sd {deviation:.3f}{unit}  "
        f"min {min(values):.3f}{unit}  max {max(values):.3f}{unit}"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=pathlib.Path)
    parser.add_argument("--tag-size", type=float, default=0.28, metavar="METRES")
    parser.add_argument("--tag-id", type=int, default=None)
    parser.add_argument("--reference-m", type=float, default=None, metavar="METRES")
    parser.add_argument("--calibration", type=pathlib.Path, default=None)
    parser.add_argument("--fx", type=float, default=None, metavar="PIXELS")
    parser.add_argument("--hfov-deg", type=float, default=NOMINAL_HFOV_DEG)
    parser.add_argument("--analysis-width", type=int, default=None)
    parser.add_argument("--analysis-height", type=int, default=480)
    parser.add_argument("--orientation", type=int, default=DOWNWARD)
    parser.add_argument("--match-tolerance-s", type=float, default=0.1)
    parser.add_argument(
        "--solve-tag-size",
        action="store_true",
        help="report the tag size implied by the reference distance",
    )
    return parser


def report_quality(observations: list[Observation]) -> None:
    print("Detection quality")
    print(_spread("decision margin", [o.margin for o in observations]))
    print(_spread("apparent width ", [o.width_px for o in observations], " px"))
    corrected = sum(1 for o in observations if o.hamming > 0)
    print(f"  hamming-corrected: {corrected}/{len(observations)} detections")
    exposures = [o.exposure_us for o in observations if o.exposure_us]
    if exposures:
        print(_spread("exposure       ", [e / 1000.0 for e in exposures], " ms"))
    focus = [o.focus_fom for o in observations if o.focus_fom]
    if focus:
        print(_spread("focus FoM      ", [float(f) for f in focus]))
    print()


def report_camera_range(
    observations: list[Observation], camera_ranges: list[float]
) -> None:
    print("Camera range (pinhole from corners)")
    print(_spread("range          ", camera_ranges, " m"))
    poses = [o.pose_distance_m for o in observations if o.pose_distance_m]
    if poses:
        print(_spread("pose distance  ", [float(p) for p in poses], " m"))
    else:
        print("  pose distance  : absent (recording had no --calibration)")
    print()


def reference_pairs(
    args: argparse.Namespace,
    observations: list[Observation],
    camera_ranges: list[float],
) -> list[tuple[float, float]] | None:
    """Pair each camera estimate with a ground truth, or None if unavailable."""
    if args.reference_m is not None:
        print(f"Reference: tape measure, {args.reference_m:.3f} m")
        return [(value, args.reference_m) for value in camera_ranges]

    telemetry_path = args.recording / "telemetry.jsonl"
    if not telemetry_path.is_file():
        print("No telemetry.jsonl and no --reference-m: cannot score accuracy.")
        return None

    ranges = read_ranges(telemetry_path, args.orientation)
    usable = [item for item in ranges if item.in_bounds]
    rejected = len(ranges) - len(usable)
    sources = sorted({item.source for item in usable})
    print(
        f"Reference: downward rangefinder, {len(usable)} usable sample(s) "
        f"from {', '.join(sources) or 'nothing'}, "
        f"{rejected} DISTANCE_SENSOR sample(s) outside RNGFND bounds"
    )
    if rejected and "RANGEFINDER" in sources:
        print(
            "  DISTANCE_SENSOR was clipped by RNGFND1_MAX; the unbounded legacy "
            "RANGEFINDER message supplied the reference instead."
        )
    if not usable:
        print(
            "  No in-bounds samples. On this drone RNGFND1_MAX is 1.0 m, so the "
            "downward range is unusable above 1 m. Raise it for the bench test, "
            "or pass --reference-m instead."
        )
        return None

    pairs = []
    for observation, estimate in zip(observations, camera_ranges, strict=True):
        match = nearest(usable, observation.seconds, args.match_tolerance_s)
        if match is not None:
            pairs.append((estimate, match.metres))
    print(f"  matched {len(pairs)} frame(s) within {args.match_tolerance_s}s")
    return pairs or None


def report_accuracy(
    args: argparse.Namespace,
    pairs: list[tuple[float, float]],
    fx: float,
) -> None:
    errors = [estimate - truth for estimate, truth in pairs]
    relative = [
        100.0 * (estimate - truth) / truth for estimate, truth in pairs if truth > 0
    ]
    print()
    print("Accuracy against reference")
    print(_spread("error          ", errors, " m"))
    print(_spread("relative error ", relative, " %"))

    if not args.solve_tag_size:
        return
    implied = [truth * args.tag_size / estimate for estimate, truth in pairs]
    print()
    print("Implied geometry (reference treated as truth)")
    print(_spread("tag size       ", implied, " m"))
    scale = statistics.fmean([truth / estimate for estimate, truth in pairs])
    print(f"  implied fx at {args.analysis_width} px wide: {fx * scale:.1f} px")


def main(arguments: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(arguments)
    if args.tag_size <= 0:
        parser.error("--tag-size must be positive")

    camera_path = args.recording / "camera.jsonl"
    if not camera_path.is_file():
        parser.error(f"{camera_path} not found")
    observations = read_camera(camera_path, args.tag_id)
    if not observations:
        print("No tag detections with corners in this recording.")
        return 1

    width, width_source = analysis_width(args.recording, args.analysis_width)
    args.analysis_width = width
    fx, fx_source = focal_length_px(args, width)
    identifiers = {o.tag_id for o in observations}
    print(f"Detections      : {len(observations)} over {len(identifiers)} ID(s)")
    print(
        f"Analysis width  : {width} px ({width_source})\n"
        f"Focal length    : fx = {fx:.1f} px ({fx_source})"
    )
    print(f"Assumed tag size: {args.tag_size:.4f} m black square")
    print()

    report_quality(observations)
    camera_ranges = [fx * args.tag_size / o.width_px for o in observations]
    report_camera_range(observations, camera_ranges)

    pairs = reference_pairs(args, observations, camera_ranges)
    if pairs:
        report_accuracy(args, pairs, fx)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
