"""Nearest-person detection on the IMX500 AI Camera.

Adapted from Sony's ``aitrios-rpi-sample-apps`` *nearest-person* example
(Apache-2.0). The model runs on the IMX500 sensor through ``modlib``. This
module keeps calibration and distance calculations independent of the Pi-only
runtime so they can be validated and tested on a development machine.

Pipeline: NanoDet object detection → keep ``person`` (class 0) detections →
ByteTrack tracking → find each person's nearest neighbour → convert the
pixel gap to metres using either per-region calibration or nadir-camera flight
geometry.

``modlib``, OpenCV, and Rich are in the Pi ``raspi`` dependency group. They are
therefore imported only in functions used by :func:`run_nearest_person`.
"""

from __future__ import annotations

import json
import logging
import math
from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, NamedTuple, Protocol, TypeAlias, cast

logger = logging.getLogger(__name__)

# Reference calibration copied from the Sony example. It is never selected by
# the runtime implicitly because it was not measured for this drone's camera.
DEFAULT_REGIONS = Path(__file__).parent / "data" / "nearest_person_regions.json"

# COCO class id for "person" in the NanoDet model.
PERSON_CLASS_ID = 0

Point: TypeAlias = tuple[float, float]
PixelPoint: TypeAlias = tuple[int, int]
OutputMode: TypeAlias = Literal["stream", "headless"]


class _ThresholdValues(Protocol):
    def __gt__(self, value: float, /) -> object: ...


class _ClassValues(Protocol):
    def __eq__(self, value: object, /) -> bool: ...


class DetectionCollection(Protocol):
    """Subset of the ``modlib`` detections API used by this module."""

    confidence: _ThresholdValues
    class_id: _ClassValues
    center_points: tuple[Sequence[float], Sequence[float]]

    def __getitem__(self, selector: object, /) -> DetectionCollection: ...

    def __len__(self) -> int: ...


class _Frame(Protocol):
    width: int
    height: int
    fps: float
    color_format: str
    image: object
    detections: DetectionCollection


class _FrameStream(Protocol):
    def __iter__(self) -> Iterator[_Frame]: ...


class _CameraDevice(Protocol):
    def __enter__(self) -> _FrameStream: ...

    def __exit__(self, *args: object) -> object: ...


class _Tracker(Protocol):
    def update(
        self, frame: _Frame, detections: DetectionCollection
    ) -> DetectionCollection: ...


class _Annotator(Protocol):
    def annotate_boxes(
        self,
        *,
        frame: _Frame,
        detections: DetectionCollection,
        labels: Sequence[str],
        color: object,
    ) -> object: ...


class _Server(Protocol):
    def shutdown(self) -> None: ...

    def server_close(self) -> None: ...


@dataclass(frozen=True, slots=True)
class _BYTETrackerArgs:
    """Tracker tuning mirrored from the Sony sample."""

    track_thresh: float = 0.30
    track_buffer: int = 30
    match_thresh: float = 0.8
    aspect_ratio_thresh: float = 3.0
    min_box_area: float = 1.0
    mot20: bool = False


@dataclass(frozen=True, slots=True)
class CalibrationRegion:
    """A normalised image polygon and its metres-per-pixel scale."""

    points: tuple[Point, ...]
    dpp: float

    def __post_init__(self) -> None:
        self.validate()

    def validate(self, *, index: int | None = None) -> None:
        """Reject unusable or physically invalid calibration geometry."""
        label = f"calibration region {index}" if index is not None else "region"
        if len(self.points) < 3:
            raise ValueError(f"{label} must contain at least three points")
        if len(set(self.points)) != len(self.points):
            raise ValueError(f"{label} must not contain duplicate points")
        for point in self.points:
            if len(point) != 2 or not all(math.isfinite(value) for value in point):
                raise ValueError(f"{label} points must contain two finite numbers")
            if not all(0.0 <= value <= 1.0 for value in point):
                raise ValueError(f"{label} points must be normalised to [0, 1]")

        twice_area = sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(
                self.points, self.points[1:] + self.points[:1], strict=True
            )
        )
        if math.isclose(twice_area, 0.0, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(f"{label} polygon must have non-zero area")
        if not math.isfinite(self.dpp) or self.dpp <= 0.0:
            raise ValueError(f"{label} dpp must be a positive finite number")


@dataclass(frozen=True, slots=True)
class NearestPersonConfig:
    """Validated runtime settings for nearest-person detection."""

    regions_file: Path | None = None
    confidence: float = 0.40
    output: OutputMode = "stream"
    port: int = 8080
    distance_threshold: float = 2.0
    rotate: int = 0
    altitude_m: float = 0.0
    hfov_deg: float = 66.0

    def __post_init__(self) -> None:
        if self.output not in ("stream", "headless"):
            raise ValueError(f"output must be stream or headless (got {self.output!r})")
        if self.rotate not in (0, 90, 180, 270):
            raise ValueError("rotate must be one of 0, 90, 180, 270")
        if not 0.0 <= self.confidence <= 1.0 or not math.isfinite(self.confidence):
            raise ValueError("confidence must be finite and between 0 and 1")
        if isinstance(self.port, bool) or not 1 <= self.port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if not math.isfinite(self.distance_threshold) or self.distance_threshold < 0.0:
            raise ValueError("distance_threshold must be a non-negative finite value")
        if not math.isfinite(self.altitude_m) or self.altitude_m < 0.0:
            raise ValueError("altitude must be a non-negative finite value")
        _validate_hfov(self.hfov_deg)

    @property
    def uses_altitude(self) -> bool:
        """Whether flight geometry replaces fixed-camera region calibration."""
        return self.altitude_m > 0.0

    @property
    def metric_enabled(self) -> bool:
        """Whether an explicit metric calibration source was selected."""

        return self.uses_altitude or self.regions_file is not None


class Pairing(NamedTuple):
    """A person and the metric distance to the nearest other person."""

    index: int  #: index of this detection
    distance_m: float  #: real-world distance to the nearest person, in metres
    p1: PixelPoint  #: pixel centre of this detection
    p2: PixelPoint  #: pixel centre of the nearest detection


def _number(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{field} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be a finite number")
    return result


def load_calibration_regions(
    regions_file: str | Path,
) -> tuple[CalibrationRegion, ...]:
    """Load and validate normalised distance-per-pixel calibration regions."""
    if not regions_file:
        raise ValueError("an explicit calibration regions file is required")
    path = Path(regions_file)
    raw: object = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"No calibration areas found in {path}")

    regions: list[CalibrationRegion] = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f"calibration region {index} must be an object")
        item_data = cast(dict[str, object], item)
        raw_points = item_data.get("points")
        if not isinstance(raw_points, list):
            raise ValueError(f"calibration region {index} points must be a list")

        points: list[Point] = []
        for point_index, raw_point in enumerate(raw_points):
            if not isinstance(raw_point, list) or len(raw_point) != 2:
                raise ValueError(
                    f"calibration region {index} point {point_index} "
                    "must contain two finite numbers"
                )
            points.append(
                (
                    _number(
                        raw_point[0],
                        field=f"calibration region {index} point {point_index} x",
                    ),
                    _number(
                        raw_point[1],
                        field=f"calibration region {index} point {point_index} y",
                    ),
                )
            )

        region = CalibrationRegion(
            points=tuple(points),
            dpp=_number(item_data.get("dpp"), field=f"calibration region {index} dpp"),
        )
        region.validate(index=index)
        regions.append(region)

    logger.info("Loaded %d calibration region(s) from %s", len(regions), path)
    return tuple(regions)


def _validate_hfov(hfov_deg: float) -> None:
    if not math.isfinite(hfov_deg) or not 0.0 < hfov_deg < 180.0:
        raise ValueError("hfov_deg must be finite and between 0 and 180 degrees")


def altitude_dpp(altitude_m: float, hfov_deg: float, image_width: int) -> float:
    """Return metres per pixel for a nadir camera above flat ground.

    This is an approximation that ignores camera tilt and lens distortion. The
    camera must be rigidly mounted straight down for the geometry to apply.
    """
    if not math.isfinite(altitude_m) or altitude_m <= 0.0:
        raise ValueError("altitude_m must be a positive finite value")
    _validate_hfov(hfov_deg)
    if isinstance(image_width, bool) or image_width <= 0:
        raise ValueError("image_width must be a positive integer")

    ground_width_m = 2.0 * altitude_m * math.tan(math.radians(hfov_deg) / 2.0)
    result = ground_width_m / image_width
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError("camera geometry produced an invalid distance-per-pixel")
    return result


def _point_on_segment(point: Point, start: Point, end: Point) -> bool:
    px, py = point
    x1, y1 = start
    x2, y2 = end
    cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1)
    return math.isclose(cross, 0.0, rel_tol=0.0, abs_tol=1e-12) and (
        min(x1, x2) <= px <= max(x1, x2) and min(y1, y2) <= py <= max(y1, y2)
    )


def _contains(point: Point, polygon: tuple[Point, ...]) -> bool:
    """Return whether *point* is inside or on the edge of *polygon*."""
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if _point_on_segment(point, previous, current):
            return True
        x1, y1 = previous
        x2, y2 = current
        if (y1 > point[1]) != (y2 > point[1]):
            crossing_x = (x2 - x1) * (point[1] - y1) / (y2 - y1) + x1
            if point[0] < crossing_x:
                inside = not inside
        previous = current
    return inside


def _dpp_at(
    point: PixelPoint,
    regions: Sequence[CalibrationRegion],
    width: int,
    height: int,
) -> float | None:
    """Return the last region scale containing a pixel-space point."""
    normalised = (point[0] / width, point[1] / height)
    found: float | None = None
    for region in regions:
        if _contains(normalised, region.points):
            # Shared edges intentionally use the last region, matching the
            # reference implementation's loop and OpenCV pointPolygonTest.
            found = region.dpp
    return found


def filter_person_detections(
    detections: DetectionCollection, confidence: float
) -> DetectionCollection:
    """Keep confident COCO ``person`` detections, preserving input order."""
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be finite and between 0 and 1")
    confident = detections[detections.confidence > confidence]
    return confident[confident.class_id == PERSON_CLASS_ID]


def _pixel_centres(
    detections: DetectionCollection, width: int, height: int
) -> tuple[tuple[float, float], ...]:
    if isinstance(width, bool) or isinstance(height, bool) or width <= 0 or height <= 0:
        raise ValueError("frame dimensions must be positive integers")
    try:
        x_values, y_values = detections.center_points
        x_centres = tuple(float(value) * width for value in x_values)
        y_centres = tuple(float(value) * height for value in y_values)
    except (TypeError, ValueError) as error:
        raise ValueError(
            "detection centres must be finite numeric sequences"
        ) from error
    if len(x_centres) != len(y_centres):
        raise ValueError("detection centre coordinate arrays must have equal lengths")
    if any(not math.isfinite(value) for value in (*x_centres, *y_centres)):
        raise ValueError("detection centres must contain only finite numbers")
    return tuple(zip(x_centres, y_centres, strict=True))


def compute_pairings(
    detections: DetectionCollection,
    width: int,
    height: int,
    regions: Sequence[CalibrationRegion] = (),
    uniform_dpp: float | None = None,
) -> list[Pairing]:
    """Find every detection's nearest neighbour and metric distance.

    ``uniform_dpp`` is used for altitude mode. Otherwise the scale is the mean
    of the two endpoints' region values. A pair with an endpoint outside all
    calibrated regions is skipped rather than reported as a false zero metres.
    """
    if uniform_dpp is not None and (
        not math.isfinite(uniform_dpp) or uniform_dpp <= 0.0
    ):
        raise ValueError("uniform_dpp must be a positive finite value")
    centres = _pixel_centres(detections, width, height)
    if len(centres) < 2:
        return []

    if uniform_dpp is None and not regions:
        raise ValueError("calibration regions are required without uniform_dpp")

    pairings: list[Pairing] = []
    for index, (x1, y1) in enumerate(centres):
        closest_index = min(
            (candidate for candidate in range(len(centres)) if candidate != index),
            key=lambda candidate: math.dist((x1, y1), centres[candidate]),
        )
        x2, y2 = centres[closest_index]
        distance_px = math.dist((x1, y1), (x2, y2))
        p1 = (int(x1), int(y1))
        p2 = (int(x2), int(y2))

        if uniform_dpp is not None:
            scale = uniform_dpp
        else:
            first_dpp = _dpp_at(p1, regions, width, height)
            second_dpp = _dpp_at(p2, regions, width, height)
            if first_dpp is None or second_dpp is None:
                continue
            scale = (first_dpp + second_dpp) / 2.0

        pairings.append(Pairing(index, round(distance_px * scale, 2), p1, p2))
    return pairings


def _process_detections(
    frame: _Frame,
    tracker: _Tracker,
    config: NearestPersonConfig,
    regions: Sequence[CalibrationRegion],
    uniform_dpp: float | None,
) -> tuple[DetectionCollection, list[Pairing]]:
    detections = filter_person_detections(frame.detections, config.confidence)
    tracked = tracker.update(frame, detections)
    if not config.metric_enabled:
        return tracked, []
    pairings = compute_pairings(
        tracked,
        frame.width,
        frame.height,
        regions=regions,
        uniform_dpp=uniform_dpp,
    )
    return tracked, pairings


def _headless_message(
    people: int,
    pairings: Sequence[Pairing],
    fps: float,
    distance_threshold: float,
    *,
    metric_enabled: bool,
) -> str:
    if not metric_enabled:
        return f"people={people}  (UNCALIBRATED; detections only)  fps={fps:.1f}"
    if not pairings:
        return f"people={people}  (no pair to measure)  fps={fps:.1f}"
    nearest = min(pairings, key=lambda pairing: pairing.distance_m)
    flag = "  [CLOSE]" if nearest.distance_m <= distance_threshold else ""
    return (
        f"people={people}  nearest pair={nearest.distance_m:.2f}m{flag}  fps={fps:.1f}"
    )


def _annotate(
    frame: _Frame,
    detections: DetectionCollection,
    pairings: Sequence[Pairing],
    annotator: _Annotator,
    distance_threshold: float,
    *,
    metric_enabled: bool,
) -> object:
    """Draw boxes, distance labels, and nearest-neighbour arrows."""
    import cv2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
    from modlib.apps.annotate import Color  # ty: ignore[unresolved-import]

    label = "Person" if metric_enabled else "Person (UNCALIBRATED)"
    labels = [label] * len(detections)
    for pairing in pairings:
        labels[pairing.index] = f"Closest: {pairing.distance_m:.2f}m"
        color = (
            [128, 255, 0] if pairing.distance_m > distance_threshold else [0, 0, 255]
        )
        cv2.arrowedLine(frame.image, pairing.p1, pairing.p2, color, 2)

    frame.image = annotator.annotate_boxes(
        frame=frame,
        detections=detections,
        labels=labels,
        color=Color(0, 255, 255),
    )
    return frame.image


def _publish_frame(
    frame: _Frame,
    rotation_code: int | None,
    push_frame: Callable[[bytes], None],
) -> None:
    import cv2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]

    image = frame.image
    if frame.color_format == "RGB":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if rotation_code is not None:
        image = cv2.rotate(image, rotation_code)
    success, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not success:
        raise RuntimeError("OpenCV failed to encode the nearest-person frame")
    push_frame(jpeg.tobytes())


def _run_camera_loop(
    device: _CameraDevice,
    tracker: _Tracker,
    annotator: _Annotator,
    config: NearestPersonConfig,
    regions: Sequence[CalibrationRegion],
    rotation_code: int | None,
    push_frame: Callable[[bytes], None],
    print_line: Callable[[str], object],
) -> None:
    uniform_dpp: float | None = None
    with device as stream:
        for frame in stream:
            if config.uses_altitude and uniform_dpp is None:
                uniform_dpp = altitude_dpp(
                    config.altitude_m, config.hfov_deg, frame.width
                )
                print_line(
                    f"[dim]Ground sampling: {uniform_dpp:.4f} m/px at "
                    f"{config.altitude_m} m[/]"
                )

            detections, pairings = _process_detections(
                frame, tracker, config, regions, uniform_dpp
            )
            if config.output == "headless":
                print_line(
                    _headless_message(
                        len(detections),
                        pairings,
                        frame.fps,
                        config.distance_threshold,
                        metric_enabled=config.metric_enabled,
                    )
                )
                continue

            _annotate(
                frame,
                detections,
                pairings,
                annotator,
                config.distance_threshold,
                metric_enabled=config.metric_enabled,
            )
            _publish_frame(frame, rotation_code, push_frame)


def run_nearest_person(
    regions_file: str | Path | None = None,
    confidence: float = 0.40,
    output: OutputMode = "stream",
    port: int = 8080,
    distance_threshold: float = 2.0,
    rotate: int = 0,
    altitude: float = 0.0,
    fov: float = 66.0,
) -> None:
    """Run nearest-person detection on the Raspberry Pi AI Camera.

    Altitude mode assumes a rigid nadir (straight-down) camera over flat ground.
    With ``altitude=0``, metric output remains disabled unless ``regions_file``
    explicitly selects a camera-specific calibration. All arguments are
    validated before importing or deploying Pi-only software.
    """
    config = NearestPersonConfig(
        regions_file=Path(regions_file) if regions_file else None,
        confidence=confidence,
        output=output,
        port=port,
        distance_threshold=distance_threshold,
        rotate=rotate,
        altitude_m=altitude,
        hfov_deg=fov,
    )
    regions = (
        load_calibration_regions(config.regions_file)
        if config.regions_file is not None
        else ()
    )

    import cv2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
    from modlib.apps import Annotator, ColorPalette  # ty: ignore[unresolved-import]
    from modlib.apps.tracker.byte_tracker import (  # ty: ignore[unresolved-import]
        BYTETracker,
    )
    from modlib.devices import AiCamera  # ty: ignore[unresolved-import]
    from modlib.models.zoo import (  # ty: ignore[unresolved-import]
        NanoDetPlus416x416,
    )
    from rich import print as rprint  # ty: ignore[unresolved-import]

    from ai_drone.stream import push_frame, start_server

    if config.uses_altitude:
        rprint(
            f"[dim]Altitude mode: {config.altitude_m} m, HFOV "
            f"{config.hfov_deg}° (ignoring JSON regions)[/]"
        )
    elif not config.metric_enabled:
        rprint(
            "[bold yellow]UNCALIBRATED: showing person detections only; "
            "metres and CLOSE thresholds are disabled.[/]"
        )

    rprint(
        "[dim]Deploying NanoDet to the IMX500 sensor (first run uploads the model)…[/]"
    )
    model = NanoDetPlus416x416()
    device = AiCamera()
    device.deploy(model)
    tracker = BYTETracker(_BYTETrackerArgs())
    annotator = Annotator(
        color=ColorPalette.default(), thickness=1, text_thickness=1, text_scale=0.4
    )

    rotation_codes = {
        0: None,
        90: cv2.ROTATE_90_CLOCKWISE,
        180: cv2.ROTATE_180,
        270: cv2.ROTATE_90_COUNTERCLOCKWISE,
    }
    server: _Server | None = None
    if config.output == "stream":
        server = start_server(port=config.port)
        rprint(f"[bold green]Nearest-person stream on http://0.0.0.0:{config.port}/[/]")
        rprint("[dim]Open this URL in a browser. Press Ctrl-C to stop.[/]")
    else:
        rprint("[bold cyan]Nearest-person running headless. Press Ctrl-C to stop.[/]")

    try:
        _run_camera_loop(
            device,
            tracker,
            annotator,
            config,
            regions,
            rotation_codes[config.rotate],
            push_frame,
            rprint,
        )
    except KeyboardInterrupt:
        rprint("\n[yellow]Nearest-person stopped.[/]")
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
