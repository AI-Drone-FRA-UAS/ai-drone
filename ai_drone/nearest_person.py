"""Nearest-person detection on the IMX500 AI Camera.

Adapted from Sony's ``aitrios-rpi-sample-apps`` *nearest-person* example
(Apache-2.0).  The original lives in
``aitrios-rpi-sample-apps/examples/nearest-person/app.py`` and is kept here as a
reference; this module reworks it to fit the AI-Drone project:

* runs entirely through ``modlib`` (the model executes **on** the IMX500
  sensor, not on the Pi CPU);
* offers three output modes so it is useful on a headless drone:
    - ``display``  — OpenCV window (needs a desktop / X session);
    - ``stream``   — annotated MJPEG over HTTP, reusing :mod:`ai_drone.stream`;
    - ``headless`` — no image, just logs the nearest-neighbour distances.

Pipeline: NanoDet object detection → keep ``person`` (class 0) detections →
ByteTrack tracking → for every person find its nearest neighbour and convert
the pixel gap to metres using per-region *distance-per-pixel* calibration.

``modlib`` is only available in the Pi ``raspi`` dependency group, so it is
imported lazily inside :func:`run_nearest_person`.
"""

from __future__ import annotations

import json
import logging
import math
from pathlib import Path
from typing import TYPE_CHECKING, NamedTuple

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = logging.getLogger(__name__)

# Default calibration shipped with the project (copied from the Sony example).
DEFAULT_REGIONS = Path(__file__).parent / "data" / "nearest_person_regions.json"

# COCO class id for "person" in the NanoDet model.
PERSON_CLASS_ID = 0


class _BYTETrackerArgs:
    """Tracker tuning — mirrors the values from the Sony sample."""

    track_thresh: float = 0.30
    track_buffer: int = 30
    match_thresh: float = 0.8
    aspect_ratio_thresh: float = 3.0
    min_box_area: float = 1.0
    mot20: bool = False


class Pairing(NamedTuple):
    """A person and the metric distance to the nearest other person."""

    index: int  #: index of this detection
    distance_m: float  #: real-world distance to the nearest person, in metres
    p1: tuple[int, int]  #: pixel centre of this detection
    p2: tuple[int, int]  #: pixel centre of the nearest detection


def load_regions(regions_file: str | Path | None) -> tuple[list, list[float]]:
    """Load calibration areas and their distance-per-pixel (dpp) values.

    Returns a tuple ``(areas, dpp)`` where ``areas`` is a list of
    ``modlib.apps.Area`` and ``dpp[i]`` is the metres-per-pixel for that area.
    """
    from modlib.apps import Area  # lazy: Pi-only dependency

    path = Path(regions_file) if regions_file else DEFAULT_REGIONS
    calibration = json.loads(path.read_text())
    if not calibration:
        raise ValueError(f"No calibration areas found in {path}")

    areas = [Area(region["points"]) for region in calibration]
    dpp = [region["dpp"] for region in calibration]
    logger.info("Loaded %d calibration region(s) from %s", len(areas), path)
    return areas, dpp


def altitude_dpp(altitude_m: float, hfov_deg: float, image_width: int) -> float:
    """Distance-per-pixel derived from drone flight geometry.

    Models a **nadir (straight down) camera over flat ground**::

        ground_width_m = 2 * altitude_m * tan(hfov / 2)
        dpp            = ground_width_m / image_width_px

    Assumes square pixels and ignores camera tilt and lens distortion, so treat
    it as an approximation — good for relative spacing; calibrate ``hfov_deg``
    to your lens for better absolute metres. A single constant dpp replaces the
    per-region calibration in altitude mode.
    """
    ground_width_m = 2.0 * altitude_m * math.tan(math.radians(hfov_deg) / 2.0)
    return ground_width_m / image_width


def _dpp_at(
    point: tuple[int, int],
    areas: list,
    dpp: list[float],
    width: int,
    height: int,
) -> float | None:
    """Return the distance-per-pixel for the region containing *point*.

    *point* is in pixels; areas are stored in normalised [0, 1] coordinates.
    Returns ``None`` when the point is outside every region — callers treat that
    as "uncalibrated", not as zero distance. When regions overlap on a shared
    edge the last match wins, matching the Sony reference's loop.
    """
    import cv2  # type: ignore[import-untyped]

    norm = (point[0] / width, point[1] / height)
    found: float | None = None
    for area_id, area in enumerate(areas):
        if cv2.pointPolygonTest(area.points, norm, False) >= 0:
            found = dpp[area_id]
    return found


def compute_pairings(
    detections,
    width: int,
    height: int,
    areas: list | None = None,
    dpp: list[float] | None = None,
    uniform_dpp: float | None = None,
) -> list[Pairing]:
    """For each detection, find its nearest neighbour and the metric distance.

    The pixel gap is scaled to metres by distance-per-pixel: a constant
    ``uniform_dpp`` (altitude mode) when provided, otherwise the average of the
    two endpoints' per-region values (fixed-camera calibration, as in the Sony
    example). Pairs whose endpoints fall outside every region are skipped rather
    than reported as a bogus 0 m.
    """
    from modlib.apps.calculate import calculate_distance_matrix

    xc, yc = detections.center_points
    xc, yc = xc * width, yc * height

    dist_matrix = calculate_distance_matrix(xc, yc)
    if len(dist_matrix) == 0:
        return []

    np.fill_diagonal(dist_matrix, np.inf)
    closest_indices = np.argmin(dist_matrix, axis=1)

    pairings: list[Pairing] = []
    for i, closest_index in enumerate(closest_indices):
        dist_px = dist_matrix[i, closest_index]
        # No neighbour (e.g. a single person in view) → distance is infinite.
        if not np.isfinite(dist_px):
            continue
        p1 = (int(xc[i]), int(yc[i]))
        p2 = (int(xc[closest_index]), int(yc[closest_index]))

        if uniform_dpp is not None:
            scale = uniform_dpp
        else:
            if areas is None or dpp is None:
                raise ValueError("areas and dpp are required without uniform_dpp")
            pd1 = _dpp_at(p1, areas, dpp, width, height)
            pd2 = _dpp_at(p2, areas, dpp, width, height)
            # Can't calibrate a point outside every region → skip the pair
            # instead of reporting 0 m (which would falsely flag "close").
            if pd1 is None or pd2 is None:
                continue
            scale = (pd1 + pd2) / 2

        real_dist = round(float(dist_px * scale), 2)
        pairings.append(Pairing(i, real_dist, p1, p2))

    return pairings


def _annotate(
    frame,
    detections,
    pairings: list[Pairing],
    annotator,
    distance_threshold: float,
) -> NDArray[np.uint8]:
    """Draw boxes, distance labels and nearest-neighbour arrows on the frame."""
    import cv2  # type: ignore[import-untyped]
    from modlib.apps.annotate import Color

    # One label per detection, aligned by index. People with no neighbour
    # (no pairing) just get a plain "Person" box.
    labels = ["Person"] * len(detections)
    for pair in pairings:
        labels[pair.index] = f"Closest: {pair.distance_m:.2f}m"
        # Green arrow when safely apart, red when within the threshold.
        color = [128, 255, 0] if pair.distance_m > distance_threshold else [0, 0, 255]
        cv2.arrowedLine(frame.image, pair.p1, pair.p2, color, 2)

    frame.image = annotator.annotate_boxes(
        frame=frame,
        detections=detections,
        labels=labels,
        color=Color(0, 255, 255),
    )
    return frame.image


def run_nearest_person(
    regions_file: str | None = None,
    confidence: float = 0.40,
    output: str = "stream",
    port: int = 8080,
    distance_threshold: float = 2.0,
    rotate: int = 0,
    altitude: float = 0.0,
    fov: float = 66.0,
) -> None:
    """Run the nearest-person pipeline on the IMX500 AI Camera.

    Args:
        regions_file: Path to a 3D-calibration JSON (areas + dpp). Defaults to
            the bundled :data:`DEFAULT_REGIONS`. Ignored when ``altitude > 0``.
        confidence: Minimum detection confidence to keep.
        output: ``"display"``, ``"stream"`` or ``"headless"``.
        port: HTTP port for ``stream`` mode.
        distance_threshold: Metres below which a pair is flagged (red arrow /
            logged as "close").
        rotate: Output rotation in degrees: 0, 90, 180 or 270. Cosmetic only
            (applied to the displayed/streamed image, not to detection maths) —
            use it when the camera is mounted rotated on the airframe.
        altitude: Drone height in metres. When > 0, distance is computed live
            from flight geometry (:func:`altitude_dpp`) instead of the static
            JSON calibration.
        fov: Horizontal field of view of the lens in degrees, used by altitude
            mode. Default 66° — set it to your lens for accurate metres.

    Raises:
        ValueError: for an unknown ``output`` mode or ``rotate`` angle.
    """
    import cv2  # type: ignore[import-untyped]
    from rich import print as rprint

    from modlib.apps import Annotator, ColorPalette
    from modlib.apps.tracker.byte_tracker import BYTETracker
    from modlib.devices import AiCamera
    from modlib.models.zoo import NanoDetPlus416x416

    from ai_drone.stream import push_frame, start_server

    # Validate inputs up front so a typo fails fast instead of, e.g., silently
    # falling through to display mode and crashing on a headless drone.
    if output not in ("stream", "headless", "display"):
        raise ValueError(
            f"output must be one of: stream, headless, display (got {output!r})"
        )

    # Map rotation degrees → cv2 code (None = no rotation).
    rot_map = {
        0: None,
        90: cv2.ROTATE_90_CLOCKWISE,
        180: cv2.ROTATE_180,
        270: cv2.ROTATE_90_COUNTERCLOCKWISE,
    }
    if rotate not in rot_map:
        raise ValueError("rotate must be one of 0, 90, 180, 270")
    rot_code = rot_map[rotate]

    # Calibration: a single constant dpp from flight geometry (altitude mode) or
    # per-region dpp from the JSON. The altitude dpp depends on the frame width,
    # which is only known once streaming starts, so it is filled in on the first
    # frame below; region calibration is loaded eagerly here.
    altitude_mode = altitude > 0
    uniform_dpp: float | None = None
    areas: list | None = None
    dpp: list[float] | None = None
    if altitude_mode:
        rprint(
            f"[dim]Altitude mode: {altitude} m, HFOV {fov}° (ignoring JSON regions)[/]"
        )
    else:
        areas, dpp = load_regions(regions_file)

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

    server = None
    if output == "stream":
        server = start_server(port=port)
        rprint(f"[bold green]Nearest-person stream on http://0.0.0.0:{port}/[/]")
        rprint("[dim]Open this URL in a browser. Press Ctrl-C to stop.[/]")
    elif output == "headless":
        rprint("[bold cyan]Nearest-person running headless. Press Ctrl-C to stop.[/]")
    else:
        rprint(
            "[bold cyan]Nearest-person display mode. Close the window or Ctrl-C to stop.[/]"
        )

    try:
        with device as stream:
            for frame in stream:
                # Altitude dpp needs the frame width, known only now → compute once.
                if altitude_mode and uniform_dpp is None:
                    uniform_dpp = altitude_dpp(altitude, fov, frame.width)
                    rprint(
                        f"[dim]Ground sampling: {uniform_dpp:.4f} m/px at {altitude} m[/]"
                    )

                # Keep only confident person detections, then track them.
                detections = frame.detections[frame.detections.confidence > confidence]
                detections = detections[detections.class_id == PERSON_CLASS_ID]
                detections = tracker.update(frame, detections)

                pairings = compute_pairings(
                    detections, frame.width, frame.height, areas, dpp, uniform_dpp
                )

                if output == "headless":
                    n = len(detections)
                    if pairings:
                        nearest = min(pairings, key=lambda p: p.distance_m)
                        flag = (
                            "  [CLOSE]"
                            if nearest.distance_m <= distance_threshold
                            else ""
                        )
                        rprint(
                            f"people={n}  nearest pair={nearest.distance_m:.2f}m{flag}  "
                            f"fps={frame.fps:.1f}"
                        )
                    else:
                        rprint(f"people={n}  (no pair to measure)  fps={frame.fps:.1f}")
                    continue

                _annotate(frame, detections, pairings, annotator, distance_threshold)

                if output == "stream":
                    # AiCamera/NanoDet frames are already BGR (what imencode
                    # expects); only convert genuine RGB sources to avoid an
                    # R/B swap. Mirrors modlib's own Frame.display() guard.
                    img = frame.image
                    if frame.color_format == "RGB":
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    if rot_code is not None:
                        img = cv2.rotate(img, rot_code)
                    _, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    push_frame(jpeg.tobytes())
                else:
                    frame.display(rotate=rot_code)
    except KeyboardInterrupt:
        rprint("\n[yellow]Nearest-person stopped.[/]")
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()  # release the listening socket / port
