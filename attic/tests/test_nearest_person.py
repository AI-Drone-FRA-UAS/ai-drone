from __future__ import annotations

import json
import math
from collections.abc import Callable
from dataclasses import FrozenInstanceError
from pathlib import Path
from typing import cast

import numpy as np
import pytest

from ai_drone.nearest_person import (
    DEFAULT_REGIONS,
    CalibrationRegion,
    DetectionCollection,
    NearestPersonConfig,
    OutputMode,
    altitude_dpp,
    compute_pairings,
    filter_person_detections,
    load_calibration_regions,
    run_nearest_person,
)


class FakeDetections:
    """Small NumPy-backed stand-in for the subset of modlib used in tests."""

    def __init__(
        self,
        rows: list[tuple[float, int, float, float]],
    ) -> None:
        self._rows = rows
        self.confidence = np.asarray([row[0] for row in rows], dtype=float)
        self.class_id = np.asarray([row[1] for row in rows], dtype=int)

    @property
    def center_points(self) -> tuple[np.ndarray, np.ndarray]:
        return (
            np.asarray([row[2] for row in self._rows], dtype=float),
            np.asarray([row[3] for row in self._rows], dtype=float),
        )

    def __getitem__(self, selector: object) -> FakeDetections:
        mask = np.asarray(selector, dtype=bool)
        return FakeDetections(
            [row for row, keep in zip(self._rows, mask, strict=True) if keep]
        )

    def __len__(self) -> int:
        return len(self._rows)


def _detections(
    *rows: tuple[float, int, float, float],
) -> DetectionCollection:
    return cast(DetectionCollection, FakeDetections(list(rows)))


def _write_json(path: Path, value: object) -> Path:
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def _region(points: tuple[tuple[float, float], ...], dpp: float) -> dict[str, object]:
    return {"points": points, "dpp": dpp}


def test_load_calibration_regions_returns_immutable_data(tmp_path: Path) -> None:
    path = _write_json(
        tmp_path / "regions.json",
        [
            _region(
                ((0.0, 0.0), (1.0, 0.0), (1.0, 0.5), (0.0, 0.5)),
                0.025,
            )
        ],
    )

    regions = load_calibration_regions(path)

    assert regions == (
        CalibrationRegion(
            points=((0.0, 0.0), (1.0, 0.0), (1.0, 0.5), (0.0, 0.5)),
            dpp=0.025,
        ),
    )
    with pytest.raises(FrozenInstanceError):
        setattr(regions[0], "dpp", 1.0)  # noqa: B010 - exercise frozen runtime guard


def test_bundled_reference_calibration_is_valid_but_never_an_implicit_default() -> None:
    assert len(load_calibration_regions(DEFAULT_REGIONS)) == 5
    with pytest.raises(ValueError, match="explicit calibration"):
        load_calibration_regions("")


def test_metric_output_requires_an_explicit_calibration_source(tmp_path) -> None:
    assert not NearestPersonConfig().metric_enabled
    assert NearestPersonConfig(regions_file=tmp_path / "regions.json").metric_enabled
    assert NearestPersonConfig(altitude_m=1.0).metric_enabled


@pytest.mark.parametrize(
    ("payload", "message"),
    [
        ([], "No calibration areas"),
        ([{"points": [[0.0, 0.0], [1.0, 0.0]], "dpp": 0.1}], "three points"),
        (
            [
                {
                    "points": [[0.0, 0.0], [1.0, 0.0], [float("nan"), 1.0]],
                    "dpp": 0.1,
                }
            ],
            "finite number",
        ),
        (
            [
                {
                    "points": [[0.0, 0.0], [1.1, 0.0], [0.0, 1.0]],
                    "dpp": 0.1,
                }
            ],
            "normalised",
        ),
        (
            [
                {
                    "points": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
                    "dpp": 0.0,
                }
            ],
            "positive finite",
        ),
        (
            [
                {
                    "points": [[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]],
                    "dpp": 0.1,
                }
            ],
            "non-zero area",
        ),
    ],
)
def test_load_regions_rejects_invalid_geometry(
    tmp_path: Path, payload: object, message: str
) -> None:
    path = _write_json(tmp_path / "regions.json", payload)

    with pytest.raises(ValueError, match=message):
        load_calibration_regions(path)


def test_altitude_dpp_uses_nadir_camera_geometry() -> None:
    expected = 2.0 * 2.5 * math.tan(math.radians(66.0) / 2.0) / 1280

    assert altitude_dpp(2.5, 66.0, 1280) == pytest.approx(expected)


@pytest.mark.parametrize(
    ("altitude", "hfov", "width"),
    [
        (0.0, 66.0, 1280),
        (-1.0, 66.0, 1280),
        (float("nan"), 66.0, 1280),
        (1.0, 0.0, 1280),
        (1.0, 180.0, 1280),
        (1.0, float("inf"), 1280),
        (1.0, 66.0, 0),
    ],
)
def test_altitude_dpp_rejects_invalid_geometry(
    altitude: float, hfov: float, width: int
) -> None:
    with pytest.raises(ValueError):
        altitude_dpp(altitude, hfov, width)


def test_filter_person_detections_applies_confidence_then_class() -> None:
    detections = _detections(
        (0.90, 0, 0.1, 0.1),
        (0.40, 0, 0.2, 0.2),
        (0.80, 2, 0.3, 0.3),
        (0.70, 0, 0.4, 0.4),
    )

    filtered = filter_person_detections(detections, 0.40)

    assert len(filtered) == 2
    x_centres, y_centres = filtered.center_points
    assert tuple(x_centres) == pytest.approx((0.1, 0.4))
    assert tuple(y_centres) == pytest.approx((0.1, 0.4))


def test_compute_pairings_with_uniform_scale() -> None:
    detections = _detections(
        (0.9, 0, 0.0, 0.0),
        (0.9, 0, 0.3, 0.4),
        (0.9, 0, 1.0, 1.0),
    )

    pairings = compute_pairings(detections, 100, 100, uniform_dpp=0.01)

    assert [(pair.index, pair.distance_m, pair.p1, pair.p2) for pair in pairings] == [
        (0, 0.5, (0, 0), (30, 40)),
        (1, 0.5, (30, 40), (0, 0)),
        (2, 0.92, (100, 100), (30, 40)),
    ]


def test_compute_pairings_needs_two_people_before_calibration() -> None:
    assert compute_pairings(_detections(), 100, 100) == []
    assert compute_pairings(_detections((0.9, 0, 0.5, 0.5)), 100, 100) == []


def test_compute_pairings_averages_endpoint_regions_and_skips_outside() -> None:
    regions = (
        CalibrationRegion(
            points=((0.0, 0.0), (1.0, 0.0), (1.0, 0.5), (0.0, 0.5)),
            dpp=0.01,
        ),
        CalibrationRegion(
            points=((0.0, 0.5), (1.0, 0.5), (1.0, 1.0), (0.0, 1.0)),
            dpp=0.03,
        ),
    )
    calibrated = _detections((0.9, 0, 0.25, 0.25), (0.9, 0, 0.25, 0.75))
    outside = _detections((0.9, 0, 0.25, 0.25), (0.9, 0, 0.25, 1.25))

    pairings = compute_pairings(calibrated, 100, 100, regions=regions)

    assert [pair.distance_m for pair in pairings] == [1.0, 1.0]
    assert compute_pairings(outside, 100, 100, regions=regions) == []


@pytest.mark.parametrize(
    ("width", "height", "uniform_dpp"),
    [
        (0, 100, 0.1),
        (100, 100, 0.0),
        (100, 100, float("nan")),
    ],
)
def test_compute_pairings_rejects_invalid_inputs(
    width: int, height: int, uniform_dpp: float
) -> None:
    detections = _detections((0.9, 0, 0.1, 0.1), (0.9, 0, 0.2, 0.2))

    with pytest.raises(ValueError):
        compute_pairings(detections, width, height, uniform_dpp=uniform_dpp)


def test_compute_pairings_rejects_non_finite_centres() -> None:
    detections = _detections(
        (0.9, 0, 0.1, 0.1),
        (0.9, 0, float("inf"), 0.2),
    )

    with pytest.raises(ValueError, match="finite"):
        compute_pairings(detections, 100, 100, uniform_dpp=0.1)


@pytest.mark.parametrize(
    "factory",
    [
        lambda: NearestPersonConfig(output=cast(OutputMode, "file")),
        lambda: NearestPersonConfig(confidence=float("nan")),
        lambda: NearestPersonConfig(port=0),
        lambda: NearestPersonConfig(distance_threshold=-1.0),
        lambda: NearestPersonConfig(rotate=45),
        lambda: NearestPersonConfig(altitude_m=-1.0),
        lambda: NearestPersonConfig(hfov_deg=180.0),
    ],
)
def test_nearest_person_config_rejects_invalid_inputs(
    factory: Callable[[], NearestPersonConfig],
) -> None:
    with pytest.raises(ValueError):
        factory()


def test_run_rejects_invalid_config_before_importing_pi_dependencies() -> None:
    with pytest.raises(ValueError, match="output must be"):
        run_nearest_person(output=cast(OutputMode, "invalid"))
