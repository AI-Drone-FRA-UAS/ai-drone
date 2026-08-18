from __future__ import annotations

import json
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from ai_drone.apriltags import (
    CameraCalibration,
    NativeAprilTagDetector,
    TagDetection,
    create_detector,
    estimate_pose,
)
from ai_drone.cli.apriltag import _resolution


def _calibration() -> CameraCalibration:
    return CameraCalibration(
        image_width=1280,
        image_height=960,
        camera_matrix=(
            (800.0, 0.0, 640.0),
            (0.0, 810.0, 480.0),
            (0.0, 0.0, 1.0),
        ),
        distortion_coefficients=(0.1, -0.2, 0.0, 0.0, 0.05),
    )


def test_calibration_loads_and_scales(tmp_path) -> None:
    path = tmp_path / "calibration.json"
    path.write_text(
        json.dumps(
            {
                "image_width": 1280,
                "image_height": 960,
                "camera_matrix": [
                    [800.0, 0.0, 640.0],
                    [0.0, 810.0, 480.0],
                    [0.0, 0.0, 1.0],
                ],
                "distortion_coefficients": [0.1, -0.2, 0.0, 0.0, 0.05],
            }
        )
    )

    calibration = CameraCalibration.load(path)
    matrix, distortion = calibration.arrays_for(640, 480)

    assert matrix.tolist() == [
        [400.0, 0.0, 320.0],
        [0.0, 405.0, 240.0],
        [0.0, 0.0, 1.0],
    ]
    assert distortion.tolist() == [0.1, -0.2, 0.0, 0.0, 0.05]


def test_calibration_rejects_aspect_ratio_change() -> None:
    with pytest.raises(ValueError, match="different aspect ratios"):
        _calibration().arrays_for(1280, 720)


def test_native_detector_normalizes_corner_order(monkeypatch) -> None:
    class FakeDetector:
        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def detect(self, _image):
            return [
                {
                    "id": 4,
                    "hamming": 0,
                    "margin": 77.5,
                    "center": np.array([20.0, 15.0]),
                    # Native order: lower-left, lower-right, upper-right, upper-left.
                    "lb-rb-rt-lt": np.array(
                        [[10.0, 20.0], [30.0, 20.0], [30.0, 10.0], [10.0, 10.0]]
                    ),
                }
            ]

    monkeypatch.setitem(sys.modules, "apriltag", SimpleNamespace(apriltag=FakeDetector))
    detector = NativeAprilTagDetector()

    detections = detector.detect(np.zeros((30, 40), dtype=np.uint8))

    assert len(detections) == 1
    assert detections[0].tag_id == 4
    assert detections[0].corners.tolist() == [
        [10.0, 10.0],
        [30.0, 10.0],
        [30.0, 20.0],
        [10.0, 20.0],
    ]
    assert detections[0].decision_margin == 77.5


def test_detector_rejects_unknown_backend() -> None:
    with pytest.raises(ValueError, match="backend must be"):
        create_detector("unknown")


def test_pose_estimate_recovers_synthetic_distance() -> None:
    cv2 = pytest.importorskip("cv2")
    calibration = _calibration()
    camera_matrix, distortion = calibration.arrays_for(1280, 960)
    tag_size = 0.16
    half = tag_size / 2.0
    object_points = np.asarray(
        [
            (-half, half, 0.0),
            (half, half, 0.0),
            (half, -half, 0.0),
            (-half, -half, 0.0),
        ],
        dtype=np.float64,
    )
    expected_translation = np.asarray((0.1, -0.05, 2.0), dtype=np.float64)
    corners, _jacobian = cv2.projectPoints(
        object_points,
        np.zeros(3, dtype=np.float64),
        expected_translation,
        camera_matrix,
        distortion,
    )
    image_corners = corners.reshape(4, 2)
    detection = TagDetection(
        tag_id=7,
        corners=image_corners,
        center=tuple(image_corners.mean(axis=0)),
    )

    pose = estimate_pose(
        detection,
        calibration,
        tag_size_m=tag_size,
        image_width=1280,
        image_height=960,
    )

    assert pose.tag_id == 7
    assert pose.translation_m == pytest.approx(expected_translation, abs=1e-7)
    assert pose.distance_m == pytest.approx(
        np.linalg.norm(expected_translation), abs=1e-7
    )
    assert pose.reprojection_error_px < 1e-7


def test_resolution_parser() -> None:
    assert _resolution("1280x960") == (1280, 960)
    with pytest.raises(Exception, match="positive even integers"):
        _resolution("641x480")
