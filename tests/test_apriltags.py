from __future__ import annotations

import json
import math
import sys
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from ai_drone.cli_parsing import parse_even_resolution
from ai_drone.vision.apriltags import (
    CameraCalibration,
    NativeAprilTagDetector,
    PoseEstimationError,
    TagDetection,
    TagPose,
    create_detector,
    estimate_pose,
)


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


@pytest.mark.parametrize("bad_value", [math.nan, math.inf, -math.inf])
def test_calibration_rejects_non_finite_matrix_values(bad_value: float) -> None:
    calibration = replace(
        _calibration(),
        camera_matrix=(
            (800.0, 0.0, 640.0),
            (0.0, 810.0, bad_value),
            (0.0, 0.0, 1.0),
        ),
    )

    with pytest.raises(ValueError, match="camera_matrix values must be finite"):
        calibration.validate()


@pytest.mark.parametrize("bad_value", [math.nan, math.inf, -math.inf])
def test_calibration_rejects_non_finite_distortion_values(
    bad_value: float,
) -> None:
    calibration = replace(
        _calibration(),
        distortion_coefficients=(0.1, -0.2, 0.0, 0.0, bad_value),
    )

    with pytest.raises(
        ValueError, match="distortion_coefficients values must be finite"
    ):
        calibration.validate()


def test_calibration_rejects_invalid_pinhole_semantics() -> None:
    outside_image = replace(
        _calibration(),
        camera_matrix=(
            (800.0, 0.0, 1280.0),
            (0.0, 810.0, 480.0),
            (0.0, 0.0, 1.0),
        ),
    )
    invalid_shape = replace(
        _calibration(),
        camera_matrix=(
            (800.0, 0.1, 640.0),
            (0.0, 810.0, 480.0),
            (0.0, 0.0, 1.0),
        ),
    )
    invalid_distortion_count = replace(
        _calibration(), distortion_coefficients=(0.0, 0.0, 0.0)
    )

    with pytest.raises(ValueError, match="principal point"):
        outside_image.validate()
    with pytest.raises(ValueError, match="OpenCV pinhole form"):
        invalid_shape.validate()
    with pytest.raises(ValueError, match="distortion_coefficients must contain"):
        invalid_distortion_count.validate()


def test_calibration_load_rejects_lossy_dimensions_and_json_nan(tmp_path) -> None:
    path = tmp_path / "calibration.json"
    document = {
        "image_width": 1280.5,
        "image_height": 960,
        "camera_matrix": [
            [800.0, 0.0, 640.0],
            [0.0, 810.0, 480.0],
            [0.0, 0.0, 1.0],
        ],
        "distortion_coefficients": [0.1, -0.2, 0.0, 0.0, 0.05],
    }
    path.write_text(json.dumps(document))

    with pytest.raises(ValueError, match="image_width must be a positive integer"):
        CameraCalibration.load(path)

    document["image_width"] = 1280
    document["camera_matrix"][1][2] = math.nan
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match=r"camera_matrix\[1\]\[2\] must be finite"):
        CameraCalibration.load(path)


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


@pytest.mark.parametrize("decimate", [math.nan, math.inf, -math.inf, 0.999])
def test_detector_rejects_invalid_decimation(decimate: float) -> None:
    with pytest.raises(ValueError, match=r"decimate must be finite and at least 1\.0"):
        create_detector("auto", decimate=decimate)


def test_tag_detection_rejects_non_finite_geometry() -> None:
    corners = np.zeros((4, 2), dtype=np.float64)
    corners[2, 1] = math.nan
    with pytest.raises(ValueError, match="tag corners must be finite"):
        TagDetection(tag_id=1, corners=corners, center=(0.0, 0.0))

    with pytest.raises(ValueError, match="tag center must contain two finite"):
        TagDetection(
            tag_id=1,
            corners=np.zeros((4, 2), dtype=np.float64),
            center=(math.inf, 0.0),
        )

    with pytest.raises(ValueError, match="decision margin must be finite"):
        TagDetection(
            tag_id=1,
            corners=np.zeros((4, 2), dtype=np.float64),
            center=(0.0, 0.0),
            decision_margin=math.nan,
        )


@pytest.mark.parametrize("tag_size", [math.nan, math.inf, -math.inf, 0.0])
def test_pose_rejects_invalid_tag_size_before_opencv(tag_size: float) -> None:
    detection = TagDetection(
        tag_id=1,
        corners=np.zeros((4, 2), dtype=np.float64),
        center=(0.0, 0.0),
    )

    with pytest.raises(ValueError, match="tag_size_m must be finite and positive"):
        estimate_pose(
            detection,
            _calibration(),
            tag_size_m=tag_size,
            image_width=1280,
            image_height=960,
        )


def test_tag_pose_rejects_non_finite_results() -> None:
    with pytest.raises(ValueError, match="reprojection error must be finite"):
        TagPose(
            tag_id=1,
            rotation_vector=(0.0, 0.0, 0.0),
            translation_m=(0.0, 0.0, 1.0),
            distance_m=1.0,
            reprojection_error_px=math.nan,
        )


def _pose_detection() -> TagDetection:
    return TagDetection(
        tag_id=7,
        corners=np.asarray(
            [[300.0, 220.0], [340.0, 220.0], [340.0, 260.0], [300.0, 260.0]]
        ),
        center=(320.0, 240.0),
    )


def _estimate_test_pose(detection: TagDetection) -> TagPose:
    return estimate_pose(
        detection,
        _calibration(),
        tag_size_m=0.16,
        image_width=1280,
        image_height=960,
    )


def test_unsolved_pose_has_recoverable_geometric_error(monkeypatch) -> None:
    monkeypatch.setitem(
        sys.modules,
        "cv2",
        SimpleNamespace(
            SOLVEPNP_IPPE_SQUARE=7,
            solvePnPGeneric=lambda *_args, **_kwargs: (False, (), ()),
        ),
    )

    with pytest.raises(PoseEstimationError, match="pose solve failed for tag 7"):
        _estimate_test_pose(_pose_detection())


@pytest.mark.parametrize(
    ("rotation", "translation", "projection_value"),
    [
        ((0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 0.0),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0),
        ((math.nan, 0.0, 0.0), (0.0, 0.0, 1.0), 0.0),
        ((0.0, math.inf, 0.0), (0.0, 0.0, 1.0), 0.0),
        ((0.0, 0.0, 0.0), (0.0, 0.0, math.nan), 0.0),
        ((0.0, 0.0, 0.0), (math.inf, 0.0, 1.0), 0.0),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.nan),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.inf),
    ],
)
def test_invalid_pose_candidates_have_recoverable_geometric_error(
    monkeypatch, rotation, translation, projection_value
) -> None:
    monkeypatch.setitem(
        sys.modules,
        "cv2",
        SimpleNamespace(
            SOLVEPNP_IPPE_SQUARE=7,
            solvePnPGeneric=lambda *_args, **_kwargs: (
                True,
                [rotation],
                [translation],
            ),
            projectPoints=lambda *_args: (
                np.full((4, 1, 2), projection_value),
                None,
            ),
        ),
    )

    with pytest.raises(PoseEstimationError, match="no positive-depth pose for tag 7"):
        _estimate_test_pose(_pose_detection())


def test_valid_pose_survives_invalid_candidates(monkeypatch) -> None:
    detection = _pose_detection()
    projected_translations = []

    def project_points(_objects, _rotation, translation, *_args):
        xyz = tuple(translation.ravel())
        projected_translations.append(xyz)
        if xyz[0] == 1.0:
            return np.full((4, 1, 2), math.nan), None
        return detection.corners.reshape(4, 1, 2), None

    expected_translation = (0.1, -0.05, 2.0)
    monkeypatch.setitem(
        sys.modules,
        "cv2",
        SimpleNamespace(
            SOLVEPNP_IPPE_SQUARE=7,
            solvePnPGeneric=lambda *_args, **_kwargs: (
                True,
                [(math.nan, 0.0, 0.0), *[(0.0, 0.0, 0.0)] * 3],
                [
                    (0.0, 0.0, 1.0),
                    (0.0, 0.0, -1.0),
                    (1.0, 0.0, 2.0),
                    expected_translation,
                ],
            ),
            projectPoints=project_points,
        ),
    )

    pose = _estimate_test_pose(detection)

    assert pose.tag_id == detection.tag_id
    assert pose.translation_m == pytest.approx(expected_translation)
    assert pose.distance_m == pytest.approx(np.linalg.norm(expected_translation))
    assert pose.reprojection_error_px == pytest.approx(0.0)
    assert projected_translations == [(1.0, 0.0, 2.0), expected_translation]


def test_missing_opencv_is_not_a_recoverable_pose_error(monkeypatch) -> None:
    monkeypatch.setitem(sys.modules, "cv2", None)

    with pytest.raises(RuntimeError, match="OpenCV is required") as error:
        _estimate_test_pose(_pose_detection())

    assert not isinstance(error.value, PoseEstimationError)


@pytest.mark.parametrize("invalid_calibration", [False, True])
def test_pose_configuration_errors_remain_fatal(monkeypatch, invalid_calibration):
    monkeypatch.setitem(sys.modules, "cv2", SimpleNamespace())
    calibration = _calibration()
    if invalid_calibration:
        calibration = replace(calibration, distortion_coefficients=(math.nan,) * 5)

    with pytest.raises(ValueError) as error:
        estimate_pose(
            _pose_detection(),
            calibration,
            tag_size_m=0.16,
            image_width=1280,
            image_height=960 if invalid_calibration else 720,
        )

    assert not isinstance(error.value, PoseEstimationError)


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
    assert parse_even_resolution("1280x960") == (1280, 960)
    with pytest.raises(Exception, match="positive even integers"):
        parse_even_resolution("641x480")
