from __future__ import annotations

import json
import math
import sys
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import ai_drone.cli.apriltag as apriltag_cli
from ai_drone.cli.apriltag import _parser, _serialize_payload, _validate_args
from ai_drone.cli_parsing import parse_even_resolution
from ai_drone.vision.apriltags import (
    CameraCalibration,
    NativeAprilTagDetector,
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


@pytest.mark.parametrize(
    "arguments",
    [
        ["--decimate", "nan"],
        ["--decimate", "inf"],
        ["--tag-size", "nan"],
        ["--tag-size", "inf"],
        ["--max-reprojection-error", "nan"],
        ["--max-reprojection-error", "inf"],
        ["--max-reprojection-error", "0"],
        ["--duration", "nan"],
        ["--duration", "inf"],
        ["--analogue-gain", "nan"],
        ["--analogue-gain", "inf"],
        ["--port", "0"],
        ["--port", "65536"],
        ["--target-id", "-1"],
        ["--target-id", "587"],
    ],
)
def test_apriltag_parser_rejects_invalid_numeric_values(
    arguments: list[str],
) -> None:
    parser = _parser()
    args = parser.parse_args(arguments)

    with pytest.raises(SystemExit):
        _validate_args(parser, args)


def test_apriltag_payload_serialization_rejects_non_finite_values() -> None:
    with pytest.raises(ValueError, match="Out of range float values"):
        _serialize_payload({"distance_m": math.nan})


def test_apriltag_cleanup_preserves_primary_error_and_attempts_all_resources(
    monkeypatch,
    capsys,
) -> None:
    calls: list[str] = []

    class FakeCamera:
        def __init__(self) -> None:
            calls.append("camera-init")

        def create_video_configuration(self, **_kwargs):
            calls.append("camera-create-configuration")
            return object()

        def configure(self, _configuration) -> None:
            calls.append("camera-configure")

        def start(self) -> None:
            calls.append("camera-start")
            raise ValueError("primary camera start failure")

        def stop(self) -> None:
            calls.append("camera-stop")
            raise RuntimeError("camera stop failure")

        def close(self) -> None:
            calls.append("camera-close")

    class FakeServer:
        def shutdown(self) -> None:
            calls.append("server-shutdown")
            raise RuntimeError("server shutdown failure")

        def server_close(self) -> None:
            calls.append("server-close")

    def fake_start_server(*, port: int) -> FakeServer:
        calls.append(f"server-start-{port}")
        return FakeServer()

    monkeypatch.setattr(apriltag_cli, "is_raspberry_pi", lambda: True)
    monkeypatch.setattr(
        apriltag_cli,
        "create_detector",
        lambda *_args, **_kwargs: SimpleNamespace(backend_name="fake"),
    )
    monkeypatch.setitem(sys.modules, "cv2", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "picamera2", SimpleNamespace(Picamera2=FakeCamera))
    from ai_drone.vision import stream

    monkeypatch.setattr(stream, "start_server", fake_start_server)

    with pytest.raises(ValueError, match="primary camera start failure"):
        apriltag_cli.run(["--output", "stream"])

    assert calls == [
        "camera-init",
        "camera-create-configuration",
        "camera-configure",
        "server-start-8081",
        "camera-start",
        "camera-stop",
        "camera-close",
        "server-shutdown",
        "server-close",
    ]
    assert "camera stop failed" in capsys.readouterr().err
