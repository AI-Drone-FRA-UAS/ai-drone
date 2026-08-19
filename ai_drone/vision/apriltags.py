"""AprilTag detection and calibrated camera-relative pose estimation.

The Raspberry Pi runtime prefers Debian's native AprilTag 3 bindings. OpenCV's
AprilTag-capable ArUco detector remains available as a fallback. Metric pose is
never inferred from field of view alone: it requires a saved camera calibration
and the measured black-border tag size.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np

_VALID_DISTORTION_COEFFICIENT_COUNTS = frozenset({4, 5, 8, 12, 14})


def _json_number(value: object, *, field: str) -> float:
    """Parse one finite JSON number without accepting booleans or strings."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{field} must be a number")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{field} must be finite")
    return parsed


def _json_positive_int(value: object, *, field: str) -> int:
    """Parse a strictly positive JSON integer without lossy coercion."""
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _validate_detector_options(
    family: str,
    *,
    threads: int,
    decimate: float | None = None,
) -> None:
    if not isinstance(family, str) or not family.strip():
        raise ValueError("family must be a non-empty string")
    if isinstance(threads, bool) or not isinstance(threads, int) or threads <= 0:
        raise ValueError("threads must be a positive integer")
    if decimate is not None and (not math.isfinite(decimate) or decimate < 1.0):
        raise ValueError("decimate must be finite and at least 1.0")


@dataclass(frozen=True)
class CameraCalibration:
    """Pinhole intrinsics and lens distortion at a known image resolution."""

    image_width: int
    image_height: int
    camera_matrix: tuple[tuple[float, ...], ...]
    distortion_coefficients: tuple[float, ...]

    @classmethod
    def load(cls, path: str | Path) -> CameraCalibration:
        """Load and validate a calibration JSON file."""
        source = Path(path)
        data = json.loads(source.read_text())
        if not isinstance(data, dict):
            raise ValueError("camera calibration must be a JSON object")
        matrix_data = data.get("camera_matrix")
        if not isinstance(matrix_data, list):
            raise ValueError("camera_matrix must be a 3x3 array")
        distortion_data = data.get("distortion_coefficients")
        if not isinstance(distortion_data, list):
            raise ValueError("distortion_coefficients must be an array")

        matrix: list[tuple[float, ...]] = []
        for row_index, row in enumerate(matrix_data):
            if not isinstance(row, list):
                raise ValueError("camera_matrix must be a 3x3 array")
            matrix.append(
                tuple(
                    _json_number(
                        value,
                        field=f"camera_matrix[{row_index}][{column_index}]",
                    )
                    for column_index, value in enumerate(row)
                )
            )
        calibration = cls(
            image_width=_json_positive_int(
                data.get("image_width"), field="image_width"
            ),
            image_height=_json_positive_int(
                data.get("image_height"), field="image_height"
            ),
            camera_matrix=tuple(matrix),
            distortion_coefficients=tuple(
                _json_number(value, field=f"distortion_coefficients[{index}]")
                for index, value in enumerate(distortion_data)
            ),
        )
        calibration.validate()
        return calibration

    def validate(self) -> None:
        """Reject malformed or physically implausible intrinsics."""
        if (
            isinstance(self.image_width, bool)
            or not isinstance(self.image_width, int)
            or self.image_width <= 0
            or isinstance(self.image_height, bool)
            or not isinstance(self.image_height, int)
            or self.image_height <= 0
        ):
            raise ValueError("calibration image dimensions must be positive integers")
        if len(self.camera_matrix) != 3 or any(
            len(row) != 3 for row in self.camera_matrix
        ):
            raise ValueError("camera_matrix must be a 3x3 array")
        if not all(math.isfinite(value) for row in self.camera_matrix for value in row):
            raise ValueError("camera_matrix values must be finite")
        fx = self.camera_matrix[0][0]
        fy = self.camera_matrix[1][1]
        if fx <= 0 or fy <= 0:
            raise ValueError("camera focal lengths must be finite and positive")
        cx = self.camera_matrix[0][2]
        cy = self.camera_matrix[1][2]
        if not (0 <= cx < self.image_width and 0 <= cy < self.image_height):
            raise ValueError(
                "camera principal point must be inside the calibrated image"
            )
        expected_fixed_entries = (
            (self.camera_matrix[0][1], 0.0),
            (self.camera_matrix[1][0], 0.0),
            (self.camera_matrix[2][0], 0.0),
            (self.camera_matrix[2][1], 0.0),
            (self.camera_matrix[2][2], 1.0),
        )
        if any(
            not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
            for actual, expected in expected_fixed_entries
        ):
            raise ValueError(
                "camera_matrix must use OpenCV pinhole form "
                "[[fx,0,cx],[0,fy,cy],[0,0,1]]"
            )
        if (
            len(self.distortion_coefficients)
            not in _VALID_DISTORTION_COEFFICIENT_COUNTS
        ):
            valid_counts = sorted(_VALID_DISTORTION_COEFFICIENT_COUNTS)
            raise ValueError(
                f"distortion_coefficients must contain {valid_counts} values"
            )
        if not all(math.isfinite(value) for value in self.distortion_coefficients):
            raise ValueError("distortion_coefficients values must be finite")

    def arrays_for(self, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
        """Return intrinsics scaled to a resolution with the same aspect ratio."""
        self.validate()
        if (
            isinstance(width, bool)
            or not isinstance(width, int)
            or width <= 0
            or isinstance(height, bool)
            or not isinstance(height, int)
            or height <= 0
        ):
            raise ValueError("target image dimensions must be positive integers")
        source_aspect = self.image_width / self.image_height
        target_aspect = width / height
        if not math.isclose(source_aspect, target_aspect, rel_tol=0.0, abs_tol=1e-3):
            raise ValueError(
                "calibration and target resolutions have different aspect ratios; "
                "crop-aware calibration is required"
            )

        scale_x = width / self.image_width
        scale_y = height / self.image_height
        matrix = np.asarray(self.camera_matrix, dtype=np.float64).copy()
        matrix[0, 0] *= scale_x
        matrix[0, 2] *= scale_x
        matrix[1, 1] *= scale_y
        matrix[1, 2] *= scale_y
        distortion = np.asarray(self.distortion_coefficients, dtype=np.float64)
        return matrix, distortion


@dataclass(frozen=True)
class TagDetection:
    """One decoded tag, with corners ordered top-left clockwise."""

    tag_id: int
    corners: np.ndarray
    center: tuple[float, float]
    hamming: int | None = None
    decision_margin: float | None = None

    def __post_init__(self) -> None:
        if (
            isinstance(self.tag_id, bool)
            or not isinstance(self.tag_id, int)
            or self.tag_id < 0
        ):
            raise ValueError("tag_id must be a non-negative integer")
        try:
            corners = np.asarray(self.corners, dtype=np.float64)
        except (TypeError, ValueError) as error:
            raise ValueError("tag corners must contain numeric values") from error
        if corners.shape != (4, 2):
            raise ValueError("tag corners must be a 4x2 array")
        if not np.isfinite(corners).all():
            raise ValueError("tag corners must be finite")
        corners = np.ascontiguousarray(corners)
        corners.setflags(write=False)
        object.__setattr__(self, "corners", corners)

        try:
            center = tuple(float(value) for value in self.center)
        except (TypeError, ValueError) as error:
            raise ValueError("tag center must contain numeric values") from error
        if len(center) != 2 or not all(math.isfinite(value) for value in center):
            raise ValueError("tag center must contain two finite values")
        object.__setattr__(self, "center", center)
        if self.hamming is not None and (
            isinstance(self.hamming, bool)
            or not isinstance(self.hamming, int)
            or self.hamming < 0
        ):
            raise ValueError("tag hamming distance must be a non-negative integer")
        if self.decision_margin is not None and (
            not math.isfinite(self.decision_margin) or self.decision_margin < 0
        ):
            raise ValueError("tag decision margin must be finite and non-negative")


@dataclass(frozen=True)
class TagPose:
    """Camera-relative pose in OpenCV's right/down/forward camera frame."""

    tag_id: int
    rotation_vector: tuple[float, float, float]
    translation_m: tuple[float, float, float]
    distance_m: float
    reprojection_error_px: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.tag_id, bool)
            or not isinstance(self.tag_id, int)
            or self.tag_id < 0
        ):
            raise ValueError("tag_id must be a non-negative integer")
        if len(self.rotation_vector) != 3 or not all(
            math.isfinite(value) for value in self.rotation_vector
        ):
            raise ValueError("rotation_vector must contain three finite values")
        if len(self.translation_m) != 3 or not all(
            math.isfinite(value) for value in self.translation_m
        ):
            raise ValueError("translation_m must contain three finite values")
        if self.translation_m[2] <= 0:
            raise ValueError("tag translation must have positive camera depth")
        if not math.isfinite(self.distance_m) or self.distance_m <= 0:
            raise ValueError("tag distance must be finite and positive")
        if (
            not math.isfinite(self.reprojection_error_px)
            or self.reprojection_error_px < 0
        ):
            raise ValueError("reprojection error must be finite and non-negative")


class Detector(Protocol):
    """Common detector interface used by the camera CLI."""

    backend_name: str

    def detect(self, grayscale: np.ndarray) -> list[TagDetection]: ...


def _validate_grayscale(grayscale: np.ndarray) -> np.ndarray:
    if grayscale.ndim != 2 or grayscale.dtype != np.uint8:
        raise ValueError("AprilTag input must be a two-dimensional uint8 image")
    return np.ascontiguousarray(grayscale)


class NativeAprilTagDetector:
    """AprilTag 3 C detector exposed by Debian's ``python3-apriltag``."""

    backend_name = "native-apriltag3"

    def __init__(
        self,
        family: str = "tag36h11",
        *,
        threads: int = 4,
        decimate: float = 1.0,
    ) -> None:
        _validate_detector_options(family, threads=threads, decimate=decimate)
        try:
            import apriltag  # ty: ignore[unresolved-import]
        except ImportError as error:
            raise RuntimeError(
                "native AprilTag bindings are unavailable; install "
                "'python3-apriltag' on Raspberry Pi OS"
            ) from error

        self._detector = apriltag.apriltag(
            family,
            threads=threads,
            decimate=decimate,
            blur=0.0,
            refine_edges=True,
            debug=False,
        )

    def detect(self, grayscale: np.ndarray) -> list[TagDetection]:
        image = _validate_grayscale(grayscale)
        detections: list[TagDetection] = []
        for raw in self._detector.detect(image):
            # Native bindings expose lb-rb-rt-lt; normalize to TL-TR-BR-BL.
            native = np.asarray(raw["lb-rb-rt-lt"], dtype=np.float64)
            corners = np.ascontiguousarray(native[[3, 2, 1, 0]])
            center_values = np.asarray(raw["center"], dtype=np.float64).reshape(2)
            detections.append(
                TagDetection(
                    tag_id=int(raw["id"]),
                    corners=corners,
                    center=(float(center_values[0]), float(center_values[1])),
                    hamming=int(raw["hamming"]),
                    decision_margin=float(raw["margin"]),
                )
            )
        return detections


class OpenCvAprilTagDetector:
    """OpenCV detector used when native AprilTag bindings are unavailable."""

    backend_name = "opencv-aruco"

    def __init__(self, family: str = "tag36h11", *, threads: int = 4) -> None:
        _validate_detector_options(family, threads=threads)
        if family != "tag36h11":
            raise ValueError("the OpenCV backend currently supports tag36h11 only")
        try:
            import cv2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
        except ImportError as error:
            raise RuntimeError("OpenCV is unavailable") from error

        cv2.setNumThreads(threads)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def detect(self, grayscale: np.ndarray) -> list[TagDetection]:
        image = _validate_grayscale(grayscale)
        corners, ids, _rejected = self._detector.detectMarkers(image)
        if ids is None:
            return []
        return [
            TagDetection(
                tag_id=int(tag_id),
                corners=np.ascontiguousarray(points.reshape(4, 2), dtype=np.float64),
                center=(
                    float(points.reshape(4, 2)[:, 0].mean()),
                    float(points.reshape(4, 2)[:, 1].mean()),
                ),
            )
            for points, tag_id in zip(corners, ids.ravel(), strict=True)
        ]


def create_detector(
    backend: str = "auto",
    family: str = "tag36h11",
    *,
    threads: int = 4,
    decimate: float = 1.0,
) -> Detector:
    """Create the requested backend, preferring native AprilTag in auto mode."""
    if backend not in {"auto", "native", "opencv"}:
        raise ValueError("backend must be auto, native, or opencv")
    _validate_detector_options(family, threads=threads, decimate=decimate)
    if backend in {"auto", "native"}:
        try:
            return NativeAprilTagDetector(
                family,
                threads=threads,
                decimate=decimate,
            )
        except RuntimeError:
            if backend == "native":
                raise
    return OpenCvAprilTagDetector(family, threads=threads)


def estimate_pose(
    detection: TagDetection,
    calibration: CameraCalibration,
    *,
    tag_size_m: float,
    image_width: int,
    image_height: int,
) -> TagPose:
    """Estimate metric pose using square-IPPE and report pixel reprojection error."""
    if not math.isfinite(tag_size_m) or tag_size_m <= 0:
        raise ValueError("tag_size_m must be finite and positive")
    camera_matrix, distortion = calibration.arrays_for(image_width, image_height)
    try:
        import cv2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
    except ImportError as error:
        raise RuntimeError(
            "OpenCV is required for calibrated pose estimation"
        ) from error

    half = tag_size_m / 2.0
    object_points = np.asarray(
        [
            (-half, half, 0.0),
            (half, half, 0.0),
            (half, -half, 0.0),
            (-half, -half, 0.0),
        ],
        dtype=np.float64,
    )
    image_points = np.asarray(detection.corners, dtype=np.float64).reshape(4, 2)

    result = cv2.solvePnPGeneric(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    solved, rotation_vectors, translation_vectors = result[:3]
    if not solved:
        raise RuntimeError(f"pose solve failed for tag {detection.tag_id}")

    candidates: list[tuple[float, np.ndarray, np.ndarray]] = []
    for rotation, translation in zip(
        rotation_vectors, translation_vectors, strict=True
    ):
        rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 1)
        translation = np.asarray(translation, dtype=np.float64).reshape(3, 1)
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            continue
        if float(translation[2, 0]) <= 0:
            continue
        projected, _jacobian = cv2.projectPoints(
            object_points,
            rotation,
            translation,
            camera_matrix,
            distortion,
        )
        if not np.isfinite(projected).all():
            continue
        residual = projected.reshape(4, 2) - image_points
        error = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        if not math.isfinite(error):
            continue
        candidates.append((error, rotation, translation))

    if not candidates:
        raise RuntimeError(f"no positive-depth pose for tag {detection.tag_id}")
    error, rotation, translation = min(candidates, key=lambda item: item[0])
    translation_values = translation.ravel()
    rotation_values = rotation.ravel()
    xyz = (
        float(translation_values[0]),
        float(translation_values[1]),
        float(translation_values[2]),
    )
    rvec = (
        float(rotation_values[0]),
        float(rotation_values[1]),
        float(rotation_values[2]),
    )
    return TagPose(
        tag_id=detection.tag_id,
        rotation_vector=rvec,
        translation_m=xyz,
        distance_m=math.sqrt(sum(value * value for value in xyz)),
        reprojection_error_px=error,
    )
