"""Run AprilTag detection on the Raspberry Pi AI Camera without flight control."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Protocol

from ai_drone.apriltags import CameraCalibration, create_detector, estimate_pose
from ai_drone.cli_parsing import parse_even_resolution
from ai_drone.platform import is_raspberry_pi

_TAG36H11_MAX_ID = 586


class _CameraCleanup(Protocol):
    def stop(self) -> object: ...

    def close(self) -> object: ...


class _ServerCleanup(Protocol):
    def shutdown(self) -> object: ...

    def server_close(self) -> object: ...


def _cleanup_resources(
    camera: _CameraCleanup | None,
    server: _ServerCleanup | None,
) -> list[Exception]:
    """Attempt every cleanup action and return failures in call order."""
    actions: list[tuple[str, Callable[[], object]]] = []
    if camera is not None:
        actions.extend(
            (
                ("camera stop", camera.stop),
                ("camera close", camera.close),
            )
        )
    if server is not None:
        actions.extend(
            (
                ("stream server shutdown", server.shutdown),
                ("stream server close", server.server_close),
            )
        )

    errors: list[Exception] = []
    for label, action in actions:
        try:
            action()
        except Exception as error:
            errors.append(error)
            print(f"WARNING: {label} failed: {error}", file=sys.stderr, flush=True)
    return errors


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Detect tag36h11 markers on the IMX500. This command never arms, "
            "moves the vehicle, or actuates the servo."
        )
    )
    parser.add_argument(
        "--backend", choices=("auto", "native", "opencv"), default="auto"
    )
    parser.add_argument("--resolution", type=parse_even_resolution, default=(1280, 960))
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--decimate", type=float, default=1.0)
    parser.add_argument("--target-id", type=int)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--tag-size", type=float, default=0.160, metavar="METRES")
    parser.add_argument("--max-reprojection-error", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--output", choices=("headless", "stream"), default="headless")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--exposure-us", type=int)
    parser.add_argument("--analogue-gain", type=float)
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.threads <= 0:
        parser.error("--threads must be positive")
    if not math.isfinite(args.decimate) or args.decimate < 1.0:
        parser.error("--decimate must be finite and at least 1.0")
    if not math.isfinite(args.tag_size) or args.tag_size <= 0:
        parser.error("--tag-size must be finite and positive")
    if (
        not math.isfinite(args.max_reprojection_error)
        or args.max_reprojection_error <= 0
    ):
        parser.error("--max-reprojection-error must be finite and positive")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and not negative")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.target_id is not None and not 0 <= args.target_id <= _TAG36H11_MAX_ID:
        parser.error(
            f"--target-id must be between 0 and {_TAG36H11_MAX_ID} for tag36h11"
        )
    if args.exposure_us is not None and args.exposure_us <= 0:
        parser.error("--exposure-us must be positive")
    if args.analogue_gain is not None and (
        not math.isfinite(args.analogue_gain) or args.analogue_gain <= 0
    ):
        parser.error("--analogue-gain must be finite and positive")


def _serialize_payload(payload: dict[str, object]) -> str:
    """Encode one standards-compliant detection payload."""
    return json.dumps(payload, allow_nan=False, sort_keys=True)


def run(arguments: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(arguments)
    _validate_args(parser, args)
    width, height = args.resolution
    if not is_raspberry_pi():
        raise SystemExit(
            "drone-apriltag is Pi-only. Deploy it from the laptop with "
            "'uv run drone-deploy --apriltag'."
        )

    import cv2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
    import numpy as np
    from picamera2 import (  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
        Picamera2,
    )

    detector = create_detector(
        args.backend,
        threads=args.threads,
        decimate=args.decimate,
    )
    calibration = CameraCalibration.load(args.calibration) if args.calibration else None
    if calibration is None:
        print(
            "WARNING: no camera calibration supplied; detections will not report "
            "metric distance or pose.",
            flush=True,
        )

    controls: dict[str, int | float] = {"FrameRate": 30}
    if args.exposure_us is not None:
        controls["ExposureTime"] = args.exposure_us
    if args.analogue_gain is not None:
        controls["AnalogueGain"] = args.analogue_gain

    camera = None
    server = None
    try:
        camera = Picamera2()
        camera.configure(
            camera.create_video_configuration(
                main={"format": "YUV420", "size": (width, height)},
                raw={"size": (2028, 1520)},
                controls=controls,
                buffer_count=4,
                queue=False,
            )
        )

        if args.output == "stream":
            from ai_drone.stream import start_server

            server = start_server(port=args.port)
            print(f"AprilTag stream: http://0.0.0.0:{args.port}/", flush=True)

        print(
            f"backend={detector.backend_name} resolution={width}x{height} "
            f"tag_size={args.tag_size:.3f}m",
            flush=True,
        )
        camera.start()
        started = time.monotonic()
        last_status = started
        frame_count = 0
        detection_count = 0
        while args.duration == 0 or time.monotonic() - started < args.duration:
            request = camera.capture_request()
            try:
                yuv = request.make_array("main")
                metadata = request.get_metadata()
            finally:
                request.release()
            luma = np.ascontiguousarray(yuv[:height, :width])
            detections = detector.detect(luma)
            if args.target_id is not None:
                detections = [
                    detection
                    for detection in detections
                    if detection.tag_id == args.target_id
                ]

            payloads = []
            for detection in detections:
                payload: dict[str, object] = {
                    "tag_id": detection.tag_id,
                    "center_px": [round(value, 3) for value in detection.center],
                    "corners_px": np.round(detection.corners, 3).tolist(),
                    "hamming": detection.hamming,
                    "decision_margin": detection.decision_margin,
                    "sensor_timestamp_ns": metadata.get("SensorTimestamp"),
                }
                if calibration is not None:
                    pose = estimate_pose(
                        detection,
                        calibration,
                        tag_size_m=args.tag_size,
                        image_width=width,
                        image_height=height,
                    )
                    payload["camera_xyz_m"] = [
                        round(value, 5) for value in pose.translation_m
                    ]
                    payload["distance_m"] = round(pose.distance_m, 5)
                    payload["reprojection_error_px"] = round(
                        pose.reprojection_error_px, 4
                    )
                    payload["pose_valid"] = (
                        pose.reprojection_error_px <= args.max_reprojection_error
                    )
                payloads.append(payload)
                print(_serialize_payload(payload), flush=True)

            frame_count += 1
            detection_count += len(detections)
            now = time.monotonic()
            if now - last_status >= 1.0:
                elapsed = now - started
                print(
                    f"status frames={frame_count} detections={detection_count} "
                    f"fps={frame_count / elapsed:.1f}",
                    flush=True,
                )
                last_status = now

            if args.output == "stream":
                from ai_drone.stream import push_frame

                annotated = cv2.cvtColor(luma, cv2.COLOR_GRAY2BGR)
                for detection, payload in zip(detections, payloads, strict=True):
                    points = (
                        np.rint(detection.corners).astype(np.int32).reshape(-1, 1, 2)
                    )
                    color = (0, 255, 0)
                    if payload.get("pose_valid") is False:
                        color = (0, 0, 255)
                    cv2.polylines(annotated, [points], True, color, 2)
                    label = f"id={detection.tag_id}"
                    if "distance_m" in payload:
                        label += f" {payload['distance_m']:.2f}m"
                    origin = tuple(np.rint(detection.corners[0]).astype(int))
                    cv2.putText(
                        annotated,
                        label,
                        origin,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )
                ok, jpeg = cv2.imencode(
                    ".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                if ok:
                    push_frame(jpeg.tobytes())
    except KeyboardInterrupt:
        print("AprilTag detection stopped.", flush=True)
    finally:
        cleanup_errors = _cleanup_resources(camera, server)
        if cleanup_errors and sys.exception() is None:
            raise RuntimeError("AprilTag resource cleanup failed") from cleanup_errors[
                0
            ]
    return 0


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
