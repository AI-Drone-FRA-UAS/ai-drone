"""Record the Pi camera and all available disarmed ArduPilot telemetry."""

from __future__ import annotations

import argparse
import json
import queue
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pymavlink import mavutil

from ai_drone.apriltags import CameraCalibration, create_detector, estimate_pose
from ai_drone.mavlink_devices import find_serial_device
from ai_drone.recording import (
    HARDWARE_TOPOLOGY,
    create_recording_paths,
    is_armed_vehicle_heartbeat,
    json_safe,
    request_telemetry_messages,
    telemetry_record,
    video_timestamp_summary,
    write_json_line,
)


def _is_raspberry_pi() -> bool:
    try:
        model = Path("/proc/device-tree/model").read_text()
    except (FileNotFoundError, PermissionError):
        return False
    return "raspberry pi" in model.lower()


def _resolution(value: str) -> tuple[int, int]:
    try:
        width_text, height_text = value.lower().split("x", 1)
        width, height = int(width_text), int(height_text)
    except (ValueError, TypeError) as error:
        raise argparse.ArgumentTypeError(
            "resolution must look like 1280x960"
        ) from error
    if width <= 0 or height <= 0 or width % 2 or height % 2:
        raise argparse.ArgumentTypeError(
            "resolution dimensions must be positive even integers"
        )
    return width, height


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record camera video, AprilTag detections, and all available FC sensor "
            "telemetry for an exact requested interval. This command never arms, "
            "changes flight mode, moves a motor, or actuates a servo."
        )
    )
    parser.add_argument("--duration", type=float, default=10.0, metavar="SECONDS")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--resolution", type=_resolution, default=(1280, 960))
    parser.add_argument("--analysis-resolution", type=_resolution, default=(640, 480))
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--bitrate", type=int, default=8_000_000)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument(
        "--backend", choices=("auto", "native", "opencv"), default="auto"
    )
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--decimate", type=float, default=1.0)
    parser.add_argument("--detect-every", type=int, default=1, metavar="FRAMES")
    parser.add_argument("--target-id", type=int)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--tag-size", type=float, default=0.160, metavar="METRES")
    parser.add_argument("--max-reprojection-error", type=float, default=2.0)
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    positive = {
        "--duration": args.duration,
        "--baud": args.baud,
        "--timeout": args.timeout,
        "--fps": args.fps,
        "--bitrate": args.bitrate,
        "--threads": args.threads,
        "--detect-every": args.detect_every,
        "--tag-size": args.tag_size,
        "--max-reprojection-error": args.max_reprojection_error,
    }
    for name, value in positive.items():
        if value <= 0:
            parser.error(f"{name} must be positive")
    if args.warmup < 0:
        parser.error("--warmup must not be negative")
    if args.decimate < 1.0:
        parser.error("--decimate must be at least 1.0")
    if args.resolution[0] / args.resolution[1] != (
        args.analysis_resolution[0] / args.analysis_resolution[1]
    ):
        parser.error(
            "recording and analysis resolutions must have the same aspect ratio"
        )


@dataclass
class CaptureState:
    """Thread-safe-enough counters written by one worker per field."""

    telemetry_counts: Counter[str] = field(default_factory=Counter)
    camera_frames: int = 0
    processed_frames: int = 0
    dropped_analysis_frames: int = 0
    tag_detections: int = 0
    tag_ids: Counter[int] = field(default_factory=Counter)
    armed_abort: bool = False
    worker_error: str | None = None


class TelemetryWorker(threading.Thread):
    """Continuously drain the UART so no camera processing blocks telemetry."""

    def __init__(
        self,
        *,
        connection: Any,
        output: Path,
        vehicle_system: int,
        started: float,
        deadline: float,
        stop: threading.Event,
        state: CaptureState,
    ) -> None:
        super().__init__(name="telemetry-recorder", daemon=True)
        self.connection = connection
        self.output = output
        self.vehicle_system = vehicle_system
        self.started = started
        self.deadline = deadline
        self.stop = stop
        self.state = state

    def run(self) -> None:
        try:
            with self.output.open("w") as handle:
                while not self.stop.is_set() and time.monotonic() < self.deadline:
                    remaining = max(0.0, self.deadline - time.monotonic())
                    message = self.connection.recv_match(
                        blocking=True,
                        timeout=min(0.1, remaining),
                    )
                    if message is None:
                        continue
                    message_type = message.get_type()
                    self.state.telemetry_counts[message_type] += 1
                    write_json_line(
                        handle,
                        telemetry_record(
                            message,
                            elapsed_s=time.monotonic() - self.started,
                        ),
                    )
                    if is_armed_vehicle_heartbeat(
                        message,
                        vehicle_system=self.vehicle_system,
                    ):
                        self.state.armed_abort = True
                        self.stop.set()
                        return
        except Exception as error:  # keep the camera cleanup path deterministic
            self.state.worker_error = f"telemetry worker: {error}"
            self.stop.set()


class DetectionWorker(threading.Thread):
    """Decode tags without delaying the video encoder or telemetry reader."""

    def __init__(
        self,
        *,
        frames: queue.Queue[Any],
        output: Path,
        detector: Any,
        calibration: CameraCalibration | None,
        tag_size: float,
        resolution: tuple[int, int],
        max_reprojection_error: float,
        target_id: int | None,
        state: CaptureState,
    ) -> None:
        super().__init__(name="apriltag-recorder", daemon=True)
        self.frames = frames
        self.output = output
        self.detector = detector
        self.calibration = calibration
        self.tag_size = tag_size
        self.resolution = resolution
        self.max_reprojection_error = max_reprojection_error
        self.target_id = target_id
        self.state = state

    def run(self) -> None:
        try:
            with self.output.open("w") as handle:
                while True:
                    item = self.frames.get()
                    try:
                        if item is None:
                            return
                        frame_index, elapsed_s, grayscale, metadata = item
                        detections = self.detector.detect(grayscale)
                        if self.target_id is not None:
                            detections = [
                                detection
                                for detection in detections
                                if detection.tag_id == self.target_id
                            ]
                        tags = []
                        for detection in detections:
                            tag: dict[str, Any] = {
                                "id": detection.tag_id,
                                "center_px": list(detection.center),
                                "corners_px": detection.corners.tolist(),
                                "hamming": detection.hamming,
                                "decision_margin": detection.decision_margin,
                            }
                            if self.calibration is not None:
                                pose = estimate_pose(
                                    detection,
                                    self.calibration,
                                    tag_size_m=self.tag_size,
                                    image_width=self.resolution[0],
                                    image_height=self.resolution[1],
                                )
                                tag.update(
                                    camera_xyz_m=list(pose.translation_m),
                                    distance_m=pose.distance_m,
                                    reprojection_error_px=pose.reprojection_error_px,
                                    pose_valid=(
                                        pose.reprojection_error_px
                                        <= self.max_reprojection_error
                                    ),
                                )
                            tags.append(tag)
                            self.state.tag_ids[detection.tag_id] += 1
                        self.state.tag_detections += len(tags)
                        self.state.processed_frames += 1
                        write_json_line(
                            handle,
                            {
                                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                                "elapsed_s": round(elapsed_s, 6),
                                "frame": frame_index,
                                "metadata": json_safe(metadata),
                                "tags": tags,
                            },
                        )
                    finally:
                        self.frames.task_done()
        except Exception as error:
            self.state.worker_error = f"AprilTag worker: {error}"


def _camera_metadata(metadata: dict[str, Any]) -> dict[str, Any]:
    names = (
        "SensorTimestamp",
        "ExposureTime",
        "AnalogueGain",
        "DigitalGain",
        "ColourTemperature",
        "Lux",
        "FrameDuration",
        "FocusFoM",
        "LensPosition",
    )
    return {name: metadata[name] for name in names if name in metadata}


def run(arguments: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(arguments)
    _validate_args(parser, args)
    if not _is_raspberry_pi():
        raise SystemExit(
            "drone-record is Pi-only. Run it through "
            "'uv run drone-deploy --record --duration SECONDS'."
        )

    import cv2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
    import numpy as np
    from picamera2 import Picamera2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
    from picamera2.encoders import H264Encoder  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
    from picamera2.outputs import FileOutput  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]

    class FirstFrameFileOutput(FileOutput):
        """Signal after Picamera2 writes the first encoded keyframe."""

        def __init__(self, file: str, pts: str) -> None:
            super().__init__(file, pts=pts)
            self.first_frame = threading.Event()

        def outputframe(
            self,
            frame: bytes,
            keyframe: bool = True,
            timestamp: int | None = None,
            packet: Any = None,
            audio: bool = False,
        ) -> None:
            should_signal = (
                not self.first_frame.is_set()
                and self.recording
                and keyframe
                and not audio
            )
            super().outputframe(frame, keyframe, timestamp, packet, audio)
            if should_signal and not self.dead:
                self.first_frame.set()

    device = find_serial_device(
        args.device,
        include_pi_uart=True,
        missing_message="No ArduPilot serial device found.",
    )
    paths = create_recording_paths(args.output_dir)
    calibration = CameraCalibration.load(args.calibration) if args.calibration else None
    detector = create_detector(
        args.backend,
        threads=args.threads,
        decimate=args.decimate,
    )

    print(f"Connecting to {device} at {args.baud} baud ...", flush=True)
    connection = mavutil.mavlink_connection(str(device), baud=args.baud)
    camera = None
    encoder = None
    camera_started = False
    encoder_started = False
    started_monotonic = 0.0
    ended_monotonic = 0.0
    started_utc: datetime | None = None
    ended_utc: datetime | None = None
    stop = threading.Event()
    state = CaptureState()
    requested_messages: list[str] = []
    initial_vehicle_state = "unknown"
    first_frame = None
    last_frame = None
    telemetry_worker = None
    detection_worker = None
    frames: queue.Queue[Any] = queue.Queue(maxsize=8)

    try:
        heartbeat = connection.wait_heartbeat(timeout=args.timeout)
        if heartbeat is None:
            raise RuntimeError("No ArduPilot heartbeat received")
        vehicle_system = connection.target_system
        if is_armed_vehicle_heartbeat(heartbeat, vehicle_system=vehicle_system):
            initial_vehicle_state = "armed"
            raise RuntimeError("Vehicle is ARMED; recording refused")
        initial_vehicle_state = "disarmed"
        print(
            f"Vehicle {vehicle_system} is DISARMED. No actuator commands will be sent.",
            flush=True,
        )

        requested_messages = request_telemetry_messages(connection)
        camera = Picamera2()
        main_resolution = args.resolution
        analysis_resolution = args.analysis_resolution
        camera.configure(
            camera.create_video_configuration(
                main={"format": "YUV420", "size": main_resolution},
                lores={"format": "YUV420", "size": analysis_resolution},
                raw={"size": (2028, 1520)},
                controls={"FrameRate": args.fps},
                buffer_count=6,
                queue=False,
            )
        )
        camera.start()
        camera_started = True
        if args.warmup:
            print(f"Camera warm-up: {args.warmup:.1f} s ...", flush=True)
            time.sleep(args.warmup)

        encoder = H264Encoder(bitrate=args.bitrate, repeat=True)
        video_output = FirstFrameFileOutput(
            str(paths.video),
            str(paths.video_timestamps),
        )
        connection.setup_logfile(str(paths.telemetry_tlog))
        camera.start_encoder(encoder, video_output, name="main")
        encoder_started = True
        if not video_output.first_frame.wait(timeout=max(2.0, 10.0 / args.fps)):
            raise RuntimeError("H.264 encoder did not produce a keyframe")

        started_monotonic = time.monotonic()
        started_utc = datetime.now(timezone.utc)
        deadline = started_monotonic + args.duration
        telemetry_worker = TelemetryWorker(
            connection=connection,
            output=paths.telemetry_events,
            vehicle_system=vehicle_system,
            started=started_monotonic,
            deadline=deadline,
            stop=stop,
            state=state,
        )
        detection_worker = DetectionWorker(
            frames=frames,
            output=paths.camera_events,
            detector=detector,
            calibration=calibration,
            tag_size=args.tag_size,
            resolution=analysis_resolution,
            max_reprojection_error=args.max_reprojection_error,
            target_id=args.target_id,
            state=state,
        )
        telemetry_worker.start()
        detection_worker.start()
        print(
            f"Recording all sensors for {args.duration:.3f} s to {paths.root} ...",
            flush=True,
        )

        frame_index = 0
        while not stop.is_set() and time.monotonic() < deadline:
            request = camera.capture_request()
            try:
                yuv = request.make_array("lores")
                metadata = _camera_metadata(request.get_metadata())
                captured_at = time.monotonic()
                height, width = analysis_resolution[1], analysis_resolution[0]
                grayscale = np.ascontiguousarray(yuv[:height, :width]).copy()
            finally:
                request.release()
            if captured_at > deadline:
                break
            state.camera_frames += 1
            if first_frame is None:
                first_frame = grayscale.copy()
            last_frame = grayscale.copy()
            if frame_index % args.detect_every == 0:
                item = (
                    frame_index,
                    captured_at - started_monotonic,
                    grayscale,
                    metadata,
                )
                try:
                    frames.put_nowait(item)
                except queue.Full:
                    state.dropped_analysis_frames += 1
            frame_index += 1

        stop.set()
        ended_monotonic = time.monotonic()
        ended_utc = datetime.now(timezone.utc)
    except KeyboardInterrupt:
        stop.set()
        ended_monotonic = time.monotonic()
        ended_utc = datetime.now(timezone.utc)
        state.worker_error = "interrupted by user"
    except Exception as error:
        stop.set()
        ended_monotonic = time.monotonic()
        ended_utc = datetime.now(timezone.utc)
        state.worker_error = str(error)
    finally:
        if encoder_started and camera is not None and encoder is not None:
            camera.stop_encoder(encoder)
        if camera_started and camera is not None:
            camera.stop()
        if camera is not None:
            camera.close()

        if telemetry_worker is not None:
            telemetry_worker.join(timeout=2.0)
        if detection_worker is not None:
            while detection_worker.is_alive():
                try:
                    frames.put(None, timeout=0.1)
                    break
                except queue.Full:
                    continue
            detection_worker.join(timeout=10.0)
            if detection_worker.is_alive() and state.worker_error is None:
                state.worker_error = "AprilTag worker did not stop within 10 seconds"

        if first_frame is not None:
            cv2.imwrite(str(paths.first_frame), first_frame)
        if last_frame is not None:
            cv2.imwrite(str(paths.last_frame), last_frame)

        logfile = getattr(connection, "logfile", None)
        if logfile is not None:
            logfile.flush()
            logfile.close()
            connection.logfile = None
        connection.close()

    actual_duration = (
        max(0.0, ended_monotonic - started_monotonic) if started_monotonic else 0.0
    )
    video_summary = video_timestamp_summary(paths.video_timestamps)
    manifest = {
        "requested_duration_s": args.duration,
        "actual_duration_s": round(actual_duration, 6),
        "started_utc": started_utc.isoformat() if started_utc else None,
        "ended_utc": ended_utc.isoformat() if ended_utc else None,
        "completed": (
            not state.armed_abort
            and state.worker_error is None
            and actual_duration >= args.duration - 0.1
        ),
        "armed_abort": state.armed_abort,
        "error": state.worker_error,
        "safety": {
            "initial_vehicle_state": initial_vehicle_state,
            "commands_never_sent": [
                "arm",
                "disarm",
                "mode change",
                "motor/throttle",
                "RC override",
                "servo",
                "mission start",
            ],
        },
        "camera": {
            "recording_resolution": list(args.resolution),
            "analysis_resolution": list(args.analysis_resolution),
            "fps_requested": args.fps,
            "bitrate": args.bitrate,
            "backend": detector.backend_name,
            "analysis_frames": state.camera_frames,
            "processed_frames": state.processed_frames,
            "dropped_analysis_frames": state.dropped_analysis_frames,
            "tag_detections": state.tag_detections,
            "tag_ids": dict(sorted(state.tag_ids.items())),
            "metric_pose_enabled": calibration is not None,
            **video_summary,
        },
        "telemetry": {
            "device": str(device),
            "baud": args.baud,
            "requested_messages": requested_messages,
            "message_counts": dict(sorted(state.telemetry_counts.items())),
        },
        "hardware_topology": HARDWARE_TOPOLOGY,
        "files": {
            "video": paths.video.name,
            "video_timestamps": paths.video_timestamps.name,
            "camera_events": paths.camera_events.name,
            "telemetry_tlog": paths.telemetry_tlog.name,
            "telemetry_events": paths.telemetry_events.name,
            "first_frame": paths.first_frame.name,
            "last_frame": paths.last_frame.name,
        },
    }
    paths.manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

    print(
        f"Finished in {actual_duration:.3f} s: camera={state.camera_frames} frames, "
        f"telemetry={sum(state.telemetry_counts.values())} messages, "
        f"tags={state.tag_detections}",
        flush=True,
    )
    print(f"Manifest: {paths.manifest}", flush=True)
    if state.armed_abort:
        print("ABORTED: the vehicle reported ARMED.", flush=True)
        return 3
    if state.worker_error is not None:
        print(f"FAILED: {state.worker_error}", flush=True)
        return 2
    return 0


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
