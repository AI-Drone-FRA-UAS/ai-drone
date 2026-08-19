"""Record the Pi camera and all available disarmed ArduPilot telemetry."""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any

from pymavlink import mavutil

from ai_drone.apriltags import (
    CameraCalibration,
    Detector,
    create_detector,
    estimate_pose,
)
from ai_drone.cli_parsing import parse_even_resolution
from ai_drone.durability import (
    DEFAULT_SYNC_INTERVAL_S,
    IntervalSync,
    atomic_write_text,
    synced_stream,
)
from ai_drone.mavlink_devices import find_serial_device
from ai_drone.mavlink_safety import (
    heartbeat_is_armed,
    is_armed_vehicle_heartbeat,
    is_vehicle_message,
)
from ai_drone.platform import is_raspberry_pi
from ai_drone.recording import (
    HARDWARE_TOPOLOGY,
    RecordingPaths,
    create_recording_paths,
    json_safe,
    request_telemetry_messages,
    telemetry_record,
    video_timestamp_summary,
    write_json_line,
)

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

_TAG36H11_MAX_ID = 586


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
    parser.add_argument("--resolution", type=parse_even_resolution, default=(1280, 960))
    parser.add_argument(
        "--analysis-resolution", type=parse_even_resolution, default=(640, 480)
    )
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
    parser.add_argument(
        "--sync-interval",
        type=float,
        default=DEFAULT_SYNC_INTERVAL_S,
        metavar="SECONDS",
        help=(
            "Bound how much recorded telemetry and detection data a sudden power "
            "loss can discard. 0 disables periodic syncing for maximum capture "
            "throughput; each stream is still synced once when it closes."
        ),
    )
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
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")
    if not math.isfinite(args.sync_interval) or args.sync_interval < 0:
        parser.error("--sync-interval must be finite and not negative")
    if not math.isfinite(args.warmup) or args.warmup < 0:
        parser.error("--warmup must be finite and not negative")
    if not math.isfinite(args.decimate) or args.decimate < 1.0:
        parser.error("--decimate must be finite and at least 1.0")
    if args.target_id is not None and not 0 <= args.target_id <= _TAG36H11_MAX_ID:
        parser.error(
            f"--target-id must be between 0 and {_TAG36H11_MAX_ID} for tag36h11"
        )
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
    _error_lock: threading.Lock = field(
        default_factory=threading.Lock,
        repr=False,
    )
    disarmed_heartbeat: threading.Event = field(
        default_factory=threading.Event,
        repr=False,
    )

    def record_error(self, message: str) -> None:
        """Retain the first capture or cleanup error across all worker threads."""

        with self._error_lock:
            if self.worker_error is None:
                self.worker_error = message


class CaptureWindow:
    """One synchronized video, telemetry, and analysis capture epoch."""

    def __init__(self, duration: float) -> None:
        self.duration = duration
        self.started = threading.Event()
        self.started_monotonic: float | None = None
        self.started_utc: datetime | None = None
        self.deadline: float | None = None
        self._lock = threading.Lock()

    def begin(self) -> None:
        """Set the epoch once, immediately after the first encoded keyframe."""

        with self._lock:
            if self.started.is_set():
                return
            started_monotonic = time.monotonic()
            self.started_monotonic = started_monotonic
            self.started_utc = datetime.now(UTC)
            self.deadline = started_monotonic + self.duration
            self.started.set()

    def require_started(self) -> tuple[float, datetime, float]:
        """Return the established epoch or fail if encoding never began."""

        if (
            self.started_monotonic is None
            or self.started_utc is None
            or self.deadline is None
        ):
            raise RuntimeError("capture epoch was not established")
        return self.started_monotonic, self.started_utc, self.deadline


@dataclass(frozen=True)
class AnalysisFrame:
    """One camera frame queued for asynchronous AprilTag analysis."""

    frame_index: int
    elapsed_s: float
    grayscale: NDArray[np.uint8]
    metadata: dict[str, object]


class TelemetryWorker(threading.Thread):
    """Monitor arming immediately, then record from the shared capture epoch."""

    def __init__(
        self,
        *,
        connection: Any,
        output: Path,
        vehicle_system: int,
        vehicle_component: int,
        window: CaptureWindow,
        stop: threading.Event,
        state: CaptureState,
        sync: IntervalSync,
    ) -> None:
        super().__init__(name="telemetry-recorder", daemon=True)
        self.connection = connection
        self.output = output
        self.vehicle_system = vehicle_system
        self.vehicle_component = vehicle_component
        self.window = window
        self.stop = stop
        self.state = state
        self.sync = sync

    def run(self) -> None:
        try:
            with synced_stream(self.output, self.sync) as handle:
                while not self.stop.is_set():
                    deadline = self.window.deadline
                    if deadline is not None and time.monotonic() >= deadline:
                        self.stop.set()
                        return
                    timeout = (
                        min(0.1, max(0.0, deadline - time.monotonic()))
                        if deadline is not None
                        else 0.1
                    )
                    message = self.connection.recv_match(
                        blocking=True,
                        timeout=timeout,
                    )
                    if message is None:
                        continue
                    received_at = time.monotonic()
                    is_vehicle_heartbeat = message.get_type() == "HEARTBEAT" and (
                        is_vehicle_message(
                            message,
                            system_id=self.vehicle_system,
                            component_id=self.vehicle_component,
                        )
                    )
                    if is_vehicle_heartbeat and heartbeat_is_armed(message):
                        self.state.armed_abort = True
                        self.state.record_error(
                            "vehicle became ARMED during camera startup or capture"
                        )
                        self.stop.set()
                        return
                    if is_vehicle_heartbeat:
                        self.state.disarmed_heartbeat.set()
                    started = self.window.started_monotonic
                    deadline = self.window.deadline
                    if started is None or received_at < started:
                        continue
                    if deadline is not None and received_at >= deadline:
                        self.stop.set()
                        return
                    message_type = message.get_type()
                    self.state.telemetry_counts[message_type] += 1
                    write_json_line(
                        handle,
                        telemetry_record(
                            message,
                            elapsed_s=received_at - started,
                        ),
                    )
                    self.sync.after_record(handle)
        except Exception as error:  # keep the camera cleanup path deterministic
            self.state.record_error(f"telemetry worker: {error}")
            self.stop.set()


class DetectionWorker(threading.Thread):
    """Decode tags without delaying the video encoder or telemetry reader."""

    def __init__(
        self,
        *,
        frames: queue.Queue[AnalysisFrame | None],
        output: Path,
        detector: Detector,
        calibration: CameraCalibration | None,
        tag_size: float,
        resolution: tuple[int, int],
        max_reprojection_error: float,
        target_id: int | None,
        stop: threading.Event,
        state: CaptureState,
        sync: IntervalSync,
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
        self.stop = stop
        self.state = state
        self.sync = sync

    def run(self) -> None:
        try:
            with synced_stream(self.output, self.sync) as handle:
                while True:
                    item = self.frames.get()
                    try:
                        if item is None:
                            return
                        detections = self.detector.detect(item.grayscale)
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
                                "timestamp_utc": datetime.now(UTC).isoformat(),
                                "elapsed_s": round(item.elapsed_s, 6),
                                "frame": item.frame_index,
                                "metadata": json_safe(item.metadata),
                                "tags": tags,
                            },
                        )
                        self.sync.after_record(handle)
                    finally:
                        self.frames.task_done()
        except Exception as error:
            self.state.record_error(f"AprilTag worker: {error}")
            self.stop.set()


def _stop_detection_worker(
    worker: DetectionWorker,
    frames: queue.Queue[AnalysisFrame | None],
    state: CaptureState,
    *,
    timeout: float = 10.0,
) -> None:
    """Enqueue a stop marker without blocking, then join for at most ``timeout``."""

    if worker.ident is None:
        worker.stop.set()
        return

    deadline = time.monotonic() + max(0.0, timeout)
    worker.stop.set()
    sentinel_queued = False
    while worker.is_alive() and not sentinel_queued and time.monotonic() < deadline:
        try:
            frames.put_nowait(None)
            sentinel_queued = True
        except queue.Full:
            try:
                discarded = frames.get_nowait()
            except queue.Empty:
                continue
            else:
                frames.task_done()
                if discarded is not None:
                    state.dropped_analysis_frames += 1

    worker.join(timeout=max(0.0, deadline - time.monotonic()))
    if worker.is_alive():
        state.record_error(
            f"AprilTag worker did not stop within {max(0.0, timeout):g} seconds"
        )


def _cleanup_action(
    state: CaptureState,
    label: str,
    action: Callable[[], object],
) -> None:
    """Run one cleanup action without skipping any later cleanup."""

    try:
        action()
    except Exception as error:
        state.record_error(f"{label}: {error}")


def _cleanup_camera(
    camera: Any | None,
    encoder: Any | None,
    state: CaptureState,
    *,
    camera_started: bool,
    encoder_started: bool,
) -> None:
    if encoder_started and camera is not None and encoder is not None:
        _cleanup_action(
            state,
            "stop H.264 encoder",
            lambda: camera.stop_encoder(encoder),
        )
    if camera_started and camera is not None:
        _cleanup_action(state, "stop camera", lambda: camera.stop())
    if camera is not None:
        _cleanup_action(state, "close camera", lambda: camera.close())


def _join_telemetry_worker(worker: TelemetryWorker, timeout: float = 2.0) -> None:
    worker.join(timeout=timeout)
    if worker.is_alive():
        raise RuntimeError(f"telemetry worker did not stop within {timeout:g} seconds")


def _stop_capture_workers(
    telemetry_worker: TelemetryWorker | None,
    detection_worker: DetectionWorker | None,
    frames: queue.Queue[AnalysisFrame | None],
    state: CaptureState,
) -> None:
    if telemetry_worker is not None and telemetry_worker.ident is not None:
        _cleanup_action(
            state,
            "stop telemetry worker",
            lambda: _join_telemetry_worker(telemetry_worker),
        )
    if detection_worker is not None and detection_worker.ident is not None:
        _cleanup_action(
            state,
            "stop AprilTag worker",
            lambda: _stop_detection_worker(detection_worker, frames, state),
        )


def _write_preview_frame(
    cv2: Any,
    path: Path,
    frame: NDArray[np.uint8],
    label: str,
) -> None:
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"OpenCV did not write the {label} preview to {path}")


def _cleanup_mavlink_connection(connection: Any, state: CaptureState) -> None:
    try:
        logfile = getattr(connection, "logfile", None)
    except Exception as error:
        state.record_error(f"access MAVLink logfile: {error}")
        logfile = None
    if logfile is not None:
        _cleanup_action(state, "flush MAVLink logfile", lambda: logfile.flush())
        _cleanup_action(state, "close MAVLink logfile", lambda: logfile.close())
        _cleanup_action(
            state,
            "detach MAVLink logfile",
            lambda: setattr(connection, "logfile", None),
        )
    _cleanup_action(state, "close MAVLink connection", lambda: connection.close())


def _cleanup_capture(
    *,
    camera: Any | None,
    encoder: Any | None,
    camera_started: bool,
    encoder_started: bool,
    telemetry_worker: TelemetryWorker | None,
    detection_worker: DetectionWorker | None,
    frames: queue.Queue[AnalysisFrame | None],
    cv2: Any,
    paths: RecordingPaths,
    first_frame: NDArray[np.uint8] | None,
    last_frame: NDArray[np.uint8] | None,
    connection: Any,
    stop: threading.Event,
    state: CaptureState,
) -> None:
    """Attempt every resource cleanup and retain only the first new failure."""

    stop.set()
    _cleanup_camera(
        camera,
        encoder,
        state,
        camera_started=camera_started,
        encoder_started=encoder_started,
    )
    _stop_capture_workers(telemetry_worker, detection_worker, frames, state)
    if first_frame is not None:
        _cleanup_action(
            state,
            "write first preview",
            lambda: _write_preview_frame(
                cv2,
                paths.first_frame,
                first_frame,
                "first-frame",
            ),
        )
    if last_frame is not None:
        _cleanup_action(
            state,
            "write last preview",
            lambda: _write_preview_frame(
                cv2,
                paths.last_frame,
                last_frame,
                "last-frame",
            ),
        )
    _cleanup_mavlink_connection(connection, state)


def _start_capture_epoch(
    connection: Any,
    telemetry_tlog: Path,
    window: CaptureWindow,
    stop: threading.Event,
    state: CaptureState,
) -> bool:
    """Start raw telemetry logging at the first successfully encoded keyframe."""

    if stop.is_set():
        return False
    try:
        connection.setup_logfile(str(telemetry_tlog))
        if stop.is_set():
            return False
        window.begin()
    except Exception as error:
        state.record_error(f"start synchronized capture epoch: {error}")
        stop.set()
        return False
    return True


def _raise_if_startup_stopped(
    stop: threading.Event,
    state: CaptureState,
    stage: str,
) -> None:
    if not stop.is_set():
        return
    if state.armed_abort:
        raise RuntimeError(f"vehicle became ARMED during {stage}")
    raise RuntimeError(state.worker_error or f"capture stopped during {stage}")


def _wait_for_capture_epoch(
    first_frame: threading.Event,
    window: CaptureWindow,
    stop: threading.Event,
    state: CaptureState,
    timeout: float,
) -> tuple[float, datetime, float]:
    """Wait boundedly for the first encoded keyframe or an early safety abort."""

    deadline = time.monotonic() + timeout
    while not first_frame.wait(
        timeout=min(0.05, max(0.0, deadline - time.monotonic()))
    ):
        _raise_if_startup_stopped(stop, state, "H.264 encoder startup")
        if time.monotonic() >= deadline:
            raise RuntimeError("H.264 encoder did not produce a keyframe")
    _raise_if_startup_stopped(stop, state, "H.264 encoder startup")
    return window.require_started()


def _wait_for_fresh_disarmed_heartbeat(
    stop: threading.Event,
    state: CaptureState,
    timeout: float,
) -> None:
    """Require a new monitored disarmed heartbeat immediately before encoding."""

    state.disarmed_heartbeat.clear()
    deadline = time.monotonic() + timeout
    while not state.disarmed_heartbeat.wait(
        timeout=min(0.1, max(0.0, deadline - time.monotonic()))
    ):
        _raise_if_startup_stopped(stop, state, "fresh heartbeat check")
        if time.monotonic() >= deadline:
            raise RuntimeError("no fresh disarmed vehicle heartbeat before capture")
    _raise_if_startup_stopped(stop, state, "fresh heartbeat check")


def _safe_video_timestamp_summary(
    path: Path,
    state: CaptureState,
) -> dict[str, int | float]:
    try:
        return video_timestamp_summary(path)
    except Exception as error:
        state.record_error(f"summarize H.264 timestamps: {error}")
        return {
            "encoded_frames": 0,
            "encoded_span_s": 0.0,
            "encoded_duration_s": 0.0,
        }


def _camera_metadata(metadata: dict[str, object]) -> dict[str, object]:
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
    if not is_raspberry_pi():
        raise SystemExit(
            "drone-record is Pi-only. Run it through "
            "'uv run drone-deploy --record --duration SECONDS'."
        )

    import cv2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
    import numpy as np
    from picamera2 import (  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
        Picamera2,
    )
    from picamera2.encoders import (  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
        H264Encoder,
    )
    from picamera2.outputs import (  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
        FileOutput,
    )

    class FirstFrameFileOutput(FileOutput):
        """Signal after Picamera2 writes the first encoded keyframe."""

        def __init__(
            self,
            file: str,
            pts: str,
            on_first_frame: Callable[[], bool],
        ) -> None:
            super().__init__(file, pts=pts)
            self.first_frame = threading.Event()
            self.on_first_frame = on_first_frame

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
            if should_signal and not self.dead and self.on_first_frame():
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
    capture_window = CaptureWindow(args.duration)
    requested_messages: list[str] = []
    initial_vehicle_state = "unknown"
    first_frame = None
    last_frame = None
    telemetry_worker = None
    detection_worker = None
    frames: queue.Queue[AnalysisFrame | None] = queue.Queue(maxsize=8)

    try:
        heartbeat = connection.wait_heartbeat(timeout=args.timeout)
        if heartbeat is None:
            raise RuntimeError("No ArduPilot heartbeat received")
        vehicle_system = connection.target_system
        if is_armed_vehicle_heartbeat(heartbeat, system_id=vehicle_system):
            initial_vehicle_state = "armed"
            raise RuntimeError("Vehicle is ARMED; recording refused")
        initial_vehicle_state = "disarmed"
        print(
            f"Vehicle {vehicle_system} is DISARMED. No actuator commands will be sent.",
            flush=True,
        )

        telemetry_worker = TelemetryWorker(
            connection=connection,
            output=paths.telemetry_events,
            vehicle_system=int(vehicle_system),
            vehicle_component=int(connection.target_component),
            window=capture_window,
            stop=stop,
            state=state,
            sync=IntervalSync(args.sync_interval),
        )
        telemetry_worker.start()
        requested_messages = request_telemetry_messages(connection)
        _raise_if_startup_stopped(stop, state, "telemetry initialization")

        camera = Picamera2()
        _raise_if_startup_stopped(stop, state, "camera initialization")
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
        _raise_if_startup_stopped(stop, state, "camera configuration")
        camera.start()
        camera_started = True
        _raise_if_startup_stopped(stop, state, "camera startup")
        if args.warmup:
            print(f"Camera warm-up: {args.warmup:.1f} s ...", flush=True)
            if stop.wait(args.warmup):
                _raise_if_startup_stopped(stop, state, "camera warm-up")
        _raise_if_startup_stopped(stop, state, "camera warm-up")
        _wait_for_fresh_disarmed_heartbeat(stop, state, args.timeout)

        encoder = H264Encoder(bitrate=args.bitrate, repeat=True)
        video_output = FirstFrameFileOutput(
            str(paths.video),
            str(paths.video_timestamps),
            lambda: _start_capture_epoch(
                connection,
                paths.telemetry_tlog,
                capture_window,
                stop,
                state,
            ),
        )
        camera.start_encoder(encoder, video_output, name="main")
        encoder_started = True
        started_monotonic, started_utc, deadline = _wait_for_capture_epoch(
            video_output.first_frame,
            capture_window,
            stop,
            state,
            timeout=max(2.0, 10.0 / args.fps),
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
            stop=stop,
            state=state,
            sync=IntervalSync(args.sync_interval),
        )
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
                item = AnalysisFrame(
                    frame_index=frame_index,
                    elapsed_s=captured_at - started_monotonic,
                    grayscale=grayscale,
                    metadata=metadata,
                )
                try:
                    frames.put_nowait(item)
                except queue.Full:
                    state.dropped_analysis_frames += 1
            frame_index += 1

        stop.set()
        ended_monotonic = time.monotonic()
        ended_utc = datetime.now(UTC)
    except KeyboardInterrupt:
        stop.set()
        ended_monotonic = time.monotonic()
        ended_utc = datetime.now(UTC)
        state.record_error("interrupted by user")
    except Exception as error:
        stop.set()
        ended_monotonic = time.monotonic()
        ended_utc = datetime.now(UTC)
        state.record_error(str(error))
    finally:
        _cleanup_capture(
            camera=camera,
            encoder=encoder,
            camera_started=camera_started,
            encoder_started=encoder_started,
            telemetry_worker=telemetry_worker,
            detection_worker=detection_worker,
            frames=frames,
            cv2=cv2,
            paths=paths,
            first_frame=first_frame,
            last_frame=last_frame,
            connection=connection,
            stop=stop,
            state=state,
        )

    if capture_window.started.is_set():
        started_monotonic, started_utc, _deadline = capture_window.require_started()

    actual_duration = (
        max(0.0, ended_monotonic - started_monotonic) if started_monotonic else 0.0
    )
    video_summary = _safe_video_timestamp_summary(paths.video_timestamps, state)
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
        "durability": {
            # Records after the final sync are the only ones a power cut can
            # discard; the manifest itself is replaced atomically.
            "sync_interval_s": args.sync_interval,
            "periodic_sync": args.sync_interval > 0,
        },
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
            "capture_epoch": "first successfully encoded H.264 keyframe",
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
    atomic_write_text(
        paths.manifest,
        json.dumps(
            json_safe(manifest),
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
        + "\n",
    )

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
