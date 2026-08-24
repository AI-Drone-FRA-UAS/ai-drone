"""Inspect and record every available disarmed camera and MAVLink stream."""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
from collections import Counter
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any

from pymavlink import mavutil

from ai_drone.cli_parsing import parse_even_resolution
from ai_drone.durability import (
    DEFAULT_SYNC_INTERVAL_S,
    IntervalSync,
    atomic_write_text,
    synced_stream,
)
from ai_drone.mavlink.devices import resolve_mavlink_endpoint
from ai_drone.mavlink.safety import (
    heartbeat_is_armed,
    is_armed_vehicle_heartbeat,
    is_vehicle_message,
)
from ai_drone.platform import is_raspberry_pi
from ai_drone.recording import (
    RecordingPaths,
    create_recording_paths,
    json_safe,
    request_telemetry_messages,
    telemetry_record,
    video_timestamp_summary,
    write_json_line,
)
from ai_drone.vision.apriltags import (
    CameraCalibration,
    Detector,
    create_detector,
    estimate_pose,
)

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

_TAG36H11_MAX_ID = 586


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect camera video, AprilTags, and all available FC sensor telemetry "
            "for an exact requested interval. This command never arms, "
            "changes flight mode, moves a motor, or actuates a servo."
        )
    )
    parser.add_argument("--duration", type=float, default=10.0, metavar="SECONDS")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--device", help="serial path or pymavlink network endpoint")
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
    parser.add_argument("--stream", action="store_true", help="serve browser MJPEG")
    parser.add_argument("--port", type=int, default=8081)
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
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
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
    distance_samples: Counter[int] = field(default_factory=Counter)
    latest_distance_m: dict[int, float] = field(default_factory=dict)
    optical_flow_samples: int = 0
    latest_flow_quality: int | None = None
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
                    _observe_sensor_message(self.state, message)
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
        visible_ids: set[int] = set()
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
                        current_ids = {int(tag["id"]) for tag in tags}
                        for event, identifiers in (
                            ("detected", current_ids - visible_ids),
                            ("lost", visible_ids - current_ids),
                        ):
                            for identifier in sorted(identifiers):
                                print(
                                    json.dumps(
                                        {
                                            "event": f"apriltag_{event}",
                                            "id": identifier,
                                            "elapsed_s": round(item.elapsed_s, 6),
                                        },
                                        sort_keys=True,
                                    ),
                                    flush=True,
                                )
                        visible_ids = current_ids
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


def _observe_sensor_message(state: CaptureState, message: Any) -> None:
    """Keep the small live summary separate from the lossless JSONL record."""

    message_type = message.get_type()
    if message_type == "DISTANCE_SENSOR":
        orientation = int(message.orientation)
        current_cm = int(message.current_distance)
        minimum_cm = int(message.min_distance)
        maximum_cm = int(message.max_distance)
        if current_cm > 0 and minimum_cm <= current_cm <= maximum_cm:
            state.distance_samples[orientation] += 1
            state.latest_distance_m[orientation] = current_cm / 100.0
    elif message_type == "RANGEFINDER":
        distance = float(message.distance)
        if math.isfinite(distance) and distance > 0:
            state.distance_samples[25] += 1
            state.latest_distance_m[25] = distance
    elif message_type in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}:
        state.optical_flow_samples += 1
        quality = getattr(message, "quality", None)
        if quality is not None:
            state.latest_flow_quality = int(quality)


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


def _cleanup_mavlink_connection(connection: Any | None, state: CaptureState) -> None:
    if connection is None:
        return
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
    cv2: Any | None,
    paths: RecordingPaths,
    first_frame: NDArray[np.uint8] | None,
    last_frame: NDArray[np.uint8] | None,
    connection: Any | None,
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
    if cv2 is not None and first_frame is not None:
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
    if cv2 is not None and last_frame is not None:
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
    connection: Any | None,
    telemetry_tlog: Path,
    window: CaptureWindow,
    stop: threading.Event,
    state: CaptureState,
) -> bool:
    """Start raw telemetry logging at the first successfully encoded keyframe."""

    if stop.is_set():
        return False
    try:
        if connection is not None:
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


def _component_report(
    state: CaptureState,
    *,
    on_pi: bool,
    flight_controller: str,
    camera: str,
    detector: str,
    details: dict[str, str],
    duration: float,
) -> dict[str, dict[str, object]]:
    def dependent(samples: int, parent: str) -> str:
        if parent != "ok":
            return "unavailable"
        return "ok" if samples else "no_data"

    downward = state.distance_samples[25]
    forward = state.distance_samples[0]
    tag_status = dependent(state.tag_detections, camera)
    if detector != "ok":
        tag_status = "unavailable"
    report: dict[str, dict[str, object]] = {
        "pi": {"status": "ok" if on_pi else "unavailable"},
        "flight_controller": {
            "status": flight_controller,
            "messages": sum(state.telemetry_counts.values()),
        },
        "camera": {"status": camera, "frames": state.camera_frames},
        "downward_rangefinder": {
            "status": dependent(downward, flight_controller),
            "samples": downward,
            "latest_m": state.latest_distance_m.get(25),
        },
        "forward_rangefinder": {
            "status": dependent(forward, flight_controller),
            "samples": forward,
            "latest_m": state.latest_distance_m.get(0),
        },
        "optical_flow": {
            "status": dependent(state.optical_flow_samples, flight_controller),
            "samples": state.optical_flow_samples,
            "quality": state.latest_flow_quality,
        },
        "apriltags": {
            "status": tag_status,
            "detections": state.tag_detections,
            "ids": dict(sorted(state.tag_ids.items())),
        },
        "servo": {
            "status": "not_detectable",
            "detail": "BCM12 has no passive servo-presence feedback",
        },
    }
    for name, detail in details.items():
        report.setdefault(name, {"status": "unavailable"})["detail"] = detail
    if duration > 0:
        for name in ("downward_rangefinder", "forward_rangefinder", "optical_flow"):
            samples = report[name]["samples"]
            if isinstance(samples, int):
                report[name]["rate_hz"] = round(samples / duration, 3)
    return report


def _print_live_status(state: CaptureState) -> None:
    downward = state.latest_distance_m.get(25)
    forward = state.latest_distance_m.get(0)
    print(
        "status "
        f"camera_frames={state.camera_frames} "
        f"telemetry={sum(state.telemetry_counts.values())} "
        f"downward_m={downward if downward is not None else 'unavailable'} "
        f"forward_m={forward if forward is not None else 'unavailable'} "
        f"flow_quality={state.latest_flow_quality if state.latest_flow_quality is not None else 'unavailable'} "
        f"tags={state.tag_detections}",
        flush=True,
    )


def run(arguments: list[str] | None = None) -> int:  # noqa: C901
    parser = _parser()
    args = parser.parse_args(arguments)
    _validate_args(parser, args)
    paths = create_recording_paths(args.output_dir)
    state = CaptureState()
    window = CaptureWindow(args.duration)
    stop = threading.Event()
    frames: queue.Queue[AnalysisFrame | None] = queue.Queue(maxsize=8)
    details: dict[str, str] = {}
    on_pi = is_raspberry_pi()
    connection = None
    candidate = None
    endpoint: str | None = None
    flight_controller_status = "unavailable"
    initial_vehicle_state = "unavailable"
    requested_messages: list[str] = []

    try:
        endpoint = resolve_mavlink_endpoint(
            args.device,
            include_pi_uart=True,
            missing_message="No ArduPilot serial device found",
        )
        candidate = mavutil.mavlink_connection(endpoint, baud=args.baud)
        heartbeat = candidate.wait_heartbeat(timeout=args.timeout)
        if heartbeat is None:
            raise TimeoutError("no ArduPilot heartbeat received")
        if is_armed_vehicle_heartbeat(
            heartbeat, system_id=int(candidate.target_system)
        ):
            connection = candidate
            initial_vehicle_state = "armed"
            state.armed_abort = True
            state.record_error("vehicle is ARMED; inspection refused")
            stop.set()
        else:
            connection = candidate
            initial_vehicle_state = "disarmed"
            flight_controller_status = "ok"
            requested_messages = request_telemetry_messages(connection)
    except (FileNotFoundError, OSError, RuntimeError, TimeoutError) as error:
        details["flight_controller"] = str(error)
        if candidate is not None:
            with suppress(OSError):
                candidate.close()

    telemetry_worker = None
    if connection is not None and not state.armed_abort:
        telemetry_worker = TelemetryWorker(
            connection=connection,
            output=paths.telemetry_events,
            vehicle_system=int(connection.target_system),
            vehicle_component=int(connection.target_component),
            window=window,
            stop=stop,
            state=state,
            sync=IntervalSync(args.sync_interval),
        )
        telemetry_worker.start()

    camera = None
    encoder = None
    cv2 = None
    camera_started = False
    encoder_started = False
    camera_status = "unavailable"
    detector_status = "unavailable"
    detector = None
    server = None
    push_stream_frame: Callable[[bytes], None] | None = None
    first_frame = None
    last_frame = None
    detection_worker = None

    if not on_pi:
        details["camera"] = "Picamera2 inspection is available only on Raspberry Pi"
    elif not state.armed_abort:
        try:
            import cv2 as cv2_module  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
            import numpy as np
            from picamera2 import Picamera2  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
            from picamera2.encoders import (  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]
                H264Encoder,
            )
            from picamera2.outputs import FileOutput  # type: ignore[import-untyped]  # ty: ignore[unresolved-import]

            cv2 = cv2_module

            class FirstFrameFileOutput(FileOutput):
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
                    signal = (
                        not self.first_frame.is_set()
                        and self.recording
                        and keyframe
                        and not audio
                    )
                    super().outputframe(frame, keyframe, timestamp, packet, audio)
                    if (
                        signal
                        and not self.dead
                        and _start_capture_epoch(
                            connection, paths.telemetry_tlog, window, stop, state
                        )
                    ):
                        self.first_frame.set()

            try:
                calibration = (
                    CameraCalibration.load(args.calibration)
                    if args.calibration
                    else None
                )
                detector = create_detector(
                    args.backend,
                    threads=args.threads,
                    decimate=args.decimate,
                )
                detector_status = "ok"
            except (OSError, RuntimeError, ValueError) as error:
                details["apriltags"] = str(error)
                calibration = None

            camera = Picamera2()
            camera.configure(
                camera.create_video_configuration(
                    main={"format": "YUV420", "size": args.resolution},
                    lores={"format": "YUV420", "size": args.analysis_resolution},
                    raw={"size": (2028, 1520)},
                    controls={"FrameRate": args.fps},
                    buffer_count=6,
                    queue=False,
                )
            )
            camera.start()
            camera_started = True
            if args.warmup and stop.wait(args.warmup):
                _raise_if_startup_stopped(stop, state, "camera warm-up")
            if connection is not None:
                _wait_for_fresh_disarmed_heartbeat(stop, state, args.timeout)

            if args.stream:
                try:
                    from ai_drone.vision.stream import push_frame, start_server

                    server = start_server(port=args.port)
                    push_stream_frame = push_frame
                    print(f"Browser stream: http://0.0.0.0:{args.port}/", flush=True)
                except (OSError, RuntimeError, ValueError) as error:
                    details["stream"] = str(error)

            encoder = H264Encoder(bitrate=args.bitrate, repeat=True)
            video_output = FirstFrameFileOutput(
                str(paths.video), str(paths.video_timestamps)
            )
            camera.start_encoder(encoder, video_output, name="main")
            encoder_started = True
            _wait_for_capture_epoch(
                video_output.first_frame,
                window,
                stop,
                state,
                timeout=max(2.0, 10.0 / args.fps),
            )
            camera_status = "ok"
            if detector is not None:
                detection_worker = DetectionWorker(
                    frames=frames,
                    output=paths.camera_events,
                    detector=detector,
                    calibration=calibration,
                    tag_size=args.tag_size,
                    resolution=args.analysis_resolution,
                    max_reprojection_error=args.max_reprojection_error,
                    target_id=args.target_id,
                    stop=stop,
                    state=state,
                    sync=IntervalSync(args.sync_interval),
                )
                detection_worker.start()
        except (ImportError, OSError, RuntimeError, ValueError) as error:
            details["camera"] = str(error)
            _cleanup_camera(
                camera,
                encoder,
                state,
                camera_started=camera_started,
                encoder_started=encoder_started,
            )
            camera = None
            encoder = None
            camera_started = False
            encoder_started = False

    if not window.started.is_set() and not state.armed_abort:
        _start_capture_epoch(connection, paths.telemetry_tlog, window, stop, state)

    started_monotonic = time.monotonic()
    started_utc: datetime | None = None
    ended_monotonic = started_monotonic
    ended_utc: datetime | None = None
    if window.started.is_set():
        started_monotonic, started_utc, deadline = window.require_started()
        next_status = started_monotonic
        try:
            frame_index = 0
            while not stop.is_set() and time.monotonic() < deadline:
                if camera is None:
                    now = time.monotonic()
                    if now >= next_status:
                        _print_live_status(state)
                        next_status = now + 1.0
                    stop.wait(min(0.1, max(0.0, deadline - now)))
                    continue
                request = camera.capture_request()
                try:
                    yuv = request.make_array("lores")
                    metadata = _camera_metadata(request.get_metadata())
                    captured_at = time.monotonic()
                    height, width = (
                        args.analysis_resolution[1],
                        args.analysis_resolution[0],
                    )
                    grayscale = np.ascontiguousarray(yuv[:height, :width]).copy()
                finally:
                    request.release()
                if captured_at >= deadline:
                    break
                state.camera_frames += 1
                first_frame = grayscale.copy() if first_frame is None else first_frame
                last_frame = grayscale.copy()
                if frame_index % args.detect_every == 0:
                    try:
                        frames.put_nowait(
                            AnalysisFrame(
                                frame_index=frame_index,
                                elapsed_s=captured_at - started_monotonic,
                                grayscale=grayscale,
                                metadata=metadata,
                            )
                        )
                    except queue.Full:
                        state.dropped_analysis_frames += 1
                    if push_stream_frame is not None and cv2 is not None:
                        ok, jpeg = cv2.imencode(
                            ".jpg", grayscale, [cv2.IMWRITE_JPEG_QUALITY, 80]
                        )
                        if ok:
                            push_stream_frame(jpeg.tobytes())
                frame_index += 1
                if captured_at >= next_status:
                    _print_live_status(state)
                    next_status = captured_at + 1.0
        except KeyboardInterrupt:
            state.record_error("interrupted by user")
        except Exception as error:
            state.record_error(str(error))
        finally:
            stop.set()
            ended_monotonic = time.monotonic()
            ended_utc = datetime.now(UTC)

    if server is not None:
        _cleanup_action(state, "stop browser stream", server.shutdown)
        _cleanup_action(state, "close browser stream", server.server_close)
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

    actual_duration = max(0.0, ended_monotonic - started_monotonic)
    components = _component_report(
        state,
        on_pi=on_pi,
        flight_controller=flight_controller_status,
        camera=camera_status,
        detector=detector_status,
        details=details,
        duration=actual_duration,
    )
    if args.stream:
        components["stream"] = {
            "status": "ok" if server is not None else "unavailable",
            **({"detail": details["stream"]} if "stream" in details else {}),
        }
    files = {
        "video": paths.video,
        "video_timestamps": paths.video_timestamps,
        "camera_events": paths.camera_events,
        "telemetry_tlog": paths.telemetry_tlog,
        "telemetry_events": paths.telemetry_events,
        "first_frame": paths.first_frame,
        "last_frame": paths.last_frame,
    }
    manifest = {
        "schema": 1,
        "requested_duration_s": args.duration,
        "actual_duration_s": round(actual_duration, 6),
        "started_utc": started_utc.isoformat() if started_utc else None,
        "ended_utc": ended_utc.isoformat() if ended_utc else None,
        "completed": not state.armed_abort and state.worker_error is None,
        "armed_abort": state.armed_abort,
        "error": state.worker_error,
        "components": components,
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
            "backend": getattr(detector, "backend_name", None),
            **_safe_video_timestamp_summary(paths.video_timestamps, state),
        },
        "telemetry": {
            "endpoint": endpoint,
            "baud": args.baud,
            "requested_messages": requested_messages,
            "message_counts": dict(sorted(state.telemetry_counts.items())),
        },
        "files": {name: path.name for name, path in files.items() if path.exists()},
    }
    try:
        atomic_write_text(
            paths.manifest,
            json.dumps(json_safe(manifest), indent=2, sort_keys=True) + "\n",
        )
    except OSError as error:
        print(f"FAILED: could not write manifest: {error}", flush=True)
        return 1

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
        return 1
    return 0


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
