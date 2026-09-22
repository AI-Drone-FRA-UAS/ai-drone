"""Passively record available camera and MAVLink streams, disarmed by default."""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import signal
import threading
import time
from collections.abc import Callable, Iterator, Sequence
from concurrent.futures import CancelledError
from contextlib import ExitStack, contextmanager, suppress
from dataclasses import dataclass, field
from datetime import UTC, datetime
from importlib import import_module
from pathlib import Path
from typing import TYPE_CHECKING, Any

from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.capture.manifest import ArtifactFiles, ComponentRecord, Manifest
from ai_drone.capture.operations import RecordingOperation, parse_operation, parser_spec
from ai_drone.capture.reporting import (
    _component_report,
    _print_live_status,
)
from ai_drone.capture.state import (
    AnalysisFrame,
    CaptureState,
    CaptureWindow,
)
from ai_drone.capture.workers import (
    DetectionWorker,
    TelemetryWorker,
)
from ai_drone.cli_parsing import parse_even_resolution
from ai_drone.durability import (
    DEFAULT_SYNC_INTERVAL_S,
    IntervalSync,
    atomic_write_text,
)
from ai_drone.mavlink.connection import open_ardupilot_connection
from ai_drone.mavlink.devices import resolve_mavlink_endpoint
from ai_drone.mavlink.metrics import TransportMetrics, attach_transport_metrics
from ai_drone.mavlink.parameters import request_parameter
from ai_drone.mavlink.safety import heartbeat_is_armed
from ai_drone.mavlink.shared import received_monotonic
from ai_drone.platform import is_raspberry_pi
from ai_drone.recording import (
    RecordingPaths,
    create_recording_paths,
    request_telemetry_messages,
    video_timestamp_summary,
)
from ai_drone.settings import load_settings
from ai_drone.storage import StorageMonitor, StoragePolicy
from ai_drone.system import handled_signals
from ai_drone.vision.apriltags import (
    CameraCalibration,
    configure_opencv_threads,
    create_detector,
)

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

_TAG36H11_MAX_ID = 586
_INSPECT_OPERATION = "inspect"
_TAG_SERVO_OPERATION = "tag-servo"
_TAG_MOUNT_OPERATION = "tag-mount"


def _parser(*, operation: str = _INSPECT_OPERATION) -> argparse.ArgumentParser:
    spec = parser_spec(operation)
    tag_servo = spec.actuation_enabled
    parser = argparse.ArgumentParser(
        description=(
            (
                "Record camera video, native AprilTag detections, and bounded FC "
                "telemetry while allowing explicitly confirmed BCM12 payload-servo "
                "pulses. This command can attach while the vehicle is armed but "
                "never arms, changes mode, or sends flight-control setpoints."
            )
            if tag_servo
            else (
                "Inspect camera video, AprilTags, and all available FC sensor "
                "telemetry for an exact requested interval. This command never "
                "arms, changes flight mode, moves a motor, or actuates a servo."
            )
        )
    )
    if operation == _TAG_MOUNT_OPERATION:
        parser.description = (
            "Record camera video, AprilTags, and all available FC telemetry; open "
            "the payload mount once when the selected tag36h11 ID is confirmed. Works disarmed "
            "or already flying under manual RC control. Keeps recording through "
            "arming/disarming until Ctrl-C or --duration. No flight-control commands."
        )
    parser.add_argument(
        "--duration",
        type=float,
        default=spec.default_duration,
        metavar="SECONDS",
        help=(
            "optional maximum runtime; omit to run until stopped"
            if tag_servo
            else "capture duration (default: %(default)s)"
        ),
    )
    parser.add_argument("--output-dir", type=Path)
    try:
        defaults = load_settings().recording
    except (OSError, ValueError) as error:
        parser.error(str(error))
    for option, help_text in (
        ("reserve-mib", "free space reserved for logs after video stops"),
        ("warning-mib", "warn below this amount of free space"),
        ("stop-mib", "final free-space margin for closing logs and the manifest"),
        ("check-interval", "seconds between free-space checks"),
    ):
        parser.add_argument(
            f"--storage-{option}",
            type=float,
            default=getattr(defaults, "storage_" + option.replace("-", "_")),
            help=help_text,
        )
    parser.add_argument(
        "--no-video",
        action="store_true",
        help="skip saving video; retain camera analysis, logs, and optional preview",
    )
    if not tag_servo:
        parser.add_argument(
            "--allow-flight",
            action="store_true",
            help="allow passive capture while already armed or arming later",
        )
    parser.add_argument("--device", help="serial path or pymavlink network endpoint")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--resolution", type=parse_even_resolution, default=(1280, 960))
    parser.add_argument(
        "--analysis-resolution", type=parse_even_resolution, default=(640, 480)
    )
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--frame-timeout",
        type=float,
        default=2.0,
        metavar="SECONDS",
        help="stop if the camera does not deliver a frame within this time (default: 2)",
    )
    parser.add_argument("--bitrate", type=int, default=8_000_000)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument(
        "--backend",
        choices=spec.backends,
        default=spec.default_backend,
    )
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--decimate", type=float, default=1.0)
    parser.add_argument("--detect-every", type=int, default=1, metavar="FRAMES")
    if not tag_servo:
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
    if operation == _TAG_MOUNT_OPERATION:
        from ai_drone.cli.tag_servo_record import mount_recording_defaults

        parser.set_defaults(**mount_recording_defaults())
        parser.add_argument(
            "--tag-id",
            dest="tag_ids",
            type=int,
            nargs=1,
            metavar="ID",
            help="tag36h11 ID that opens the mount once (default: 3)",
        )
    elif tag_servo:
        from ai_drone.cli.tag_servo_record import add_arguments

        add_arguments(parser)
    return parser


def _validate_positive_args(
    parser: argparse.ArgumentParser, args: argparse.Namespace
) -> None:
    positive = {
        "--baud": args.baud,
        "--timeout": args.timeout,
        "--fps": args.fps,
        "--frame-timeout": args.frame_timeout,
        "--bitrate": args.bitrate,
        "--threads": args.threads,
        "--detect-every": args.detect_every,
        "--tag-size": args.tag_size,
        "--max-reprojection-error": args.max_reprojection_error,
    }
    if args.duration is not None:
        positive["--duration"] = args.duration
    for name, value in positive.items():
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")


def _validate_args(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
    *,
    operation: str = _INSPECT_OPERATION,
) -> None:
    tag_servo = operation in {_TAG_SERVO_OPERATION, _TAG_MOUNT_OPERATION}
    _validate_positive_args(parser, args)
    try:
        _storage_policy(args)
    except ValueError as error:
        parser.error(str(error))
    if not math.isfinite(args.sync_interval) or args.sync_interval < 0:
        parser.error("--sync-interval must be finite and not negative")
    if not math.isfinite(args.warmup) or args.warmup < 0:
        parser.error("--warmup must be finite and not negative")
    if not math.isfinite(args.decimate) or args.decimate < 1.0:
        parser.error("--decimate must be finite and at least 1.0")
    target_id = getattr(args, "target_id", None)
    if target_id is not None and not 0 <= target_id <= _TAG36H11_MAX_ID:
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
    if tag_servo:
        from ai_drone.cli.tag_servo_record import validate_args

        validate_args(parser, args)


def _storage_policy(args: argparse.Namespace) -> StoragePolicy:
    return StoragePolicy(
        reserve_mib=args.storage_reserve_mib,
        warning_mib=args.storage_warning_mib,
        stop_mib=args.storage_stop_mib,
        check_interval=args.storage_check_interval,
        video_bytes_per_second=args.bitrate / 8,
        max_pause_s=args.timeout + args.frame_timeout + max(2, 10 / args.fps),
    )


def _check_storage(
    storage: StorageMonitor,
    paths: RecordingPaths,
    state: CaptureState,
    stop: threading.Event,
    *,
    video: bool,
    camera: Any = None,
    encoder: Any = None,
    encoder_started: bool = False,
    force: bool = False,
) -> bool:
    try:
        decision = storage.check(time.monotonic(), video=video, force=force)
        if decision is None:
            return encoder_started
        if decision.stop_capture:
            state.set_stop_reason("storage_full")
            state.record_error(
                "free space reached the log-stop threshold; capture finalized early"
            )
            stop.set()
            storage.event("capture_stopped_storage", free_bytes=decision.free_bytes)
            raise RuntimeError(state.worker_error)
        if decision.stop_video and storage.video_stopped_reason is None:
            if encoder_started:
                camera.stop_encoder(encoder)
                for path in (paths.video, paths.video_timestamps):
                    _sync_existing_file(path)
            storage.video_stopped(was_recording=encoder_started)
            return False
        return encoder_started
    except Exception as error:
        state.set_stop_reason("storage_error")
        state.record_error(f"storage monitor: {error}")
        stop.set()
        raise


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
                    with state.lock:
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
) -> bool:
    """Run one cleanup action without skipping any later cleanup."""

    try:
        action()
    except BaseException as error:
        state.record_error(f"{label}: {error}")
        return False
    return True


def _cleanup_camera(
    camera: Any | None,
    encoder: Any | None,
    state: CaptureState,
    *,
    camera_started: bool,
    encoder_started: bool,
) -> bool:
    if encoder_started and camera is not None and encoder is not None:
        _cleanup_action(
            state,
            "stop H.264 encoder",
            lambda: camera.stop_encoder(encoder),
        )
    if camera_started and camera is not None:
        _cleanup_action(state, "stop camera", lambda: camera.stop())
    if camera is not None:
        return _cleanup_action(state, "close camera", lambda: camera.close())
    return True


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
    except BaseException as error:
        state.record_error(f"access MAVLink logfile: {error}")
        logfile = None
    if logfile is not None:
        _cleanup_action(state, "flush MAVLink logfile", lambda: logfile.flush())
        _cleanup_action(
            state,
            "sync MAVLink logfile",
            lambda: os.fsync(logfile.fileno()),
        )
        _cleanup_action(state, "close MAVLink logfile", lambda: logfile.close())
        _cleanup_action(
            state,
            "detach MAVLink logfile",
            lambda: setattr(connection, "logfile", None),
        )
    _cleanup_action(state, "close MAVLink connection", lambda: connection.close())


def _sync_existing_file(path: Path) -> None:
    """Persist a completed camera artifact before publishing its manifest."""

    if not path.exists():
        return
    with path.open("rb") as handle:
        os.fsync(handle.fileno())


def _finalize_camera_artifacts(
    cv2: Any,
    paths: RecordingPaths,
    first_frame: NDArray[np.uint8] | None,
    last_frame: NDArray[np.uint8] | None,
    state: CaptureState,
) -> None:
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
    for label, path in (
        ("H.264 video", paths.video),
        ("H.264 timestamps", paths.video_timestamps),
    ):
        _cleanup_action(
            state, f"sync {label}", lambda path=path: _sync_existing_file(path)
        )


def _start_capture_epoch(
    connection: Any | None,
    telemetry_tlog: Path,
    window: CaptureWindow,
    stop: threading.Event,
    state: CaptureState,
) -> bool:
    """Start raw telemetry logging when the camera capture becomes available."""

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


def _retry_pi_uart_heartbeat(
    connection: Any, *, endpoint: str, baud: int, timeout: float
) -> Any:
    """Retry one timed-out GPIO UART startup without sending MAVLink traffic."""

    def counters() -> str:
        parser = getattr(connection, "mav", None)
        return " ".join(
            f"{label}={getattr(parser, attribute, 'unknown')}"
            for label, attribute in (
                ("bytes", "total_bytes_received"),
                ("packets", "total_packets_received"),
                ("parse_errors", "total_receive_errors"),
            )
        )

    print(
        f"Pi UART heartbeat timeout: endpoint={endpoint} baud={baud} {counters()}; "
        "reopening the local serial connection once",
        flush=True,
    )
    started = time.monotonic()
    # mavserial.reset is a local descriptor reopen at the requested baud. It
    # does not reset the FC or transmit packets. Keep its public transport API
    # and the existing robust MAVLink 2 decoder instead of editing parser state.
    if not connection.reset():
        print(
            f"Pi UART retry: reopen failed after {time.monotonic() - started:.3f}s",
            flush=True,
        )
        raise OSError("Pi UART reopen failed after the initial heartbeat timeout")
    heartbeat = connection.wait_heartbeat(timeout=timeout)
    outcome = "recovered" if heartbeat is not None else "no heartbeat"
    print(
        f"Pi UART retry: {outcome} after {time.monotonic() - started:.3f}s "
        f"{counters()}",
        flush=True,
    )
    if heartbeat is None:
        raise TimeoutError("no ArduPilot heartbeat received after one Pi UART retry")
    return heartbeat


def _wait_for_capture_epoch(
    first_frame: threading.Event,
    window: CaptureWindow,
    stop: threading.Event,
    state: CaptureState,
    timeout: float,
) -> tuple[float, datetime, float | None]:
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


def _video_output(paths: RecordingPaths, begin: Callable[[], bool]) -> Any:
    from picamera2.outputs import FileOutput  # ty: ignore[unresolved-import]

    class FirstFrameFileOutput(FileOutput):
        def __init__(self) -> None:
            super().__init__(str(paths.video), pts=str(paths.video_timestamps))
            self.first_frame = threading.Event()

        def outputframe(
            self,
            frame: bytes,
            keyframe: bool = True,
            timestamp: int | None = None,
            packet: Any = None,
            audio: bool = False,
        ) -> None:
            first = (
                not self.first_frame.is_set()
                and self.recording
                and keyframe
                and not audio
            )
            super().outputframe(frame, keyframe, timestamp, packet, audio)
            if first and not self.dead and begin():
                self.first_frame.set()

    return FirstFrameFileOutput()


@dataclass
class _PendingCameraRequest:
    camera: Any
    state: CaptureState
    abandoned: threading.Event = field(default_factory=threading.Event)
    released: bool = False
    release_lock: Any = field(default_factory=threading.Lock)


def _release_abandoned_request(pending: _PendingCameraRequest, job: Any) -> None:
    # A completed job can leave Picamera2's queue just before cancellation,
    # with its completion signal arriving later. Cover that race as well as
    # results already available at cancellation, without releasing twice.
    with pending.release_lock:
        if not pending.abandoned.is_set() or pending.released:
            return
        try:
            request = pending.camera.wait(job, timeout=0)
        except (TimeoutError, CancelledError):
            return
        except Exception as error:
            pending.state.record_error(f"camera request failed: {error}")
            return
        pending.released = True
    _cleanup_action(pending.state, "release cancelled camera request", request.release)


def _wait_camera_request(
    pending: _PendingCameraRequest,
    job: Any,
    *,
    stop: threading.Event,
    deadline: float | None,
    frame_deadline: float,
    frame_timeout: float,
) -> tuple[bool, Any]:
    while not stop.is_set():
        now = time.monotonic()
        if deadline is not None and deadline <= frame_deadline and now >= deadline:
            pending.state.set_stop_reason("duration_elapsed")
            stop.set()
            return False, None
        if now >= frame_deadline:
            message = f"camera did not deliver a frame within {frame_timeout:g} seconds"
            pending.state.record_error(message)
            pending.state.set_stop_reason("camera_stalled")
            stop.set()
            raise TimeoutError(message)
        remaining = frame_deadline - now
        if deadline is not None:
            remaining = min(remaining, deadline - now)
        try:
            request = pending.camera.wait(job, timeout=min(0.1, max(0.0, remaining)))
        except TimeoutError:
            continue
        if stop.is_set():
            return False, None
        if deadline is not None and time.monotonic() >= deadline:
            pending.state.set_stop_reason("duration_elapsed")
            stop.set()
            return False, None
        return True, request
    return False, None


def _capture_request_bounded(
    camera: Any,
    *,
    stop: threading.Event,
    state: CaptureState,
    deadline: float | None,
    frame_timeout: float,
) -> Any | None:
    """Bound camera waiting and release results that race with cancellation."""
    if stop.is_set():
        return None
    if deadline is not None and time.monotonic() >= deadline:
        state.set_stop_reason("duration_elapsed")
        stop.set()
        return None
    frame_deadline = time.monotonic() + frame_timeout
    pending = _PendingCameraRequest(camera, state)
    job = camera.capture_request(
        wait=False,
        signal_function=lambda job: _release_abandoned_request(pending, job),
    )
    delivered = False
    try:
        delivered, request = _wait_camera_request(
            pending,
            job,
            stop=stop,
            deadline=deadline,
            frame_deadline=frame_deadline,
            frame_timeout=frame_timeout,
        )
        return request
    finally:
        if not delivered:
            pending.abandoned.set()
            _cleanup_action(
                state, "cancel pending camera requests", camera.cancel_all_and_flush
            )
            _release_abandoned_request(pending, job)


def _wait_for_fresh_heartbeat(
    stop: threading.Event,
    state: CaptureState,
    timeout: float,
    *,
    require_disarmed: bool = True,
) -> None:
    heartbeat = (
        state.disarmed_heartbeat if require_disarmed else state.vehicle_heartbeat
    )
    missing = (
        "no fresh disarmed vehicle heartbeat before capture"
        if require_disarmed
        else "no fresh selected flight-controller heartbeat"
    )

    requested_at = time.monotonic()
    deadline = requested_at + timeout
    heartbeat.clear()
    while True:
        heartbeat.wait(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
        _raise_if_startup_stopped(stop, state, "fresh heartbeat check")
        observed_at = state.last_vehicle_heartbeat_monotonic
        if (
            heartbeat.is_set()
            and observed_at is not None
            and observed_at >= requested_at
        ):
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(missing)
        heartbeat.clear()


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


def _queue_analysis_frame(
    frames: queue.Queue[AnalysisFrame | None],
    item: AnalysisFrame,
    state: CaptureState,
    *,
    latest_wins: bool,
) -> None:
    """Queue analysis without blocking; active mode replaces stale queued work."""

    try:
        frames.put_nowait(item)
        return
    except queue.Full:
        with state.lock:
            state.dropped_analysis_frames += 1
    if not latest_wins:
        return
    try:
        discarded = frames.get_nowait()
    except queue.Empty:
        return
    else:
        frames.task_done()
        if discarded is None:
            return
    try:
        frames.put_nowait(item)
    except queue.Full:
        with state.lock:
            state.dropped_analysis_frames += 1


@dataclass
class _FlightCapture:
    connection: Any = None
    endpoint: str | None = None
    status: str = "unavailable"
    initial_vehicle_state: str = "unavailable"
    requested_messages: list[str] = field(default_factory=list)
    arming_skipchk: float | None = None
    worker: TelemetryWorker | None = None
    transport: TransportMetrics | None = None


@dataclass
class _CameraCapture:
    device: Any = None
    encoder: Any = None
    cv2: Any = None
    numpy: Any = None
    started: bool = False
    encoder_started: bool = False
    status: str = "unavailable"
    detector_status: str = "unavailable"
    detector: Any = None
    calibration: CameraCalibration | None = None
    worker: DetectionWorker | None = None
    server: Any = None
    push_stream_frame: Callable[[bytes], None] | None = None
    first_frame: NDArray[np.uint8] | None = None
    last_frame: NDArray[np.uint8] | None = None


@dataclass
class _Recording:
    args: argparse.Namespace
    operation: RecordingOperation
    paths: RecordingPaths
    window: CaptureWindow
    stop: threading.Event
    frames: queue.Queue[AnalysisFrame | None]
    on_pi: bool
    state: CaptureState = field(default_factory=CaptureState)
    details: dict[str, str] = field(default_factory=dict)
    flight: _FlightCapture = field(default_factory=_FlightCapture)
    camera: _CameraCapture = field(default_factory=_CameraCapture)
    servo_session: Any = None
    servo_config: Any = None
    storage: StorageMonitor | None = None
    started_monotonic: float = field(default_factory=time.monotonic)
    started_utc: datetime | None = None
    ended_monotonic: float = 0.0
    ended_utc: datetime | None = None
    signals: ExitStack = field(default_factory=ExitStack)

    @property
    def tag_servo(self) -> bool:
        return self.operation.actuation_enabled

    @property
    def allow_flight(self) -> bool:
        return self.operation.allow_flight


def _recording(args: argparse.Namespace, operation: str) -> _Recording:
    tag_servo = operation in {_TAG_SERVO_OPERATION, _TAG_MOUNT_OPERATION}
    paths = create_recording_paths(args.output_dir)
    window = CaptureWindow(args.duration)
    if tag_servo:
        from ai_drone.cli.tag_servo_record import ActuationStop

        stop = ActuationStop()
    else:
        stop = threading.Event()
    return _Recording(
        args=args,
        operation=parse_operation(
            operation, allow_flight=getattr(args, "allow_flight", False)
        ),
        paths=paths,
        window=window,
        stop=stop,
        frames=queue.Queue(maxsize=1 if tag_servo else 8),
        on_pi=is_raspberry_pi(),
    )


def _install_capture_signals(recording: _Recording) -> None:
    if not recording.tag_servo:
        return

    def stop_signal(signum: int, _frame: Any) -> None:
        recording.stop.set()
        recording.state.set_stop_reason(
            f"operator_signal_{signal.Signals(signum).name}"
        )

    recording.signals.enter_context(
        handled_signals(
            {
                signum: stop_signal
                for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
            }
        )
    )


def _initial_heartbeat(recording: _Recording) -> Any:
    flight, args = recording.flight, recording.args
    flight.endpoint = resolve_mavlink_endpoint(
        args.device,
        include_pi_uart=True,
        missing_message="No ArduPilot serial device found",
    )
    flight.connection = open_ardupilot_connection(flight.endpoint, baud=args.baud)
    flight.transport = attach_transport_metrics(flight.connection)
    heartbeat = flight.connection.wait_heartbeat(timeout=args.timeout)
    _raise_if_startup_stopped(
        recording.stop, recording.state, "flight-controller startup"
    )
    if (
        heartbeat is None
        and recording.on_pi
        and flight.endpoint in ("/dev/serial0", "/dev/ttyAMA0")
    ):
        heartbeat = _retry_pi_uart_heartbeat(
            flight.connection,
            endpoint=flight.endpoint,
            baud=args.baud,
            timeout=args.timeout,
        )
        _raise_if_startup_stopped(
            recording.stop, recording.state, "flight-controller startup"
        )
    if heartbeat is None:
        raise TimeoutError("no ArduPilot heartbeat received")
    return heartbeat


def _select_servo_vehicle(connection: Any, heartbeat: Any) -> None:
    source_system = int(heartbeat.get_srcSystem())
    source_component = int(heartbeat.get_srcComponent())
    if (source_system, source_component) != (1, 1):
        raise RuntimeError(
            "armed tag-servo recording requires the project FC at MAVLink "
            f"target 1/1, received {source_system}/{source_component}"
        )
    if int(heartbeat.autopilot) != mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        raise RuntimeError("selected heartbeat is not from ArduPilot")
    if int(heartbeat.type) != mavlink.MAV_TYPE_QUADROTOR:
        raise RuntimeError("selected ArduPilot vehicle is not a quadrotor")
    connection.target_system = source_system
    connection.target_component = source_component


def _configure_flight_capture(recording: _Recording, heartbeat: Any) -> None:
    flight, state = recording.flight, recording.state
    if recording.tag_servo:
        _select_servo_vehicle(flight.connection, heartbeat)
    initially_armed = heartbeat_is_armed(heartbeat)
    flight.initial_vehicle_state = "armed" if initially_armed else "disarmed"
    if not state.observe_vehicle_state(
        armed=initially_armed,
        observed_at=received_monotonic(heartbeat),
    ):
        raise RuntimeError("initial vehicle heartbeat is stale")
    if recording.tag_servo:
        flight.arming_skipchk = request_parameter(
            flight.connection,
            "ARMING_SKIPCHK",
            timeout=recording.args.timeout,
            require_disarmed=False,
        )
        if flight.arming_skipchk != 0.0:
            raise RuntimeError(
                f"ARMING_SKIPCHK={flight.arming_skipchk:g}; armed tag-servo recording "
                "requires exact ARMING_SKIPCHK=0"
            )
    elif initially_armed and not recording.allow_flight:
        state.armed_abort = True
        state.record_error("vehicle is ARMED; inspection refused")
        recording.stop.set()
        return
    flight.status = "ok"
    flight.requested_messages = request_telemetry_messages(flight.connection)


def _start_flight_capture(recording: _Recording) -> None:
    flight = recording.flight
    try:
        _configure_flight_capture(recording, _initial_heartbeat(recording))
    except (
        FileNotFoundError,
        OSError,
        RuntimeError,
        TimeoutError,
        ValueError,
    ) as error:
        recording.details["flight_controller"] = str(error)
        if flight.connection is not None:
            _cleanup_mavlink_connection(flight.connection, recording.state)
            if flight.transport is not None:
                _cleanup_action(
                    recording.state,
                    "finish failed transport metrics",
                    flight.transport.close,
                )
            flight.connection = None
            flight.status = "unavailable"
    if flight.connection is None or recording.state.armed_abort:
        return
    flight.worker = TelemetryWorker(
        connection=flight.connection,
        output=recording.paths.telemetry_events,
        vehicle_system=int(flight.connection.target_system),
        vehicle_component=int(flight.connection.target_component),
        window=recording.window,
        stop=recording.stop,
        state=recording.state,
        sync=IntervalSync(recording.args.sync_interval),
        allow_armed_at_any_time=recording.allow_flight,
        stop_after_disarm=recording.operation.stop_after_disarm,
    )
    flight.worker.start()


def _prepare_detector(recording: _Recording) -> None:
    camera, args = recording.camera, recording.args
    try:
        camera.calibration = (
            CameraCalibration.load(args.calibration) if args.calibration else None
        )
        camera.detector = create_detector(
            args.backend, threads=args.threads, decimate=args.decimate
        )
        camera.detector_status = "ok"
    except (OSError, RuntimeError, ValueError) as error:
        recording.details["apriltags"] = str(error)
        camera.calibration = None


def _open_camera(recording: _Recording) -> None:
    import cv2
    import numpy
    from picamera2 import Picamera2  # ty: ignore[unresolved-import]

    camera, args = recording.camera, recording.args
    camera.cv2, camera.numpy = cv2, numpy
    configure_opencv_threads(args.threads)
    _prepare_detector(recording)
    camera.device = Picamera2()
    camera.device.configure(
        camera.device.create_video_configuration(
            main={"format": "YUV420", "size": args.resolution},
            lores={"format": "YUV420", "size": args.analysis_resolution},
            raw={"size": (2028, 1520)},
            controls={"FrameRate": args.fps},
            buffer_count=6,
            queue=False,
        )
    )
    camera.device.start()
    camera.started = True
    if args.warmup and recording.stop.wait(args.warmup):
        _raise_if_startup_stopped(recording.stop, recording.state, "camera warm-up")
    if recording.flight.connection is not None and (
        recording.tag_servo or not recording.allow_flight
    ):
        _wait_for_fresh_heartbeat(
            recording.stop,
            recording.state,
            args.timeout,
            require_disarmed=not recording.tag_servo,
        )


def _start_browser_stream(recording: _Recording) -> None:
    if not recording.args.stream:
        return
    try:
        from ai_drone.vision.stream import push_frame, start_server

        recording.camera.server = start_server(port=recording.args.port)
        recording.camera.push_stream_frame = push_frame
        print(f"Browser stream: http://0.0.0.0:{recording.args.port}/", flush=True)
    except (OSError, RuntimeError, ValueError) as error:
        recording.details["stream"] = str(error)


def _begin_recording_epoch(recording: _Recording) -> bool:
    return _start_capture_epoch(
        recording.flight.connection,
        recording.paths.telemetry_tlog,
        recording.window,
        recording.stop,
        recording.state,
    )


def _start_camera_epoch(recording: _Recording) -> None:
    camera, args = recording.camera, recording.args
    storage = recording.storage
    assert storage is not None
    _check_storage(
        storage,
        recording.paths,
        recording.state,
        recording.stop,
        video=not args.no_video and storage.video_stopped_reason is None,
        force=True,
    )
    if args.no_video or storage.video_stopped_reason is not None:
        request = _capture_request_bounded(
            camera.device,
            stop=recording.stop,
            state=recording.state,
            deadline=None,
            frame_timeout=args.frame_timeout,
        )
        if request is None:
            _raise_if_startup_stopped(recording.stop, recording.state, "camera startup")
        else:
            request.release()
            _begin_recording_epoch(recording)
        _raise_if_startup_stopped(recording.stop, recording.state, "camera startup")
        return
    from picamera2.encoders import H264Encoder  # ty: ignore[unresolved-import]

    camera.encoder = H264Encoder(bitrate=args.bitrate, repeat=True)
    output = _video_output(recording.paths, lambda: _begin_recording_epoch(recording))
    camera.device.start_encoder(camera.encoder, output, name="main")
    camera.encoder_started = True
    _wait_for_capture_epoch(
        output.first_frame,
        recording.window,
        recording.stop,
        recording.state,
        timeout=max(2.0, 10.0 / args.fps),
    )


def _start_tag_servo(recording: _Recording) -> None:
    if not recording.tag_servo:
        return
    if recording.camera.detector is None:
        raise RuntimeError(
            "armed tag-servo recording requires the native AprilTag detector"
        )
    from ai_drone.cli.tag_servo_record import TagServoConfig, TagServoSession

    recording.servo_config = TagServoConfig.from_args(recording.args)
    recording.servo_session = TagServoSession(
        config=recording.servo_config,
        event_path=recording.paths.actuation_events,
        state=recording.state,
        capture_stop=recording.stop,
        ready=recording.window.ready,
    )
    recording.servo_session.install_signal_handlers()


def _start_detection(recording: _Recording) -> None:
    camera, args = recording.camera, recording.args
    if camera.detector is None:
        return
    camera.worker = DetectionWorker(
        frames=recording.frames,
        output=recording.paths.camera_events,
        detector=camera.detector,
        calibration=camera.calibration,
        tag_size=args.tag_size,
        resolution=args.analysis_resolution,
        max_reprojection_error=args.max_reprojection_error,
        target_id=getattr(args, "target_id", None),
        stop=recording.stop,
        state=recording.state,
        sync=IntervalSync(args.sync_interval),
        observer=recording.servo_session,
    )
    camera.worker.start()


def _camera_startup_failed(recording: _Recording, error: Exception) -> None:
    camera = recording.camera
    recording.details["camera"] = str(error)
    if recording.tag_servo:
        recording.state.record_error(f"armed tag-servo startup: {error}")
        recording.state.set_stop_reason("startup_failed")
        recording.stop.set()
    if recording.servo_session is not None:
        closed = _cleanup_action(
            recording.state,
            "close failed payload servo session",
            recording.servo_session.close,
        )
        if closed:
            recording.servo_session = None
    camera_closed = _cleanup_camera(
        camera.device,
        camera.encoder,
        recording.state,
        camera_started=camera.started,
        encoder_started=camera.encoder_started,
    )
    recording.camera = _CameraCapture(
        device=None if camera_closed else camera.device,
        encoder=None if camera_closed else camera.encoder,
        started=camera.started and not camera_closed,
        encoder_started=camera.encoder_started and not camera_closed,
        cv2=camera.cv2,
        numpy=camera.numpy,
        detector=camera.detector,
        detector_status=camera.detector_status,
        calibration=camera.calibration,
        worker=camera.worker,
        server=camera.server,
        push_stream_frame=camera.push_stream_frame,
        first_frame=camera.first_frame,
        last_frame=camera.last_frame,
    )


def _start_camera_capture(recording: _Recording) -> None:
    if not recording.on_pi:
        recording.details["camera"] = (
            "Picamera2 inspection is available only on Raspberry Pi"
        )
        return
    if recording.state.armed_abort or recording.stop.is_set():
        return
    try:
        _open_camera(recording)
        _start_browser_stream(recording)
        _start_camera_epoch(recording)
        recording.camera.status = "ok"
        _start_tag_servo(recording)
        _start_detection(recording)
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        _camera_startup_failed(recording, error)


def _ready_to_capture(recording: _Recording) -> None:
    state, window, stop = recording.state, recording.window, recording.stop
    camera = recording.camera
    if recording.tag_servo and (
        recording.flight.connection is None
        or camera.status != "ok"
        or camera.detector_status != "ok"
        or recording.servo_session is None
    ):
        state.record_error(
            "armed tag-servo recording requires selected FC telemetry, camera, "
            "native AprilTag detection, and exclusive BCM12 servo access"
        )
        state.set_stop_reason("startup_failed")
        stop.set()
    if not window.started.is_set() and not state.armed_abort:
        _begin_recording_epoch(recording)
    if not window.started.is_set() or stop.is_set():
        return
    if recording.tag_servo:
        try:
            _wait_for_fresh_heartbeat(
                stop, state, recording.args.timeout, require_disarmed=False
            )
        except (RuntimeError, TimeoutError) as error:
            state.record_error(f"final armed tag-servo readiness check: {error}")
            state.set_stop_reason("startup_failed")
            stop.set()
            return
        window.ready.set()
        print(
            "READY: camera and telemetry synchronized; no flight-control commands. "
            "BCM12 stays detached until a tag qualifies.",
            flush=True,
        )
    else:
        window.ready.set()
        print(
            "READY: recording available sources; no flight-control commands.",
            flush=True,
        )


def _start_recording(recording: _Recording) -> None:
    recording.storage = StorageMonitor(
        recording.paths.storage_events,
        _storage_policy(recording.args),
        IntervalSync(recording.args.sync_interval),
    )
    _check_storage(
        recording.storage,
        recording.paths,
        recording.state,
        recording.stop,
        video=not recording.args.no_video,
        force=True,
    )
    _install_capture_signals(recording)
    if recording.on_pi and not recording.stop.is_set():
        # Native OpenCV initialization can hold the GIL long enough to overflow
        # a live telemetry subscription. Report optional import failures later
        # through normal camera startup, without opening any camera here.
        with suppress(ImportError, OSError, RuntimeError, ValueError):
            import_module("cv2")
    _start_flight_capture(recording)
    _start_camera_capture(recording)
    _ready_to_capture(recording)


def _check_capture_health(recording: _Recording, now: float) -> bool:
    camera = recording.camera
    assert recording.storage is not None
    camera.encoder_started = _check_storage(
        recording.storage,
        recording.paths,
        recording.state,
        recording.stop,
        video=camera.encoder_started,
        camera=camera.device,
        encoder=camera.encoder,
        encoder_started=camera.encoder_started,
    )
    if recording.servo_session is not None:
        error = recording.servo_session.health_error(now)
        if error is not None:
            recording.state.record_error(error)
            recording.state.set_stop_reason("runtime_watchdog")
            recording.stop.set()
            return False
    return True


def _read_camera_frame(
    camera: _CameraCapture, request: Any, resolution: tuple[int, int]
) -> tuple[Any, dict[str, object], float]:
    try:
        yuv = request.make_array("lores")
        metadata = _camera_metadata(request.get_metadata())
        captured_at = time.monotonic()
        width, height = resolution
        grayscale = camera.numpy.ascontiguousarray(yuv[:height, :width]).copy()
        return grayscale, metadata, captured_at
    finally:
        request.release()


def _preview_frame(camera: _CameraCapture, grayscale: Any) -> None:
    if camera.push_stream_frame is None or camera.cv2 is None:
        return
    ok, jpeg = camera.cv2.imencode(
        ".jpg", grayscale, [camera.cv2.IMWRITE_JPEG_QUALITY, 80]
    )
    if ok:
        camera.push_stream_frame(jpeg.tobytes())


def _capture_frame(
    recording: _Recording, frame_index: int, deadline: float | None
) -> bool:
    camera, args, state = recording.camera, recording.args, recording.state
    request = _capture_request_bounded(
        camera.device,
        stop=recording.stop,
        state=state,
        deadline=deadline,
        frame_timeout=args.frame_timeout,
    )
    if request is None:
        return False
    grayscale, metadata, captured_at = _read_camera_frame(
        camera, request, args.analysis_resolution
    )
    if deadline is not None and captured_at >= deadline:
        state.set_stop_reason("duration_elapsed")
        return False
    with state.lock:
        state.camera_frames += 1
    if camera.first_frame is None:
        camera.first_frame = grayscale.copy()
    camera.last_frame = grayscale.copy()
    if frame_index % args.detect_every == 0:
        _queue_analysis_frame(
            recording.frames,
            AnalysisFrame(
                frame_index=frame_index,
                elapsed_s=captured_at - recording.started_monotonic,
                grayscale=grayscale,
                metadata=metadata,
                captured_monotonic=captured_at,
            ),
            state,
            latest_wins=recording.tag_servo,
        )
        _preview_frame(camera, grayscale)
    return True


def _capture_loop(recording: _Recording, deadline: float | None) -> None:
    next_status = recording.started_monotonic
    frame_index = 0
    while not recording.stop.is_set():
        now = time.monotonic()
        if deadline is not None and now >= deadline:
            recording.state.set_stop_reason("duration_elapsed")
            break
        if not _check_capture_health(recording, now):
            break
        if now >= next_status:
            _print_live_status(
                recording.state,
                tag_servo=recording.tag_servo,
                stop_after=(
                    recording.servo_config.stop_after
                    if recording.servo_config is not None
                    else None
                ),
            )
            next_status = now + 1.0
        if recording.camera.status != "ok":
            wait_time = 0.1 if deadline is None else min(0.1, max(0.0, deadline - now))
            recording.stop.wait(wait_time)
            continue
        if not _capture_frame(recording, frame_index, deadline):
            break
        frame_index += 1


def _record_capture(recording: _Recording) -> None:
    if not recording.window.started.is_set():
        return
    recording.started_monotonic, recording.started_utc, deadline = (
        recording.window.require_started()
    )
    try:
        _capture_loop(recording, deadline)
    except KeyboardInterrupt:
        if recording.tag_servo:
            recording.state.set_stop_reason("operator_interrupt")
            if recording.servo_session is not None:
                recording.servo_session.stop_accepting()
        else:
            recording.state.record_error("interrupted by user")
    except Exception as error:
        recording.state.record_error(str(error))
        recording.state.set_stop_reason("runtime_error")


def _close_stream(recording: _Recording) -> None:
    if recording.camera.server is not None:
        _cleanup_action(
            recording.state, "stop browser stream", recording.camera.server.shutdown
        )
        _cleanup_action(
            recording.state,
            "close browser stream",
            recording.camera.server.server_close,
        )


@contextmanager
def _recording_lifecycle(recording: _Recording) -> Iterator[None]:
    """Stop actuation and telemetry before slower camera/artifact cleanup."""
    with ExitStack() as resources:
        # Explicit dependency order, rather than reverse acquisition order.
        for label, close in (
            ("restore capture signal handlers", recording.signals.close),
            (
                "close storage event log",
                lambda: recording.storage.close() if recording.storage else None,
            ),
            (
                "finalize camera artifacts",
                lambda: _finalize_camera_artifacts(
                    recording.camera.cv2,
                    recording.paths,
                    recording.camera.first_frame,
                    recording.camera.last_frame,
                    recording.state,
                ),
            ),
            (
                "stop capture workers",
                lambda: _stop_capture_workers(
                    None,
                    recording.camera.worker,
                    recording.frames,
                    recording.state,
                ),
            ),
            (
                "close camera capture",
                lambda: _cleanup_camera(
                    recording.camera.device,
                    recording.camera.encoder,
                    recording.state,
                    camera_started=recording.camera.started,
                    encoder_started=recording.camera.encoder_started,
                ),
            ),
            ("close browser stream", lambda: _close_stream(recording)),
            (
                "close flight capture",
                lambda: _cleanup_mavlink_connection(
                    recording.flight.connection, recording.state
                ),
            ),
            (
                "finish transport metrics",
                lambda: (
                    recording.flight.transport.close()
                    if recording.flight.transport
                    else None
                ),
            ),
            (
                "stop telemetry worker",
                lambda: (
                    _join_telemetry_worker(recording.flight.worker)
                    if recording.flight.worker is not None
                    and recording.flight.worker.ident is not None
                    else None
                ),
            ),
            (
                "close payload servo session",
                lambda: (
                    recording.servo_session.close() if recording.servo_session else None
                ),
            ),
        ):
            resources.callback(_cleanup_action, recording.state, label, close)
        try:
            yield
        finally:
            recording.stop.set()
            recording.ended_monotonic = time.monotonic()
            recording.ended_utc = datetime.now(UTC)


def _write_minimal_manifest(recording: _Recording, error: BaseException) -> None:
    if recording.paths.manifest.exists():
        return
    state = recording.state.snapshot()
    manifest = {
        "schema": 1,
        "operation": recording.operation.name,
        "error": state.worker_error or str(error) or type(error).__name__,
        "finalization_error": str(error) or type(error).__name__,
        **({"errors": list(state.errors)} if state.errors else {}),
        "stop_reason": state.stop_reason or "capture_failed",
        "completed": False,
        "armed_abort": state.armed_abort,
        "started_utc": recording.started_utc.isoformat()
        if recording.started_utc
        else None,
        "ended_utc": recording.ended_utc.isoformat() if recording.ended_utc else None,
        "components": {
            "servo": {
                "completed_commanded_pulses": state.servo_pulses_completed,
                "feedback_available": False,
            }
        },
    }
    with suppress(OSError):
        atomic_write_text(
            recording.paths.manifest,
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        )


def run(
    arguments: Sequence[str] | None = None,
    *,
    operation: str = "inspect",
) -> int:
    parser = _parser(operation=operation)
    args = parser.parse_args(arguments)
    _validate_args(parser, args, operation=operation)
    recording = _recording(args, operation)
    try:
        with _recording_lifecycle(recording):
            try:
                _start_recording(recording)
                _record_capture(recording)
            except KeyboardInterrupt:
                recording.state.set_stop_reason("operator_interrupt")
                recording.state.record_error(
                    "interrupted by user during startup or capture"
                )
            except Exception as error:
                recording.state.record_error(str(error))
                recording.state.set_stop_reason("capture_failed")
        return _finish_recording(recording)
    except (Exception, KeyboardInterrupt) as error:
        _write_minimal_manifest(recording, error)
        print(f"FAILED: recording terminated unexpectedly: {error}", flush=True)
        return 1


def _transport_report(recording: _Recording) -> dict[str, object]:
    meter = recording.flight.transport
    if meter is None:
        return {
            "schema": 1,
            "supported": False,
            "scope": "unavailable",
            "rx_bytes": None,
            "tx_bytes": None,
            "reason": "No directly owned transport meter; shared-owner counters require runtime status.",
        }
    report = meter.snapshot()
    duration = report["elapsed_s"]
    report["includes_startup_requests"] = True
    for direction in ("rx", "tx"):
        measured = report[f"{direction}_bytes"]
        report[f"{direction}_bytes_per_s"] = (
            measured / duration if measured is not None and duration > 0 else None
        )
    report["nominal_serial_capacity_bytes_per_direction_s"] = (
        recording.args.baud / 10
        if (recording.flight.endpoint or "").startswith("/dev/")
        else None
    )
    return report


def _finish_recording(recording: _Recording) -> int:
    timestamp_summary = _safe_video_timestamp_summary(
        recording.paths.video_timestamps, recording.state
    )
    state = recording.state.snapshot()
    actual_duration = max(0.0, recording.ended_monotonic - recording.started_monotonic)
    if state.stop_reason == "camera_stalled":
        recording.camera.status = "error"
        recording.details["camera"] = (
            state.worker_error or "camera frame acquisition stalled"
        )
    components = _component_report(
        state,
        on_pi=recording.on_pi,
        flight_controller=recording.flight.status,
        camera=recording.camera.status,
        detector=recording.camera.detector_status,
        details=recording.details,
        duration=actual_duration,
        observed_at=recording.ended_monotonic,
    )
    if recording.args.stream:
        components["stream"] = {
            "status": "ok" if recording.camera.server is not None else "unavailable",
            **(
                {"detail": recording.details["stream"]}
                if "stream" in recording.details
                else {}
            ),
        }
    if recording.tag_servo:
        components["servo"] = {
            "status": (
                "commanded"
                if state.servo_pulses_completed
                else ("ready" if recording.servo_session is not None else "unavailable")
            ),
            "gpio": 12,
            "feedback_available": False,
            "completed_commanded_pulses": state.servo_pulses_completed,
        }
    files = ArtifactFiles(
        video=recording.paths.video,
        video_timestamps=recording.paths.video_timestamps,
        camera_events=recording.paths.camera_events,
        telemetry_tlog=recording.paths.telemetry_tlog,
        telemetry_events=recording.paths.telemetry_events,
        actuation_events=recording.paths.actuation_events,
        storage_events=recording.paths.storage_events,
        first_frame=recording.paths.first_frame,
        last_frame=recording.paths.last_frame,
    )
    manifest = Manifest(
        operation=recording.operation.name,
        requested_duration_s=recording.args.duration,
        actual_duration_s=round(actual_duration, 6),
        started_utc=recording.started_utc.isoformat()
        if recording.started_utc
        else None,
        ended_utc=recording.ended_utc.isoformat() if recording.ended_utc else None,
        completed=not state.armed_abort and state.worker_error is None,
        armed_abort=state.armed_abort,
        error=state.worker_error,
        errors=state.errors,
        stop_reason=state.stop_reason,
        components={
            name: ComponentRecord.from_dict(item) for name, item in components.items()
        },
        storage=recording.storage.manifest() if recording.storage is not None else None,
        safety={
            "initial_vehicle_state": recording.flight.initial_vehicle_state,
            "allow_flight": recording.allow_flight,
            "saw_armed": state.saw_armed,
            "saw_disarmed_after_arm": state.saw_disarmed_after_arm,
            "last_vehicle_state": state.last_vehicle_state or "unavailable",
            "arming_skipchk": recording.flight.arming_skipchk,
            "mavlink_commands_never_sent": [
                "arm",
                "disarm",
                "mode change",
                "motor/throttle",
                "RC override",
                "flight-controller servo",
                "mission start",
            ],
            "gpio_servo_actuation_enabled": recording.tag_servo,
        },
        camera={
            "video_enabled": not recording.args.no_video,
            "frame_timeout_s": recording.args.frame_timeout,
            "recording_resolution": list(recording.args.resolution),
            "analysis_resolution": list(recording.args.analysis_resolution),
            "backend": getattr(recording.camera.detector, "backend_name", None),
            **timestamp_summary,
        },
        telemetry={
            "delivery": state.delivery.to_dict(
                actual_duration, observed_at=recording.ended_monotonic
            ),
            "transport": _transport_report(recording),
            "endpoint": recording.flight.endpoint,
            "baud": recording.args.baud,
            "requested_messages": recording.flight.requested_messages,
            "message_counts": dict(sorted(state.telemetry_counts.items())),
            "vehicle_message_counts": dict(
                sorted(state.vehicle_telemetry_counts.items())
            ),
            "outbound": (
                [
                    "PARAM_REQUEST_READ ARMING_SKIPCHK",
                    "MAV_CMD_SET_MESSAGE_INTERVAL for requested_messages",
                ]
                if recording.tag_servo
                else ["MAV_CMD_SET_MESSAGE_INTERVAL for requested_messages"]
            ),
        },
        tag_servo=(
            recording.servo_session.manifest()
            if recording.servo_session is not None
            else None
        ),
        files=files,
    )
    try:
        atomic_write_text(
            recording.paths.manifest,
            manifest.to_json(),
        )
    except OSError as error:
        print(f"FAILED: could not write manifest: {error}", flush=True)
        return 1

    print(
        f"Finished in {actual_duration:.3f} s: camera={state.camera_frames} frames, "
        f"telemetry={sum(state.telemetry_counts.values())} messages, "
        f"tags={state.tag_detections}, "
        f"servo_pulses={state.servo_pulses_completed}, "
        f"stop_reason={state.stop_reason or 'unspecified'}",
        flush=True,
    )
    print(f"Manifest: {recording.paths.manifest}", flush=True)
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
