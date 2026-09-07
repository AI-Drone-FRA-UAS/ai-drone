"""Inspect and record every available disarmed camera and MAVLink stream."""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import signal
import threading
import time
from collections.abc import Callable
from concurrent.futures import CancelledError
from datetime import UTC, datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.capture.reporting import (
    _component_report,
    _print_live_status,
)
from ai_drone.capture.reporting import (
    _downward_range_summary as _downward_range_summary,
)
from ai_drone.capture.reporting import (
    _observe_sensor_message as _observe_sensor_message,
)
from ai_drone.capture.state import (
    AnalysisFrame,
    CaptureState,
    CaptureWindow,
)
from ai_drone.capture.state import (
    DetectionObserver as DetectionObserver,
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
from ai_drone.mavlink.devices import resolve_mavlink_endpoint
from ai_drone.mavlink.parameters import request_parameter
from ai_drone.mavlink.safety import (
    heartbeat_is_armed,
    is_armed_vehicle_heartbeat,
)
from ai_drone.platform import is_raspberry_pi
from ai_drone.recording import (
    RecordingPaths,
    create_recording_paths,
    json_safe,
    request_telemetry_messages,
    video_timestamp_summary,
)
from ai_drone.vision.apriltags import (
    CameraCalibration,
    create_detector,
)

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

_TAG36H11_MAX_ID = 586
MANUAL_FLIGHT_RECORDING_CONFIRMATION = "PASSIVE_MANUAL_FLIGHT_RECORDING"
_INSPECT_OPERATION = "inspect"
_TAG_SERVO_OPERATION = "tag-servo"


def _parser(*, operation: str = _INSPECT_OPERATION) -> argparse.ArgumentParser:
    if operation not in {_INSPECT_OPERATION, _TAG_SERVO_OPERATION}:
        raise ValueError(f"unknown recording operation {operation!r}")
    tag_servo = operation == _TAG_SERVO_OPERATION
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
    parser.add_argument(
        "--duration",
        type=float,
        default=None if tag_servo else 10.0,
        metavar="SECONDS",
        help=(
            "optional maximum runtime; omit to run until stopped"
            if tag_servo
            else "capture duration (default: 10)"
        ),
    )
    parser.add_argument("--output-dir", type=Path)
    if not tag_servo:
        parser.add_argument(
            "--confirm-manual-flight-recording",
            metavar="ACKNOWLEDGEMENT",
            help=(
                "permit the pilot to arm only after the synchronized recorder "
                "prints READY; must be exactly "
                f"{MANUAL_FLIGHT_RECORDING_CONFIRMATION}"
            ),
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
        choices=("native",) if tag_servo else ("auto", "native", "opencv"),
        default="native" if tag_servo else "auto",
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
    if tag_servo:
        from ai_drone.cli.tag_servo_record import add_arguments

        add_arguments(parser)
    return parser


def _validate_args(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
    *,
    operation: str = _INSPECT_OPERATION,
) -> None:
    tag_servo = operation == _TAG_SERVO_OPERATION
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
    if not math.isfinite(args.sync_interval) or args.sync_interval < 0:
        parser.error("--sync-interval must be finite and not negative")
    if not math.isfinite(args.warmup) or args.warmup < 0:
        parser.error("--warmup must be finite and not negative")
    if not math.isfinite(args.decimate) or args.decimate < 1.0:
        parser.error("--decimate must be finite and at least 1.0")
    if getattr(args, "confirm_manual_flight_recording", None) not in (
        None,
        MANUAL_FLIGHT_RECORDING_CONFIRMATION,
    ):
        parser.error(
            "--confirm-manual-flight-recording must be exactly "
            f"{MANUAL_FLIGHT_RECORDING_CONFIRMATION}"
        )
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
    except BaseException as error:
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
    for label, path in (
        ("H.264 video", paths.video),
        ("H.264 timestamps", paths.video_timestamps),
    ):
        _cleanup_action(
            state, f"sync {label}", lambda path=path: _sync_existing_file(path)
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


def _capture_request_bounded(
    camera: Any,
    *,
    stop: threading.Event,
    state: CaptureState,
    deadline: float | None,
    frame_timeout: float,
) -> Any | None:
    """Wait for a frame without hiding stop or duration behind a camera job.

    Cancellation uses Picamera2's queue-aware API. Camera driver cancellation,
    encoder shutdown and filesystem syncing retain their own backend latency.
    """

    if stop.is_set():
        return None
    if deadline is not None and time.monotonic() >= deadline:
        state.set_stop_reason("duration_elapsed")
        stop.set()
        return None
    frame_deadline = time.monotonic() + frame_timeout
    abandoned = threading.Event()
    released = False
    release_lock = threading.Lock()

    def release_abandoned_result(job: Any) -> None:
        nonlocal released
        # A completed job can leave Picamera2's queue just before cancellation,
        # with its completion signal arriving later. Cover that race as well as
        # results already available at cancellation, without releasing twice.
        with release_lock:
            if not abandoned.is_set() or released:
                return
            try:
                request = camera.wait(job, timeout=0)
            except (TimeoutError, CancelledError):
                return
            except Exception as error:
                state.record_error(f"camera request failed: {error}")
                return
            released = True
        _cleanup_action(state, "release cancelled camera request", request.release)

    job = camera.capture_request(wait=False, signal_function=release_abandoned_result)
    delivered = False
    try:
        while not stop.is_set():
            now = time.monotonic()
            if deadline is not None and deadline <= frame_deadline and now >= deadline:
                state.set_stop_reason("duration_elapsed")
                stop.set()
                return None
            if now >= frame_deadline:
                message = (
                    f"camera did not deliver a frame within {frame_timeout:g} seconds"
                )
                state.record_error(message)
                state.set_stop_reason("camera_stalled")
                stop.set()
                raise TimeoutError(message)
            remaining = frame_deadline - now
            if deadline is not None:
                remaining = min(remaining, deadline - now)
            try:
                request = camera.wait(job, timeout=min(0.1, max(0.0, remaining)))
            except TimeoutError:
                continue
            if stop.is_set():
                return None
            if deadline is not None and time.monotonic() >= deadline:
                state.set_stop_reason("duration_elapsed")
                stop.set()
                return None
            delivered = True
            return request
        return None
    finally:
        if not delivered:
            abandoned.set()
            _cleanup_action(
                state, "cancel pending camera requests", camera.cancel_all_and_flush
            )
            release_abandoned_result(job)


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


def _wait_for_fresh_vehicle_heartbeat(
    stop: threading.Event,
    state: CaptureState,
    timeout: float,
) -> None:
    """Require a new selected-FC heartbeat without requiring an arm state."""

    state.vehicle_heartbeat.clear()
    deadline = time.monotonic() + timeout
    while not state.vehicle_heartbeat.wait(
        timeout=min(0.1, max(0.0, deadline - time.monotonic()))
    ):
        _raise_if_startup_stopped(stop, state, "fresh vehicle heartbeat check")
        if time.monotonic() >= deadline:
            raise RuntimeError("no fresh selected flight-controller heartbeat")
    _raise_if_startup_stopped(stop, state, "fresh vehicle heartbeat check")


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
        state.dropped_analysis_frames += 1


def run(  # noqa: C901
    arguments: list[str] | None = None,
    *,
    operation: str = _INSPECT_OPERATION,
) -> int:
    parser = _parser(operation=operation)
    args = parser.parse_args(arguments)
    _validate_args(parser, args, operation=operation)
    tag_servo = operation == _TAG_SERVO_OPERATION
    manual_flight_recording = (
        getattr(args, "confirm_manual_flight_recording", None)
        == MANUAL_FLIGHT_RECORDING_CONFIRMATION
    )
    paths = create_recording_paths(args.output_dir)
    state = CaptureState()
    window = CaptureWindow(args.duration)
    if tag_servo:
        from ai_drone.cli.tag_servo_record import ActuationStop

        stop = ActuationStop()
    else:
        stop = threading.Event()
    frames: queue.Queue[AnalysisFrame | None] = queue.Queue(
        maxsize=1 if tag_servo else 8
    )
    details: dict[str, str] = {}
    on_pi = is_raspberry_pi()
    connection = None
    candidate = None
    endpoint: str | None = None
    flight_controller_status = "unavailable"
    initial_vehicle_state = "unavailable"
    requested_messages: list[str] = []
    arming_skipchk: float | None = None
    tag_servo_session: Any | None = None
    tag_servo_config: Any | None = None

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
    telemetry_worker = None

    started_monotonic = time.monotonic()
    started_utc: datetime | None = None
    ended_monotonic = started_monotonic
    ended_utc: datetime | None = None
    previous_signals: dict[int, Any] = {}

    def stop_signal(signum: int, _frame: Any) -> None:
        stop.set()
        state.set_stop_reason(f"operator_signal_{signal.Signals(signum).name}")

    try:
        if tag_servo:
            for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                previous_signals[signum] = signal.getsignal(signum)
                signal.signal(signum, stop_signal)
        try:
            endpoint = resolve_mavlink_endpoint(
                args.device,
                include_pi_uart=True,
                missing_message="No ArduPilot serial device found",
            )
            candidate = mavutil.mavlink_connection(endpoint, baud=args.baud)
            heartbeat = candidate.wait_heartbeat(timeout=args.timeout)
            _raise_if_startup_stopped(stop, state, "flight-controller startup")
            if heartbeat is None:
                raise TimeoutError("no ArduPilot heartbeat received")
            if tag_servo:
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
                candidate.target_system = source_system
                candidate.target_component = source_component
                initially_armed = heartbeat_is_armed(heartbeat)
                initial_vehicle_state = "armed" if initially_armed else "disarmed"
                state.observe_vehicle_state(armed=initially_armed)
                arming_skipchk = request_parameter(
                    candidate,
                    "ARMING_SKIPCHK",
                    timeout=args.timeout,
                    require_disarmed=False,
                )
                if arming_skipchk != 0.0:
                    raise RuntimeError(
                        f"ARMING_SKIPCHK={arming_skipchk:g}; armed tag-servo recording "
                        "requires exact ARMING_SKIPCHK=0"
                    )
                connection = candidate
                flight_controller_status = "ok"
                requested_messages = request_telemetry_messages(connection)
            elif is_armed_vehicle_heartbeat(
                heartbeat, system_id=int(candidate.target_system)
            ):
                connection = candidate
                initial_vehicle_state = "armed"
                state.observe_vehicle_state(armed=True)
                state.armed_abort = True
                state.record_error("vehicle is ARMED; inspection refused")
                stop.set()
            else:
                connection = candidate
                initial_vehicle_state = "disarmed"
                state.observe_vehicle_state(armed=False)
                flight_controller_status = "ok"
                requested_messages = request_telemetry_messages(connection)
        except (
            FileNotFoundError,
            OSError,
            RuntimeError,
            TimeoutError,
            ValueError,
        ) as error:
            details["flight_controller"] = str(error)
            if candidate is not None:
                _cleanup_mavlink_connection(candidate, state)
                candidate = None
                connection = None
                flight_controller_status = "unavailable"

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
                allow_armed_after_ready=manual_flight_recording,
                allow_armed_at_any_time=tag_servo,
                stop_after_disarm=tag_servo,
            )
            telemetry_worker.start()

        if not on_pi:
            details["camera"] = "Picamera2 inspection is available only on Raspberry Pi"
        elif not state.armed_abort and not stop.is_set():
            try:
                import cv2 as cv2_module  # ty: ignore[unresolved-import]
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
                    if tag_servo:
                        _wait_for_fresh_vehicle_heartbeat(stop, state, args.timeout)
                    else:
                        _wait_for_fresh_disarmed_heartbeat(stop, state, args.timeout)

                if args.stream:
                    try:
                        from ai_drone.vision.stream import push_frame, start_server

                        server = start_server(port=args.port)
                        push_stream_frame = push_frame
                        print(
                            f"Browser stream: http://0.0.0.0:{args.port}/", flush=True
                        )
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
                if tag_servo:
                    if detector is None:
                        raise RuntimeError(
                            "armed tag-servo recording requires the native AprilTag "
                            "detector"
                        )
                    from ai_drone.cli.tag_servo_record import (
                        TagServoConfig,
                        TagServoSession,
                    )

                    tag_servo_config = TagServoConfig.from_args(args)
                    tag_servo_session = TagServoSession(
                        config=tag_servo_config,
                        event_path=paths.actuation_events,
                        state=state,
                        capture_stop=stop,
                        ready=window.ready,
                    )
                    tag_servo_session.install_signal_handlers()
                if detector is not None:
                    detection_worker = DetectionWorker(
                        frames=frames,
                        output=paths.camera_events,
                        detector=detector,
                        calibration=calibration,
                        tag_size=args.tag_size,
                        resolution=args.analysis_resolution,
                        max_reprojection_error=args.max_reprojection_error,
                        target_id=getattr(args, "target_id", None),
                        stop=stop,
                        state=state,
                        sync=IntervalSync(args.sync_interval),
                        observer=tag_servo_session,
                    )
                    detection_worker.start()
            except (ImportError, OSError, RuntimeError, ValueError) as error:
                details["camera"] = str(error)
                if tag_servo:
                    state.record_error(f"armed tag-servo startup: {error}")
                    state.set_stop_reason("startup_failed")
                    stop.set()
                if tag_servo_session is not None:
                    tag_servo_session.close()
                    tag_servo_session = None
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

        if (
            manual_flight_recording
            and not state.armed_abort
            and (connection is None or camera_status != "ok")
        ):
            state.record_error(
                "manual-flight recording requires monitored flight-controller telemetry "
                "and camera video"
            )
            stop.set()

        if tag_servo and (
            connection is None
            or camera_status != "ok"
            or detector_status != "ok"
            or tag_servo_session is None
        ):
            state.record_error(
                "armed tag-servo recording requires selected FC telemetry, camera, "
                "native AprilTag detection, and exclusive BCM12 servo access"
            )
            state.set_stop_reason("startup_failed")
            stop.set()

        if not window.started.is_set() and not state.armed_abort:
            _start_capture_epoch(connection, paths.telemetry_tlog, window, stop, state)

        if manual_flight_recording and window.started.is_set() and not stop.is_set():
            try:
                _wait_for_fresh_disarmed_heartbeat(stop, state, args.timeout)
            except (RuntimeError, TimeoutError) as error:
                state.record_error(f"final manual-flight readiness check: {error}")
                stop.set()
            else:
                window.ready.set()
                print(
                    "READY: passive manual-flight recording is synchronized; the pilot "
                    "may arm now. This recorder sends no flight-control commands.",
                    flush=True,
                )

        if tag_servo and window.started.is_set() and not stop.is_set():
            try:
                _wait_for_fresh_vehicle_heartbeat(stop, state, args.timeout)
            except (RuntimeError, TimeoutError) as error:
                state.record_error(f"final armed tag-servo readiness check: {error}")
                state.set_stop_reason("startup_failed")
                stop.set()
            else:
                window.ready.set()
                print(
                    "READY: synchronized armed-flight recording and native AprilTag "
                    "detection are active; BCM12 is detached until a tag qualifies. "
                    "No arm, mode, motor, throttle, RC, mission, or FC-servo command "
                    "will be sent.",
                    flush=True,
                )

        if window.started.is_set():
            started_monotonic, started_utc, deadline = window.require_started()
            next_status = started_monotonic
            try:
                frame_index = 0
                while not stop.is_set():
                    now = time.monotonic()
                    if deadline is not None and now >= deadline:
                        state.set_stop_reason("duration_elapsed")
                        break
                    if tag_servo_session is not None:
                        health_error = tag_servo_session.health_error(now)
                        if health_error is not None:
                            state.record_error(health_error)
                            state.set_stop_reason("runtime_watchdog")
                            stop.set()
                            break
                    if camera is None:
                        if now >= next_status:
                            _print_live_status(
                                state,
                                tag_servo=tag_servo,
                                stop_after=(
                                    tag_servo_config.stop_after
                                    if tag_servo_config is not None
                                    else None
                                ),
                            )
                            next_status = now + 1.0
                        wait_time = (
                            0.1
                            if deadline is None
                            else min(0.1, max(0.0, deadline - now))
                        )
                        stop.wait(wait_time)
                        continue
                    request = _capture_request_bounded(
                        camera,
                        stop=stop,
                        state=state,
                        deadline=deadline,
                        frame_timeout=args.frame_timeout,
                    )
                    if request is None:
                        break
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
                    if deadline is not None and captured_at >= deadline:
                        state.set_stop_reason("duration_elapsed")
                        break
                    state.camera_frames += 1
                    first_frame = (
                        grayscale.copy() if first_frame is None else first_frame
                    )
                    last_frame = grayscale.copy()
                    if frame_index % args.detect_every == 0:
                        _queue_analysis_frame(
                            frames,
                            AnalysisFrame(
                                frame_index=frame_index,
                                elapsed_s=captured_at - started_monotonic,
                                grayscale=grayscale,
                                metadata=metadata,
                                captured_monotonic=captured_at,
                            ),
                            state,
                            latest_wins=tag_servo,
                        )
                        if push_stream_frame is not None and cv2 is not None:
                            ok, jpeg = cv2.imencode(
                                ".jpg", grayscale, [cv2.IMWRITE_JPEG_QUALITY, 80]
                            )
                            if ok:
                                push_stream_frame(jpeg.tobytes())
                    frame_index += 1
                    if captured_at >= next_status:
                        _print_live_status(
                            state,
                            tag_servo=tag_servo,
                            stop_after=(
                                tag_servo_config.stop_after
                                if tag_servo_config is not None
                                else None
                            ),
                        )
                        next_status = captured_at + 1.0
            except KeyboardInterrupt:
                if tag_servo:
                    state.set_stop_reason("operator_interrupt")
                    if tag_servo_session is not None:
                        tag_servo_session.stop_accepting()
                else:
                    state.record_error("interrupted by user")
            except Exception as error:
                state.record_error(str(error))
                state.set_stop_reason("runtime_error")
            finally:
                stop.set()
                ended_monotonic = time.monotonic()
                ended_utc = datetime.now(UTC)

    except KeyboardInterrupt:
        state.set_stop_reason("operator_interrupt")
        state.record_error("interrupted by user during startup or capture")
    except Exception as error:
        state.record_error(str(error))
        state.set_stop_reason("capture_failed")
    finally:
        stop.set()
        ended_monotonic = time.monotonic()
        ended_utc = datetime.now(UTC)
        if tag_servo_session is not None:
            _cleanup_action(
                state, "close payload servo session", tag_servo_session.close
            )
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
            connection=connection if connection is not None else candidate,
            stop=stop,
            state=state,
        )

        for signum, previous in previous_signals.items():
            _cleanup_action(
                state,
                "restore capture signal handler",
                lambda signum=signum, previous=previous: signal.signal(
                    signum, previous
                ),
            )

    actual_duration = max(0.0, ended_monotonic - started_monotonic)
    if state.stop_reason == "camera_stalled":
        camera_status = "error"
        details["camera"] = state.worker_error or "camera frame acquisition stalled"
    components = _component_report(
        state,
        on_pi=on_pi,
        flight_controller=flight_controller_status,
        camera=camera_status,
        detector=detector_status,
        details=details,
        duration=actual_duration,
        observed_at=ended_monotonic,
    )
    if args.stream:
        components["stream"] = {
            "status": "ok" if server is not None else "unavailable",
            **({"detail": details["stream"]} if "stream" in details else {}),
        }
    if tag_servo:
        components["servo"] = {
            "status": (
                "commanded"
                if state.servo_pulses_completed
                else ("ready" if tag_servo_session is not None else "unavailable")
            ),
            "gpio": 12,
            "feedback_available": False,
            "completed_commanded_pulses": state.servo_pulses_completed,
        }
    files = {
        "video": paths.video,
        "video_timestamps": paths.video_timestamps,
        "camera_events": paths.camera_events,
        "telemetry_tlog": paths.telemetry_tlog,
        "telemetry_events": paths.telemetry_events,
        "actuation_events": paths.actuation_events,
        "first_frame": paths.first_frame,
        "last_frame": paths.last_frame,
    }
    timestamp_summary = _safe_video_timestamp_summary(paths.video_timestamps, state)
    manifest = {
        "schema": 1,
        "operation": operation,
        "requested_duration_s": args.duration,
        "actual_duration_s": round(actual_duration, 6),
        "started_utc": started_utc.isoformat() if started_utc else None,
        "ended_utc": ended_utc.isoformat() if ended_utc else None,
        "completed": not state.armed_abort and state.worker_error is None,
        "armed_abort": state.armed_abort,
        "error": state.worker_error,
        "stop_reason": state.stop_reason,
        "components": components,
        "safety": {
            "initial_vehicle_state": initial_vehicle_state,
            "manual_flight_recording": manual_flight_recording,
            "saw_armed": state.saw_armed,
            "saw_disarmed_after_arm": state.saw_disarmed_after_arm,
            "last_vehicle_state": state.last_vehicle_state or "unavailable",
            "arming_skipchk": arming_skipchk,
            "mavlink_commands_never_sent": [
                "arm",
                "disarm",
                "mode change",
                "motor/throttle",
                "RC override",
                "flight-controller servo",
                "mission start",
            ],
            "commands_never_sent": (
                [
                    "arm",
                    "disarm",
                    "mode change",
                    "motor/throttle",
                    "RC override",
                    "mission start",
                ]
                if tag_servo
                else [
                    "arm",
                    "disarm",
                    "mode change",
                    "motor/throttle",
                    "RC override",
                    "servo",
                    "mission start",
                ]
            ),
            "gpio_servo_actuation_enabled": tag_servo,
        },
        "camera": {
            "frame_timeout_s": args.frame_timeout,
            "recording_resolution": list(args.resolution),
            "analysis_resolution": list(args.analysis_resolution),
            "backend": getattr(detector, "backend_name", None),
            **timestamp_summary,
        },
        "telemetry": {
            "endpoint": endpoint,
            "baud": args.baud,
            "requested_messages": requested_messages,
            "message_counts": dict(sorted(state.telemetry_counts.items())),
            "vehicle_message_counts": dict(
                sorted(state.vehicle_telemetry_counts.items())
            ),
            "outbound": (
                [
                    "PARAM_REQUEST_READ ARMING_SKIPCHK",
                    "MAV_CMD_SET_MESSAGE_INTERVAL for requested_messages",
                ]
                if tag_servo
                else ["MAV_CMD_SET_MESSAGE_INTERVAL for requested_messages"]
            ),
        },
        "tag_servo": (
            tag_servo_session.manifest() if tag_servo_session is not None else None
        ),
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
        f"tags={state.tag_detections}, "
        f"servo_pulses={state.servo_pulses_completed}, "
        f"stop_reason={state.stop_reason or 'unspecified'}",
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
