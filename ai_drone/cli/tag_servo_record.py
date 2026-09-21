"""Optional guarded BCM12 servo pulses for the shared AprilTag capture engine."""

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
from contextlib import ExitStack, suppress
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from types import FrameType
from typing import Any

from ai_drone.capture.state import AnalysisFrame, CaptureState
from ai_drone.cli.servo import ACTUATION_CONFIRMATION
from ai_drone.mavlink.safety import is_fresh
from ai_drone.mount import (
    ABSOLUTE_MAX_PULSE_US,
    ABSOLUTE_MIN_PULSE_US,
    DEFAULT_MAX_PULSE_US,
    DEFAULT_MIN_PULSE_US,
    DEFAULT_SETTLE_S,
    MOUNT_OPEN_VALUE,
    SERVO_GPIO_PIN,
    ServoProcessLock,
    create_servo,
    parse_servo_input,
    pulse_us,
)
from ai_drone.recording import json_safe, write_json_line
from ai_drone.system import handled_signals
from ai_drone.vision.apriltags import TagDetection

ARMED_FLIGHT_CONFIRMATION = "ARMED_FLIGHT_TAG_SERVO_CLEAR"
MAX_PULSE_DURATION_S = 2.0
MAX_SETTLE_DURATION_S = 2.0
MAX_DETECTION_AGE_S = 1.0
MAX_HEARTBEAT_AGE_S = 5.0


class ActuationStop(threading.Event):
    """Serialize stop publication with the final active GPIO assignment."""

    def __init__(self) -> None:
        super().__init__()
        self.command_lock = threading.RLock()

    def set(self) -> None:
        with self.command_lock:
            super().set()


@dataclass(frozen=True)
class TagServoConfig:
    """Validated, mechanism-specific actuation and detection settings."""

    allowed_tag_ids: frozenset[int] | None
    stop_after: int | None
    confirmation_frames: int
    minimum_decision_margin: float
    maximum_detection_age_s: float
    maximum_heartbeat_age_s: float
    minimum_pulse_us: int
    maximum_pulse_us: int
    active_pulse_us: int
    rest_pulse_us: int
    pulse_duration_s: float
    settle_duration_s: float
    open_mount: bool = False

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> TagServoConfig:
        """Build a config after :func:`validate_args` has accepted it."""

        return cls(
            allowed_tag_ids=(None if args.all_tags else frozenset(args.tag_ids or ())),
            stop_after=args.stop_after,
            confirmation_frames=args.confirmation_frames,
            minimum_decision_margin=args.min_decision_margin,
            maximum_detection_age_s=args.max_detection_age,
            maximum_heartbeat_age_s=args.max_heartbeat_age,
            minimum_pulse_us=args.min_us,
            maximum_pulse_us=args.max_us,
            active_pulse_us=args.active_us,
            rest_pulse_us=args.rest_us,
            pulse_duration_s=args.pulse_duration,
            settle_duration_s=args.settle_duration,
            open_mount=getattr(args, "open_mount", False),
        )


def mount_recording_defaults() -> dict[str, object]:
    """Proven mount motion with tag 3 as the default trigger.

    Match the mount helper's pulse geometry, position and hold time. Invoking
    this dedicated script selects actuation while armed; it needs no separate
    flight acknowledgement or servo calibration arguments.
    """
    active_us = pulse_us(MOUNT_OPEN_VALUE)
    return {
        "all_tags": False,
        "tag_ids": [3],
        "stop_after": None,
        "confirmation_frames": 3,
        "min_decision_margin": 30.0,
        "max_detection_age": 0.5,
        "max_heartbeat_age": 2.5,
        "min_us": DEFAULT_MIN_PULSE_US,
        "max_us": DEFAULT_MAX_PULSE_US,
        "active_us": active_us,
        "rest_us": DEFAULT_MIN_PULSE_US,  # Unused: this operation never re-closes.
        "pulse_duration": DEFAULT_SETTLE_S,
        "settle_duration": 0.0,
        "open_mount": True,
    }


@dataclass(frozen=True)
class ServoTrigger:
    """One fresh, confirmed, lifetime-deduplicated tag encounter."""

    tag_id: int
    frame_index: int
    elapsed_s: float
    captured_monotonic: float


@dataclass(frozen=True)
class Qualification:
    visible: tuple[int, ...]
    qualifying: frozenset[int]
    reasons: tuple[tuple[int, str], ...]


@dataclass(frozen=True)
class StreakState:
    counts: tuple[tuple[int, int], ...] = ()
    captured: float | None = None
    ready: bool = False


def qualify(
    detections: list[TagDetection],
    config: TagServoConfig,
    now: float,
    captured: float | None,
) -> Qualification:
    """Choose the best detection per ID and explain each quality decision."""
    by_id: dict[int, TagDetection] = {}
    for detection in detections:
        previous = by_id.get(detection.tag_id)
        margin = (
            detection.decision_margin if detection.decision_margin is not None else -1.0
        )
        old_margin = previous.decision_margin if previous is not None else None
        if previous is None or margin > (
            old_margin if old_margin is not None else -1.0
        ):
            by_id[detection.tag_id] = detection
    reasons: dict[int, str] = {}
    for tag_id, detection in by_id.items():
        if config.allowed_tag_ids is not None and tag_id not in config.allowed_tag_ids:
            reason = "id_not_allowed"
        elif detection.hamming != 0:
            reason = "hamming_not_zero"
        elif detection.decision_margin is None:
            reason = "decision_margin_unavailable"
        elif detection.decision_margin < config.minimum_decision_margin:
            reason = "decision_margin_too_low"
        elif not is_fresh(captured, now, config.maximum_detection_age_s):
            reason = "stale_frame"
        else:
            reason = "qualifying"
        reasons[tag_id] = reason
    return Qualification(
        tuple(sorted(by_id)),
        frozenset(
            tag_id for tag_id, reason in reasons.items() if reason == "qualifying"
        ),
        tuple(sorted(reasons.items())),
    )


def advance_streaks(
    previous: StreakState,
    qualifying: frozenset[int],
    captured: float | None,
    ready: bool,
    config: TagServoConfig,
) -> StreakState:
    """Advance only consecutive qualifying frames in the same ready epoch."""
    counts = dict(previous.counts)
    contiguous = (
        ready
        and previous.ready
        and captured is not None
        and is_fresh(previous.captured, captured, config.maximum_detection_age_s)
    )
    return StreakState(
        tuple(
            (tag_id, counts.get(tag_id, 0) + 1 if contiguous else 1)
            for tag_id in sorted(qualifying)
        ),
        captured,
        ready,
    )


def select_trigger(
    streak: StreakState,
    scheduled: set[int],
    completed: set[int],
    config: TagServoConfig,
) -> int | None:
    """Choose at most one confirmed ID within the run's lifetime budget."""
    if not streak.ready or (
        config.stop_after is not None
        and len(completed) + len(scheduled) >= config.stop_after
    ):
        return None
    return next(
        (
            tag_id
            for tag_id, count in streak.counts
            if count >= config.confirmation_frames
            and tag_id not in scheduled
            and tag_id not in completed
        ),
        None,
    )


def _tag_range(value: str) -> range:
    try:
        start, end = map(int, value.split(":"))
        if not 0 <= start <= end <= 586:
            raise ValueError
    except ValueError:
        raise argparse.ArgumentTypeError(
            "tag range requires 0 <= START <= END <= 586"
        ) from None
    return range(start, end + 1)


def add_arguments(parser: argparse.ArgumentParser) -> None:
    """Add the explicit armed-flight and GPIO-actuation arguments."""

    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument(
        "--all-tags",
        action="store_true",
        help="allow every valid tag36h11 ID (each ID can trigger only once)",
    )
    selection.add_argument(
        "--tag-id",
        dest="tag_ids",
        action="append",
        type=int,
        metavar="ID",
        help="allow one tag36h11 ID; repeat to allow multiple IDs",
    )
    selection.add_argument(
        "--tag-range",
        dest="tag_ids",
        action="extend",
        type=_tag_range,
        metavar="START:END",
        help="allow an inclusive tag36h11 ID range; repeat for disjoint ranges",
    )
    parser.add_argument(
        "--stop-after",
        type=int,
        metavar="COMPLETED_PULSES",
        help=(
            "stop after this many distinct tag IDs have completed a pulse and "
            "return-to-rest; omit to run until Ctrl-C, SIGTERM, disarm, or failure"
        ),
    )
    parser.add_argument(
        "--confirmation-frames",
        type=int,
        default=3,
        metavar="FRAMES",
        help="required consecutive fresh native detections (default: 3)",
    )
    parser.add_argument(
        "--min-decision-margin",
        type=float,
        default=30.0,
        metavar="MARGIN",
        help="minimum native AprilTag decision margin (default: 30)",
    )
    parser.add_argument(
        "--max-detection-age",
        type=float,
        default=0.5,
        metavar="SECONDS",
        help="maximum frame age when a pulse begins (default: 0.5, hard max: 1)",
    )
    parser.add_argument(
        "--max-heartbeat-age",
        type=float,
        default=2.5,
        metavar="SECONDS",
        help="maximum selected-FC heartbeat age (default: 2.5, hard max: 5)",
    )
    parser.add_argument(
        "--min-us",
        type=int,
        default=DEFAULT_MIN_PULSE_US,
        metavar="MICROSECONDS",
        help="calibrated servo minimum used for GPIO mapping (default: 900)",
    )
    parser.add_argument(
        "--max-us",
        type=int,
        default=DEFAULT_MAX_PULSE_US,
        metavar="MICROSECONDS",
        help="calibrated servo maximum used for GPIO mapping (default: 2100)",
    )
    parser.add_argument(
        "--active-us",
        type=int,
        required=True,
        metavar="MICROSECONDS",
        help="mechanism-tested active pulse width; no default is assumed",
    )
    parser.add_argument(
        "--rest-us",
        type=int,
        required=True,
        metavar="MICROSECONDS",
        help="mechanism-tested safe rest pulse width; no default is assumed",
    )
    parser.add_argument(
        "--pulse-duration",
        type=float,
        required=True,
        metavar="SECONDS",
        help="active hold time greater than 0 and at most 2 seconds",
    )
    parser.add_argument(
        "--settle-duration",
        type=float,
        default=0.5,
        metavar="SECONDS",
        help="return-to-rest hold before PWM detaches (default: 0.5, max: 2)",
    )
    parser.add_argument(
        "--confirm-actuation",
        choices=[ACTUATION_CONFIRMATION],
        required=True,
        help=f"required mechanism-clear acknowledgement: {ACTUATION_CONFIRMATION}",
    )
    parser.add_argument(
        "--confirm-armed-flight",
        choices=[ARMED_FLIGHT_CONFIRMATION],
        required=True,
        help=(
            "required acknowledgement that GPIO servo motion is permitted while "
            f"the vehicle may be armed: {ARMED_FLIGHT_CONFIRMATION}"
        ),
    )


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    """Reject unsafe settings before platform, GPIO, camera, or MAVLink access."""
    _validate_detection_args(parser, args)
    _validate_pulse_args(parser, args)
    _validate_tag_selection(parser, args)


def _validate_detection_args(
    parser: argparse.ArgumentParser, args: argparse.Namespace
) -> None:
    if args.stop_after is not None and args.stop_after <= 0:
        parser.error("--stop-after must be a positive integer when provided")
    if args.stop_after is not None and args.stop_after > 587:
        parser.error("--stop-after cannot exceed the 587 tag36h11 IDs")
    if not 2 <= args.confirmation_frames <= 30:
        parser.error("--confirmation-frames must be between 2 and 30")
    if not math.isfinite(args.min_decision_margin) or args.min_decision_margin <= 0:
        parser.error("--min-decision-margin must be finite and greater than zero")
    if (
        not math.isfinite(args.max_detection_age)
        or not 0 < args.max_detection_age <= MAX_DETECTION_AGE_S
    ):
        parser.error(
            f"--max-detection-age must be greater than 0 and at most "
            f"{MAX_DETECTION_AGE_S:g}"
        )
    if (
        not math.isfinite(args.max_heartbeat_age)
        or not 1.0 < args.max_heartbeat_age <= MAX_HEARTBEAT_AGE_S
    ):
        parser.error(
            f"--max-heartbeat-age must be greater than 1 and at most "
            f"{MAX_HEARTBEAT_AGE_S:g}"
        )


def _validate_pulse_args(
    parser: argparse.ArgumentParser, args: argparse.Namespace
) -> None:
    if args.min_us < ABSOLUTE_MIN_PULSE_US:
        parser.error(
            f"--min-us must be at least the absolute software limit "
            f"{ABSOLUTE_MIN_PULSE_US}"
        )
    if args.max_us > ABSOLUTE_MAX_PULSE_US:
        parser.error(
            f"--max-us must be at most the absolute software limit "
            f"{ABSOLUTE_MAX_PULSE_US}"
        )
    if not args.min_us < 1500 < args.max_us:
        parser.error("pulse geometry must satisfy --min-us < 1500 < --max-us")
    for name, value in (("--active-us", args.active_us), ("--rest-us", args.rest_us)):
        if not args.min_us <= value <= args.max_us:
            parser.error(f"{name} must be between --min-us and --max-us")
    if args.active_us == args.rest_us:
        parser.error("--active-us and --rest-us must be different")
    if (
        not math.isfinite(args.pulse_duration)
        or not 0 < args.pulse_duration <= MAX_PULSE_DURATION_S
    ):
        parser.error(
            f"--pulse-duration must be greater than 0 and at most "
            f"{MAX_PULSE_DURATION_S:g}"
        )
    if (
        not math.isfinite(args.settle_duration)
        or not 0 <= args.settle_duration <= MAX_SETTLE_DURATION_S
    ):
        parser.error(
            f"--settle-duration must be between 0 and {MAX_SETTLE_DURATION_S:g}"
        )


def _validate_tag_selection(
    parser: argparse.ArgumentParser, args: argparse.Namespace
) -> None:
    if args.tag_ids is not None:
        if any(tag_id < 0 or tag_id > 586 for tag_id in args.tag_ids):
            parser.error("--tag-id must be between 0 and 586 for tag36h11")
        if len(set(args.tag_ids)) != len(args.tag_ids):
            parser.error("--tag-id values must not be repeated")
        if args.stop_after is not None and args.stop_after > len(args.tag_ids):
            parser.error(
                "--stop-after cannot exceed the number of allowed --tag-id values"
            )


class ServoEventWriter:
    """Low-rate actuator audit log synced after every event."""

    def __init__(self, path: Path) -> None:
        self._handle = path.open("x", encoding="utf-8")
        self._lock = threading.Lock()

    def write(self, event: str, **fields: object) -> None:
        record = {
            "timestamp_utc": datetime.now(UTC).isoformat(),
            "event": event,
            "feedback_available": False,
            **fields,
        }
        with self._lock:
            write_json_line(self._handle, record)
            self._handle.flush()
            os.fsync(self._handle.fileno())

    def close(self) -> None:
        with self._lock:
            if self._handle.closed:
                return
            try:
                self._handle.flush()
                os.fsync(self._handle.fileno())
            finally:
                self._handle.close()


@dataclass
class _PulseTiming:
    active_commanded_utc: str | None = None
    active_started: float | None = None
    rest_commanded_utc: str | None = None
    rest_started: float | None = None
    detached_utc: str | None = None


class TagServoSession:
    """Qualify detections and serialize bounded, lifetime-deduplicated pulses."""

    def __init__(
        self,
        *,
        config: TagServoConfig,
        event_path: Path,
        state: CaptureState,
        capture_stop: threading.Event,
        ready: threading.Event,
        servo_factory: Any | None = None,
        process_lock_factory: Any = ServoProcessLock,
    ) -> None:
        if not isinstance(capture_stop, ActuationStop):
            raise TypeError("payload servo requires a synchronized ActuationStop")
        self.config = config
        self.state = state
        self.capture_stop = capture_stop
        self.ready = ready
        self._lock = capture_stop.command_lock
        self._queue: queue.Queue[ServoTrigger | None] = queue.Queue(maxsize=1)
        self._scheduled_ids: set[int] = set()
        self._confirmed_ids: set[int] = set()
        self._completed_ids: set[int] = set()
        self._attempted_ids: set[int] = set()
        self._issued_ids: set[int] = set()
        self._hold_completed_ids: set[int] = set()
        self._detached_ids: set[int] = set()
        self._interrupted_ids: set[int] = set()
        self._streak = StreakState()
        self._accepting = True
        self._ever_commanded = False
        self._closed = False
        self._gpio_closed = False
        self._events_closed = False
        self._last_processed_monotonic: float | None = None
        self._health_started_monotonic: float | None = None
        self._signals = ExitStack()
        self._signals_installed = False

        self._process_lock = process_lock_factory()
        servo_instance: Any = None
        event_writer: ServoEventWriter | None = None
        try:
            servo_instance = create_servo(
                min_us=config.minimum_pulse_us,
                max_us=config.maximum_pulse_us,
                factory=servo_factory,
            )
            event_writer = ServoEventWriter(event_path)
            self._active_value = parse_servo_input(
                f"{config.active_pulse_us}us",
                min_us=config.minimum_pulse_us,
                max_us=config.maximum_pulse_us,
            )
            self._rest_value = parse_servo_input(
                f"{config.rest_pulse_us}us",
                min_us=config.minimum_pulse_us,
                max_us=config.maximum_pulse_us,
            )
            event_writer.write(
                "servo_ready_detached",
                gpio=SERVO_GPIO_PIN,
                active_us=config.active_pulse_us,
                rest_us=config.rest_pulse_us,
                pulse_duration_s=config.pulse_duration_s,
                settle_duration_s=config.settle_duration_s,
            )
        except BaseException:
            if event_writer is not None:
                with suppress(BaseException):
                    event_writer.close()
            if servo_instance is not None:
                with suppress(BaseException):
                    servo_instance.detach()
                with suppress(BaseException):
                    servo_instance.close()
            with suppress(BaseException):
                self._process_lock.close()
            raise

        self._servo: Any = servo_instance
        self._events: ServoEventWriter = event_writer

        self._worker = threading.Thread(
            target=self._actuator_loop,
            name="apriltag-servo-actuator",
            daemon=True,
        )
        try:
            self._worker.start()
            self._publish_state()
        except BaseException:
            self.close()
            raise

    def install_signal_handlers(self) -> None:
        """Convert process termination signals into a cleanup-capable stop."""

        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError("signal handlers must be installed from the main thread")
        if not self._signals_installed:
            self._signals.enter_context(
                handled_signals(
                    {
                        signum: self._handle_stop_signal
                        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
                    }
                )
            )
            self._signals_installed = True

    def _handle_stop_signal(self, signum: int, _frame: FrameType | None) -> None:
        with self._lock:
            self.stop_accepting()
            self.capture_stop.set()
            self.state.set_stop_reason(f"operator_signal_{signal.Signals(signum).name}")

    def _restore_signal_handlers(self) -> None:
        if threading.current_thread() is not threading.main_thread():
            return
        self._signals.close()
        self._signals_installed = False

    def _publish_state(self) -> None:
        with self._lock, self.state.lock:
            self.state.confirmed_tag_ids = tuple(sorted(self._confirmed_ids))
            self.state.completed_servo_tag_ids = tuple(sorted(self._completed_ids))
            self.state.pending_servo_tag_ids = tuple(sorted(self._scheduled_ids))
            self.state.servo_pulses_completed = len(self._completed_ids)

    def _annotate_tag_records(
        self,
        tag_records: list[dict[str, Any]],
        reasons: dict[int, str],
    ) -> None:
        record_by_id = {int(record["id"]): record for record in tag_records}
        with self._lock:
            completed_ids = self._completed_ids.copy()
            scheduled_ids = self._scheduled_ids.copy()
        for tag_id, record in record_by_id.items():
            record["actuation_quality"] = reasons.get(tag_id, "duplicate_detection")
            record["confirmation_streak"] = dict(self._streak.counts).get(tag_id, 0)
            if tag_id in completed_ids:
                record["actuation_state"] = "completed_for_run"
            elif tag_id in scheduled_ids:
                record["actuation_state"] = "pending"
            else:
                record["actuation_state"] = "not_scheduled"

    def observe(
        self,
        frame: AnalysisFrame,
        detections: list[TagDetection],
        tag_records: list[dict[str, Any]],
    ) -> None:
        """Update live state and enqueue at most one fresh confirmed trigger."""

        now = time.monotonic()
        self._last_processed_monotonic = now
        captured = frame.captured_monotonic
        quality = qualify(detections, self.config, now, captured)
        with self.state.lock:
            self.state.visible_tag_ids = quality.visible
        ready_now = self.ready.is_set()
        self._streak = advance_streaks(
            self._streak, quality.qualifying, captured, ready_now, self.config
        )
        self._annotate_tag_records(tag_records, dict(quality.reasons))
        if not ready_now or captured is None:
            return
        with self._lock:
            if not self._accepting or self.capture_stop.is_set():
                return
            unavailable = self._completed_ids | (
                self._issued_ids if self.config.open_mount else set()
            )
            tag_id = select_trigger(
                self._streak, self._scheduled_ids, unavailable, self.config
            )
            if tag_id is None:
                return
            self._scheduled_ids.add(tag_id)
        self._enqueue_trigger(
            ServoTrigger(tag_id, frame.frame_index, frame.elapsed_s, captured),
            tag_records,
        )

    def _enqueue_trigger(
        self, trigger: ServoTrigger, tag_records: list[dict[str, Any]]
    ) -> None:
        tag_id = trigger.tag_id
        # Ownership is reserved, but the actuator cannot see this work until
        # the intent is durable. Keep the command gate available during I/O.
        try:
            self._events.write(
                "tag_confirmed_intent",
                tag_id=tag_id,
                frame=trigger.frame_index,
                elapsed_s=round(trigger.elapsed_s, 6),
                confirmation_frames=dict(self._streak.counts)[tag_id],
            )
        except BaseException:
            self.capture_stop.set()
            self.stop_accepting()
            self._release_scheduled(tag_id)
            raise
        with self._lock:
            if not self._accepting or self.capture_stop.is_set():
                self._release_scheduled(tag_id)
                return
            try:
                self._queue.put_nowait(trigger)
            except queue.Full:
                self._scheduled_ids.discard(tag_id)
                return
            self._confirmed_ids.add(tag_id)
        for record in tag_records:
            if int(record["id"]) == tag_id:
                record["actuation_state"] = "pending"
                record["actuation_queued"] = True
        self._publish_state()
        self._events.write(
            "tag_confirmed_queued",
            tag_id=tag_id,
            frame=trigger.frame_index,
            elapsed_s=round(trigger.elapsed_s, 6),
            confirmation_frames=dict(self._streak.counts)[tag_id],
        )
        print(
            json.dumps(
                {
                    "event": "apriltag_confirmed",
                    "id": tag_id,
                    "frame": trigger.frame_index,
                    "elapsed_s": round(trigger.elapsed_s, 6),
                },
                sort_keys=True,
            ),
            flush=True,
        )

    def _selected_heartbeat_is_fresh(self, now: float) -> bool:
        with self.state.lock:
            observed = self.state.last_vehicle_heartbeat_monotonic
        return is_fresh(observed, now, self.config.maximum_heartbeat_age_s)

    def health_error(self, now: float) -> str | None:
        """Return a fail-closed runtime-watchdog error after READY."""

        if not self.ready.is_set():
            return None
        if self._health_started_monotonic is None:
            self._health_started_monotonic = now
        if not self._selected_heartbeat_is_fresh(now):
            return "selected flight-controller heartbeat became stale"
        processed = self._last_processed_monotonic
        detector_timeout = max(2.0, 4.0 * self.config.maximum_detection_age_s)
        if processed is None:
            if now - self._health_started_monotonic <= detector_timeout:
                return None
            return "AprilTag analysis worker did not produce a frame"
        if now - processed > detector_timeout:
            return "AprilTag analysis worker became stale"
        return None

    def _record_and_print(
        self, event: str, trigger: ServoTrigger, **fields: object
    ) -> None:
        self._events.write(
            event,
            tag_id=trigger.tag_id,
            frame=trigger.frame_index,
            elapsed_s=round(trigger.elapsed_s, 6),
            **fields,
        )
        print(
            json.dumps(
                {
                    "event": event,
                    "id": trigger.tag_id,
                    "elapsed_s": round(trigger.elapsed_s, 6),
                    **json_safe(fields),
                },
                sort_keys=True,
            ),
            flush=True,
        )

    def _release_pulse(self, timing: _PulseTiming) -> BaseException | None:
        """Restore pulse-session rest and always attempt to detach PWM."""
        release_error: BaseException | None = None
        try:
            if not self.config.open_mount:
                timing.rest_commanded_utc = datetime.now(UTC).isoformat()
                timing.rest_started = time.monotonic()
                with self._lock:
                    if not self._gpio_closed:
                        self._servo.value = self._rest_value
                time.sleep(self.config.settle_duration_s)
        except BaseException as error:
            release_error = error
        try:
            with self._lock:
                if not self._gpio_closed:
                    self._servo.detach()
            timing.detached_utc = datetime.now(UTC).isoformat()
        except BaseException as error:
            if release_error is None:
                release_error = error
        return release_error

    def _record_pulse_commands(
        self, trigger: ServoTrigger, timing: _PulseTiming
    ) -> None:
        if timing.active_started is not None:
            self._record_and_print(
                "mount_open_commanded"
                if self.config.open_mount
                else "servo_active_commanded",
                trigger,
                commanded_us=self.config.active_pulse_us,
                commanded_utc=timing.active_commanded_utc,
                actual_active_s=round(
                    max(
                        0.0,
                        (timing.rest_started or time.monotonic())
                        - timing.active_started,
                    ),
                    6,
                ),
            )
        if timing.rest_started is not None:
            self._record_and_print(
                "servo_rest_commanded",
                trigger,
                commanded_us=self.config.rest_pulse_us,
                commanded_utc=timing.rest_commanded_utc,
            )
        if timing.detached_utc is not None:
            self._record_and_print(
                "servo_pwm_detached",
                trigger,
                commanded_utc=timing.detached_utc,
            )

    def _command_pulse(self, trigger: ServoTrigger) -> bool:
        """Command active and detach; return to rest only for pulse sessions."""

        completed_hold = False
        pulse_error: BaseException | None = None
        timing = _PulseTiming()
        self._record_and_print(
            "servo_pulse_starting",
            trigger,
            active_us=self.config.active_pulse_us,
            rest_us=self.config.rest_pulse_us,
            requested_active_s=self.config.pulse_duration_s,
        )
        # This is the final command boundary. No journal/stdout I/O may separate
        # these checks from GPIO, and every stop uses this same lock.
        with self._lock:
            now = time.monotonic()
            detection_age = now - trigger.captured_monotonic
            if (
                not self._accepting
                or self.capture_stop.is_set()
                or not self.ready.is_set()
                or trigger.tag_id not in self._scheduled_ids
                or not 0 <= detection_age <= self.config.maximum_detection_age_s
                or not self._selected_heartbeat_is_fresh(now)
            ):
                return False
            self._ever_commanded = True
            self._attempted_ids.add(trigger.tag_id)
            try:
                timing.active_commanded_utc = datetime.now(UTC).isoformat()
                self._servo.value = self._active_value
                timing.active_started = time.monotonic()
                self._issued_ids.add(trigger.tag_id)
            except BaseException as error:
                pulse_error = error
        try:
            if pulse_error is None:
                completed_hold = not self.capture_stop.wait(
                    self.config.pulse_duration_s
                )
        except BaseException as error:
            pulse_error = error
        finally:
            release_error = self._release_pulse(timing)
            if pulse_error is None:
                pulse_error = release_error
        with self._lock:
            if completed_hold:
                self._hold_completed_ids.add(trigger.tag_id)
            else:
                self._interrupted_ids.add(trigger.tag_id)
            if timing.detached_utc is not None:
                self._detached_ids.add(trigger.tag_id)
        self._record_pulse_commands(trigger, timing)
        self._record_and_print(
            "servo_pulse_outcome",
            trigger,
            command_attempted=True,
            command_issued=timing.active_started is not None,
            hold_completed=completed_hold,
            pwm_detached=timing.detached_utc is not None,
            interrupted=not completed_hold,
            error=str(pulse_error) if pulse_error is not None else None,
        )
        if pulse_error is not None:
            raise RuntimeError(
                f"payload servo command failed: {pulse_error}"
            ) from pulse_error
        return completed_hold

    def _release_scheduled(self, tag_id: int) -> None:
        with self._lock:
            self._scheduled_ids.discard(tag_id)
        self._publish_state()

    def _complete_scheduled(self, tag_id: int) -> None:
        """Atomically replace the pending latch with the lifetime latch."""

        with self._lock:
            self._scheduled_ids.discard(tag_id)
            self._completed_ids.add(tag_id)
        self._publish_state()

    def _actuator_loop(self) -> None:
        while True:
            trigger = self._queue.get()
            try:
                if trigger is None:
                    return
                now = time.monotonic()
                if self.capture_stop.is_set() or not self._accepting:
                    self._record_and_print("servo_trigger_cancelled", trigger)
                    self._release_scheduled(trigger.tag_id)
                    continue
                if (
                    now - trigger.captured_monotonic
                    > self.config.maximum_detection_age_s
                ):
                    self._record_and_print(
                        "servo_trigger_rejected_stale",
                        trigger,
                        age_s=round(now - trigger.captured_monotonic, 6),
                    )
                    self._release_scheduled(trigger.tag_id)
                    continue
                if not self._selected_heartbeat_is_fresh(now):
                    self._record_and_print(
                        "servo_trigger_rejected_stale_heartbeat", trigger
                    )
                    self._release_scheduled(trigger.tag_id)
                    self.state.record_error(
                        "selected flight-controller heartbeat became stale before "
                        "payload servo actuation"
                    )
                    self.state.set_stop_reason("heartbeat_stale")
                    self.stop_accepting()
                    self.capture_stop.set()
                    continue
                completed = self._command_pulse(trigger)
                if not completed:
                    self._release_scheduled(trigger.tag_id)
                    self._record_and_print("servo_pulse_interrupted", trigger)
                    continue
                self._complete_scheduled(trigger.tag_id)
                self._record_and_print(
                    "servo_pulse_completed_commanded",
                    trigger,
                    completed_count=len(self._completed_ids),
                )
                if (
                    self.config.stop_after is not None
                    and len(self._completed_ids) >= self.config.stop_after
                ):
                    self.state.set_stop_reason("tag_limit_reached")
                    self.stop_accepting()
                    self.capture_stop.set()
            except Exception as error:
                if trigger is not None:
                    self._release_scheduled(trigger.tag_id)
                    with suppress(Exception):
                        self._record_and_print(
                            "servo_pulse_failed", trigger, error=str(error)
                        )
                self.state.record_error(str(error))
                self.state.set_stop_reason("servo_error")
                self.stop_accepting()
                self.capture_stop.set()
            finally:
                self._queue.task_done()

    def stop_accepting(self) -> None:
        with self._lock:
            self._accepting = False

    def _cancel_queued_triggers(self) -> None:
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                return
            else:
                try:
                    if item is not None:
                        self._release_scheduled(item.tag_id)
                        with suppress(Exception):
                            self._record_and_print("servo_trigger_cancelled", item)
                finally:
                    self._queue.task_done()

    def _stop_worker(self) -> None:
        self._cancel_queued_triggers()
        self._queue.put_nowait(None)
        if self._worker.ident is not None:
            self._worker.join(
                timeout=self.config.pulse_duration_s + self.config.settle_duration_s + 2
            )
        if self._worker.is_alive():
            raise RuntimeError("payload servo worker did not stop boundedly")

    def _restore_rest(self) -> None:
        with self._lock:
            self._servo.value = self._rest_value
        time.sleep(self.config.settle_duration_s)

    def _close_resources(
        self, cleanup: Callable[[str, Callable[[], object]], bool]
    ) -> None:
        if (
            self._ever_commanded
            and not self._gpio_closed
            and not self.config.open_mount
        ):
            cleanup("restore payload servo rest position", self._restore_rest)
        with self._lock:
            if not self._gpio_closed:
                cleanup("detach payload servo PWM", self._servo.detach)
                self._gpio_closed = cleanup(
                    "close payload servo GPIO", self._servo.close
                )
        if not self._events_closed:
            cleanup(
                "write payload servo close event",
                lambda: self._events.write("servo_session_closed"),
            )
            self._events_closed = cleanup(
                "close payload servo event log", self._events.close
            )
        if self._gpio_closed:
            released = cleanup("release payload servo lock", self._process_lock.close)
            self._closed = released and self._events_closed

    def close(self) -> None:
        """Cancel work, restore rest for pulse sessions, detach, and release BCM12."""

        if self._closed:
            return
        self.stop_accepting()
        self.capture_stop.set()
        interruption: BaseException | None = None

        def cleanup(label: str, action: Callable[[], object]) -> bool:
            nonlocal interruption
            try:
                action()
                return True
            except BaseException as error:
                self.state.record_error(f"{label}: {error}")
                if not isinstance(error, Exception) and interruption is None:
                    interruption = error
                return False

        try:
            cleanup("stop payload servo worker", self._stop_worker)
            self._close_resources(cleanup)
        finally:
            # Termination remains a cooperative stop throughout rest/detach and
            # resource release, including repeated signals during cleanup.
            cleanup(
                "restore payload servo signal handlers", self._restore_signal_handlers
            )
        if interruption is not None:
            raise interruption

    def manifest(self) -> dict[str, object]:
        with self._lock, self.state.lock:
            return {
                "gpio": SERVO_GPIO_PIN,
                "feedback_available": False,
                "open_mount": self.config.open_mount,
                "active_us": self.config.active_pulse_us,
                "rest_us": self.config.rest_pulse_us,
                "pulse_duration_s": self.config.pulse_duration_s,
                "settle_duration_s": self.config.settle_duration_s,
                "confirmation_frames": self.config.confirmation_frames,
                "minimum_decision_margin": self.config.minimum_decision_margin,
                "allowed_tag_ids": (
                    "all"
                    if self.config.allowed_tag_ids is None
                    else sorted(self.config.allowed_tag_ids)
                ),
                "stop_after": self.config.stop_after,
                "confirmed_tag_ids": list(self.state.confirmed_tag_ids),
                "completed_tag_ids": list(self.state.completed_servo_tag_ids),
                "completed_commanded_pulses": self.state.servo_pulses_completed,
                "attempted_tag_ids": sorted(self._attempted_ids),
                "issued_tag_ids": sorted(self._issued_ids),
                "hold_completed_tag_ids": sorted(self._hold_completed_ids),
                "pwm_detached_tag_ids": sorted(self._detached_ids),
                "interrupted_tag_ids": sorted(self._interrupted_ids),
            }


def main(arguments: list[str] | None = None) -> int:
    from ai_drone.cli.record import run

    return run(arguments, operation="tag-servo")


if __name__ == "__main__":
    raise SystemExit(main())
