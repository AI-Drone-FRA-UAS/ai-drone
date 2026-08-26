"""Armed-flight AprilTag recording with bounded BCM12 payload-servo pulses.

This is deliberately a separate entry point from :mod:`ai_drone.cli.record`.
The ordinary ``drone-inspect`` command remains passive and continues to reject
an initially armed vehicle.  This module supplies the explicit actuation
policy used by ``drone-tag-servo-record`` while reusing the synchronized camera
and telemetry capture engine.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import signal
import threading
import time
from contextlib import suppress
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from types import FrameType
from typing import Any, Protocol

from ai_drone.cli.servo import (
    ABSOLUTE_MAX_PULSE_US,
    ABSOLUTE_MIN_PULSE_US,
    ACTUATION_CONFIRMATION,
    SERVO_GPIO_PIN,
    ServoProcessLock,
    _target_value_from_input,
)
from ai_drone.recording import json_safe, write_json_line

ARMED_FLIGHT_CONFIRMATION = "ARMED_FLIGHT_TAG_SERVO_CLEAR"
DEFAULT_MIN_PULSE_US = 900
DEFAULT_MAX_PULSE_US = 2100
MAX_PULSE_DURATION_S = 2.0
MAX_SETTLE_DURATION_S = 2.0
MAX_DETECTION_AGE_S = 1.0
MAX_HEARTBEAT_AGE_S = 5.0


class AnalysisFrameLike(Protocol):
    """Capture fields needed to qualify an actuation request."""

    frame_index: int
    elapsed_s: float
    captured_monotonic: float | None


class DetectionLike(Protocol):
    """AprilTag quality fields required by the active command."""

    tag_id: int
    hamming: int | None
    decision_margin: float | None


class CaptureStateLike(Protocol):
    """Thread-shared recording state updated by the active session."""

    last_vehicle_heartbeat_monotonic: float | None
    visible_tag_ids: tuple[int, ...]
    confirmed_tag_ids: tuple[int, ...]
    completed_servo_tag_ids: tuple[int, ...]
    pending_servo_tag_ids: tuple[int, ...]
    servo_pulses_completed: int
    stop_reason: str | None

    def record_error(self, message: str) -> None: ...

    def set_stop_reason(self, reason: str) -> None: ...


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
        )


@dataclass(frozen=True)
class ServoTrigger:
    """One fresh, confirmed, lifetime-deduplicated tag encounter."""

    tag_id: int
    frame_index: int
    elapsed_s: float
    captured_monotonic: float


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
            self._handle.flush()
            os.fsync(self._handle.fileno())
            self._handle.close()


class TagServoSession:
    """Qualify detections and serialize bounded, lifetime-deduplicated pulses."""

    def __init__(
        self,
        *,
        config: TagServoConfig,
        event_path: Path,
        state: CaptureStateLike,
        capture_stop: threading.Event,
        ready: threading.Event,
        servo_factory: Any | None = None,
        process_lock_factory: Any = ServoProcessLock,
    ) -> None:
        self.config = config
        self.state = state
        self.capture_stop = capture_stop
        self.ready = ready
        self._lock = threading.Lock()
        self._queue: queue.Queue[ServoTrigger | None] = queue.Queue(maxsize=1)
        self._scheduled_ids: set[int] = set()
        self._confirmed_ids: set[int] = set()
        self._completed_ids: set[int] = set()
        self._streaks: dict[int, int] = {}
        self._previous_qualifying_ids: set[int] = set()
        self._previous_capture_monotonic: float | None = None
        self._was_ready = False
        self._accepting = True
        self._ever_commanded = False
        self._closed = False
        self._last_processed_monotonic: float | None = None
        self._health_started_monotonic: float | None = None
        self._signal_handlers: dict[int, Any] = {}

        self._process_lock = process_lock_factory()
        servo_instance: Any = None
        event_writer: ServoEventWriter | None = None
        try:
            if servo_factory is None:
                from gpiozero import Servo  # ty: ignore[unresolved-import]

                servo_factory = Servo
            servo_instance = servo_factory(
                SERVO_GPIO_PIN,
                min_pulse_width=config.minimum_pulse_us / 1_000_000.0,
                max_pulse_width=config.maximum_pulse_us / 1_000_000.0,
                initial_value=None,
            )
            event_writer = ServoEventWriter(event_path)
            self._active_value = _target_value_from_input(
                f"{config.active_pulse_us}us",
                min_us=config.minimum_pulse_us,
                max_us=config.maximum_pulse_us,
            )
            self._rest_value = _target_value_from_input(
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
        except Exception:
            if event_writer is not None:
                with suppress(Exception):
                    event_writer.close()
            if servo_instance is not None:
                with suppress(Exception):
                    servo_instance.detach()
                with suppress(Exception):
                    servo_instance.close()
            self._process_lock.close()
            raise

        if event_writer is None or servo_instance is None:
            self._process_lock.close()
            raise RuntimeError("payload servo session did not initialize completely")
        self._servo: Any = servo_instance
        self._events: ServoEventWriter = event_writer

        self._worker = threading.Thread(
            target=self._actuator_loop,
            name="apriltag-servo-actuator",
            daemon=True,
        )
        self._worker.start()
        self._publish_state()

    def install_signal_handlers(self) -> None:
        """Convert process termination signals into a cleanup-capable stop."""

        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError("signal handlers must be installed from the main thread")
        for signum in (signal.SIGTERM, signal.SIGHUP):
            previous = signal.getsignal(signum)
            self._signal_handlers[signum] = previous
            signal.signal(signum, self._handle_stop_signal)

    def _handle_stop_signal(self, signum: int, _frame: FrameType | None) -> None:
        self.state.set_stop_reason(f"operator_signal_{signal.Signals(signum).name}")
        self.stop_accepting()
        self.capture_stop.set()

    def _restore_signal_handlers(self) -> None:
        if threading.current_thread() is not threading.main_thread():
            return
        for signum, previous in self._signal_handlers.items():
            signal.signal(signum, previous)
        self._signal_handlers.clear()

    def _publish_state(self) -> None:
        with self._lock:
            self.state.confirmed_tag_ids = tuple(sorted(self._confirmed_ids))
            self.state.completed_servo_tag_ids = tuple(sorted(self._completed_ids))
            self.state.pending_servo_tag_ids = tuple(sorted(self._scheduled_ids))
            self.state.servo_pulses_completed = len(self._completed_ids)

    def _allowed(self, tag_id: int) -> bool:
        allowed = self.config.allowed_tag_ids
        return allowed is None or tag_id in allowed

    def _best_detection_by_id(
        self, detections: list[DetectionLike]
    ) -> dict[int, DetectionLike]:
        by_id: dict[int, DetectionLike] = {}
        for detection in detections:
            current = by_id.get(detection.tag_id)
            current_margin = (
                -1.0
                if current is None or current.decision_margin is None
                else float(current.decision_margin)
            )
            candidate_margin = (
                -1.0
                if detection.decision_margin is None
                else float(detection.decision_margin)
            )
            if current is None or candidate_margin > current_margin:
                by_id[detection.tag_id] = detection
        return by_id

    def _quality_results(
        self,
        by_id: dict[int, DetectionLike],
        *,
        fresh_frame: bool,
    ) -> tuple[set[int], dict[int, str]]:
        qualifying: set[int] = set()
        reasons: dict[int, str] = {}
        for tag_id, detection in by_id.items():
            if not self._allowed(tag_id):
                reasons[tag_id] = "id_not_allowed"
            elif detection.hamming != 0:
                reasons[tag_id] = "hamming_not_zero"
            elif detection.decision_margin is None:
                reasons[tag_id] = "decision_margin_unavailable"
            elif detection.decision_margin < self.config.minimum_decision_margin:
                reasons[tag_id] = "decision_margin_too_low"
            elif not fresh_frame:
                reasons[tag_id] = "stale_frame"
            else:
                qualifying.add(tag_id)
                reasons[tag_id] = "qualifying"
        return qualifying, reasons

    def _advance_confirmation_streaks(
        self,
        qualifying: set[int],
        *,
        captured_monotonic: float | None,
        ready_now: bool,
    ) -> None:
        if not ready_now or not self._was_ready:
            self._streaks = {}
            self._previous_qualifying_ids = set()
            self._previous_capture_monotonic = None
        self._was_ready = ready_now
        gap_ok = (
            captured_monotonic is not None
            and self._previous_capture_monotonic is not None
            and 0.0
            <= captured_monotonic - self._previous_capture_monotonic
            <= self.config.maximum_detection_age_s
        )
        self._streaks = {
            tag_id: (
                self._streaks.get(tag_id, 0) + 1
                if gap_ok and tag_id in self._previous_qualifying_ids
                else 1
            )
            for tag_id in qualifying
        }
        self._previous_qualifying_ids = qualifying
        self._previous_capture_monotonic = captured_monotonic

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
            record["confirmation_streak"] = self._streaks.get(tag_id, 0)
            if tag_id in completed_ids:
                record["actuation_state"] = "completed_for_run"
            elif tag_id in scheduled_ids:
                record["actuation_state"] = "pending"
            else:
                record["actuation_state"] = "not_scheduled"

    def observe(
        self,
        frame: AnalysisFrameLike,
        detections: list[DetectionLike],
        tag_records: list[dict[str, Any]],
    ) -> None:
        """Update live state and enqueue at most one fresh confirmed trigger."""

        now = time.monotonic()
        self._last_processed_monotonic = now
        by_id = self._best_detection_by_id(detections)

        self.state.visible_tag_ids = tuple(sorted(by_id))
        captured = frame.captured_monotonic
        fresh_frame = (
            captured is not None
            and 0.0 <= now - captured <= self.config.maximum_detection_age_s
        )
        qualifying, reasons = self._quality_results(by_id, fresh_frame=fresh_frame)
        ready_now = self.ready.is_set()
        self._advance_confirmation_streaks(
            qualifying,
            captured_monotonic=captured,
            ready_now=ready_now,
        )
        self._annotate_tag_records(tag_records, reasons)

        if not ready_now or not self._accepting or captured is None:
            return
        for tag_id in sorted(qualifying):
            if self._streaks[tag_id] < self.config.confirmation_frames:
                continue
            with self._lock:
                if tag_id in self._completed_ids or tag_id in self._scheduled_ids:
                    continue
                if (
                    self.config.stop_after is not None
                    and len(self._completed_ids) + len(self._scheduled_ids)
                    >= self.config.stop_after
                ):
                    return
                trigger = ServoTrigger(
                    tag_id=tag_id,
                    frame_index=frame.frame_index,
                    elapsed_s=frame.elapsed_s,
                    captured_monotonic=captured,
                )
                self._scheduled_ids.add(tag_id)
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
                frame=frame.frame_index,
                elapsed_s=round(frame.elapsed_s, 6),
                confirmation_frames=self._streaks[tag_id],
            )
            print(
                json.dumps(
                    {
                        "event": "apriltag_confirmed",
                        "id": tag_id,
                        "frame": frame.frame_index,
                        "elapsed_s": round(frame.elapsed_s, 6),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            return

    def _selected_heartbeat_is_fresh(self, now: float) -> bool:
        observed = self.state.last_vehicle_heartbeat_monotonic
        return observed is not None and (
            0.0 <= now - observed <= self.config.maximum_heartbeat_age_s
        )

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

    def _command_pulse(self, trigger: ServoTrigger) -> bool:
        """Command active, then always command rest, settle, and detach."""

        completed_hold = False
        pulse_error: Exception | None = None
        self._ever_commanded = True
        active_commanded_utc: str | None = None
        active_started: float | None = None
        rest_commanded_utc: str | None = None
        rest_started: float | None = None
        detached_utc: str | None = None
        self._record_and_print(
            "servo_pulse_starting",
            trigger,
            active_us=self.config.active_pulse_us,
            rest_us=self.config.rest_pulse_us,
            requested_active_s=self.config.pulse_duration_s,
        )
        try:
            active_commanded_utc = datetime.now(UTC).isoformat()
            self._servo.value = self._active_value
            active_started = time.monotonic()
            completed_hold = not self.capture_stop.wait(self.config.pulse_duration_s)
        except Exception as error:
            pulse_error = error
        finally:
            try:
                rest_commanded_utc = datetime.now(UTC).isoformat()
                rest_started = time.monotonic()
                self._servo.value = self._rest_value
                time.sleep(self.config.settle_duration_s)
            except Exception as error:
                if pulse_error is None:
                    pulse_error = error
            try:
                self._servo.detach()
                detached_utc = datetime.now(UTC).isoformat()
            except Exception as error:
                if pulse_error is None:
                    pulse_error = error
        if active_started is not None:
            self._record_and_print(
                "servo_active_commanded",
                trigger,
                commanded_us=self.config.active_pulse_us,
                commanded_utc=active_commanded_utc,
                actual_active_s=round(
                    max(0.0, (rest_started or time.monotonic()) - active_started),
                    6,
                ),
            )
        if rest_started is not None:
            self._record_and_print(
                "servo_rest_commanded",
                trigger,
                commanded_us=self.config.rest_pulse_us,
                commanded_utc=rest_commanded_utc,
            )
        if detached_utc is not None:
            self._record_and_print(
                "servo_pwm_detached",
                trigger,
                commanded_utc=detached_utc,
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

    def close(self) -> None:
        """Cancel queued work, neutralize if used, detach, and release BCM12."""

        if self._closed:
            return
        self._closed = True
        self._restore_signal_handlers()
        self.stop_accepting()
        self._cancel_queued_triggers()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            self._cancel_queued_triggers()
            self._queue.put_nowait(None)
        self._worker.join(
            timeout=(self.config.pulse_duration_s + self.config.settle_duration_s + 2.0)
        )
        if self._worker.is_alive():
            self.state.record_error("payload servo worker did not stop boundedly")
        if self._ever_commanded:
            try:
                self._servo.value = self._rest_value
                time.sleep(self.config.settle_duration_s)
            except Exception as error:
                self.state.record_error(f"restore payload servo rest position: {error}")
        try:
            self._servo.detach()
        except Exception as error:
            self.state.record_error(f"detach payload servo PWM: {error}")
        try:
            self._servo.close()
        except Exception as error:
            self.state.record_error(f"close payload servo GPIO: {error}")
        try:
            self._events.write("servo_session_closed")
        except Exception as error:
            self.state.record_error(f"write payload servo close event: {error}")
        try:
            self._events.close()
        except Exception as error:
            self.state.record_error(f"close payload servo event log: {error}")
        self._process_lock.close()

    def manifest(self) -> dict[str, object]:
        return {
            "gpio": SERVO_GPIO_PIN,
            "feedback_available": False,
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
        }


def main() -> None:
    from ai_drone.cli.record import run

    raise SystemExit(run(operation="tag-servo"))


if __name__ == "__main__":
    main()
