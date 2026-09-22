"""Capture counters, synchronization epoch, and analyzed-frame contracts."""

from __future__ import annotations

import threading
import time
from collections import Counter
from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import UTC, datetime
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Protocol

from ai_drone.capture.metrics import DeliveryMetrics, DeliverySnapshot
from ai_drone.mavlink.safety import is_fresh

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from ai_drone.vision.apriltags import TagDetection


@dataclass(frozen=True)
class CaptureSnapshot:
    """One immutable observation; contains neither worker resources nor locks."""

    delivery: DeliverySnapshot
    telemetry_counts: Mapping[str, int]
    vehicle_telemetry_counts: Mapping[str, int]
    camera_frames: int
    processed_frames: int
    dropped_analysis_frames: int
    tag_detections: int
    tag_ids: Mapping[int, int]
    distance_samples: Mapping[int, int]
    latest_distance_m: Mapping[int, float]
    distance_observed_monotonic: Mapping[int, float]
    legacy_range_samples: int
    latest_legacy_range_m: float | None
    legacy_range_observed_monotonic: float | None
    optical_flow_samples: int
    latest_flow_quality: int | None
    flow_observed_monotonic: float | None
    saw_armed: bool
    saw_disarmed_after_arm: bool
    last_vehicle_state: str | None
    last_vehicle_heartbeat_monotonic: float | None
    visible_tag_ids: tuple[int, ...]
    confirmed_tag_ids: tuple[int, ...]
    pending_servo_tag_ids: tuple[int, ...]
    completed_servo_tag_ids: tuple[int, ...]
    servo_pulses_completed: int
    stop_reason: str | None
    armed_abort: bool
    worker_error: str | None
    errors: tuple[str, ...]

    def snapshot(self) -> CaptureSnapshot:
        return self


@dataclass
class CaptureState:
    """Cross-thread sink; mutations use lock and readers take coherent snapshots."""

    delivery: DeliveryMetrics = field(default_factory=DeliveryMetrics, repr=False)
    telemetry_counts: Counter[str] = field(default_factory=Counter)
    vehicle_telemetry_counts: Counter[str] = field(default_factory=Counter)
    camera_frames: int = 0
    processed_frames: int = 0
    dropped_analysis_frames: int = 0
    tag_detections: int = 0
    tag_ids: Counter[int] = field(default_factory=Counter)
    distance_samples: Counter[int] = field(default_factory=Counter)
    latest_distance_m: dict[int, float] = field(default_factory=dict)
    distance_observed_monotonic: dict[int, float] = field(default_factory=dict)
    legacy_range_samples: int = 0
    latest_legacy_range_m: float | None = None
    legacy_range_observed_monotonic: float | None = None
    optical_flow_samples: int = 0
    latest_flow_quality: int | None = None
    flow_observed_monotonic: float | None = None
    saw_armed: bool = False
    saw_disarmed_after_arm: bool = False
    last_vehicle_state: str | None = None
    last_vehicle_heartbeat_monotonic: float | None = None
    visible_tag_ids: tuple[int, ...] = ()
    confirmed_tag_ids: tuple[int, ...] = ()
    pending_servo_tag_ids: tuple[int, ...] = ()
    completed_servo_tag_ids: tuple[int, ...] = ()
    servo_pulses_completed: int = 0
    stop_reason: str | None = None
    armed_abort: bool = False
    worker_error: str | None = None
    errors: list[str] = field(default_factory=list)
    lock: threading.RLock = field(
        default_factory=threading.RLock,
        repr=False,
    )
    disarmed_heartbeat: threading.Event = field(
        default_factory=threading.Event,
        repr=False,
    )
    vehicle_heartbeat: threading.Event = field(
        default_factory=threading.Event,
        repr=False,
    )

    def record_error(self, message: str) -> None:
        """Retain the first capture or cleanup error across all worker threads."""

        with self.lock:
            self.errors.append(message)
            if self.worker_error is None:
                self.worker_error = message

    def snapshot(self) -> CaptureSnapshot:
        """Freeze all counters and status from one locked observation."""
        with self.lock:
            return CaptureSnapshot(
                delivery=self.delivery.snapshot(),
                telemetry_counts=MappingProxyType(self.telemetry_counts.copy()),
                vehicle_telemetry_counts=MappingProxyType(
                    self.vehicle_telemetry_counts.copy()
                ),
                camera_frames=self.camera_frames,
                processed_frames=self.processed_frames,
                dropped_analysis_frames=self.dropped_analysis_frames,
                tag_detections=self.tag_detections,
                tag_ids=MappingProxyType(self.tag_ids.copy()),
                distance_samples=MappingProxyType(self.distance_samples.copy()),
                latest_distance_m=MappingProxyType(self.latest_distance_m.copy()),
                distance_observed_monotonic=MappingProxyType(
                    self.distance_observed_monotonic.copy()
                ),
                legacy_range_samples=self.legacy_range_samples,
                latest_legacy_range_m=self.latest_legacy_range_m,
                legacy_range_observed_monotonic=self.legacy_range_observed_monotonic,
                optical_flow_samples=self.optical_flow_samples,
                latest_flow_quality=self.latest_flow_quality,
                flow_observed_monotonic=self.flow_observed_monotonic,
                saw_armed=self.saw_armed,
                saw_disarmed_after_arm=self.saw_disarmed_after_arm,
                last_vehicle_state=self.last_vehicle_state,
                last_vehicle_heartbeat_monotonic=self.last_vehicle_heartbeat_monotonic,
                visible_tag_ids=self.visible_tag_ids,
                confirmed_tag_ids=self.confirmed_tag_ids,
                pending_servo_tag_ids=self.pending_servo_tag_ids,
                completed_servo_tag_ids=self.completed_servo_tag_ids,
                servo_pulses_completed=self.servo_pulses_completed,
                stop_reason=self.stop_reason,
                armed_abort=self.armed_abort,
                worker_error=self.worker_error,
                errors=tuple(self.errors),
            )

    def heartbeat_is_current(self, observed_at: float) -> bool:
        with self.lock:
            return is_fresh(observed_at, time.monotonic(), 100.0) and (
                self.last_vehicle_heartbeat_monotonic is None
                or observed_at > self.last_vehicle_heartbeat_monotonic
            )

    def observe_vehicle_state(
        self, *, armed: bool, observed_at: float | None = None
    ) -> bool:
        """Track selected-vehicle arm transitions from the telemetry worker."""

        with self.lock:
            observed_at = time.monotonic() if observed_at is None else observed_at
            if not self.heartbeat_is_current(observed_at):
                return False
            self.last_vehicle_heartbeat_monotonic = observed_at
            self.vehicle_heartbeat.set()
            if armed:
                self.disarmed_heartbeat.clear()
                self.saw_armed = True
                self.last_vehicle_state = "armed"
            else:
                if self.saw_armed:
                    self.saw_disarmed_after_arm = True
                self.last_vehicle_state = "disarmed"
                self.disarmed_heartbeat.set()
            return True

    def set_stop_reason(self, reason: str) -> None:
        """Retain the first intentional or error stop reason."""

        with self.lock:
            if self.stop_reason is None:
                self.stop_reason = reason


class CaptureWindow:
    """One synchronized video, telemetry, and analysis capture epoch."""

    def __init__(self, duration: float | None) -> None:
        self.duration = duration
        self.started = threading.Event()
        self.ready = threading.Event()
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
            self.deadline = (
                None if self.duration is None else started_monotonic + self.duration
            )
            self.started.set()

    def require_started(self) -> tuple[float, datetime, float | None]:
        """Return the established epoch or fail if encoding never began."""

        if self.started_monotonic is None or self.started_utc is None:
            raise RuntimeError("capture epoch was not established")
        return self.started_monotonic, self.started_utc, self.deadline


@dataclass(frozen=True)
class AnalysisFrame:
    """One camera frame queued for asynchronous AprilTag analysis."""

    frame_index: int
    elapsed_s: float
    grayscale: NDArray[np.uint8]
    metadata: dict[str, object]
    captured_monotonic: float | None = None


class DetectionObserver(Protocol):
    """Optional active-mode hook called for each analyzed camera frame."""

    def observe(
        self,
        frame: AnalysisFrame,
        detections: list[TagDetection],
        tag_records: list[dict[str, Any]],
    ) -> None: ...
