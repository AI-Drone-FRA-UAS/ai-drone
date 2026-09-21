"""Capture counters, synchronization epoch, and analyzed-frame contracts."""

from __future__ import annotations

import threading
import time
from collections import Counter
from dataclasses import dataclass, field, replace
from datetime import UTC, datetime
from typing import TYPE_CHECKING, Any, Protocol

from ai_drone.mavlink.safety import is_fresh

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from ai_drone.vision.apriltags import TagDetection


@dataclass
class CaptureState:
    """Cross-thread sink; mutations use lock and readers take coherent snapshots."""

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

    def snapshot(self) -> CaptureState:
        """Return an isolated copy of the counters from one locked observation."""
        with self.lock:
            return replace(
                self,
                telemetry_counts=self.telemetry_counts.copy(),
                vehicle_telemetry_counts=self.vehicle_telemetry_counts.copy(),
                tag_ids=self.tag_ids.copy(),
                distance_samples=self.distance_samples.copy(),
                latest_distance_m=self.latest_distance_m.copy(),
                distance_observed_monotonic=self.distance_observed_monotonic.copy(),
                errors=self.errors.copy(),
                lock=threading.RLock(),
            )

    def heartbeat_is_current(self, observed_at: float) -> bool:
        with self.lock:
            return is_fresh(observed_at, time.monotonic(), 2.5) and (
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
