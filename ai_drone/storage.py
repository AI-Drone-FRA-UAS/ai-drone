from __future__ import annotations

import math
import shutil
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path

from ai_drone.durability import IntervalSync
from ai_drone.recording import write_json_line

MIB = 1024 * 1024


@dataclass(frozen=True)
class StoragePolicy:
    reserve_mib: float = 256
    warning_mib: float = 1024
    stop_mib: float = 16
    check_interval: float = 2
    video_bytes_per_second: float = 1_000_000
    max_pause_s: float = 12

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value) and value > 0 for value in asdict(self).values()
        ):
            raise ValueError(
                "storage thresholds, intervals and rates must be finite and positive"
            )
        if not 0 < self.stop_mib < self.reserve_mib <= self.warning_mib:
            raise ValueError("storage thresholds require 0 < stop < reserve <= warning")
        if not math.isfinite(self.video_stop_bytes):
            raise ValueError("storage video margin is too large")

    @property
    def video_stop_bytes(self) -> float:
        # Cover delayed checks and encoder shutdown before spending the log reserve.
        margin = max(
            16 * MIB,
            self.video_bytes_per_second * (2 * self.check_interval + self.max_pause_s),
        )
        return self.reserve_mib * MIB + margin


@dataclass(frozen=True)
class StorageDecision:
    free_bytes: int
    warning: bool
    stop_video: bool
    stop_capture: bool
    estimated_video_seconds: float | None


def storage_decision(
    policy: StoragePolicy, free_bytes: int, *, video: bool
) -> StorageDecision:
    stop_capture = free_bytes <= policy.stop_mib * MIB
    stop_video = video and free_bytes <= policy.video_stop_bytes
    return StorageDecision(
        free_bytes=free_bytes,
        warning=free_bytes <= policy.warning_mib * MIB or stop_video or stop_capture,
        stop_video=stop_video,
        stop_capture=stop_capture,
        estimated_video_seconds=(
            round(
                max(0, free_bytes - policy.video_stop_bytes)
                / policy.video_bytes_per_second,
                1,
            )
            if video
            else None
        ),
    )


class StorageMonitor:
    def __init__(self, path: Path, policy: StoragePolicy, sync: IntervalSync) -> None:
        self.path, self.policy, self.sync = path, policy, sync
        self.last: StorageDecision | None = None
        self.minimum_free_bytes: int | None = None
        self.next_check = 0.0
        self.video_stopped_reason: str | None = None
        self.handle = path.open("w", encoding="utf-8")

    def event(self, name: str, **fields: object) -> None:
        write_json_line(
            self.handle,
            {
                "timestamp_utc": datetime.now(UTC).isoformat(),
                "event": name,
                **fields,
            },
        )
        self.sync.after_record(self.handle)

    def check(
        self, now: float, *, video: bool, force: bool = False
    ) -> StorageDecision | None:
        if not force and now < self.next_check:
            return None
        current = storage_decision(
            self.policy, shutil.disk_usage(self.path.parent).free, video=video
        )
        self.next_check = now + self.policy.check_interval
        self.minimum_free_bytes = min(
            current.free_bytes,
            current.free_bytes
            if self.minimum_free_bytes is None
            else self.minimum_free_bytes,
        )
        self.event("storage_sample", **asdict(current))
        if current.warning and (self.last is None or not self.last.warning):
            self.event("storage_warning", free_bytes=current.free_bytes)
        self.last = current
        estimate = (
            f", estimated video remaining={current.estimated_video_seconds:.0f}s"
            if current.estimated_video_seconds is not None
            else ""
        )
        print(
            f"Storage{' WARNING' if current.warning else ''}: {current.free_bytes / MIB:.0f} MiB free{estimate}",
            flush=True,
        )
        return current

    def video_stopped(self, *, was_recording: bool) -> None:
        self.video_stopped_reason = "storage_reserve"
        self.event(
            "video_stopped_storage" if was_recording else "video_skipped_storage"
        )
        print(
            "Video disabled to preserve log space; camera analysis and logs continue.",
            flush=True,
        )

    def manifest(self) -> dict[str, object]:
        return {
            **asdict(self.policy),
            "video_stop_bytes": self.policy.video_stop_bytes,
            "minimum_free_bytes": self.minimum_free_bytes,
            "last_sample": asdict(self.last) if self.last is not None else None,
            "video_stopped_reason": self.video_stopped_reason,
        }

    def close(self) -> None:
        try:
            self.sync.finalize(self.handle)
        finally:
            self.handle.close()
