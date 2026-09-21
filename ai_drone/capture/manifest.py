"""Version-one capture manifests, independent of resource-owning sessions."""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ComponentRecord:
    status: str
    observations: Mapping[str, object]

    @classmethod
    def from_dict(cls, value: Mapping[str, object]) -> ComponentRecord:
        status = value.get("status")
        if not isinstance(status, str):
            raise ValueError("manifest component status must be text")
        return cls(
            status, {key: item for key, item in value.items() if key != "status"}
        )

    def to_dict(self) -> dict[str, object]:
        return {"status": self.status, **self.observations}


@dataclass(frozen=True)
class ArtifactFiles:
    """Named artifact paths; only present files become historical relative names."""

    video: Path
    video_timestamps: Path
    camera_events: Path
    telemetry_tlog: Path
    telemetry_events: Path
    actuation_events: Path
    storage_events: Path
    first_frame: Path
    last_frame: Path

    def to_dict(self) -> dict[str, str]:
        entries = (
            ("video", self.video),
            ("video_timestamps", self.video_timestamps),
            ("camera_events", self.camera_events),
            ("telemetry_tlog", self.telemetry_tlog),
            ("telemetry_events", self.telemetry_events),
            ("actuation_events", self.actuation_events),
            ("storage_events", self.storage_events),
            ("first_frame", self.first_frame),
            ("last_frame", self.last_frame),
        )
        return {name: path.name for name, path in entries if path.exists()}


@dataclass(frozen=True)
class Manifest:
    operation: str
    requested_duration_s: float | None
    actual_duration_s: float
    started_utc: str | None
    ended_utc: str | None
    completed: bool
    armed_abort: bool
    error: str | None
    errors: tuple[str, ...]
    stop_reason: str | None
    components: Mapping[str, ComponentRecord]
    storage: Mapping[str, object] | None
    safety: Mapping[str, object]
    camera: Mapping[str, object]
    telemetry: Mapping[str, object]
    tag_servo: Mapping[str, object] | None
    files: ArtifactFiles

    def to_dict(self) -> dict[str, Any]:
        """Keep schema keys explicit; do not deep-copy locks or stringify resources."""
        return {
            "schema": 1,
            "operation": self.operation,
            "requested_duration_s": self.requested_duration_s,
            "actual_duration_s": self.actual_duration_s,
            "started_utc": self.started_utc,
            "ended_utc": self.ended_utc,
            "completed": self.completed,
            "armed_abort": self.armed_abort,
            "error": self.error,
            **({"errors": list(self.errors)} if self.errors else {}),
            "stop_reason": self.stop_reason,
            "components": {
                name: record.to_dict() for name, record in self.components.items()
            },
            "storage": None if self.storage is None else dict(self.storage),
            "safety": dict(self.safety),
            "camera": dict(self.camera),
            "telemetry": dict(self.telemetry),
            "tag_servo": None if self.tag_servo is None else dict(self.tag_servo),
            "files": self.files.to_dict(),
        }

    def to_json(self) -> str:
        # Unlike telemetry normalization, a manifest must reject accidental
        # non-finite values/resources so finalization can publish an honest failure.
        return (
            json.dumps(self.to_dict(), indent=2, sort_keys=True, allow_nan=False) + "\n"
        )
