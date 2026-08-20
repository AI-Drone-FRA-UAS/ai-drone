from __future__ import annotations

import json
import os
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from ai_drone.durability import IntervalSync, atomic_write_text
from ai_drone.flight.dataflash import DataFlashLog
from ai_drone.flight.provenance import flight_code_sha256
from ai_drone.recording import json_safe, request_telemetry_messages, write_json_line


class FlightRecorder:
    def __init__(
        self, connection: Any, metadata: dict[str, Any], root: Path | None = None
    ) -> None:
        now = datetime.now(UTC)
        self.connection = connection
        self.metadata = metadata
        self.root = root or Path("artifacts/flights") / now.strftime(
            "%Y%m%dT%H%M%S.%fZ"
        )
        self.root.mkdir(parents=True)
        self.started_at = now
        self.started = time.monotonic()
        self.error: str | None = None
        self.completed = False
        self.code_sha256 = flight_code_sha256()
        self.dataflash_log: DataFlashLog | None = None
        self.requested_messages: list[str] = []
        self._sync = IntervalSync()
        self._events = (self.root / "events.jsonl").open("w", encoding="utf-8")
        try:
            connection.setup_logfile(str(self.root / "telemetry.tlog"))
            self.requested_messages = request_telemetry_messages(connection)
            self.event("recording_started")
        except BaseException:
            self.close()
            raise

    def event(self, name: str, **fields: Any) -> None:
        write_json_line(
            self._events,
            {
                "elapsed_s": round(time.monotonic() - self.started, 6),
                "event": name,
                "timestamp_utc": datetime.now(UTC).isoformat(),
                **fields,
            },
        )
        self._sync.after_record(self._events)

    def finish(self, error: BaseException | None = None) -> None:
        self.error = None if error is None else f"{type(error).__name__}: {error}"
        self.completed = error is None
        self.event("completed" if self.completed else "failed", error=self.error)

    def set_dataflash_log(self, log: DataFlashLog | None) -> None:
        self.dataflash_log = log

    def close(self) -> None:
        if self._events.closed:
            return
        self._sync.finalize(self._events)
        self._events.close()
        logfile = getattr(self.connection, "logfile", None)
        if logfile is not None:
            logfile.flush()
            os.fsync(logfile.fileno())
            logfile.close()
            self.connection.logfile = None
        manifest = {
            "schema": 1,
            "started_utc": self.started_at.isoformat(),
            "ended_utc": datetime.now(UTC).isoformat(),
            "duration_s": round(time.monotonic() - self.started, 6),
            "completed": self.completed,
            "code_sha256": self.code_sha256,
            "dataflash_log": self.dataflash_log,
            "error": self.error,
            "metadata": self.metadata,
            "requested_messages": self.requested_messages,
            "files": {"events": "events.jsonl", "telemetry": "telemetry.tlog"},
        }
        atomic_write_text(
            self.root / "manifest.json",
            json.dumps(json_safe(manifest), indent=2, sort_keys=True) + "\n",
        )
