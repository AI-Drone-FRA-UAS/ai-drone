"""Flight artifacts on a dedicated MAVLink subscription, isolated from control."""

from __future__ import annotations

import json
import logging
import queue
import shutil
import threading
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from ai_drone.durability import IntervalSync, atomic_write_text
from ai_drone.flight.dataflash import DataFlashLog
from ai_drone.flight.provenance import flight_code_sha256
from ai_drone.recording import json_safe, request_telemetry_messages, write_json_line
from ai_drone.settings import RecordingSettings, load_settings
from ai_drone.storage import MIB

_LOG = logging.getLogger(__name__)


class FlightRecorder:
    def __init__(
        self,
        connection: Any,
        metadata: dict[str, Any],
        root: Path | None = None,
        *,
        settings: RecordingSettings | None = None,
    ) -> None:
        """Take ownership of a recording endpoint, never the controller endpoint."""
        self.settings = settings or load_settings().recording
        now = datetime.now(UTC)
        self.connection = connection
        self.metadata = metadata
        self.root = root or Path("artifacts/flights") / now.strftime(
            "%Y%m%dT%H%M%S.%fZ"
        )
        self.root.mkdir(parents=True)
        self.started_at = now
        self._ended_at: datetime | None = None
        self.started = time.monotonic()
        self.error: str | None = None
        self.recording_error: str | None = None
        self.completed = False
        self.code_sha256 = flight_code_sha256()
        self.dataflash_log: DataFlashLog | None = None
        self.requested_messages: list[str] = []
        self._sync = IntervalSync()
        self._stop = threading.Event()
        self._failed = threading.Event()
        self._closed = threading.Event()
        self._lock = threading.Lock()
        self._pending: queue.Queue[dict] = queue.Queue(maxsize=256)
        self._thread: threading.Thread | None = None
        self._events = None
        try:
            self._check_storage()
            self._events = (self.root / "events.jsonl").open("w", encoding="utf-8")
            connection.setup_logfile(str(self.root / "telemetry.tlog"))
            self.requested_messages = request_telemetry_messages(connection)
            self._write_event(self._event("recording_started"))
            self._write_manifest()
            self._thread = threading.Thread(
                target=self._drain, name="flight-recording", daemon=True
            )
            self._thread.start()
        except BaseException as error:
            self._failure(error)
            self._finalize()
            raise

    def _event(self, name: str, **fields: Any) -> dict:
        return {
            "elapsed_s": round(time.monotonic() - self.started, 6),
            "event": name,
            "timestamp_utc": datetime.now(UTC).isoformat(),
            **fields,
        }

    def _write_event(self, value: dict) -> None:
        assert self._events is not None
        write_json_line(self._events, value)
        self._sync.after_record(self._events)

    def _failure(self, error: BaseException) -> None:
        with self._lock:
            if self.recording_error is not None:
                return
            self.recording_error = f"{type(error).__name__}: {error}"
            self.error = self.error or self.recording_error
            self.completed = False
            self._failed.set()
        _LOG.warning(
            "Flight recording disabled; flight control continues: %s",
            self.recording_error,
        )

    def _check_storage(self) -> None:
        if shutil.disk_usage(self.root).free <= self.settings.storage_stop_mib * MIB:
            raise RuntimeError(
                "storage floor reached; preserving space for flight control"
            )

    def _write_pending(self) -> None:
        # The bounded queue prevents event producers from starving telemetry drain.
        for _ in range(self._pending.maxsize):
            try:
                value = self._pending.get_nowait()
            except queue.Empty:
                break
            self._write_event(value)

    def _drain(self) -> None:
        next_storage = time.monotonic() + self.settings.storage_check_interval
        try:
            while not self._stop.is_set() and not self._failed.is_set():
                self._write_pending()
                now = time.monotonic()
                if now >= next_storage:
                    self._check_storage()
                    next_storage = now + self.settings.storage_check_interval
                # Logging is performed by this endpoint's independent broker sink.
                self.connection.recv_match(blocking=True, timeout=0.05)
        except Exception as error:
            self._failure(error)
        finally:
            self._finalize()

    def event(self, name: str, **fields: Any) -> None:
        if self._stop.is_set() or self._failed.is_set() or self._closed.is_set():
            return
        try:
            self._pending.put_nowait(self._event(name, **fields))
        except queue.Full:
            self._failure(
                RuntimeError("flight event queue overflow; log is incomplete")
            )

    def finish(self, error: BaseException | None = None) -> None:
        with self._lock:
            if error is not None:
                self.error = self.error or f"{type(error).__name__}: {error}"
            self.completed = error is None and self.error is None
        self.event("completed" if self.completed else "failed", error=self.error)

    def set_dataflash_log(self, log: DataFlashLog | None) -> None:
        self.dataflash_log = log

    def _finalize(self) -> None:
        try:
            if self.recording_error is None:
                self._write_pending()
        except Exception as error:
            self._failure(error)
        if self._events is not None:
            try:
                self._sync.finalize(self._events)
            except Exception as error:
                self._failure(error)
            finally:
                try:
                    self._events.close()
                except Exception as error:
                    self._failure(error)
        try:
            # Shared endpoints stop and flush only their own sink, with a bound.
            self.connection.close(timeout=2)
        except Exception as error:
            self._failure(error)
        self._ended_at = datetime.now(UTC)
        try:
            self._write_manifest()
        except Exception as error:
            self._failure(error)
        finally:
            self._closed.set()

    def _write_manifest(self) -> None:
        manifest = {
            "schema": 1,
            "started_utc": self.started_at.isoformat(),
            "ended_utc": self._ended_at.isoformat() if self._ended_at else None,
            "duration_s": round(time.monotonic() - self.started, 6),
            "completed": self.completed,
            "code_sha256": self.code_sha256,
            "dataflash_log": self.dataflash_log,
            "error": self.error,
            "recording_error": self.recording_error,
            "metadata": self.metadata,
            "requested_messages": self.requested_messages,
            "files": {"events": "events.jsonl", "telemetry": "telemetry.tlog"},
        }
        atomic_write_text(
            self.root / "manifest.json",
            json.dumps(json_safe(manifest), indent=2, sort_keys=True) + "\n",
        )

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.ident is not None:
            self._thread.join(timeout=2.5)
            if self._thread.is_alive():
                self._failure(
                    TimeoutError(
                        "flight recording cleanup did not finish within 2.5 seconds"
                    )
                )
