"""Durable artifact writes for a companion computer that can lose power.

A sudden power loss discards whatever the SD card has not yet persisted, so
artifacts need an explicit durability policy rather than the operating system's
default write-back timing.

Two policies cover every artifact this package produces:

* Small, infrequent artifacts (manifests, parameter snapshots) are replaced
  atomically. Writing them costs one fsync each, which is irrelevant next to
  how rarely they are written, so a reader never observes partial contents.
* Append-only recording streams are written at sensor rate, where an fsync per
  record would stall capture on a Raspberry Pi SD card. They instead flush
  every record and fsync on a bounded interval, which caps how much a power cut
  can discard without slowing the recording itself.
"""

from __future__ import annotations

import math
import os
import tempfile
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import TextIO

# Bounds how much of an append-only stream a power cut may discard. Five
# seconds keeps the fsync cost per recording negligible at the bounded
# telemetry rates in `recording.TELEMETRY_RATES_HZ`.
DEFAULT_SYNC_INTERVAL_S = 5.0


def fsync_directory(path: Path) -> None:
    """Persist a directory entry so a completed rename survives a power loss.

    `os.replace` is atomic for readers, but the rename itself is only durable
    once the containing directory is synced. Directory descriptors cannot be
    opened for this purpose outside POSIX, where the rename is durable through
    the platform's own semantics instead.
    """

    if os.name != "posix":
        return
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_write_text(path: Path, content: str, *, encoding: str = "utf-8") -> None:
    """Replace one artifact so readers never observe partial contents."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding=encoding,
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as handle:
            temporary = Path(handle.name)
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = None
        fsync_directory(path.parent)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


@dataclass
class IntervalSync:
    """Bound the power-loss window of an append-only stream.

    Every record is flushed to the operating system, which is a cheap syscall
    and keeps a killed process from losing buffered records. The expensive
    fsync that reaches the SD card runs at most once per `interval_s`.
    """

    interval_s: float = DEFAULT_SYNC_INTERVAL_S
    _due_at: float | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if not math.isfinite(self.interval_s) or self.interval_s < 0:
            raise ValueError("Sync interval must be finite and non-negative")

    @property
    def periodic(self) -> bool:
        """Report whether periodic fsync is enabled for this stream."""

        return self.interval_s > 0

    def after_record(self, handle: TextIO, *, now: float | None = None) -> bool:
        """Flush one record and report whether it also reached the device."""

        handle.flush()
        if not self.periodic:
            return False
        current = time.monotonic() if now is None else now
        if self._due_at is None:
            self._due_at = current + self.interval_s
            return False
        if current < self._due_at:
            return False
        os.fsync(handle.fileno())
        self._due_at = current + self.interval_s
        return True

    def finalize(self, handle: TextIO) -> None:
        """Persist the tail of a finished stream.

        This runs once per stream, so it stays enabled even when periodic
        syncing is switched off for throughput.
        """

        handle.flush()
        os.fsync(handle.fileno())


@contextmanager
def synced_stream(path: Path, sync: IntervalSync) -> Iterator[TextIO]:
    """Open an append-only recording stream that always persists its tail.

    The writer may return from anywhere inside the block; the closing sync runs
    on every path so a completed recording is durable even when the capture
    stopped early.
    """

    with path.open("w", encoding="utf-8") as handle:
        try:
            yield handle
        finally:
            sync.finalize(handle)
