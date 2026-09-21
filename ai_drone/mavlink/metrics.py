"""Measure a MAVLink owner's actual transport returns without changing writes.

Pymavlink's encoded-byte count is not a successful-write count: its serial
adapter can return -1 while the encoder still increments total_bytes_sent.
Unknown or ambiguous transport returns therefore make exact byte totals unknown.
"""

from __future__ import annotations

import threading
import time
from typing import Any


def _counter(parser: Any, name: str) -> int | None:
    value = getattr(parser, name, None)
    return value if type(value) is int and value >= 0 else None


def _message_id(data: bytes) -> int | None:
    if len(data) >= 12 and data[0] == 0xFD:
        size = 12 + data[1] + (13 if data[2] & 1 else 0)
        return int.from_bytes(data[7:10], "little") if len(data) == size else None
    if len(data) >= 8 and data[0] == 0xFE and len(data) == 8 + data[1]:
        return data[5]
    return None


class TransportMetrics:
    """One owner-scoped, locked transport measurement interval."""

    def __init__(self, connection: Any, *, scope: str) -> None:
        self._connection = connection
        self._receive = connection.recv
        self._write = connection.write
        self._lock = threading.RLock()
        self._started = time.monotonic()
        self._scope = scope
        self._parser = connection.mav
        self._rx: int | None = 0
        self._tx: int | None = 0
        self._rx_errors = 0
        self._tx_errors = 0
        self._last: dict[str, float] = {}
        self._gaps: dict[str, float] = {}
        self._counts: dict[str, int] = {}
        self._final: dict[str, Any] | None = None
        self._baseline = {
            name: _counter(connection.mav, name)
            for name in (
                "total_bytes_received",
                "total_bytes_sent",
                "total_receive_errors",
            )
        }
        connection.recv = self.receive
        connection.write = self.write

    def receive(self, *args: Any, **kwargs: Any) -> Any:
        try:
            data = self._receive(*args, **kwargs)
        except BaseException:
            with self._lock:
                if self._final is None:
                    self._rx_errors += 1
                    self._rx = None
            raise
        # pymavlink's nonblocking socket adapters use an empty text string for
        # EAGAIN/EWOULDBLOCK. It proves zero bytes arrived; preserve the sentinel.
        if isinstance(data, str) and not data:
            return data
        with self._lock:
            if self._final is None and self._rx is not None:
                self._rx = (
                    self._rx + len(data)
                    if isinstance(data, bytes | bytearray)
                    else None
                )
        return data

    def write(self, data: Any, *args: Any, **kwargs: Any) -> Any:
        try:
            written = self._write(data, *args, **kwargs)
        except BaseException:
            with self._lock:
                if self._final is None:
                    self._tx_errors += 1
                    self._tx = None
            raise
        with self._lock:
            if self._final is None:
                self._written(data, written, time.monotonic())
        return written

    def _written(self, data: Any, written: Any, now: float) -> None:
        if not isinstance(data, bytes | bytearray) or type(written) is not int:
            self._tx = None
            return
        if not 0 <= written <= len(data):
            self._tx_errors += 1
            self._tx = None
            return
        if self._tx is not None:
            self._tx += written
        if written != len(data):
            self._tx_errors += 1
            return
        message = _message_id(bytes(data))
        kind = {0: "heartbeat", 82: "setpoint"}.get(message)
        if kind is None:
            return
        previous = self._last.get(kind)
        if previous is not None:
            self._gaps[kind] = max(self._gaps.get(kind, 0), now - previous)
        self._last[kind] = now
        self._counts[kind] = self._counts.get(kind, 0) + 1

    def _delta(self, name: str) -> int | None:
        if self._connection.mav is not self._parser:
            return None
        start = self._baseline[name]
        current = _counter(self._connection.mav, name)
        return (
            current - start
            if start is not None and current is not None and current >= start
            else None
        )

    def _snapshot(self, now: float, *, ended: bool) -> dict[str, Any]:
        return {
            "schema": 1,
            "supported": True,
            "scope": self._scope,
            "transport_type": type(self._connection).__name__,
            "started_monotonic": self._started,
            "sampled_monotonic": now,
            "ended_monotonic": now if ended else None,
            "elapsed_s": max(0, now - self._started),
            "rx_bytes": self._rx,
            "tx_bytes": self._tx,
            "rx_errors": self._rx_errors,
            "tx_errors": self._tx_errors,
            "parser_rx_bytes": self._delta("total_bytes_received"),
            "encoded_tx_bytes": self._delta("total_bytes_sent"),
            "parser_receive_errors": self._delta("total_receive_errors"),
            **{
                f"{kind}_tx_{field}": value
                for kind in ("heartbeat", "setpoint")
                for field, value in (
                    ("count", self._counts.get(kind, 0)),
                    ("gap_max_s", self._gaps.get(kind)),
                    ("age_s", now - self._last[kind] if kind in self._last else None),
                )
            },
        }

    def snapshot(self, now: float | None = None) -> dict[str, Any]:
        with self._lock:
            return (
                dict(self._final)
                if self._final is not None
                else self._snapshot(
                    time.monotonic() if now is None else now, ended=False
                )
            )

    def close(self) -> None:
        with self._lock:
            if self._final is not None:
                return
            self._final = self._snapshot(time.monotonic(), ended=True)
            if self._connection.recv == self.receive:
                self._connection.recv = self._receive
            if self._connection.write == self.write:
                self._connection.write = self._write
            if getattr(self._connection, "_ai_drone_transport_metrics", None) is self:
                del self._connection._ai_drone_transport_metrics


def attach_transport_metrics(
    connection: Any, *, scope: str = "direct_physical_connection"
) -> TransportMetrics | None:
    """Attach only to a directly owned parser/file pair, never a socket proxy."""
    existing = getattr(connection, "_ai_drone_transport_metrics", None)
    if isinstance(existing, TransportMetrics):
        return existing
    parser = getattr(connection, "mav", None)
    if (
        getattr(parser, "file", None) is not connection
        or not callable(getattr(connection, "recv", None))
        or not callable(getattr(connection, "write", None))
        or not hasattr(connection, "__dict__")
    ):
        return None
    meter = TransportMetrics(connection, scope=scope)
    connection._ai_drone_transport_metrics = meter
    return meter
