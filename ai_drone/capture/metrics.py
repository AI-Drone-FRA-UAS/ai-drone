"""Bounded-cost capture delivery measurements, separate from transport byte counts."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class StreamSnapshot:
    identity: str
    messages: int
    known_packet_bytes: int
    first_received: float
    last_received: float
    maximum_receipt_gap_s: float | None


@dataclass(frozen=True)
class DeliverySnapshot:
    known_packet_bytes: int
    unknown_packet_size_messages: int
    sequence_gap_estimate: int
    duplicate_sequences: int
    reordered_or_reset_sequences: int
    unknown_sequence_messages: int
    maximum_delivery_lag_s: float | None
    streams: tuple[StreamSnapshot, ...]

    def to_dict(
        self, duration: float, *, observed_at: float | None = None
    ) -> dict[str, Any]:
        return {
            "schema": 1,
            "scope": "capture_window_decoded_messages",
            "duration_s": duration,
            "observed_monotonic": observed_at,
            "known_packet_bytes": self.known_packet_bytes,
            "packet_sizes_complete": self.unknown_packet_size_messages == 0,
            "unknown_packet_size_messages": self.unknown_packet_size_messages,
            "known_packet_bytes_per_s": self.known_packet_bytes / duration
            if duration > 0
            else None,
            "sequence_gap_estimate": self.sequence_gap_estimate,
            "duplicate_sequences": self.duplicate_sequences,
            "reordered_or_reset_sequences": self.reordered_or_reset_sequences,
            "unknown_sequence_messages": self.unknown_sequence_messages,
            "maximum_delivery_lag_s": self.maximum_delivery_lag_s,
            "streams": {
                stream.identity: {
                    "messages": stream.messages,
                    "delivered_hz": stream.messages / duration
                    if duration > 0
                    else None,
                    "known_packet_bytes": stream.known_packet_bytes,
                    "first_received_monotonic": stream.first_received,
                    "last_received_monotonic": stream.last_received,
                    "last_receipt_age_s": max(0, observed_at - stream.last_received)
                    if observed_at is not None
                    else None,
                    "maximum_receipt_gap_s": stream.maximum_receipt_gap_s,
                }
                for stream in self.streams
            },
            "limitation": "Decoded packet bytes exclude serial noise and startup. Sequence gaps estimate missing sequence numbers per source, not proven packet loss; filtering, reordering, counter wrap and resets can affect them.",
        }


class DeliveryMetrics:
    """Mutable counters protected independently of capture reporting or disk I/O."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._streams: dict[str, StreamSnapshot] = {}
        self._sequences: dict[tuple[int, int], int] = {}
        self._packet_bytes = 0
        self._unknown_sizes = 0
        self._sequence_gaps = 0
        self._duplicates = 0
        self._reordered = 0
        self._unknown_sequences = 0
        self._maximum_lag: float | None = None

    def _sequence(self, message: Any, source: tuple[int, int]) -> None:
        getter = getattr(message, "get_seq", None)
        sequence = getter() if callable(getter) else None
        if type(sequence) is not int or not 0 <= sequence <= 255:
            self._unknown_sequences += 1
            return
        previous = self._sequences.get(source)
        if previous is not None:
            delta = (sequence - previous) % 256
            if delta == 0:
                self._duplicates += 1
                return
            if delta >= 128:
                self._reordered += 1
                return
            self._sequence_gaps += delta - 1
        self._sequences[source] = sequence

    def observe(self, message: Any, *, received_at: float, delivered_at: float) -> None:
        source = (message.get_srcSystem(), message.get_srcComponent())
        identity = f"{source[0]}/{source[1]}/{message.get_type()}"
        for field in ("id", "sensor_id", "orientation"):
            value = getattr(message, field, None)
            if type(value) is int:
                identity += f"/{field}={value}"
        getter = getattr(message, "get_msgbuf", None)
        packet = getter() if callable(getter) else None
        size = (
            len(packet) if isinstance(packet, bytes | bytearray | memoryview) else None
        )
        with self._lock:
            if size is None:
                self._unknown_sizes += 1
            else:
                self._packet_bytes += size
            self._sequence(message, source)
            lag = delivered_at - received_at
            if lag >= 0:
                self._maximum_lag = (
                    lag if self._maximum_lag is None else max(lag, self._maximum_lag)
                )
            previous = self._streams.get(identity)
            if previous is None:
                current = StreamSnapshot(
                    identity, 1, size or 0, received_at, received_at, None
                )
            else:
                gap = received_at - previous.last_received
                maximum = previous.maximum_receipt_gap_s
                if gap >= 0:
                    maximum = gap if maximum is None else max(gap, maximum)
                current = StreamSnapshot(
                    identity,
                    previous.messages + 1,
                    previous.known_packet_bytes + (size or 0),
                    min(received_at, previous.first_received),
                    max(received_at, previous.last_received),
                    maximum,
                )
            self._streams[identity] = current

    def snapshot(self) -> DeliverySnapshot:
        with self._lock:
            return DeliverySnapshot(
                self._packet_bytes,
                self._unknown_sizes,
                self._sequence_gaps,
                self._duplicates,
                self._reordered,
                self._unknown_sequences,
                self._maximum_lag,
                tuple(self._streams[key] for key in sorted(self._streams)),
            )
