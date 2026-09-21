"""Physical-byte accounting uses only mocked transport return values."""

from __future__ import annotations

import errno
from types import SimpleNamespace
from typing import Any

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from ai_drone.mavlink import metrics


class Wire:
    def __init__(self):
        self.incoming = b""
        self.result = "complete"
        self.packets = []
        self.mav = mavlink.MAVLink(self)
        self.mav.robust_parsing = True

    def recv(self):
        return self.incoming

    def write(self, data):
        self.packets.append(bytes(data))
        if isinstance(self.result, BaseException):
            raise self.result
        return len(data) if self.result == "complete" else self.result


@pytest.fixture
def measured(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(metrics.time, "monotonic", lambda: clock[0])
    wire = Wire()
    meter = metrics.attach_transport_metrics(wire)
    assert meter is not None
    yield wire, meter, clock
    meter.close()


def heartbeat(wire):
    wire.mav.heartbeat_send(
        mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 4
    )


def test_transport_bytes_are_actual_returns_and_encoder_counts_are_separate(measured):
    wire, meter, _ = measured
    wire.incoming = b"invalid"
    wire.mav.parse_char(wire.recv())
    heartbeat(wire)
    wire.result = 4
    heartbeat(wire)
    result = meter.snapshot()
    assert result["rx_bytes"] == result["parser_rx_bytes"] == len(b"invalid")
    assert result["tx_bytes"] == len(wire.packets[0]) + 4
    assert result["encoded_tx_bytes"] == sum(map(len, wire.packets))
    assert result["tx_errors"] == 1
    assert result["heartbeat_tx_count"] == 1
    assert result["heartbeat_tx_gap_max_s"] is None


@pytest.mark.parametrize("returned", [None, -1, True, 10000])
def test_unknown_or_impossible_write_return_never_fabricates_bytes(measured, returned):
    wire, meter, _ = measured
    wire.result = returned
    heartbeat(wire)
    result = meter.snapshot()
    assert result["tx_bytes"] is None
    assert result["encoded_tx_bytes"] == len(wire.packets[0])
    assert result["heartbeat_tx_count"] == 0


def test_write_exception_propagates_and_retains_unknown_completion(measured):
    wire, meter, _ = measured
    wire.result = OSError("serial write failed")
    with pytest.raises(OSError, match="serial write failed"):
        heartbeat(wire)
    assert meter.snapshot()["tx_errors"] == 1
    assert meter.snapshot()["tx_bytes"] is None


def test_completed_gap_and_current_silence_are_both_reported(measured):
    wire, meter, clock = measured
    heartbeat(wire)
    clock[0] = 11.2
    heartbeat(wire)
    wire.mav.set_attitude_target_send(0, 1, 1, 7, [1, 0, 0, 0], 0, 0, 0, 0.5)
    clock[0] = 11.25
    wire.mav.set_attitude_target_send(0, 1, 1, 7, [1, 0, 0, 0], 0, 0, 0, 0.5)
    clock[0] = 14
    result = meter.snapshot()
    assert result["heartbeat_tx_gap_max_s"] == pytest.approx(1.2)
    assert result["heartbeat_tx_age_s"] == pytest.approx(2.8)
    assert result["setpoint_tx_gap_max_s"] == pytest.approx(0.05)
    assert result["setpoint_tx_age_s"] == pytest.approx(2.75)


def test_close_restores_transport_and_freezes_measurement_interval(measured):
    wire, meter, clock = measured
    heartbeat(wire)
    clock[0] = 12
    meter.close()
    snapshot = meter.snapshot()
    clock[0] = 20
    heartbeat(wire)
    meter.close()
    assert meter.snapshot() == snapshot
    assert snapshot["ended_monotonic"] == 12
    assert snapshot["elapsed_s"] == 2
    assert snapshot["heartbeat_tx_count"] == 1
    assert wire.write.__func__ is Wire.write


def test_remote_or_unknown_transport_is_not_reported_as_zero_bytes():
    assert (
        metrics.attach_transport_metrics(SimpleNamespace(mav=SimpleNamespace())) is None
    )


def test_repeated_attachment_does_not_double_count(measured):
    wire, meter, _ = measured
    assert metrics.attach_transport_metrics(wire) is meter
    heartbeat(wire)
    assert meter.snapshot()["tx_bytes"] == len(wire.packets[0])


def test_parser_replacement_cannot_reset_a_measurement_to_apparent_zero(measured):
    wire, meter, _ = measured
    heartbeat(wire)
    wire.mav = mavlink.MAVLink(wire)
    snapshot = meter.snapshot()
    assert snapshot["encoded_tx_bytes"] is None
    assert snapshot["parser_rx_bytes"] is None
    assert snapshot["tx_bytes"] == len(wire.packets[0])


def test_real_tcp_idle_sentinel_preserves_exact_received_byte_total():
    responses = iter([b"first", BlockingIOError(errno.EAGAIN, "idle"), b"next"])

    def receive(_size):
        value = next(responses)
        if isinstance(value, OSError):
            raise value
        return value

    raw: Any = object.__new__(mavutil.mavtcp)
    mavutil.mavfile.__init__(raw, None, "MOCK", input=False)
    raw.port = SimpleNamespace(recv=receive)
    meter = metrics.attach_transport_metrics(raw)
    assert meter is not None
    try:
        assert raw.recv(16) == b"first"
        assert raw.recv(16) == ""  # The real pinned adapter catches EAGAIN.
        assert meter.snapshot()["rx_bytes"] == len(b"first")
        assert raw.recv(16) == b"next"
        assert meter.snapshot()["rx_bytes"] == len(b"firstnext")
        assert meter.snapshot()["rx_errors"] == 0
    finally:
        meter.close()


def test_nonempty_text_receive_remains_unknown_after_idle_poll(measured):
    wire, meter, _ = measured
    wire.incoming = "unknown encoding"
    assert wire.recv() == "unknown encoding"
    wire.incoming = ""
    assert wire.recv() == ""
    wire.incoming = b"next"
    assert wire.recv() == b"next"
    assert meter.snapshot()["rx_bytes"] is None
