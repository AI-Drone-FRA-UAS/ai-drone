"""Exercise real pymavlink encoders against isolated physical-port doubles."""

from __future__ import annotations

import errno
import time
from types import SimpleNamespace
from typing import Any

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from ai_drone.flight.controller import DroneController
from ai_drone.flight.phase import Armed, Climb, cleanup
from ai_drone.mavlink.connection import (
    open_ardupilot_connection,
    require_complete_writes,
)
from ai_drone.mavlink.metrics import attach_transport_metrics
from ai_drone.mavlink.shared import SharedMavlink

KINDS = ("serial", "tcp", "tcpin", "udpout", "udpin")


class Port:
    def __init__(self, result="complete"):
        self.result = result
        self.calls = []
        self.closed = False
        self.write_timeout = None

    def write(self, data):
        self.calls.append(bytes(data))
        if isinstance(self.result, BaseException):
            raise self.result
        if self.result == "complete":
            return len(data)
        if self.result == "short":
            return len(data) - 1
        return self.result

    send = write

    def sendto(self, data, address):
        self.address = address
        return self.write(data)

    def close(self):
        self.closed = True


def raw_connection(kind, result="complete"):
    adapter = {
        "serial": mavutil.mavserial,
        "tcp": mavutil.mavtcp,
        "tcpin": mavutil.mavtcpin,
        "udpout": mavutil.mavudp,
        "udpin": mavutil.mavudp,
    }[kind]
    raw: Any = object.__new__(adapter)
    mavutil.mavfile.__init__(raw, None, "MOCK", input=False)
    raw.device = "MOCK"
    raw.autoreconnect = False
    raw.port = Port(result)
    raw.mav = mavlink.MAVLink(raw, srcSystem=255, srcComponent=190)
    raw.recv_match = lambda **_kwargs: time.sleep(0.001)
    raw.recv = lambda *_args: b""
    raw.listen = SimpleNamespace(fileno=lambda: 42, close=lambda: None)
    raw.udp_server = kind == "udpin"
    raw.clients = {("127.0.0.1", 1234)}
    raw.clients_last_alive = {("127.0.0.1", 1234): 0}
    raw.timeout = 0
    raw.last_address = None
    raw.broadcast = False
    raw.destination_addr = ("127.0.0.1", 1234)
    raw.resolved_destination_addr = "127.0.0.1"
    return raw


@pytest.mark.parametrize("kind", KINDS)
@pytest.mark.parametrize("shared", [False, True])
@pytest.mark.parametrize("result", [OSError(errno.EIO, "port failed"), "short", None])
def test_real_encoder_failure_cannot_claim_written_or_clear_cleanup(
    monkeypatch, kind, shared, result
):
    raw = raw_connection(kind, result)
    hub = SharedMavlink(raw) if shared else None
    if hub is None:
        monkeypatch.setattr(mavutil, "mavlink_connection", lambda *_args, **_kw: raw)
        connection = open_ardupilot_connection("MOCK")
    else:
        connection = hub.subscribe("control")
    drone = DroneController(device="tcp:127.0.0.1:5760")
    drone.phase = Armed()
    drone.connection = connection
    try:
        with pytest.raises(OSError):
            drone._write_command(Climb(0.3, 0))
        assert drone._command_outcomes == {"written": 0, "queued": 0, "failed": 1}
        assert cleanup(drone.phase) == "land"
        assert raw.mav.total_packets_sent == raw.mav.total_bytes_sent == 0
        assert len(raw.port.calls) == 1
        decoded = mavlink.MAVLink(None).parse_char(raw.port.calls[0])
        assert decoded is not None
        assert decoded.get_type() == "SET_ATTITUDE_TARGET"
        # Cleanup is still allowed a subsequent best-effort write; the transport
        # does not permanently latch a failure or replay a possibly partial frame.
        raw.port.result = "complete"
        connection.mav.heartbeat_send(6, 8, 0, 0, 0)
        assert raw.mav.total_packets_sent == 1
        assert len(raw.port.calls) == 2
    finally:
        if hub is not None:
            hub.close()
        else:
            raw.close()


@pytest.mark.parametrize("kind", KINDS)
def test_real_encoder_success_has_exact_unchanged_bytes_and_measured_count(kind):
    raw = raw_connection(kind)
    require_complete_writes(raw)
    first_writer = raw.write
    require_complete_writes(raw)
    assert raw.write is first_writer
    meter = attach_transport_metrics(raw)
    assert meter is not None
    expected_encoder = mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
    expected = expected_encoder.heartbeat_encode(6, 8, 0, 0, 0).pack(expected_encoder)
    raw.mav.heartbeat_send(6, 8, 0, 0, 0)
    assert raw.port.calls == [expected]
    assert raw.mav.total_bytes_sent == len(expected)
    assert meter.snapshot()["tx_bytes"] == len(expected)
    assert meter.snapshot()["tx_errors"] == 0
    meter.close()
    raw.close()


@pytest.mark.parametrize("result", [-1, 0, True, 10000])
def test_invalid_counts_raise_before_the_real_encoder_counts_success(result):
    raw = raw_connection("serial", result)
    require_complete_writes(raw)
    with pytest.raises(OSError, match="incomplete MAVLink transport write"):
        raw.mav.heartbeat_send(6, 8, 0, 0, 0)
    assert raw.mav.total_packets_sent == 0


@pytest.mark.parametrize("timeout", [None, 1, 0.01, 0])
def test_serial_write_keeps_stricter_timeouts_and_reapplies_after_reset(timeout):
    raw = raw_connection("serial")
    raw.port.write_timeout = timeout
    require_complete_writes(raw)
    raw.write(b"frame")
    assert raw.port.write_timeout == (0.05 if timeout is None else min(timeout, 0.05))
    raw.port = Port()
    raw.write(b"next")
    assert raw.port.write_timeout == 0.05


def test_serial_failure_retains_upstream_portdead_and_autoreconnect():
    raw = raw_connection("serial", OSError("serial failure"))
    raw.autoreconnect = True
    resets = []
    raw.reset = lambda: resets.append(raw.portdead)
    require_complete_writes(raw)
    with pytest.raises(OSError):
        raw.write(b"frame")
    assert raw.portdead
    assert resets == [True]


def test_tcp_disconnect_retains_reconnect_policy_and_raises_original_error():
    error = OSError(errno.EPIPE, "disconnected")
    raw = raw_connection("tcp", error)
    reconnects = []
    raw.reconnect = lambda: reconnects.append(True)
    require_complete_writes(raw)
    with pytest.raises(OSError) as raised:
        raw.write(b"frame")
    assert raised.value is error
    assert reconnects == [True]


def test_tcp_listener_disconnect_retains_listener_descriptor():
    raw = raw_connection("tcpin", OSError(errno.EPIPE, "disconnected"))
    port = raw.port
    require_complete_writes(raw)
    with pytest.raises(OSError):
        raw.write(b"frame")
    assert port.closed and raw.port is None and raw.fd == 42


@pytest.mark.parametrize("kind", ["tcp", "tcpin", "udpin"])
def test_missing_peer_fails_without_claiming_a_write(kind):
    raw = raw_connection(kind)
    if kind == "udpin":
        raw.clients.clear()
    else:
        raw.port = None
    require_complete_writes(raw)
    with pytest.raises(OSError):
        raw.mav.heartbeat_send(6, 8, 0, 0, 0)
    assert raw.mav.total_packets_sent == 0


def test_udp_fanout_does_not_fabricate_one_datagram_as_exact_total_bytes():
    raw = raw_connection("udpin")
    raw.clients.add(("127.0.0.1", 1235))
    require_complete_writes(raw)
    meter = attach_transport_metrics(raw)
    assert meter is not None
    raw.mav.heartbeat_send(6, 8, 0, 0, 0)
    assert len(raw.port.calls) == 2
    assert meter.snapshot()["tx_bytes"] is None
    assert meter.snapshot()["tx_errors"] == 0
    meter.close()


def test_unverified_real_adapter_refuses_before_calling_its_writer():
    raw: Any = mavutil.mavfile(None, "MOCK", input=False)
    raw.write = lambda _data: pytest.fail("unverified write was executed")
    require_complete_writes(raw)
    with pytest.raises(OSError, match="does not support verified physical writes"):
        raw.mav.heartbeat_send(6, 8, 0, 0, 0)
    assert raw.mav.total_packets_sent == 0


def test_shared_endpoint_retains_queued_submission_capability():
    raw = raw_connection("serial")
    require_complete_writes(raw)
    raw.write_confirmation = "queued"
    with SharedMavlink(raw) as hub:
        assert hub.subscribe("client").write_confirmation == "queued"
