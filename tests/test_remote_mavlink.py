from __future__ import annotations

import json
import socket
import time
from contextlib import closing
from itertools import count
from pathlib import Path

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from ai_drone.mavlink import remote
from ai_drone.mavlink.shared import received_monotonic


@pytest.fixture
def wires():
    first, second = socket.socketpair()
    writer, reader = remote.JsonSocket(first), remote.JsonSocket(second)
    try:
        yield writer, reader
    finally:
        writer.close()
        reader.close()


@pytest.fixture
def client(monkeypatch):
    local, peer = socket.socketpair()
    transport = remote.JsonSocket(local)
    monkeypatch.setattr(remote, "connect_socket", lambda _path: transport)
    connection = remote.RemoteConnection("unused-local-socket")
    server = remote.JsonSocket(peer)
    try:
        yield connection, server
    finally:
        connection.close()
        server.close()


def _heartbeat(mode: int = 5, *, armed: bool = True):
    return mavlink.MAVLink_heartbeat_message(
        mavlink.MAV_TYPE_QUADROTOR,
        mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        | (mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0),
        mode,
        mavlink.MAV_STATE_ACTIVE,
        3,
    )


def _packet(message, *, system: int = 1, component: int = 1):
    encoder = mavlink.MAVLink(None, srcSystem=system, srcComponent=component)
    return bytes(message.pack(encoder))


def _record(
    message=None,
    *,
    received: float = 75.5,
    timestamp: float = 1_760_000_000.25,
    **source,
):
    return {
        "packet": _packet(message or _heartbeat(), **source).hex(),
        "received_monotonic": received,
        "timestamp": timestamp,
    }


def _raw_json(wire, value):
    wire.socket.sendall(json.dumps(value).encode() + b"\n")


def test_arguments_use_json_and_recover_binary_parameter_identifiers():
    original = {
        "args": (1, 1, b"ARMING_SKIPCHK\x00", -1),
        "kwargs": {"force_mavlink1": False, "payload": bytearray(b"\x00\xff")},
    }
    encoded = remote.wire_record(remote.encode_value(original))
    decoded = remote.decode_value(json.loads(encoded))
    assert decoded == {
        "args": [1, 1, b"ARMING_SKIPCHK\x00", -1],
        "kwargs": {"force_mavlink1": False, "payload": b"\x00\xff"},
    }
    assert encoded.startswith(b'{"args":')
    with pytest.raises(ValueError, match="unsupported MAVLink argument"):
        remote.encode_value(object())


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_outgoing_json_rejects_nonfinite_numbers(value):
    with pytest.raises(ValueError):
        remote.wire_record({"number": value})


def test_wire_record_enforces_the_whole_record_limit():
    overhead = len(remote.wire_record({"payload": ""}))
    assert (
        len(remote.wire_record({"payload": "x" * (remote.MAX_RECORD - overhead)}))
        == remote.MAX_RECORD
    )
    with pytest.raises(ValueError, match="too large"):
        remote.wire_record({"payload": "x" * (remote.MAX_RECORD - overhead + 1)})


def test_socket_preserves_fragmented_and_multiple_records(wires):
    writer, reader = wires
    writer.socket.sendall(b'{"part":')
    assert reader.receive(0) is None
    writer.socket.sendall(b'1}\n{"part":2}\n')
    assert reader.receive(0.1) == {"part": 1}
    assert reader.receive(0) == {"part": 2}
    assert reader.receive(0) is None


@pytest.mark.parametrize(
    "line",
    [b"[]\n", b"null\n", b"not-json\n", b'{"value":NaN}\n', b'{"value":Infinity}\n'],
)
def test_socket_rejects_invalid_json_records(wires, line):
    writer, reader = wires
    writer.socket.sendall(line)
    with pytest.raises(ValueError):
        reader.receive(0.1)


@pytest.mark.parametrize("terminated", [False, True])
def test_socket_rejects_oversized_records(wires, terminated):
    writer, reader = wires
    payload = b"x" * (remote.MAX_RECORD + 1) + (b"\n" if terminated else b"")
    writer.socket.sendall(payload)
    with pytest.raises(ValueError, match="size limit"):
        reader.receive(0.1)


def test_socket_reports_server_error_and_peer_disconnect(wires):
    writer, reader = wires
    writer.send({"error": "subscription overflow"})
    with pytest.raises(RuntimeError, match="subscription overflow"):
        reader.receive(0.1)
    writer.close()
    with pytest.raises(ConnectionError, match="disconnected"):
        reader.receive(0.1)


def test_unix_connection_and_close_leave_listener_usable(tmp_path):
    path = tmp_path / "vehicle.sock"
    with closing(socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)) as listener:
        listener.bind(str(path))
        listener.listen(1)
        client = remote.connect_socket(path)
        peer, _address = listener.accept()
        with closing(peer):
            client.send({"request": "status"})
            assert json.loads(peer.recv(4096)) == {"request": "status"}
            client.close()
            client.close()
            assert peer.recv(1) == b""
            assert listener.fileno() >= 0


def test_remote_send_encodes_bytes_without_python_object_transport(client):
    connection, peer = client
    connection.mav.param_request_read_send(1, 1, b"ARMING_SKIPCHK", -1)
    request = peer.receive(0.1)
    assert request == {
        "send": "param_request_read_send",
        "args": [1, 1, {"bytes": "41524d494e475f534b495043484b"}, -1],
        "kwargs": {},
    }
    assert remote.decode_value(request["args"])[2] == b"ARMING_SKIPCHK"
    with pytest.raises(AttributeError):
        connection.mav.parse_char(b"unsafe")


def test_remote_preserves_packet_order_receipt_times_and_selected_flightmode(client):
    connection, peer = client
    peer.send(_record(_heartbeat(5), received=10.0))
    peer.send(_record(_heartbeat(9), system=2, received=11.0))
    peer.send(_record(_heartbeat(2), received=12.0))
    assert connection.flightmode is None
    first = connection.recv_match()
    assert first.custom_mode == 5
    assert received_monotonic(first) == 10.0
    assert connection.flightmode == "LOITER"
    assert connection.recv_match().get_srcSystem() == 2
    assert connection.flightmode == "LOITER"
    last = connection.recv_match()
    assert last.custom_mode == 2
    assert last._timestamp == 1_760_000_000.25
    assert received_monotonic(last) == 12.0
    assert connection.flightmode == "ALT_HOLD"


def test_other_component_or_nonvehicle_heartbeat_cannot_change_flightmode(client):
    connection, peer = client
    peer.send(_record(_heartbeat(5)))
    peer.send(_record(_heartbeat(9), component=42))
    gcs = mavlink.MAVLink_heartbeat_message(
        mavlink.MAV_TYPE_GCS,
        mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavlink.MAV_STATE_ACTIVE,
        3,
    )
    peer.send(_record(gcs))
    for _ in range(3):
        assert connection.recv_match() is not None
        assert connection.flightmode == "LOITER"


def test_remote_filter_and_logging_keep_intermediate_packets(client, tmp_path):
    connection, peer = client
    path = tmp_path / "telemetry.tlog"
    connection.setup_logfile(path)
    peer.send(_record(mavlink.MAVLink_attitude_message(100, 0, 0, 0, 0, 0, 0)))
    peer.send(_record(_heartbeat(5)))
    peer.send(_record(mavlink.MAVLink_attitude_message(200, 0, 0, 0, 0, 0, 0)))
    assert connection.recv_match(type=["HEARTBEAT"]).get_type() == "HEARTBEAT"
    assert connection.recv_match(type="ATTITUDE").time_boot_ms == 200
    connection.close()
    reader = mavutil.mavlink_connection(str(path), dialect="ardupilotmega")
    try:
        messages = []
        while (message := reader.recv_match()) is not None:
            messages.append(message)
    finally:
        reader.close()
    assert [message.get_type() for message in messages] == [
        "ATTITUDE",
        "HEARTBEAT",
        "ATTITUDE",
    ]
    assert [message._timestamp for message in messages] == [1_760_000_000.25] * 3


def test_mavlink_logs_require_a_binary_write_mode(client, tmp_path):
    connection, _peer = client
    with pytest.raises(ValueError, match="binary"):
        connection.setup_logfile(tmp_path / "bad.tlog", mode="w")


def test_remote_wait_heartbeat_ignores_foreign_vehicle(client):
    connection, peer = client
    peer.send(_record(system=2))
    peer.send(_record())
    assert connection.wait_heartbeat(timeout=0.1).get_srcSystem() == 1


def test_remote_receive_timeout_is_bounded_and_nonblocking_is_immediate(client):
    connection, _peer = client
    started = time.monotonic()
    assert connection.recv_match() is None
    assert connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.01) is None
    assert 0.005 <= time.monotonic() - started < 0.5
    with pytest.raises(ValueError, match="condition"):
        connection.recv_match(condition="ARMED")


@pytest.mark.parametrize("timeout", [-1, float("nan"), float("inf"), True])
def test_remote_rejects_invalid_timeout_before_waiting(client, timeout):
    connection, _peer = client
    with pytest.raises(ValueError):
        connection.recv_match(timeout=timeout)


def test_control_record_chatter_cannot_extend_receive_deadline(client, monkeypatch):
    connection, peer = client
    for index in range(20):
        peer.send({"status": index})
    peer.send(_record())
    ticks = count(100.0, 0.1)
    monkeypatch.setattr(remote.time, "monotonic", lambda: next(ticks))
    assert connection.recv_match(blocking=True, timeout=0.3) is None


def test_runtime_request_skips_notifications_and_closes_its_socket(monkeypatch):
    local, server = socket.socketpair()
    peer = remote.JsonSocket(server)
    monkeypatch.setattr(
        remote, "connect_socket", lambda _path: remote.JsonSocket(local)
    )
    try:
        peer.send({"notice": "waiting"})
        peer.send({"result": {"armed": False}})
        assert remote.runtime_request("unused", {"status": True}, timeout=0.1) == {
            "armed": False
        }
        assert peer.receive(0.1) == {"status": True}
        assert local.fileno() == -1
    finally:
        local.close()
        peer.close()


def test_runtime_request_timeout_closes_socket(monkeypatch):
    local, server = socket.socketpair()
    monkeypatch.setattr(
        remote, "connect_socket", lambda _path: remote.JsonSocket(local)
    )
    with closing(server):
        try:
            with pytest.raises(TimeoutError, match="not acknowledged"):
                remote.runtime_request("unused", {"status": True}, timeout=0.01)
            assert local.fileno() == -1
        finally:
            local.close()


@pytest.mark.parametrize(
    "field,value",
    [
        ("received_monotonic", True),
        ("received_monotonic", float("nan")),
        ("received_monotonic", float("inf")),
        ("timestamp", float("nan")),
    ],
)
def test_remote_rejects_invalid_packet_timestamps(client, field, value):
    connection, peer = client
    _raw_json(peer, _record() | {field: value})
    with pytest.raises(ValueError):
        connection.recv_match()


@pytest.mark.parametrize("packet", ["xyz", "", "fd000000"])
def test_remote_rejects_malformed_or_incomplete_packets(client, packet):
    connection, peer = client
    peer.send(_record() | {"packet": packet})
    with pytest.raises(ValueError):
        connection.recv_match()


def test_one_record_cannot_smuggle_an_additional_frame_with_wrong_timestamps(client):
    connection, peer = client
    packets = _packet(_heartbeat(5)) + _packet(_heartbeat(2))
    peer.send(_record() | {"packet": packets.hex()})
    with pytest.raises(ValueError):
        connection.recv_match()


def test_remote_close_does_not_close_unrelated_file_or_socket(client, tmp_path):
    connection, peer = client
    unrelated = Path(tmp_path / "keep-open.txt")
    first, second = socket.socketpair()
    with unrelated.open("w") as handle, closing(first), closing(second):
        connection.close()
        connection.close()
        handle.write("still open")
        first.sendall(b"alive")
        assert second.recv(5) == b"alive"
        assert peer.socket.recv(1) == b""
    assert unrelated.read_text() == "still open"
