"""Private Unix-socket fanout and command ownership without physical hardware."""

from __future__ import annotations

import os
import queue
import socket
import stat
import threading
import time
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from ai_drone.mavlink.remote import (
    JsonSocket,
    RemoteConnection,
    connect_socket,
    encode_value,
)
from ai_drone.mavlink.server import VehicleServer
from ai_drone.mavlink.shared import SharedMavlink, received_monotonic

pytestmark = pytest.mark.skipif(
    os.name != "posix", reason="private Unix socket service"
)


class PhysicalConnection:
    def __init__(self):
        self.incoming = queue.Queue()
        self.writes = queue.Queue()
        self.closed = False
        self.mav = SimpleNamespace()
        for name in (
            "command_long_send",
            "heartbeat_send",
            "param_request_read_send",
            "param_request_list_send",
            "log_request_list_send",
            "set_mode_send",
            "set_attitude_target_send",
        ):
            setattr(self.mav, name, self._sender(name))

    def _sender(self, name):
        return lambda *args, **kwargs: self.writes.put((name, args, kwargs))

    def recv_match(self, **kwargs):
        try:
            return self.incoming.get(timeout=kwargs["timeout"])
        except queue.Empty:
            return None

    def close(self):
        self.closed = True
        self.incoming.put(None)


def heartbeat(*, armed=False, mode=5):
    message = mavlink.MAVLink_heartbeat_message(
        type=mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        | (mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0),
        custom_mode=mode,
        system_status=mavlink.MAV_STATE_ACTIVE,
        mavlink_version=3,
    )
    message.pack(mavlink.MAVLink(None, srcSystem=1, srcComponent=1))
    return message


@pytest.fixture
def runtime(tmp_path):
    physical = PhysicalConnection()
    hub = SharedMavlink(physical)
    server = VehicleServer(
        tmp_path / "runtime" / "vehicle.sock",
        hub,
        request_handler=lambda request: {"echo": request},
    ).start()
    value = SimpleNamespace(physical=physical, hub=hub, server=server, clients=[])
    try:
        yield value
    finally:
        for client in value.clients:
            client.close()
        server.close()
        hub.close()


def receive(client, key, *, timeout=1):
    deadline = time.monotonic() + timeout
    while (remaining := deadline - time.monotonic()) > 0:
        record = client.receive(remaining)
        if record is not None and key in record:
            return record[key]
    raise TimeoutError(f"no {key} response")


def client(runtime):
    connection = connect_socket(runtime.server.path)
    runtime.clients.append(connection)
    connection.send({"ping": True})
    assert receive(connection, "result") == {"echo": {"ping": True}}
    return connection


def observe(runtime, **fields):
    observer = runtime.hub.subscribe("test-observer")
    message = heartbeat(**fields)
    runtime.physical.incoming.put(message)
    try:
        assert observer.wait_heartbeat(timeout=1) is message
    finally:
        observer.close()
    return message


def send(connection, method, *args, **kwargs):
    connection.send(
        {"send": method, "args": encode_value(args), "kwargs": encode_value(kwargs)}
    )


def command(connection, identifier, first=0, second=0):
    send(
        connection,
        "command_long_send",
        1,
        1,
        identifier,
        0,
        first,
        second,
        0,
        0,
        0,
        0,
        0,
    )


def claim(connection):
    send(
        connection,
        "heartbeat_send",
        mavlink.MAV_TYPE_GCS,
        mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavlink.MAV_STATE_ACTIVE,
    )


def wait_for(predicate):
    deadline = time.monotonic() + 1
    while not predicate():
        assert time.monotonic() < deadline
        threading.Event().wait(0.005)


def test_private_socket_streams_identical_packets_and_original_times(runtime):
    first, second = client(runtime), client(runtime)
    message = observe(runtime)
    records = [connection.receive(1) for connection in (first, second)]
    assert all(
        record["packet"] == bytes(message.get_msgbuf()).hex() for record in records
    )
    assert all(
        record["received_monotonic"] == received_monotonic(message)
        for record in records
    )
    assert stat.S_IMODE(runtime.server.path.stat().st_mode) == 0o600
    assert stat.S_IMODE(runtime.server.path.parent.stat().st_mode) == 0o700
    assert runtime.server.status()["clients"] == 2


def test_passive_requests_and_encoded_parameter_names_need_no_control_owner(runtime):
    connection = client(runtime)
    send(connection, "param_request_read_send", 1, 1, b"ARMING_SKIPCHK", -1)
    name, _args, kwargs = runtime.physical.writes.get(timeout=1)
    assert name == "param_request_read_send" and kwargs["param_id"] == b"ARMING_SKIPCHK"
    for identifier in (
        mavlink.MAV_CMD_REQUEST_MESSAGE,
        mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
    ):
        command(connection, identifier)
        assert runtime.physical.writes.get(timeout=1)[2]["command"] == identifier
    assert runtime.server.control_owner is None


@pytest.mark.parametrize("state", ["unknown", "armed", "stale"])
def test_first_control_claim_requires_fresh_disarmed_fc(runtime, state):
    connection = client(runtime)
    if state != "unknown":
        message = observe(runtime, armed=state == "armed")
        if state == "stale":
            message._received_monotonic -= 10
    claim(connection)
    with pytest.raises(RuntimeError, match="fresh disarmed"):
        receive(connection, "result")
    assert runtime.server.control_owner is None and runtime.physical.writes.empty()


def test_one_control_owner_survives_arming_and_releases_on_disconnect(runtime):
    first, second = client(runtime), client(runtime)
    observe(runtime)
    claim(first)
    assert runtime.physical.writes.get(timeout=1)[0] == "heartbeat_send"
    owner = runtime.server.control_owner
    assert owner is not None
    observe(runtime, armed=True)
    command(first, mavlink.MAV_CMD_NAV_LAND)
    assert (
        runtime.physical.writes.get(timeout=1)[2]["command"] == mavlink.MAV_CMD_NAV_LAND
    )
    claim(second)
    with pytest.raises(RuntimeError, match="another peer"):
        receive(second, "result")
    assert runtime.server.control_owner == owner
    first.close()
    wait_for(lambda: runtime.server.control_owner is None)
    observe(runtime)
    third = client(runtime)
    claim(third)
    assert runtime.physical.writes.get(timeout=1)[0] == "heartbeat_send"
    assert runtime.server.control_owner not in (None, owner)


def test_network_maintenance_and_control_claim_are_mutually_exclusive(runtime):
    recorder = client(runtime)
    observe(runtime)
    runtime.server.set_network_busy(True)
    assert runtime.server.is_network_busy
    send(recorder, "param_request_list_send", 1, 1)
    assert runtime.physical.writes.get(timeout=1)[0] == "param_request_list_send"
    controller = client(runtime)
    command(controller, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    with pytest.raises(RuntimeError, match="network maintenance"):
        receive(controller, "result")
    assert runtime.server.control_owner is None
    runtime.server.set_network_busy(False)
    controller = client(runtime)
    claim(controller)
    runtime.physical.writes.get(timeout=1)
    with pytest.raises(RuntimeError, match="control owner"):
        runtime.server.set_network_busy(True)
    assert not runtime.server.is_network_busy


def test_network_claim_cannot_race_control_ownership(runtime, monkeypatch):
    connection = client(runtime)
    observe(runtime)
    entered, release, started = (threading.Event() for _ in range(3))
    result = queue.Queue()
    original_status = runtime.hub.status

    def paused_status():
        entered.set()
        assert release.wait(timeout=1)
        return original_status()

    def maintenance():
        started.set()
        try:
            runtime.server.set_network_busy(True)
        except RuntimeError as error:
            result.put(str(error))
        else:
            result.put("network claim succeeded")

    monkeypatch.setattr(runtime.hub, "status", paused_status)
    claim(connection)
    assert entered.wait(timeout=1)
    thread = threading.Thread(target=maintenance)
    thread.start()
    try:
        assert started.wait(timeout=1)
    finally:
        release.set()
        thread.join(timeout=1)
    assert "control owner" in result.get(timeout=1)
    assert runtime.physical.writes.get(timeout=1)[0] == "heartbeat_send"
    assert runtime.server.control_owner is not None
    assert not runtime.server.is_network_busy


def test_remote_connection_roundtrip_preserves_receipt_and_parameter_bytes(runtime):
    connection = RemoteConnection(runtime.server.path)
    runtime.clients.append(connection)
    wait_for(lambda: runtime.server.status()["clients"] == 1)
    message = observe(runtime)
    received = connection.wait_heartbeat(timeout=1)
    assert received is not None
    assert received.get_msgbuf() == message.get_msgbuf()
    assert received_monotonic(received) == received_monotonic(message)
    assert connection.flightmode == "LOITER"
    connection.param_fetch_one("ARMING_SKIPCHK")
    assert runtime.physical.writes.get(timeout=1)[2]["param_id"] == b"ARMING_SKIPCHK"
    assert runtime.server.control_owner is None


def test_external_network_busy_callback_blocks_existing_owner_arm(runtime):
    connection = client(runtime)
    observe(runtime)
    claim(connection)
    runtime.physical.writes.get(timeout=1)
    runtime.server._external_network_busy = lambda: True
    command(connection, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    with pytest.raises(RuntimeError, match="arming is blocked"):
        receive(connection, "result")
    assert runtime.physical.writes.empty()


def test_stream_rates_keep_fastest_request_even_after_peer_disconnect(runtime):
    first, second = client(runtime), client(runtime)
    for connection, interval, expected in (
        (first, 100_000, 100_000),
        (second, 200_000, 100_000),
        (second, 50_000, 50_000),
        (first, 200_000, 50_000),
    ):
        command(connection, mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 193, interval)
        assert runtime.physical.writes.get(timeout=1)[2]["param2"] == expected
    second.close()
    command(first, mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 193, 100_000)
    assert runtime.physical.writes.get(timeout=1)[2]["param2"] == 50_000
    assert runtime.server.control_owner is None


@pytest.mark.parametrize(
    "payload, reason",
    [
        ({"send": "param_set_send", "args": []}, "unsupported"),
        ({"send": "param_request_list_send", "args": [42, 1]}, "1/1"),
        ({"send": "param_request_list_send", "args": "bad"}, "arguments"),
        ({"send": "heartbeat_send", "args": [2, 3, 0, 0, 4]}, "GCS"),
    ],
)
def test_invalid_send_requests_fail_without_transport_write(runtime, payload, reason):
    connection = client(runtime)
    connection.send(payload)
    with pytest.raises(RuntimeError, match=reason):
        receive(connection, "result")
    assert runtime.physical.writes.empty()


def test_exponent_overflow_cannot_send_nonfinite_control_values(runtime):
    connection = client(runtime)
    observe(runtime)
    connection.socket.setblocking(True)
    connection.socket.sendall(
        b'{"send":"command_long_send","args":[1,1,400,0,1e999,0,0,0,0,0,0]}\n'
    )
    connection.socket.setblocking(False)
    with pytest.raises(RuntimeError, match="finite"):
        receive(connection, "result")
    assert runtime.server.control_owner is None and runtime.physical.writes.empty()


def test_maximum_eight_clients_is_enforced(runtime):
    for _ in range(8):
        client(runtime)
    extra = connect_socket(runtime.server.path)
    try:
        with pytest.raises(RuntimeError, match="maximum"):
            extra.receive(1)
    finally:
        extra.close()
    assert runtime.server.status()["clients"] == 8


def test_live_socket_is_never_replaced_and_shutdown_keeps_hub_open(runtime):
    connection = client(runtime)
    duplicate = VehicleServer(runtime.server.path, runtime.hub, lambda _request: None)
    with pytest.raises(FileExistsError):
        duplicate.start()
    connection.send({"still": "alive"})
    assert receive(connection, "result") == {"echo": {"still": "alive"}}
    runtime.server.close()
    assert not runtime.server.path.exists()
    assert not runtime.physical.closed
    observe(runtime)
    assert runtime.hub.status()["fresh"]


def test_shutdown_timeout_retains_exclusion_until_handler_finishes(runtime):
    connection = client(runtime)
    entered, release = threading.Event(), threading.Event()

    def handler(_request):
        entered.set()
        assert release.wait(2)
        return {"done": True}

    runtime.server._request_handler = handler
    connection.send({"block": True})
    assert entered.wait(1)
    try:
        with pytest.raises(TimeoutError, match="did not stop boundedly"):
            runtime.server.close(timeout=0.02)
        replacement = VehicleServer(runtime.server.path, runtime.hub, handler)
        with pytest.raises(FileExistsError, match="already owned"):
            replacement.start()
    finally:
        release.set()
    runtime.server.close()
    with VehicleServer(runtime.server.path, runtime.hub, handler):
        pass


def test_rejected_owner_disconnects_and_cannot_be_replaced_while_armed(runtime):
    owner = client(runtime)
    observe(runtime)
    claim(owner)
    assert runtime.physical.writes.get(timeout=1)[0] == "heartbeat_send"
    observe(runtime, armed=True)
    owner.send({"send": "param_set_send", "args": []})
    with pytest.raises(RuntimeError, match="unsupported"):
        receive(owner, "result")
    wait_for(lambda: runtime.server.control_owner is None)
    replacement = client(runtime)
    claim(replacement)
    with pytest.raises(RuntimeError, match="fresh disarmed"):
        receive(replacement, "result")
    assert runtime.physical.writes.empty()
    observe(runtime)
    grounded = client(runtime)
    claim(grounded)
    assert runtime.physical.writes.get(timeout=1)[0] == "heartbeat_send"


def test_stale_owned_socket_is_recovered_under_exclusive_lock(runtime):
    runtime.server.close()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stale:
        stale.bind(str(runtime.server.path))
    replacement = VehicleServer(runtime.server.path, runtime.hub, lambda _request: {})
    with replacement:
        assert runtime.server.path.is_socket()
        contender = VehicleServer(runtime.server.path, runtime.hub, lambda _request: {})
        with pytest.raises(FileExistsError, match="already owned"):
            contender.start()
        assert runtime.server.path.is_socket()
    assert not runtime.server.path.exists()


def test_socket_cleanup_never_replaces_an_ordinary_file(runtime):
    runtime.server.close()
    runtime.server.path.write_text("keep")
    server = VehicleServer(runtime.server.path, runtime.hub, lambda _request: {})
    with pytest.raises(FileExistsError, match="non-socket"):
        server.start()
    server.close()
    assert runtime.server.path.read_text() == "keep"


def test_maintenance_requires_only_one_idle_client_and_fresh_disarmed_fc(runtime):
    first, second = client(runtime), client(runtime)
    observe(runtime)
    first.send({"maintenance": True})
    with pytest.raises(RuntimeError, match="no other clients"):
        receive(first, "result")
    wait_for(lambda: runtime.server.status()["clients"] == 1)
    observe(runtime, armed=True)
    second.send({"maintenance": True})
    with pytest.raises(RuntimeError, match="fresh disarmed"):
        receive(second, "result")
    assert not runtime.server.is_maintenance


def test_maintenance_latches_across_disconnect_and_allows_status_and_release(runtime):
    installer = client(runtime)
    observe(runtime)
    installer.send({"maintenance": True})
    assert receive(installer, "result") == {"maintenance": True}
    installer.close()
    wait_for(lambda: runtime.server.status()["clients"] == 0)
    assert runtime.server.is_maintenance
    with pytest.raises(RuntimeError, match="maintenance"):
        runtime.server.set_network_busy(True)
    status = connect_socket(runtime.server.path)
    runtime.clients.append(status)
    status.send({"status": True})
    assert receive(status, "result") == {"echo": {"status": True}}
    observe(runtime)
    assert status.receive(0.05) is None  # No telemetry during maintenance.
    send(status, "param_request_list_send", 1, 1)
    with pytest.raises(RuntimeError, match="maintenance"):
        receive(status, "result")
    assert runtime.physical.writes.empty()
    duplicate = connect_socket(runtime.server.path)
    runtime.clients.append(duplicate)
    duplicate.send({"maintenance": True})
    with pytest.raises(RuntimeError, match="maintenance requires"):
        receive(duplicate, "result")
    assert runtime.server.is_maintenance
    recovery = connect_socket(runtime.server.path)
    runtime.clients.append(recovery)
    recovery.send({"maintenance": False})
    assert receive(recovery, "result") == {"maintenance": False}
    assert not runtime.server.is_maintenance
    message = observe(runtime)
    assert receive(recovery, "packet") == bytes(message.get_msgbuf()).hex()


def test_runtime_parent_requires_private_permissions(runtime, tmp_path):
    parent = tmp_path / "public"
    parent.mkdir(mode=0o755)
    server = VehicleServer(parent / "vehicle.sock", runtime.hub, lambda _request: None)
    with pytest.raises(PermissionError, match="0700"):
        server.start()
    assert stat.S_IMODE(parent.stat().st_mode) == 0o755


def test_slow_socket_peer_receives_overflow_error_without_silent_loss(
    runtime, monkeypatch
):
    connection = client(runtime)
    entered, release = threading.Event(), threading.Event()
    original_send = JsonSocket.send

    def blocked(self, value, timeout=2):
        if "packet" in value:
            entered.set()
            assert release.wait(timeout=3)
        return original_send(self, value, timeout)

    monkeypatch.setattr(JsonSocket, "send", blocked)
    try:
        runtime.physical.incoming.put(heartbeat())
        assert entered.wait(timeout=1)
        for _ in range(513):
            runtime.physical.incoming.put(heartbeat())
        runtime.physical.incoming.put(heartbeat(mode=9))
        wait_for(lambda: runtime.hub.status()["mode"] == "LAND")
    finally:
        release.set()
    with pytest.raises(RuntimeError, match="queue overflow"):
        receive(connection, "result")
    assert runtime.hub.status()["fresh"]
