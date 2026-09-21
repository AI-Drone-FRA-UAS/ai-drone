"""Private local JSON service for one MAVLink owner and bounded client streams."""

from __future__ import annotations

import inspect
import math
import os
import socket
import stat
import threading
import time
import uuid
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from ai_drone.mavlink.remote import JsonSocket, decode_value, encode_value
from ai_drone.mavlink.shared import MavlinkEndpoint, SharedMavlink, received_monotonic

_METHODS = {
    name: inspect.signature(getattr(mavlink.MAVLink, name))
    for name in (
        "param_request_read_send",
        "param_request_list_send",
        "log_request_list_send",
        "command_long_send",
        "heartbeat_send",
        "set_mode_send",
        "set_attitude_target_send",
    )
}
_PASSIVE_COMMANDS = {
    mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    mavlink.MAV_CMD_REQUEST_MESSAGE,
    mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
}


def _require_finite(value: Any) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("MAVLink arguments must be finite")
    if isinstance(value, list):
        for item in value:
            _require_finite(item)
    elif isinstance(value, dict):
        for item in value.values():
            _require_finite(item)


def _send_arguments(request: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    if request.keys() - {"send", "args", "kwargs"}:
        raise ValueError("unsupported MAVLink request fields")
    method = request["send"]
    if not isinstance(method, str) or method not in _METHODS:
        raise ValueError("unsupported MAVLink send method")
    args = decode_value(request.get("args", []))
    kwargs = decode_value(request.get("kwargs", {}))
    if not isinstance(args, list) or not isinstance(kwargs, dict):
        raise ValueError("MAVLink arguments require a list and keyword object")
    _require_finite(args)
    _require_finite(kwargs)
    bound = _METHODS[method].bind(None, *args, **kwargs)
    bound.apply_defaults()
    values = dict(bound.arguments)
    values.pop("self")
    for name in ("target_system", "target_component"):
        if name in values and (type(values[name]) is not int or values[name] != 1):
            raise ValueError("vehicle runtime only addresses the selected FC at 1/1")
    if method == "command_long_send" and type(values["command"]) is not int:
        raise ValueError("MAVLink command must be an integer")
    if method == "heartbeat_send" and values["type"] != mavlink.MAV_TYPE_GCS:
        raise ValueError("only GCS heartbeats are supported")
    return method, values


def _is_control(method: str, values: dict[str, Any]) -> bool:
    if method == "command_long_send":
        return values["command"] not in _PASSIVE_COMMANDS
    return method in {"heartbeat_send", "set_mode_send", "set_attitude_target_send"}


@dataclass
class _Peer:
    name: str
    wire: JsonSocket
    endpoint: MavlinkEndpoint
    thread: threading.Thread | None = None


class VehicleServer:
    """Expose an existing hub; server shutdown leaves hub ownership with its caller."""

    def __init__(
        self,
        path: str | Path,
        hub: SharedMavlink,
        request_handler: Callable[[dict[str, Any]], Any],
        network_busy: Callable[[], bool] | None = None,
        *,
        max_clients: int = 8,
    ) -> None:
        if (
            isinstance(max_clients, bool)
            or not isinstance(max_clients, int)
            or not 1 <= max_clients <= 8
        ):
            raise ValueError("vehicle runtime supports between 1 and 8 clients")
        if (hub.target_system, hub.target_component) != (1, 1):
            raise ValueError("vehicle runtime requires the project FC at 1/1")
        self.path = Path(path)
        self.hub = hub
        self._request_handler = request_handler
        self._external_network_busy = network_busy or (lambda: False)
        self._max_clients = max_clients
        self._lock = threading.RLock()
        self._stream_lock = threading.Lock()
        self._peers: dict[str, _Peer] = {}
        self._control_owner: str | None = None
        self._network_busy = False
        self._maintenance = False
        self._intervals: dict[int, float] = {}
        self._stopped = threading.Event()
        self._listener: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._socket_identity: tuple[int, int] | None = None
        self._lock_descriptor: int | None = None

    @property
    def control_owner(self) -> str | None:
        with self._lock:
            return self._control_owner

    @property
    def is_network_busy(self) -> bool:
        with self._lock:
            return self._network_busy or self._external_network_busy()

    @property
    def is_maintenance(self) -> bool:
        with self._lock:
            return self._maintenance

    def set_network_busy(self, active: bool) -> None:
        if type(active) is not bool:
            raise ValueError("network maintenance state must be a boolean")
        with self._lock:
            if active and self._maintenance:
                raise RuntimeError("network switching is blocked during maintenance")
            if active and self._control_owner is not None:
                raise RuntimeError("network switching is blocked by the control owner")
            self._network_busy = active

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "control_owner": self._control_owner,
                "network_busy": self.is_network_busy,
                "maintenance": self._maintenance,
                "clients": len(self._peers),
                "max_clients": self._max_clients,
            }

    def _prepare_parent(self) -> None:
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        parent = self.path.parent.lstat()
        if (
            not stat.S_ISDIR(parent.st_mode)
            or parent.st_uid != os.geteuid()
            or stat.S_IMODE(parent.st_mode) != 0o700
        ):
            raise PermissionError(
                "runtime directory must be owned by this user with mode 0700"
            )

    def _claim_socket(self) -> None:
        import fcntl

        descriptor = os.open(
            self.path.with_suffix(".lock"),
            os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW,
            0o600,
        )
        try:
            info = os.fstat(descriptor)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid():
                raise PermissionError("runtime lock must be an owned regular file")
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise FileExistsError(
                    "vehicle runtime socket is already owned"
                ) from None
        except BaseException:
            os.close(descriptor)
            raise
        self._lock_descriptor = descriptor
        if not self.path.exists() and not self.path.is_symlink():
            return
        info = self.path.lstat()
        if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.geteuid():
            raise FileExistsError("refusing to replace an unowned or non-socket path")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
            probe.settimeout(0.2)
            try:
                probe.connect(str(self.path))
            except ConnectionRefusedError:
                current = self.path.lstat()
                if (current.st_dev, current.st_ino) != (info.st_dev, info.st_ino):
                    raise FileExistsError(
                        "runtime socket changed during stale check"
                    ) from None
                self.path.unlink()
            else:
                raise FileExistsError("vehicle runtime socket is accepting connections")

    def _release_socket(self) -> None:
        if self._lock_descriptor is not None:
            os.close(self._lock_descriptor)
            self._lock_descriptor = None

    def start(self) -> VehicleServer:
        with self._lock:
            if self._stopped.is_set():
                raise RuntimeError("vehicle server is closed")
            if self._listener is not None:
                return self
            self._prepare_parent()
            listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                self._claim_socket()
                listener.bind(str(self.path))
                info = self.path.lstat()
                self._socket_identity = (info.st_dev, info.st_ino)
                self.path.chmod(0o600)
                listener.listen(self._max_clients)
                listener.settimeout(0.1)
                self._listener = listener
                self._thread = threading.Thread(
                    target=self._accept, name="vehicle-accept", daemon=True
                )
                self._thread.start()
            except BaseException:
                listener.close()
                self._listener = None
                self._remove_socket()
                self._release_socket()
                raise
        return self

    def _accept(self) -> None:
        listener = self._listener
        assert listener is not None
        while not self._stopped.is_set():
            try:
                connection, _address = listener.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            wire = JsonSocket(connection)
            endpoint = None
            try:
                with self._lock:
                    if self._stopped.is_set():
                        raise RuntimeError("vehicle server is stopping")
                    if len(self._peers) >= self._max_clients:
                        raise RuntimeError("maximum vehicle runtime clients reached")
                    name = f"peer-{uuid.uuid4().hex[:12]}"
                    endpoint = self.hub.subscribe(name, capacity=512)
                    peer = _Peer(name, wire, endpoint)
                    peer.thread = threading.Thread(
                        target=self._serve_peer, args=(peer,), name=name, daemon=True
                    )
                    self._peers[name] = peer
                    peer.thread.start()
            except Exception as error:
                with suppress(Exception):
                    wire.send({"error": str(error)}, timeout=0.1)
                wire.close()
                if endpoint is not None:
                    endpoint.close()
                    with self._lock:
                        self._peers.pop(endpoint.name, None)

    def _authorize_control(
        self, peer: _Peer, method: str, values: dict[str, Any]
    ) -> None:
        with self._lock:
            if self._maintenance:
                raise RuntimeError("control is blocked during maintenance")
            if self._control_owner is None:
                if self.is_network_busy:
                    raise RuntimeError("control is blocked during network maintenance")
                if self.hub.status().get("status") != "disarmed":
                    raise RuntimeError("control ownership requires a fresh disarmed FC")
                self._control_owner = peer.name
            elif self._control_owner != peer.name:
                raise RuntimeError("another peer owns flight control")
            if (
                method == "command_long_send"
                and values["command"] == mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                and values["param1"] != 0
                and self.is_network_busy
            ):
                raise RuntimeError("arming is blocked during network maintenance")

    def _set_interval(self, peer: _Peer, values: dict[str, Any]) -> None:
        message_id, interval = values["param1"], values["param2"]
        if (
            isinstance(message_id, bool)
            or not isinstance(message_id, int | float)
            or not math.isfinite(message_id)
            or not 0 <= message_id <= 0xFFFFFF
            or int(message_id) != message_id
            or isinstance(interval, bool)
            or not isinstance(interval, int | float)
            or not math.isfinite(interval)
            or interval <= 0
        ):
            raise ValueError(
                "shared telemetry requires a valid message ID and positive interval"
            )
        with self._stream_lock:
            identifier = int(message_id)
            fastest = min(interval, self._intervals.get(identifier, interval))
            values["param2"] = fastest
            peer.endpoint.mav.command_long_send(**values)
            self._intervals[identifier] = fastest

    def _request(self, peer: _Peer, request: dict[str, Any]) -> None:
        with self._lock:
            if self._stopped.is_set():
                raise RuntimeError("vehicle server is stopping")
            if set(request) == {"maintenance"}:
                active = request["maintenance"]
                if type(active) is not bool:
                    raise ValueError("maintenance state must be a boolean")
                if active and (
                    self._maintenance
                    or self._control_owner is not None
                    or self.is_network_busy
                    or len(self._peers) != 1
                    or self.hub.status().get("status") != "disarmed"
                ):
                    raise RuntimeError(
                        "maintenance requires fresh disarmed FC and no other clients or network operation"
                    )
                self._maintenance = active
                peer.wire.send({"result": {"maintenance": active}}, timeout=2)
                return
            if self._maintenance and request != {"status": True}:
                raise RuntimeError("vehicle runtime is in maintenance")
        if "send" not in request:
            result = self._request_handler(request)
            peer.wire.send({"result": encode_value(result)}, timeout=2)
            return
        method, values = _send_arguments(request)
        if _is_control(method, values):
            self._authorize_control(peer, method, values)
        if (
            method == "command_long_send"
            and values["command"] == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        ):
            self._set_interval(peer, values)
        else:
            getattr(peer.endpoint.mav, method)(**values)

    def _serve_peer(self, peer: _Peer) -> None:
        try:
            while not self._stopped.is_set():
                request = peer.wire.receive(timeout=0)
                if request is not None:
                    self._request(peer, request)
                message = peer.endpoint.recv_match(blocking=False)
                if (
                    message is not None
                    and message.get_type() != "BAD_DATA"
                    and not self.is_maintenance
                ):
                    peer.wire.send(
                        {
                            "packet": bytes(message.get_msgbuf()).hex(),
                            "received_monotonic": received_monotonic(message),
                            "timestamp": getattr(message, "_timestamp", None)
                            or time.time(),
                        },
                        timeout=2,
                    )
                if request is None and message is None:
                    self._stopped.wait(0.005)
        except Exception as error:
            # A rejected request invalidates this peer, including its control
            # lease. A replacement still has to prove fresh disarmed FC state;
            # an armed vehicle relies on its existing FC-side GCS failsafe.
            if not self._stopped.is_set():
                with suppress(Exception):
                    peer.wire.send({"error": str(error)}, timeout=2)
        finally:
            peer.wire.close()
            with suppress(Exception):
                peer.endpoint.close()
            with self._lock:
                self._peers.pop(peer.name, None)
                if self._control_owner == peer.name:
                    self._control_owner = None

    def _remove_socket(self) -> None:
        with suppress(FileNotFoundError):
            current = self.path.lstat()
            if self._socket_identity == (current.st_dev, current.st_ino):
                self.path.unlink()

    def close(self, *, timeout: float = 2.0) -> None:
        if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("close timeout must be finite and positive")
        deadline = time.monotonic() + timeout
        self._stopped.set()
        if not self._lock.acquire(timeout=max(0, deadline - time.monotonic())):
            raise TimeoutError("vehicle server request did not stop boundedly")
        try:
            if self._listener is not None:
                self._listener.close()
            peers = list(self._peers.values())
        finally:
            self._lock.release()
        errors = self._stop_peers(peers, deadline)
        if errors:
            for secondary in errors[1:]:
                errors[0].add_note(f"Additional shutdown error: {secondary}")
            # A handler can still be inside an external effect. Retain the lock
            # and socket identity until a later close confirms quiescence.
            raise errors[0]
        try:
            self._remove_socket()
        finally:
            self._release_socket()

    def _stop_peers(self, peers: list[_Peer], deadline: float) -> list[BaseException]:
        errors: list[BaseException] = []
        for peer in peers:
            try:
                peer.wire.close()
            except BaseException as error:
                errors.append(error)
            try:
                peer.endpoint.close(timeout=max(0.001, deadline - time.monotonic()))
            except BaseException as error:
                errors.append(error)
        threads = [self._thread, *(peer.thread for peer in peers)]
        for thread in threads:
            if thread is not None and thread.ident is not None:
                try:
                    thread.join(max(0, deadline - time.monotonic()))
                except BaseException as error:
                    errors.append(error)
        if any(thread is not None and thread.is_alive() for thread in threads):
            errors.append(TimeoutError("vehicle server did not stop boundedly"))
        return errors

    def __enter__(self) -> VehicleServer:
        return self.start()

    def __exit__(self, *_args: Any) -> None:
        self.close()
