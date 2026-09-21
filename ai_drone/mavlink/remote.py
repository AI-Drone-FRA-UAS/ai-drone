"""Local MAVLink clients for the single Pi serial owner.

The Unix socket carries JSON and wire packets, never Python objects. Original
receipt times survive transport so a delayed heartbeat cannot become fresh.
"""

from __future__ import annotations

import builtins
import json
import math
import select
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

MAX_RECORD = 16_384


def _invalid_constant(value: str):
    raise ValueError(f"non-finite JSON constant: {value}")


def encode_value(value: Any) -> Any:
    if isinstance(value, bytes | bytearray):
        return {"bytes": bytes(value).hex()}
    if isinstance(value, list | tuple):
        return [encode_value(item) for item in value]
    if isinstance(value, dict):
        return {key: encode_value(item) for key, item in value.items()}
    if value is None or isinstance(value, str | int | float | bool):
        return value
    raise ValueError(f"unsupported MAVLink argument type: {type(value).__name__}")


def decode_value(value: Any) -> Any:
    if isinstance(value, dict):
        if set(value) == {"bytes"}:
            return bytes.fromhex(value["bytes"])
        return {key: decode_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [decode_value(item) for item in value]
    return value


def wire_record(value: dict) -> bytes:
    result = json.dumps(value, separators=(",", ":"), allow_nan=False).encode() + b"\n"
    if len(result) > MAX_RECORD:
        raise ValueError("runtime request is too large")
    return result


class JsonSocket:
    def __init__(self, connection: socket.socket):
        self.socket = connection
        self.socket.setblocking(False)
        self._buffer = bytearray()
        self._send_lock = threading.Lock()

    def send(self, value: dict, timeout: float = 2) -> None:
        remaining = memoryview(wire_record(value))
        deadline = time.monotonic() + timeout
        with self._send_lock:
            while remaining:
                wait = deadline - time.monotonic()
                if wait <= 0 or not select.select([], [self.socket], [], wait)[1]:
                    raise TimeoutError("runtime socket write timed out")
                try:
                    count = self.socket.send(remaining)
                except BlockingIOError:
                    continue
                if not count:
                    raise ConnectionError("runtime socket closed")
                remaining = remaining[count:]

    def receive(self, timeout: float | None = 0) -> dict | None:
        deadline = None if timeout is None else time.monotonic() + timeout
        while b"\n" not in self._buffer:
            wait = None if deadline is None else max(0, deadline - time.monotonic())
            if not select.select([self.socket], [], [], wait)[0]:
                return None
            data = self.socket.recv(4096)
            if not data:
                raise ConnectionError("vehicle runtime disconnected")
            self._buffer.extend(data)
            if len(self._buffer) > MAX_RECORD and b"\n" not in self._buffer:
                raise ValueError("runtime record exceeds size limit")
        line, _, rest = self._buffer.partition(b"\n")
        self._buffer = bytearray(rest)
        if len(line) > MAX_RECORD:
            raise ValueError("runtime record exceeds size limit")
        value = json.loads(line, parse_constant=_invalid_constant)
        if not isinstance(value, dict):
            raise ValueError("runtime record must be an object")
        if "error" in value:
            raise RuntimeError(str(value["error"]))
        return value

    def close(self) -> None:
        self.socket.close()


def connect_socket(path: str | Path) -> JsonSocket:
    connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        connection.settimeout(2)
        connection.connect(str(path))
        return JsonSocket(connection)
    except BaseException:
        connection.close()
        raise


def runtime_request(path: str | Path, request: dict, timeout: float = 5) -> Any:
    connection = connect_socket(path)
    try:
        connection.send(request)
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            result = connection.receive(remaining)
            if result is not None and "result" in result:
                return result["result"]
        raise TimeoutError("runtime request was not acknowledged")
    finally:
        connection.close()


class _RemoteMav:
    def __init__(self, wire: JsonSocket):
        self.wire = wire

    def __getattr__(self, name: str):
        if not name.endswith("_send") or name.startswith("_"):
            raise AttributeError(name)

        def send(*args, **kwargs):
            self.wire.send(
                {
                    "send": name,
                    "args": encode_value(args),
                    "kwargs": encode_value(kwargs),
                }
            )

        return send


class RemoteConnection:
    @property
    def write_confirmation(self) -> str:
        """Submission to the local runtime is not a confirmed physical write."""
        return "queued"

    def __init__(self, path: str | Path):
        self._wire = connect_socket(path)
        self._parser = mavlink.MAVLink(None)
        self.mav = _RemoteMav(self._wire)
        self.target_system = 1
        self.target_component = 1
        self.flightmode = None
        self.logfile = None
        self._closed = False
        self._last_received = 0.0

    def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
        if condition is not None:
            raise ValueError("MAVLink condition expressions are not supported")
        if timeout is not None and (
            builtins.type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout < 0
        ):
            raise ValueError("timeout must be finite and nonnegative")
        kinds = (
            None if type is None else ({type} if isinstance(type, str) else set(type))
        )
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            remaining = (
                None if deadline is None else max(0, deadline - time.monotonic())
            )
            record = self._wire.receive(remaining if blocking else 0)
            if record is None:
                return None
            if "packet" not in record:
                if deadline is not None and time.monotonic() >= deadline:
                    return None
                continue
            message = self._decode_record(record)
            self._observe_message(message)
            if kinds is None or message.get_type() in kinds:
                return message
            if deadline is not None and time.monotonic() >= deadline:
                return None

    def _observe_message(self, message: Any) -> None:
        if (
            message.get_type() == "HEARTBEAT"
            and (message.get_srcSystem(), message.get_srcComponent()) == (1, 1)
            and message.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            and message.type == mavlink.MAV_TYPE_QUADROTOR
        ):
            self.flightmode = mavutil.mode_string_v10(message)
        if self.logfile is not None and message.get_type() != "BAD_DATA":
            self.logfile.write(
                struct.pack(">Q", int(message._timestamp * 1_000_000) & ~3)
                + message.get_msgbuf()
            )

    def _decode_record(self, record: dict[str, Any]) -> Any:
        packet = bytes.fromhex(record["packet"])
        if len(packet) < 8 or packet[0] not in (253, 254):
            raise ValueError("invalid MAVLink frame")
        length = packet[1] + (
            8 if packet[0] == 254 else 12 + (13 if packet[2] & 1 else 0)
        )
        if len(packet) != length:
            raise ValueError("runtime record must contain exactly one MAVLink frame")
        received, timestamp = record["received_monotonic"], record["timestamp"]
        if (
            builtins.type(received) not in (int, float)
            or not math.isfinite(received)
            or not self._last_received <= received <= time.monotonic() + 0.1
            or builtins.type(timestamp) not in (int, float)
            or not math.isfinite(timestamp)
            or timestamp < 0
        ):
            raise ValueError("invalid runtime receipt timestamps")
        self._last_received = received
        message: Any = self._parser.parse_char(packet)
        if message is None:
            raise ValueError("runtime supplied an incomplete MAVLink packet")
        message._received_monotonic = received
        message._timestamp = timestamp
        return message

    def wait_heartbeat(self, blocking=True, timeout=None):
        from ai_drone.mavlink.safety import require_ardupilot_heartbeat

        if not blocking:
            return self.recv_match(type="HEARTBEAT")
        try:
            return require_ardupilot_heartbeat(
                self,
                system_id=1,
                component_id=1,
                timeout=15 if timeout is None else timeout,
            )
        except TimeoutError:
            return None

    def mode_mapping(self):
        return mavutil.mode_mapping_acm

    def arducopter_arm(self):
        self.mav.command_long_send(
            1, 1, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )

    def arducopter_disarm(self):
        self.mav.command_long_send(
            1, 1, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )

    def param_fetch_one(self, name):
        self.mav.param_request_read_send(
            1, 1, name.encode() if isinstance(name, str) else name, -1
        )

    def setup_logfile(self, filename, mode="wb"):
        if mode not in {"wb", "ab"}:
            raise ValueError("MAVLink logs require binary mode")
        if self.logfile is not None:
            raise RuntimeError("MAVLink log is already open")
        self.logfile = open(filename, mode)  # noqa: SIM115 -- owned until close()

    def reset(self):
        raise RuntimeError("shared FC transport cannot be reset by a client")

    def close(self):
        if not self._closed:
            self._closed = True
            self._wire.close()
            if self.logfile is not None:
                self.logfile.close()
                self.logfile = None
