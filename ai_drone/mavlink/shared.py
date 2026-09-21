"""One MAVLink reader with independent, bounded consumers and log sinks."""

from __future__ import annotations

import math
import os
import queue
import struct
import threading
import time
from collections import deque
from collections.abc import Callable
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from ai_drone.durability import IntervalSync
from ai_drone.mavlink.safety import heartbeat_is_armed, is_vehicle_message

_RECEIVED_AT = "_received_monotonic"


class SharedMavlinkError(RuntimeError):
    """A shared reader, consumer, or log sink can no longer deliver complete data."""


def received_monotonic(message: Any, *, default: float | None = None) -> float:
    """Use wire-reception time; retain explicit fallback for unbrokered messages."""
    value = getattr(message, _RECEIVED_AT, None)
    if value is None:
        return time.monotonic() if default is None else default
    if (
        isinstance(value, bool)
        or not isinstance(value, int | float)
        or not math.isfinite(value)
    ):
        raise ValueError("invalid MAVLink reception timestamp")
    return float(value)


def _positive(value: float, name: str) -> float:
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _stamp_message(message: Any) -> tuple[float, float]:
    received = received_monotonic(message)
    setattr(message, _RECEIVED_AT, received)
    timestamp = getattr(message, "_timestamp", None)
    if timestamp is None:
        timestamp = time.time()
        message._timestamp = timestamp
    if (
        isinstance(timestamp, bool)
        or not isinstance(timestamp, int | float)
        or not math.isfinite(timestamp)
        or timestamp < 0
    ):
        raise ValueError("invalid MAVLink wall receipt timestamp")
    return received, timestamp


class _LogSink:
    """A bounded writer; queue overflow and disk failures invalidate its endpoint."""

    def __init__(
        self,
        path: str | Path,
        mode: str,
        capacity: int,
        failed: Callable[[BaseException], None],
    ) -> None:
        if mode not in {"wb", "ab", "xb"}:
            raise ValueError("MAVLink logs require a binary write mode")
        self._handle = Path(path).open(mode)  # noqa: SIM115 - writer thread owns the file
        self._queue: queue.Queue[bytes | threading.Event] = queue.Queue(capacity)
        self._sync = IntervalSync()
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._closed = threading.Event()
        self._error: BaseException | None = None
        self._failed = failed
        self._thread = threading.Thread(
            target=self._write, name="mavlink-tlog", daemon=True
        )
        try:
            self._thread.start()
        except BaseException:
            self._handle.close()
            raise

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    def _fail(self, error: BaseException) -> None:
        with self._lock:
            if self._error is not None:
                return
            self._error = error
            self._closing.set()
        self._failed(error)

    def enqueue(self, packet: bytes) -> None:
        error = None
        with self._lock:
            if self._closing.is_set():
                return
            try:
                self._queue.put_nowait(packet)
            except queue.Full:
                error = SharedMavlinkError(
                    "MAVLink tlog queue overflow; log is incomplete"
                )
        if error is not None:
            self._fail(error)

    def _write(self) -> None:
        try:
            while not self._closing.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                try:
                    if isinstance(item, threading.Event):
                        self._handle.flush()
                        item.set()
                    else:
                        self._handle.write(item)
                        self._sync.after_record(self._handle)
                finally:
                    self._queue.task_done()
        except BaseException as error:
            self._fail(error)
        finally:
            try:
                self._handle.flush()
                os.fsync(self._handle.fileno())
            except BaseException as error:
                self._fail(error)
            finally:
                try:
                    self._handle.close()
                except BaseException as error:
                    self._fail(error)
                self._closed.set()

    def flush(self, *, timeout: float = 2.0) -> None:
        deadline = time.monotonic() + _positive(timeout, "timeout")
        barrier = threading.Event()
        with self._lock:
            if self._error is not None:
                raise SharedMavlinkError(
                    f"MAVLink log failed: {self._error}"
                ) from self._error
            if self._closing.is_set():
                raise SharedMavlinkError("MAVLink log is closing")
        try:
            self._queue.put(barrier, timeout=max(0, deadline - time.monotonic()))
        except queue.Full:
            raise TimeoutError("MAVLink log flush timed out") from None
        while not barrier.wait(min(0.05, max(0, deadline - time.monotonic()))):
            if self._error is not None:
                raise SharedMavlinkError(
                    f"MAVLink log failed: {self._error}"
                ) from self._error
            if time.monotonic() >= deadline:
                raise TimeoutError("MAVLink log flush timed out")

    def fileno(self) -> int:
        return self._handle.fileno()

    def request_close(self) -> None:
        with self._lock:
            self._closing.set()

    def close(self, *, timeout: float = 2.0) -> None:
        _positive(timeout, "timeout")
        self.request_close()
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise TimeoutError("MAVLink log writer did not stop")
        if self._error is not None:
            raise SharedMavlinkError(
                f"MAVLink log failed: {self._error}"
            ) from self._error


class _Sender:
    __slots__ = ("_endpoint",)

    def __init__(self, endpoint: MavlinkEndpoint) -> None:
        self._endpoint = endpoint

    def __getattr__(self, name: str) -> Any:
        value = getattr(self._endpoint._hub._raw.mav, name)
        if callable(value):
            if name != "send" and not name.endswith("_send"):
                raise AttributeError(
                    f"shared MAVLink does not expose parser method {name}"
                )

            def send(*args: Any, **kwargs: Any) -> Any:
                return self._endpoint._hub._send(self._endpoint, value, *args, **kwargs)

            return send
        if name.startswith("_") or name.endswith("callback"):
            raise AttributeError(name)
        return value


class MavlinkEndpoint:
    """One independent receive cursor; closing it never closes the transport."""

    def __init__(self, hub: SharedMavlink, name: str, capacity: int) -> None:
        self._hub = hub
        self.name = name
        self.capacity = capacity
        self._messages: deque[Any] = deque()
        self._closed = False
        self._error: BaseException | None = None
        self._heartbeat: Any = None
        self._log: _LogSink | None = None
        self.flightmode: str | None = None
        self.mav = _Sender(self)

    @property
    def target_system(self) -> int:
        return self._hub.target_system

    @target_system.setter
    def target_system(self, value: int) -> None:
        if value != self._hub.target_system:
            raise ValueError("shared MAVLink target system is fixed")

    @property
    def target_component(self) -> int:
        return self._hub.target_component

    @target_component.setter
    def target_component(self, value: int) -> None:
        if value != self._hub.target_component:
            raise ValueError("shared MAVLink target component is fixed")

    @property
    def logfile(self) -> _LogSink | None:
        return self._log

    @logfile.setter
    def logfile(self, value: None) -> None:
        if value is not None:
            raise ValueError("use setup_logfile to attach a shared MAVLink log")
        if self._log is not None:
            self._log.close()
            self._log = None

    def setup_logfile(self, path: str | Path, mode: str = "wb") -> None:
        with self._hub._condition:
            self._check_receive()
            if self._log is not None:
                raise RuntimeError("this MAVLink endpoint already owns a log")
        log = _LogSink(path, mode, self.capacity, self._fail)
        try:
            with self._hub._condition:
                self._check_receive()
                if self._log is not None:
                    raise RuntimeError("this MAVLink endpoint already owns a log")
                self._log = log
        except BaseException:
            log.close()
            raise

    def _fail(self, error: BaseException) -> None:
        with self._hub._condition:
            if self._error is None:
                self._error = error
                self._messages.clear()
            if self._log is not None:
                self._log.request_close()
            self._hub._condition.notify_all()

    def _check_receive(self) -> None:
        if self._error is not None:
            raise SharedMavlinkError(f"{self.name}: {self._error}") from self._error
        if self._closed or self._hub._closed:
            raise SharedMavlinkError(f"MAVLink endpoint {self.name} is closed")

    def _accept(self, message: Any, packet: bytes | None) -> None:
        if self._closed or self._error is not None:
            return
        if len(self._messages) >= self.capacity:
            self._fail(SharedMavlinkError(f"{self.name} receive queue overflow"))
            return
        self._messages.append(message)
        if self._log is not None and packet is not None:
            self._log.enqueue(packet)

    def recv_match(
        self,
        condition: str | None = None,
        type: str | list[str] | set[str] | None = None,
        blocking: bool = False,
        timeout: float | None = None,
    ) -> Any | None:
        if condition is not None:
            raise ValueError("shared MAVLink recv_match does not support conditions")
        if timeout is not None and (
            isinstance(timeout, bool) or not math.isfinite(timeout) or timeout < 0
        ):
            raise ValueError("timeout must be finite and non-negative")
        types = {type} if isinstance(type, str) else type
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._hub._condition:
            while True:
                self._check_receive()
                while self._messages:
                    message = self._messages.popleft()
                    if self._hub._selected_heartbeat(message):
                        self._heartbeat = message
                        self.flightmode = mavutil.mode_string_v10(message)
                    if types is None or message.get_type() in types:
                        return message
                    if deadline is not None and time.monotonic() >= deadline:
                        return None
                if not blocking:
                    return None
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return None
                self._hub._condition.wait(remaining)

    def wait_heartbeat(
        self, blocking: bool = True, timeout: float | None = None
    ) -> Any:
        return self.recv_match(type="HEARTBEAT", blocking=blocking, timeout=timeout)

    def mode_mapping(self) -> dict[str, int]:
        mapping = (
            mavutil.mode_mapping_byname(self._heartbeat.type)
            if self._heartbeat is not None
            else None
        )
        if mapping is None:
            mapping = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR)
        if mapping is None:
            raise SharedMavlinkError("vehicle mode mapping unavailable")
        return dict(mapping)

    def arducopter_arm(self) -> None:
        self._arm_disarm(1)

    def arducopter_disarm(self) -> None:
        self._arm_disarm(0)

    def _arm_disarm(self, armed: int) -> None:
        self.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            armed,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def param_fetch_one(self, name: str | bytes | int) -> None:
        try:
            index = int(name)
        except ValueError:
            identifier = name if isinstance(name, bytes) else str(name).encode("ascii")
            self.mav.param_request_read_send(
                self.target_system, self.target_component, identifier, -1
            )
        else:
            self.mav.param_request_read_send(
                self.target_system, self.target_component, b"", index
            )

    def reset(self) -> None:
        raise RuntimeError(
            "retry serial startup before sharing; shared MAVLink cannot reset"
        )

    def _detach(self) -> None:
        self._closed = True
        self._messages.clear()
        if self._hub._endpoints.get(self.name) is self:
            self._hub._endpoints.pop(self.name)
        if self._log is not None:
            self._log.request_close()
        self._hub._condition.notify_all()

    def close(self, *, timeout: float = 2.0) -> None:
        _positive(timeout, "timeout")
        with self._hub._condition:
            self._detach()
        if self._log is not None:
            self._log.close(timeout=timeout)


class SharedMavlink:
    """Own a physical connection, its sole reader, and serialized outgoing writes."""

    def __init__(
        self,
        raw_connection: Any,
        *,
        target_system: int = 1,
        target_component: int = 1,
        read_timeout: float = 0.05,
    ) -> None:
        for value in (target_system, target_component):
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or not 1 <= value <= 255
            ):
                raise ValueError(
                    "MAVLink target IDs must be integers between 1 and 255"
                )
        self.target_system = target_system
        self.target_component = target_component
        self._read_timeout = _positive(read_timeout, "read_timeout")
        self._raw = raw_connection
        self._raw.target_system = target_system
        self._raw.target_component = target_component
        self._condition = threading.Condition(threading.RLock())
        self._send_lock = threading.RLock()
        self._endpoints: dict[str, MavlinkEndpoint] = {}
        self._stopped = threading.Event()
        self._closed = False
        self._error: BaseException | None = None
        self._heartbeat: Any = None
        self._close_thread: threading.Thread | None = None
        self._close_error: BaseException | None = None
        self._closing_endpoints: list[MavlinkEndpoint] = []
        self._reader = threading.Thread(
            target=self._read, name="mavlink-reader", daemon=True
        )
        try:
            self._reader.start()
        except BaseException:
            self._closed = True
            self._raw.close()
            raise

    def subscribe(self, name: str, capacity: int = 512) -> MavlinkEndpoint:
        if (
            not name
            or isinstance(capacity, bool)
            or not isinstance(capacity, int)
            or capacity <= 0
        ):
            raise ValueError("subscription requires a name and positive queue capacity")
        with self._condition:
            if self._closed or self._error is not None:
                raise SharedMavlinkError("MAVLink reader is closed or failed")
            if name in self._endpoints:
                raise ValueError(f"MAVLink subscriber {name!r} already exists")
            endpoint = MavlinkEndpoint(self, name, capacity)
            self._endpoints[name] = endpoint
            return endpoint

    def _selected_heartbeat(self, message: Any) -> bool:
        return (
            message.get_type() == "HEARTBEAT"
            and is_vehicle_message(
                message,
                system_id=self.target_system,
                component_id=self.target_component,
            )
            and getattr(message, "autopilot", None)
            == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            and getattr(message, "type", None) == mavlink.MAV_TYPE_QUADROTOR
        )

    def _read(self) -> None:
        try:
            while not self._stopped.is_set():
                message = self._raw.recv_match(
                    blocking=True, timeout=self._read_timeout
                )
                if message is None:
                    continue
                received, timestamp = _stamp_message(message)
                with self._condition:
                    if self._closed:
                        return
                    if self._selected_heartbeat(message) and (
                        self._heartbeat is None
                        or received >= received_monotonic(self._heartbeat)
                    ):
                        self._heartbeat = message
                    packet = None
                    if message.get_type() != "BAD_DATA" and any(
                        endpoint._log is not None
                        for endpoint in self._endpoints.values()
                    ):
                        packet = struct.pack(
                            ">Q", int(timestamp * 1_000_000) & ~3
                        ) + bytes(message.get_msgbuf())
                    for endpoint in self._endpoints.values():
                        endpoint._accept(message, packet)
                    self._condition.notify_all()
        except BaseException as error:
            with self._condition:
                if not self._closed:
                    self._error = error
                    for endpoint in self._endpoints.values():
                        endpoint._fail(error)
                    self._condition.notify_all()

    def _send(
        self,
        endpoint: MavlinkEndpoint,
        function: Callable[..., Any],
        *args: Any,
        **kwargs: Any,
    ) -> Any:
        with self._send_lock:
            if self._closed or endpoint._closed:
                raise SharedMavlinkError(
                    "cannot send through a closed MAVLink endpoint"
                )
            # Receive failure does not prohibit a final best-effort LAND write.
            return function(*args, **kwargs)

    def status(self, *, max_age: float = 2.0) -> dict[str, Any]:
        _positive(max_age, "max_age")
        with self._condition:
            heartbeat = self._heartbeat
            received = None if heartbeat is None else received_monotonic(heartbeat)
            age = None if received is None else time.monotonic() - received
            fresh = age is not None and 0 <= age <= max_age
            healthy = not self._closed and self._error is None
            armed = None if heartbeat is None else heartbeat_is_armed(heartbeat)
            return {
                "system_id": self.target_system,
                "component_id": self.target_component,
                "source_known": heartbeat is not None,
                "received_monotonic": received,
                "heartbeat_monotonic": received,
                "heartbeat_age_s": age,
                "armed": armed,
                "mode": None
                if heartbeat is None
                else mavutil.mode_string_v10(heartbeat),
                "fresh": fresh and healthy,
                "status": ("armed" if armed else "disarmed")
                if fresh and healthy
                else "unknown",
                "closed": self._closed,
                "error": None if self._error is None else str(self._error),
            }

    def _close_transport(self, timeout: float) -> None:
        try:
            if not self._send_lock.acquire(timeout=timeout):
                raise TimeoutError("MAVLink sender did not stop")
            try:
                self._raw.close()
            finally:
                self._send_lock.release()
        except BaseException as error:
            self._close_error = error

    def close(self, *, timeout: float = 2.0) -> None:
        deadline = time.monotonic() + _positive(timeout, "timeout")
        with self._condition:
            if not self._closed:
                self._closed = True
                self._stopped.set()
                self._closing_endpoints = list(self._endpoints.values())
                for endpoint in self._closing_endpoints:
                    endpoint._detach()
                self._close_thread = threading.Thread(
                    target=self._close_transport,
                    args=(timeout,),
                    name="mavlink-close",
                    daemon=True,
                )
                self._close_thread.start()
            self._condition.notify_all()
        errors: list[BaseException] = []
        for endpoint in self._closing_endpoints:
            try:
                endpoint.close(timeout=max(0.001, deadline - time.monotonic()))
            except BaseException as error:
                errors.append(error)
        for thread in (self._reader, self._close_thread):
            if thread is not None:
                thread.join(max(0, deadline - time.monotonic()))
                if thread.is_alive():
                    errors.append(TimeoutError(f"{thread.name} did not stop"))
        if self._close_error is not None:
            errors.append(self._close_error)
        if errors:
            raise SharedMavlinkError(
                f"MAVLink cleanup failed: {errors[0]}"
            ) from errors[0]

    def __enter__(self) -> SharedMavlink:
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()
