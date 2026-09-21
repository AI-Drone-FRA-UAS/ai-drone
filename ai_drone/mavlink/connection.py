"""Consistent ArduPilot wire decoding for every maintained connection."""

from __future__ import annotations

import errno
import os
import socket
import time
from collections.abc import Callable
from typing import Any


def _require_complete_write(written: object, expected: int) -> int:
    if type(written) is not int or written != expected:
        raise OSError(
            f"incomplete MAVLink transport write: expected {expected} bytes, got {written!r}"
        )
    return written


def _serial_writer(connection: Any) -> Callable[[bytes], int]:
    original = connection.write

    def write(data: bytes) -> int:
        # pyserial defaults to an unbounded write. Keep any tighter caller limit,
        # including nonblocking mode, and reapply after an upstream serial reset.
        port = connection.port
        if hasattr(port, "write_timeout") and (
            port.write_timeout is None or port.write_timeout > 0.05
        ):
            port.write_timeout = 0.05
        # Retain pymavlink's portdead/autoreconnect bookkeeping, then reject the
        # negative or partial count that its generated encoder otherwise ignores.
        return _require_complete_write(original(data), len(data))

    return write


def _tcp_writer(connection: Any) -> Callable[[bytes], int]:
    def write(data: bytes) -> int:
        if connection.port is None:
            connection.reconnect()
        if connection.port is None:
            raise OSError("MAVLink TCP transport is disconnected")
        try:
            # mavtcp makes its socket nonblocking. One send preserves that
            # behavior; sendall/retry loops would delay the controller's cleanup.
            written = connection.port.send(data)
        except OSError as error:
            if error.errno in (errno.ECONNRESET, errno.EPIPE):
                connection.handle_disconnect()
            raise
        return _require_complete_write(written, len(data))

    return write


def _tcp_listener_writer(connection: Any) -> Callable[[bytes], int]:
    def write(data: bytes) -> int:
        if connection.port is None:
            raise OSError("MAVLink TCP listener has no connected peer")
        try:
            written = connection.port.send(data)
        except OSError as error:
            if error.errno == errno.EPIPE:
                connection.port.close()
                connection.port = None
                connection.fd = connection.listen.fileno()
            raise
        return _require_complete_write(written, len(data))

    return write


def _udp_server_write(connection: Any, data: bytes) -> int | None:
    now = time.time()
    expired = set()
    recipients = 0
    for address in connection.clients:
        if (
            len(connection.clients) == 1
            or connection.timeout <= 0
            or connection.clients_last_alive[address] + connection.timeout > now
        ):
            _require_complete_write(connection.port.sendto(data, address), len(data))
            recipients += 1
        elif len(connection.clients) > 1 and len(expired) < len(connection.clients) - 1:
            expired.add(address)
            connection.clients_last_alive.pop(address)
    connection.clients -= expired
    if not recipients:
        raise OSError("MAVLink UDP listener has no current peer")
    # One logical write can fan out to several sockets. The observer's byte
    # contract is per call, so do not claim one datagram is the complete TX sum.
    return len(data) if recipients == 1 else None


def _udp_writer(connection: Any) -> Callable[[bytes], int | None]:
    def write(data: bytes) -> int | None:
        if connection.udp_server:
            return _udp_server_write(connection, data)
        if connection.last_address and connection.broadcast:
            connection.destination_addr = connection.last_address
            connection.broadcast = False
            connection.port.connect(connection.destination_addr)
        if connection.destination_addr[0] != connection.resolved_destination_addr:
            connection.resolved_destination_addr = connection.destination_addr[0]
            connection.destination_addr = (
                socket.gethostbyname(connection.destination_addr[0]),
                connection.destination_addr[1],
            )
        return _require_complete_write(
            connection.port.sendto(data, connection.destination_addr), len(data)
        )

    return write


def _unsupported_write(_data: bytes) -> int:
    raise OSError("MAVLink transport does not support verified physical writes")


def require_complete_writes(connection: Any) -> None:
    """Make physical failures visible before pymavlink's encoder hides them."""
    from pymavlink import mavutil

    if getattr(connection, "_ai_drone_checked_write", False):
        return
    writer: Callable[[bytes], int | None]
    if isinstance(connection, mavutil.mavserial):
        writer = _serial_writer(connection)
    elif isinstance(connection, mavutil.mavtcp):
        writer = _tcp_writer(connection)
    elif isinstance(connection, mavutil.mavtcpin):
        writer = _tcp_listener_writer(connection)
    elif isinstance(connection, mavutil.mavudp):
        writer = _udp_writer(connection)
    elif isinstance(connection, mavutil.mavfile):
        writer = _unsupported_write
    else:
        return
    connection.write = writer
    connection._ai_drone_checked_write = True
    connection.write_confirmation = (
        "unsupported" if writer is _unsupported_write else "physical"
    )


def open_ardupilot_connection(device: str, **options: Any) -> Any:
    """Open with the MAVLink 2 decoder, which also accepts MAVLink 1 frames.

    Autodetection can cache V1 messages before switching to V2, then crash on
    V2 instance fields such as RAW_IMU.id. Select the complete dialect before
    receiving anything, including when pymavlink was imported earlier.
    """

    if device.startswith("unix:"):
        from ai_drone.mavlink.remote import RemoteConnection

        return RemoteConnection(device.removeprefix("unix:"))

    from pymavlink import mavutil

    os.environ["MAVLINK20"] = "1"
    connection = mavutil.mavlink_connection(device, dialect="ardupilotmega", **options)
    try:
        require_complete_writes(connection)
    except BaseException:
        connection.close()
        raise
    return connection
