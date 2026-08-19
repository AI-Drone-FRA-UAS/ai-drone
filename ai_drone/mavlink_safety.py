"""Shared, side-effect-free MAVLink source and armed-state checks."""

from __future__ import annotations

import math
import time
from typing import Any, Protocol

from pymavlink.dialects.v10 import ardupilotmega as mavlink


class SourceMessage(Protocol):
    """The source-identification surface common to received MAVLink messages."""

    def get_type(self) -> str: ...

    def get_srcSystem(self) -> int: ...

    def get_srcComponent(self) -> int: ...


class HeartbeatMessage(SourceMessage, Protocol):
    """The fields used to decode one MAVLink HEARTBEAT."""

    base_mode: int


def heartbeat_is_armed(message: HeartbeatMessage) -> bool:
    """Return whether a heartbeat has the MAVLink safety-armed flag set."""

    return bool(int(message.base_mode) & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def is_vehicle_message(
    message: SourceMessage,
    *,
    system_id: int,
    component_id: int | None = None,
) -> bool:
    """Return whether a message came from the selected vehicle/component.

    A component of ``None`` or ``0`` means any component on the selected vehicle.
    Unknown source IDs are rejected instead of being treated as the vehicle.
    """

    try:
        if int(message.get_srcSystem()) != system_id:
            return False
        return (
            component_id in (None, 0) or int(message.get_srcComponent()) == component_id
        )
    except (AttributeError, TypeError, ValueError):
        return False


def is_armed_vehicle_heartbeat(
    message: HeartbeatMessage,
    *,
    system_id: int,
    component_id: int | None = None,
) -> bool:
    """Return whether a selected vehicle heartbeat reports the armed state."""

    if message.get_type() != "HEARTBEAT":
        return False
    if not is_vehicle_message(
        message,
        system_id=system_id,
        component_id=component_id,
    ):
        return False
    return heartbeat_is_armed(message)


def require_fresh_disarmed_heartbeat(
    connection: Any,
    *,
    system_id: int,
    component_id: int | None = None,
    timeout: float,
) -> HeartbeatMessage:
    """Drain queued heartbeats, then require a new disarmed vehicle heartbeat.

    Any matching armed heartbeat observed while draining or waiting aborts
    immediately. Draining first prevents an old disarmed message from being
    mistaken for a fresh safety check immediately before an operation.
    """

    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("heartbeat timeout must be finite and greater than zero")

    while True:
        queued = connection.recv_match(type="HEARTBEAT", blocking=False)
        if queued is None:
            break
        if is_armed_vehicle_heartbeat(
            queued,
            system_id=system_id,
            component_id=component_id,
        ):
            raise RuntimeError("Vehicle reported ARMED")

    deadline = time.monotonic() + timeout
    while (remaining := deadline - time.monotonic()) > 0:
        heartbeat = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=remaining,
        )
        if heartbeat is None:
            break
        if not is_vehicle_message(
            heartbeat,
            system_id=system_id,
            component_id=component_id,
        ):
            continue
        if heartbeat.get_type() != "HEARTBEAT":
            continue
        if heartbeat_is_armed(heartbeat):
            raise RuntimeError("Vehicle reported ARMED")
        return heartbeat
    raise TimeoutError("No fresh disarmed vehicle heartbeat received")
