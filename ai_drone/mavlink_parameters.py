"""Small, source-filtered helpers for reading ArduPilot parameters."""

from __future__ import annotations

import math
import time
from typing import Any

from ai_drone.config_snapshot import PARAMETER_NAME_PATTERN, decode_parameter_name
from ai_drone.mavlink_safety import heartbeat_is_armed, is_vehicle_message


def request_parameter(
    connection: Any,
    name: str,
    *,
    timeout: float = 3.0,
    require_disarmed: bool = True,
) -> float:
    """Read one finite parameter from the selected vehicle.

    Responses from other MAVLink systems/components are ignored.  When
    ``require_disarmed`` is true, an armed heartbeat aborts the request so a
    pre-flight check cannot continue after vehicle state changes underneath it.
    """

    if PARAMETER_NAME_PATTERN.fullmatch(name) is None:
        raise ValueError(f"invalid ArduPilot parameter name: {name!r}")
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("parameter timeout must be finite and greater than zero")

    system_id = int(connection.target_system)
    component_id = int(connection.target_component)
    connection.mav.param_request_read_send(
        system_id,
        component_id,
        name.encode("ascii"),
        -1,
    )

    deadline = time.monotonic() + timeout
    while (remaining := deadline - time.monotonic()) > 0.0:
        message = connection.recv_match(
            type=["PARAM_VALUE", "HEARTBEAT"],
            blocking=True,
            timeout=min(remaining, 0.25),
        )
        if message is None or not is_vehicle_message(
            message,
            system_id=system_id,
            component_id=component_id,
        ):
            continue
        if message.get_type() == "HEARTBEAT":
            if require_disarmed and heartbeat_is_armed(message):
                raise RuntimeError(f"vehicle became ARMED while reading {name}")
            continue
        if decode_parameter_name(message.param_id) != name:
            continue

        value = float(message.param_value)
        if not math.isfinite(value):
            raise RuntimeError(f"{name} returned a non-finite value")
        return value

    raise TimeoutError(f"flight controller did not return {name}")
