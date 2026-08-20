"""The project's only general parameter write, with the one bypass it forbids.

Writing flight-controller parameters is how an aircraft gets configured and
also how it gets quietly made unsafe.  This module keeps the write in one
place so there is a single thing to audit, and it hard-codes the one rule the
rest of the project depends on: ``ARMING_CHECK`` can be written to ``1`` and to
nothing else.  No caller, and no argument, can turn the pre-arm checks off
through this code.

Every write requires a fresh disarmed heartbeat, and every write is read back
and verified.  A write this module cannot confirm is reported as a failure, not
as a success.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.mavlink import arming_checks
from ai_drone.mavlink.parameters import PARAMETER_NAME_PATTERN, request_parameter
from ai_drone.mavlink.safety import require_fresh_disarmed_heartbeat

# Parameters this module refuses to set to an arbitrary value, each mapped to
# the values the project permits.  This is a plain module-level table rather
# than a registry something has to populate: a guard that depends on an import
# having happened is a guard that can be lost by deleting an import.
PROTECTED_PARAMETERS: dict[str, Callable[[float], bool]] = {
    arming_checks.PARAMETER: arming_checks.is_acceptable,
}


class ParameterWriteError(RuntimeError):
    """Raised when a parameter write was refused or could not be confirmed."""


@dataclass(frozen=True)
class WriteResult:
    """What the vehicle reported before and after one write."""

    name: str
    previous: float
    current: float

    @property
    def changed(self) -> bool:
        return self.previous != self.current

    def describe(self) -> str:
        if not self.changed:
            return f"{self.name} was already {self.current:g}; nothing was written"
        return f"{self.name} {self.previous:g} -> {self.current:g}"


def _matches(written: float, reported: float) -> bool:
    return math.isclose(written, reported, rel_tol=1e-6, abs_tol=1e-6)


def set_parameter(
    connection: Any,
    name: str,
    value: float,
    *,
    timeout: float = 5.0,
) -> WriteResult:
    """Write one parameter to the selected vehicle and verify the readback."""

    if PARAMETER_NAME_PATTERN.fullmatch(name) is None:
        raise ValueError(f"invalid ArduPilot parameter name: {name!r}")
    if isinstance(value, bool) or not math.isfinite(value):
        raise ValueError("parameter value must be a finite number")

    permitted = PROTECTED_PARAMETERS.get(name)
    if permitted is not None and not permitted(value):
        raise ParameterWriteError(
            f"{name}={value:g} is not a value this project will write. It guards a "
            "safety gate, so this command refuses."
        )

    system = int(connection.target_system)
    component = int(connection.target_component)
    require_fresh_disarmed_heartbeat(
        connection, system_id=system, component_id=component, timeout=timeout
    )

    previous = request_parameter(connection, name, timeout=timeout)
    if _matches(previous, value):
        return WriteResult(name=name, previous=previous, current=previous)

    connection.mav.param_set_send(
        system,
        component,
        name.encode("ascii"),
        float(value),
        mavlink.MAV_PARAM_TYPE_REAL32,
    )

    deadline = time.monotonic() + timeout
    current = previous
    while time.monotonic() < deadline:
        time.sleep(0.2)
        current = request_parameter(connection, name, timeout=timeout)
        if _matches(current, value):
            return WriteResult(name=name, previous=previous, current=current)

    raise ParameterWriteError(
        f"vehicle did not confirm {name}={value:g}; it still reports {current:g}"
    )
