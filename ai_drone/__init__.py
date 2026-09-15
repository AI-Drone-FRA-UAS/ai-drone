"""Camera, sensor, and ArduPilot tooling for the AI Drone project.

The flight-control names below stay importable from the package root, but are
resolved lazily: importing ``ai_drone`` for a link, config, or vision helper
must not pull the MAVLink flight-control stack into the process.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ai_drone.flight.controller import DroneController, FlightSafetyError
    from ai_drone.flight.guards import (
        FlightController,
        FlightGuardError,
        check_safety_guardrails,
    )

_LAZY_EXPORTS = {
    "DroneController": "ai_drone.flight.controller",
    "FlightSafetyError": "ai_drone.flight.controller",
    "FlightController": "ai_drone.flight.guards",
    "FlightGuardError": "ai_drone.flight.guards",
    "check_safety_guardrails": "ai_drone.flight.guards",
    "MountController": "ai_drone.mount",
    "openMount": "ai_drone.mount",
    "closeMount": "ai_drone.mount",
    "open_mount": "ai_drone.mount",
    "close_mount": "ai_drone.mount",
    "releaseMount": "ai_drone.mount",
}

__all__ = [
    "DroneController",
    "FlightController",
    "FlightGuardError",
    "FlightSafetyError",
    "MountController",
    "check_safety_guardrails",
    "closeMount",
    "close_mount",
    "openMount",
    "open_mount",
    "releaseMount",
]


def __getattr__(name: str) -> Any:
    """Import a flight-control export on first attribute access."""
    module_name = _LAZY_EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from importlib import import_module

    return getattr(import_module(module_name), name)


def __dir__() -> list[str]:
    return sorted(__all__)
