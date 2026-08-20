"""What pre-arm check configurations this project is willing to fly with.

ArduPilot's ``ARMING_CHECK`` is a bitmask.  ``1`` means "run every available
check".  ``0`` means "run none", and an aircraft in that state does not report
*why* it will not arm -- which is how this airframe sat for weeks with a
misconfigured rangefinder nobody could see.

This aircraft has no GPS receiver and its whole purpose is indoor flight
without one, so the two GPS checks can never pass and block a mode that
otherwise has everything it needs.  That is the one relaxation this project
allows, and it is expressed as a specific value rather than as "any non-zero
mask": every other check -- IMU, compass, barometer, battery, board voltage,
logging, rangefinder, parameters -- stays on, and stays able to speak.

There is no configuration here that disables everything.  ``0`` is not
reachable through this module, and neither is an arbitrary mask.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ai_drone.mavlink.parameters import request_parameter

PARAMETER = "ARMING_CHECK"

# ArduPilot Copter 4.6 ARMING_CHECK bits.
BAROMETER = 2
COMPASS = 4
GPS_LOCK = 8
INS = 16
PARAMETERS = 32
RC_CHANNELS = 64
BOARD_VOLTAGE = 128
BATTERY_LEVEL = 256
LOGGING = 1_024
SAFETY_SWITCH = 2_048
GPS_CONFIG = 4_096
SYSTEM = 8_192
MISSION = 16_384
RANGEFINDER = 32_768
CAMERA = 65_536
AUX_AUTH = 131_072
VISION_ODOMETRY = 262_144
FFT = 524_288

GPS_CHECKS = GPS_LOCK | GPS_CONFIG

ALL_CHECKS = 1.0
ALL_EXCEPT_GPS = float(
    BAROMETER
    | COMPASS
    | INS
    | PARAMETERS
    | RC_CHANNELS
    | BOARD_VOLTAGE
    | BATTERY_LEVEL
    | LOGGING
    | SAFETY_SWITCH
    | SYSTEM
    | MISSION
    | RANGEFINDER
    | CAMERA
    | AUX_AUTH
    | VISION_ODOMETRY
    | FFT
)

# The only two values this project will write, or fly with.
ACCEPTABLE: dict[float, str] = {
    ALL_CHECKS: "every configurable pre-arm check",
    ALL_EXCEPT_GPS: "every pre-arm check except the two GPS checks",
}


def _fmt(value: float) -> str:
    """Format a bitmask the way ArduPilot shows it: an integer, never 1e+06."""

    return f"{int(value)}" if float(value).is_integer() else f"{value:g}"


class ArmingCheckError(RuntimeError):
    """Raised when the pre-arm check set is not one this project will fly."""


def is_acceptable(value: float) -> bool:
    """Whether ``value`` is a pre-arm configuration this project permits."""

    return value in ACCEPTABLE


def describe(value: float) -> str:
    """Explain one live ``ARMING_CHECK`` value in operational terms."""

    if value in ACCEPTABLE:
        return f"{PARAMETER}={_fmt(value)}: {ACCEPTABLE[value]} is enabled"
    if value == 0.0:
        return (
            f"{PARAMETER}=0: all pre-arm checks are disabled. The vehicle does not "
            "report PreArm failures in this state, so it cannot tell anyone what is "
            "wrong with it. "
            f"This project flies with {_fmt(ALL_CHECKS)} or {_fmt(ALL_EXCEPT_GPS)}."
        )
    missing = [
        name
        for name, bit in (
            ("barometer", BAROMETER),
            ("compass", COMPASS),
            ("INS", INS),
            ("parameters", PARAMETERS),
            ("battery", BATTERY_LEVEL),
            ("logging", LOGGING),
            ("rangefinder", RANGEFINDER),
        )
        if not int(value) & bit
    ]
    detail = f", missing: {', '.join(missing)}" if missing else ""
    return (
        f"{PARAMETER}={_fmt(value)}: an arbitrary subset of pre-arm checks{detail}. "
        f"This project flies with {_fmt(ALL_CHECKS)} or {_fmt(ALL_EXCEPT_GPS)}."
    )


def require_acceptable(value: float) -> None:
    """Raise unless ``value`` is a permitted pre-arm configuration."""

    if not is_acceptable(value):
        raise ArmingCheckError(describe(value))


@dataclass(frozen=True)
class RestoreResult:
    """What the vehicle reported before and after the write."""

    previous: float
    current: float

    @property
    def changed(self) -> bool:
        return self.previous != self.current

    def describe(self) -> str:
        if not self.changed:
            return f"{PARAMETER} was already {_fmt(self.current)}; nothing was written"
        return (
            f"{PARAMETER} {_fmt(self.previous)} -> {_fmt(self.current)}: "
            f"{ACCEPTABLE[self.current]} is now enabled"
        )


def read_arming_checks(connection: Any, *, timeout: float = 3.0) -> float:
    """Read the live ``ARMING_CHECK`` value from the selected vehicle."""

    return request_parameter(connection, PARAMETER, timeout=timeout)
