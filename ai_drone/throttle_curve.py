"""ArduPilot's STABILIZE throttle curve, as the vehicle actually applies it.

Copter does not treat the throttle stick as a thrust fraction. It shapes the
stick so that *mid stick* produces hover thrust, whatever hover turns out to
be, with a cubic expo derived from ``MOT_THST_HOVER``.

Getting this wrong is why this aircraft never left the ground in STABILIZE.
The flight code placed the stick at ``hover`` -- 0.313 of travel -- believing
that asked for hover thrust. Through this curve 0.313 asks for 0.135, and the
2026-08-21 dataflash recorded ``ThrOut`` 0.128 against a learned hover of
0.263. Twice, for thirteen seconds each, the aircraft was commanded roughly
half the thrust needed to lift it.

This lives at the package root so the flight controller and the simulated
vehicle share one model rather than each carrying its own guess.
"""

from __future__ import annotations

# Copter's expo is clamped to this range whatever hover is learned to be.
MINIMUM_EXPO = -0.5
MAXIMUM_EXPO = 1.0
# The constant Copter divides the hover offset by when deriving expo.
EXPO_SCALE = 0.375
MID_STICK = 0.5


def expo_for_hover(hover: float) -> float:
    """Copter's throttle expo for a given learned hover thrust."""

    return min(MAXIMUM_EXPO, max(MINIMUM_EXPO, -(hover - MID_STICK) / EXPO_SCALE))


def thrust_for_stick(stick: float, hover: float) -> float:
    """The motor thrust Copter produces from a 0.0-1.0 stick fraction."""

    clamped = min(1.0, max(0.0, stick))
    expo = expo_for_hover(hover)
    return clamped * (1.0 - expo) + expo * clamped**3


def stick_for_thrust(thrust: float, hover: float) -> float:
    """Invert :func:`thrust_for_stick`.

    The curve is monotonic on [0, 1], so a bisection is exact enough and
    cannot be tripped up by the cubic having roots outside that interval.
    """

    target = min(1.0, max(0.0, thrust))
    low, high = 0.0, 1.0
    for _ in range(60):
        middle = (low + high) / 2.0
        if thrust_for_stick(middle, hover) < target:
            low = middle
        else:
            high = middle
    return (low + high) / 2.0
