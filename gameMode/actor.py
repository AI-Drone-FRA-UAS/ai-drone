"""GameMode Drohnen-Actor & Vektor-Mathematik.

Re-exportiert die Klassen und Funktionen aus `ai_drone.game_actor` für Rückwärtskompatibilität.
"""

from __future__ import annotations

from ai_drone.game_actor import (
    DEFAULT_FAILSAFE_CONFIG,
    DroneGameActor,
    FailsafeConfig,
    FailsafeException,
    PWM_NEUTRAL,
    PWM_THROTTLE_DISARM,
    PWM_THROTTLE_HOVER,
    Vector3,
)

__all__ = [
    "DEFAULT_FAILSAFE_CONFIG",
    "DroneGameActor",
    "FailsafeConfig",
    "FailsafeException",
    "PWM_NEUTRAL",
    "PWM_THROTTLE_DISARM",
    "PWM_THROTTLE_HOVER",
    "Vector3",
]
