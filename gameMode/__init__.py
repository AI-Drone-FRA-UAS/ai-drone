"""GameMode Drohnensteuerung: Intuitive, spiele-engine-artige Abstraktionsschicht.

Bietet die Klassen `DroneGameActor` und `Vector3` zur einfachen und sicheren
Ansteuerung der Drohne im ALT_HOLD Modus mit integriertem Failsafe-System.
"""

from __future__ import annotations

from gameMode.actor import (
    DEFAULT_FAILSAFE_CONFIG,
    DroneGameActor,
    FailsafeConfig,
    FailsafeException,
    Vector3,
)

__all__ = [
    "DEFAULT_FAILSAFE_CONFIG",
    "DroneGameActor",
    "FailsafeConfig",
    "FailsafeException",
    "Vector3",
]
