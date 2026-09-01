#!/usr/bin/env python3
"""Interaktives Servo-Kalibrierungs- und Steuerungswerkzeug für die AI-Drone.

Ermöglicht das schrittweise manuelle Einstellen des Servos (z.B. SG90 / MS18-F),
um mechanische Anschläge, Minimum, Maximum sowie die Abwurf-Positionen (Open/Closed)
präzise zu ermitteln und direkt für mission_drop.py zu exportieren.

Starten auf dem Raspberry Pi:
    ./calibrate_servo.py
    -- ODER --
    uv run ./calibrate_servo.py --pin 18

Starten vom Laptop (Deploy zum Pi):
    uv run drone-deploy --servo
"""

from __future__ import annotations

import sys
from ai_drone.cli.servo import main

if __name__ == "__main__":
    sys.exit(main())
