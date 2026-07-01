#!/usr/bin/env python3
"""CLI-Skript für autonomes Verfolgen von Personen im Flugsicherheitsnetz (oder simuliert am Schreibtisch).

Verwendet MAVLink 2 für die Flugsteuerung und die IMX500 AI-Kamera (NanoDet) für die
optische Personenerkennung.

Beispielaufrufe:
    # Simuliertes Tracking am Schreibtisch (ohne Drohnensprung und ohne Kamera)
    python follow_person.py --sim-target --duration 15

    # Echter Flug und visuelles Tracking im Netz (nur auf dem Raspberry Pi mit Kamera)
    python follow_person.py --device /dev/serial0 --takeoff-alt 0.5 --max-alt 1.0
"""

from __future__ import annotations

import argparse
import logging
import sys

from ai_drone.controller import DroneController
from ai_drone.follower import AutonomousFollower


def main() -> int:
    """Hauptfunktion für das autonome Follow-Person-Skript."""
    parser = argparse.ArgumentParser(
        description="Autonomes Verfolgen von Personen mit IMX500 AI-Kamera und MAVLink."
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="MAVLink Schnittstelle (z.B. /dev/serial0 am Pi oder /dev/ttyACM0 am USB-Port). Standard: Auto-Detect.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=921600,
        help="Baudrate für serielle Schnittstellen. Standard: 921600.",
    )
    parser.add_argument(
        "--takeoff-alt",
        type=float,
        default=0.5,
        help="Zielhöhe für den Start in Metern. Standard: 0.5 m.",
    )
    parser.add_argument(
        "--max-alt",
        type=float,
        default=0.8,
        help="Sicherheits-Maximalhöhe in Metern. Standard: 0.8 m.",
    )
    parser.add_argument(
        "--target-dist",
        type=float,
        default=2.0,
        help="Gewünschter Halteabstand zur Person in Metern. Standard: 2.0 m.",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=0.3,
        help="Maximale Vorwärts-/Rückwärtsgeschwindigkeit in m/s. Standard: 0.3 m/s.",
    )
    parser.add_argument(
        "--max-yaw",
        type=float,
        default=20.0,
        help="Maximale Drehgeschwindigkeit in °/s. Standard: 20.0 °/s.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Maximale Dauer des Trackings in Sekunden (0 für unendlich). Standard: 30.0 s.",
    )
    parser.add_argument(
        "--sim-target",
        action="store_true",
        help="Aktiviere simuliertes Target-Tracking am Schreibtisch (ohne Kamera und ohne Takeoff).",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Erweiterte Debug-Ausgaben."
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    logger = logging.getLogger("follow_person")

    logger.info("=== AI-Drone: Autonomes Follow-Person Skript ===")
    logger.info("Konfiguration: Target-Dist=%.1fm, Max-Speed=%.2fm/s, Max-Alt=%.1fm", args.target_dist, args.max_speed, args.max_alt)

    try:
        with DroneController(
            device=args.device,
            baud=args.baud,
            max_altitude=args.max_alt,
        ) as drone:
            follower = AutonomousFollower(
                drone=drone,
                target_dist_m=args.target_dist,
                max_vx=args.max_speed,
                max_yaw_rate_deg=args.max_yaw,
            )

            if args.sim_target:
                logger.info("Modus: SIMULIERT (kein Takeoff, keine Kamera).")
                follower.run_simulated_tracking(duration_s=args.duration if args.duration > 0 else 15.0)
            else:
                logger.info("Modus: ECHTER FLUG MIT IMX500 AI-KAMERA.")
                # 1. Arm und Takeoff
                drone.arm()
                drone.takeoff(alt=args.takeoff_alt)
                logger.info("Takeoff abgeschlossen. Schwebe für 2 Sekunden...")
                import time
                time.sleep(2.0)

                # 2. Starte visuelles Tracking
                follower.run_live_tracking(max_duration_s=args.duration if args.duration > 0 else None)

                # 3. Land nach Beendigung
                logger.info("Tracking beendet. Leite Landung ein...")
                drone.land()

    except Exception as exc:
        logger.error("Fehler im Follow-Person-Skript: %s", exc, exc_info=args.verbose)
        return 1

    logger.info("Follow-Person-Skript sicher und erfolgreich beendet.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
