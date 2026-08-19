#!/usr/bin/env python3
"""Einfacher Schwebeflug-Test für die ai-drone mit SOFORT-NOT-AUS.

Lässt die Drohne im GUIDED_NOGPS Modus sanft auf die gewünschte Höhe abheben,
schwebt für die angegebene Dauer stabil an Ort und Stelle und landet dann sicher.

SOFORT-NOT-AUS (Motoren sofort 100% AUS):
- Einfach 'ENTER' oder 'LEERTASTE' im Terminal drücken!
- Oder 'STRG + C' drücken!
"""

from __future__ import annotations

import argparse
import logging
import select
import sys
import threading
import time

from ai_drone.controller import DroneController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("test_hover")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Einfacher Schwebeflug-Test (Abheben -> Schweben -> Landen)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=0.35,
        help="Zielhöhe in Metern (Standard: 0.35m)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Schwebeflug-Dauer in Sekunden vor der Landung",
    )
    parser.add_argument(
        "--max-alt",
        type=float,
        default=0.80,
        help="Sicherheits-Höhenlimit in Metern",
    )
    parser.add_argument(
        "--min-battery",
        type=float,
        default=14.4,
        help="Mindestspannung in Volt für Notlandung",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="MAVLink-Schnittstelle (Standard: automatische Erkennung)",
    )
    args = parser.parse_args()

    logger.info("====================================================================")
    logger.info(
        "  SCHWEBEFLUG-TEST: Start auf %.2f m für %.1f s", args.alt, args.duration
    )
    logger.info("  *** NOT-AUS: Drücke jederzeit 'ENTER' oder 'STRG+C' zum Killen! ***")
    logger.info("====================================================================")

    drone_instance: DroneController | None = None
    stop_event = threading.Event()

    def keyboard_kill_listener() -> None:
        """Lauscht im Hintergrund auf Tastendruck für sofortigen Motor-Stopp."""
        while not stop_event.is_set():
            if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                _ = sys.stdin.readline()
                logger.warning(
                    "Tastendruck erkannt! LÖSE SOFORT-NOT-AUS AUS (MOTOREN AUS)!"
                )
                if drone_instance is not None:
                    drone_instance.hard_emergency_kill()
                stop_event.set()
                break

    kill_thread = threading.Thread(target=keyboard_kill_listener, daemon=True)
    kill_thread.start()

    try:
        with DroneController(device=args.device, max_altitude=args.max_alt) as drone:
            drone_instance = drone

            # 1. Start / Steigflug
            logger.info(">>> 1. STARTE ABHEBEN auf %.2f m...", args.alt)
            drone.takeoff(target_alt=args.alt)

            # 2. Schwebeflug
            logger.info(
                ">>> 2. ZIELHÖHE ERREICHT! Schwebe für %.1f Sekunden...", args.duration
            )
            started = time.monotonic()
            last_log = 0.0

            while (
                time.monotonic() - started < args.duration and not stop_event.is_set()
            ):
                drone.update_telemetry()
                now = time.monotonic()

                # Batterie-Check
                if (
                    drone.battery_voltage is not None
                    and 0.0 < drone.battery_voltage < args.min_battery
                ):
                    logger.error(
                        "Batterie kritisch (%.2f V)! Leite Notlandung ein.",
                        drone.battery_voltage,
                    )
                    break

                # Halteposition (0 m/s Horizontal- und Vertikal-Geschwindigkeit)
                drone.send_velocity_body(0.0, 0.0, 0.0, 0.0)

                if now - last_log >= 1.0:
                    remaining = args.duration - (now - started)
                    alt_str = (
                        f"{drone.current_altitude:.2f} m"
                        if drone.current_altitude is not None
                        else "N/A"
                    )
                    bat_str = (
                        f"{drone.battery_voltage:.2f} V"
                        if drone.battery_voltage is not None
                        else "N/A"
                    )
                    logger.info(
                        "Schwebe... Verbleibend: %.1fs | Höhe: %s | Akku: %s | (ENTER = Not-Aus)",
                        remaining,
                        alt_str,
                        bat_str,
                    )
                    last_log = now

                time.sleep(0.05)

            # 3. Landung
            if not stop_event.is_set():
                logger.info(">>> 3. SCHWEBEZEIT BEENDET! Leite sanfte Landung ein...")
                drone.land()
                logger.info("Schwebeflug-Test erfolgreich beendet!")

    except KeyboardInterrupt:
        logger.warning("STRG+C erkannt! LÖSE SOFORT-NOT-AUS AUS (MOTOREN SOFORT AUS)!")
        if drone_instance is not None:
            drone_instance.hard_emergency_kill()
        return 130
    except Exception as exc:
        logger.error("Fehler beim Schwebeflug: %s", exc)
        if drone_instance is not None:
            drone_instance.hard_emergency_kill()
        return 1
    finally:
        stop_event.set()

    return 0


if __name__ == "__main__":
    sys.exit(main())
