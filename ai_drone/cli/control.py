"""Ausführbares MAVLink-Steuerungstool für den Raspberry Pi und lokale Bench-Tests.

Ermöglicht das passive Überwachen der Sensorik (`status`), einen autonomen Schwebeflug (`hover`)
sowie das Testen der Body-Frame-Geschwindigkeitssteuerung (`velocity-test`) für autonomes Fliegen.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

from ai_drone.controller import DroneController
from ai_drone.follower import AutonomousFollower

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("control_drone")


def cmd_status(args: argparse.Namespace) -> int:
    """Passives Sensor-Monitoring ohne Arming (Sicherheits-Check am Boden)."""
    logger.info("Starte passives Monitoring für %.1f Sekunden...", args.duration)
    try:
        with DroneController(
            device=args.device,
            baud=args.baud,
            max_altitude=args.max_alt,
        ) as drone:
            started = time.monotonic()
            last_print = 0.0
            while time.monotonic() - started < args.duration:
                drone.update_telemetry()
                now = time.monotonic()
                if now - last_print >= 1.0:
                    alt_str = (
                        f"{drone.current_altitude:.2f} m"
                        if drone.current_altitude is not None
                        else "Keine Daten"
                    )
                    bat_str = (
                        f"{drone.battery_voltage:.2f} V"
                        if drone.battery_voltage is not None
                        else "Keine Daten"
                    )
                    logger.info(
                        "Status: Modus=%s | Armed=%s | Höhe=%s | Batterie=%s",
                        drone.flight_mode or "Unbekannt",
                        "JA" if drone.is_armed else "NEIN",
                        alt_str,
                        bat_str,
                    )
                    last_print = now
                time.sleep(0.05)
    except KeyboardInterrupt:
        logger.info("Monitoring durch Benutzer beendet.")
    except Exception as exc:
        logger.error("Fehler im Status-Monitoring: %s", exc)
        return 1
    return 0


def cmd_hover(args: argparse.Namespace) -> int:
    """Autonomer Schwebeflug-Test mit Sicherheitsüberwachung."""
    logger.info(
        "Starte Schwebeflug-Test (Takeoff: %.2f m, Dauer: %.1f s, Max-Alt: %.2f m)...",
        args.takeoff_alt,
        args.duration,
        args.max_alt,
    )
    try:
        with DroneController(
            device=args.device,
            baud=args.baud,
            max_altitude=args.max_alt,
        ) as drone:
            drone.takeoff(target_alt=args.takeoff_alt)

            logger.info("Halte Position für %.1f Sekunden...", args.duration)
            started = time.monotonic()
            last_print = 0.0
            while time.monotonic() - started < args.duration:
                drone.update_telemetry()
                now = time.monotonic()
                if now - last_print >= 1.0:
                    alt_str = (
                        f"{drone.current_altitude:.2f} m"
                        if drone.current_altitude is not None
                        else "?"
                    )
                    logger.info(
                        "Schwebe... Höhe: %s | Batterie: %.2f V",
                        alt_str,
                        drone.battery_voltage or 0.0,
                    )
                    last_print = now
                time.sleep(0.05)

            drone.land()
    except KeyboardInterrupt:
        logger.warning("Abbruch durch Benutzer! Context-Manager löst Notlandung aus...")
        return 130
    except Exception as exc:
        logger.error("Flugfehler aufgetreten: %s", exc)
        return 1
    return 0


def cmd_velocity_test(args: argparse.Namespace) -> int:
    """Testet die Body-Frame-Geschwindigkeitssteuerung für autonome Kamera-Missionen."""
    logger.info(
        "Starte Velocity-Test (Takeoff: %.2f m, Dauer: %.1f s, vx=%.2f, yaw_rate=%.1f°/s)...",
        args.takeoff_alt,
        args.duration,
        args.vx,
        args.yaw_rate,
    )
    try:
        with DroneController(
            device=args.device,
            baud=args.baud,
            max_altitude=args.max_alt,
        ) as drone:
            drone.takeoff(target_alt=args.takeoff_alt)

            logger.info("Sende Body-Frame-Geschwindigkeitsbefehle...")
            started = time.monotonic()
            last_cmd = 0.0
            last_print = 0.0
            while time.monotonic() - started < args.duration:
                drone.update_telemetry()
                now = time.monotonic()

                if now - last_cmd >= 0.2:
                    drone.send_velocity_body(
                        vx=args.vx,
                        vy=args.vy,
                        vz=args.vz,
                        yaw_rate_deg=args.yaw_rate,
                    )
                    last_cmd = now

                if now - last_print >= 1.0:
                    alt_str = (
                        f"{drone.current_altitude:.2f} m"
                        if drone.current_altitude is not None
                        else "?"
                    )
                    logger.info("Velocity Bewegung aktiv... Höhe: %s", alt_str)
                    last_print = now

                time.sleep(0.05)

            logger.info("Stoppe Bewegung (0 m/s)...")
            drone.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            time.sleep(1.0)

            drone.land()
    except KeyboardInterrupt:
        logger.warning("Abbruch durch Benutzer! Notlandung wird eingeleitet...")
        return 130
    except Exception as exc:
        logger.error("Fehler im Velocity-Test: %s", exc)
        return 1
    return 0


def cmd_follow(args: argparse.Namespace) -> int:
    """Autonomes Verfolgen einer Person mit IMX500 Kamera oder als Simulation."""
    logger.info(
        "Starte Follow-Person-Modus (Target-Dist: %.1fm, Max-Speed: %.2fm/s, Sim: %s)...",
        args.target_dist,
        args.max_speed,
        args.sim_target,
    )
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
            )
            if args.sim_target:
                follower.run_simulated_tracking(duration_s=args.duration)
            else:
                drone.arm()
                drone.takeoff(target_alt=args.takeoff_alt)
                time.sleep(2.0)
                follower.run_live_tracking(
                    max_duration_s=args.duration if args.duration > 0 else None
                )
                drone.land()
    except KeyboardInterrupt:
        logger.warning("Follow-Modus durch Benutzer beendet!")
        return 130
    except Exception as exc:
        logger.error("Fehler im Follow-Modus: %s", exc)
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MAVLink-Steuerungstool für die autonome AI-Drone",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Pfad zum Serial-/USB-Gerät (z. B. /dev/serial0 oder /dev/ttyACM0). Falls nicht angegeben, wird auto-detektiert.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baudrate der seriellen Schnittstelle (Standard: 115200 für Pi UART4).",
    )
    parser.add_argument(
        "--takeoff-alt",
        type=float,
        default=0.4,
        help="Zielhöhe für Takeoff in Metern.",
    )
    parser.add_argument(
        "--max-alt",
        type=float,
        default=0.8,
        help="Sicherheits-Höhenlimit in Metern (löst bei Überschreitung Notlandung aus).",
    )

    subparsers = parser.add_subparsers(
        dest="subcommand", required=True, help="Aktions-Modus"
    )

    p_status = subparsers.add_parser(
        "status", help="Passives Sensor-Monitoring ohne Motorschärfung"
    )
    p_status.add_argument(
        "--duration", type=float, default=5.0, help="Monitoring-Dauer in Sekunden."
    )
    p_status.set_defaults(func=cmd_status)

    p_hover = subparsers.add_parser(
        "hover", help="Autonomer Schwebeflug-Test (Takeoff -> Halten -> Landen)"
    )
    p_hover.add_argument(
        "--duration", type=float, default=5.0, help="Schwebeflug-Dauer in Sekunden."
    )
    p_hover.set_defaults(func=cmd_hover)

    p_vel = subparsers.add_parser(
        "velocity-test", help="Testet Body-Frame-Geschwindigkeitssteuerung im Flug"
    )
    p_vel.add_argument(
        "--duration",
        type=float,
        default=4.0,
        help="Dauer des Bewegungs-Tests in Sekunden.",
    )
    p_vel.add_argument(
        "--vx", type=float, default=0.2, help="Vorwärts-Geschwindigkeit in m/s."
    )
    p_vel.add_argument(
        "--vy", type=float, default=0.0, help="Seitwärts-Geschwindigkeit in m/s."
    )
    p_vel.add_argument(
        "--vz", type=float, default=0.0, help="Vertikal-Geschwindigkeit in m/s."
    )
    p_vel.add_argument(
        "--yaw-rate",
        type=float,
        default=10.0,
        help="Gier-Rate (Drehung) in Grad/Sekunde.",
    )
    p_vel.set_defaults(func=cmd_velocity_test)

    p_follow = subparsers.add_parser(
        "follow", help="Autonomes Verfolgen von Personen mit AI-Kamera (oder simuliert)"
    )
    p_follow.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="Maximale Tracking-Dauer in Sekunden.",
    )
    p_follow.add_argument(
        "--target-dist",
        type=float,
        default=2.0,
        help="Gewünschter Halteabstand zur Person in Metern.",
    )
    p_follow.add_argument(
        "--max-speed", type=float, default=0.3, help="Maximale Geschwindigkeit in m/s."
    )
    p_follow.add_argument(
        "--sim-target",
        action="store_true",
        help="Simulierter Test am Schreibtisch ohne Kamera.",
    )
    p_follow.set_defaults(func=cmd_follow)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
