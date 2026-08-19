#!/usr/bin/env python3
"""Autonome Abwurf-Mission für die ai-drone (mit sicherem --dry-run Modus).

Fliegt ein expandierendes Spiral-Suchmuster ab, begrenzt auf maximal 2 Meter Radius
um den Startpunkt. Sobald das AprilTag erkannt wird, zentriert sich die Drohne
präzise darüber, löst den Servo aus und landet sicher.
Im `--dry-run` Modus bleiben die Motoren komplett AUS (sicherer Schreibtisch-Test).
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path


# Wir fangen Fehler ab, falls das Skript nicht auf dem Pi läuft
def _is_raspberry_pi() -> bool:
    try:
        model = Path("/proc/device-tree/model").read_text()
    except (FileNotFoundError, PermissionError):
        return False
    return "raspberry pi" in model.lower()


if _is_raspberry_pi():
    try:
        import numpy as np
        from gpiozero import Servo  # type: ignore[import-not-found]  # ty: ignore[unresolved-import]
        from picamera2 import Picamera2  # type: ignore[import-not-found,import-untyped]  # ty: ignore[unresolved-import]

        from ai_drone.apriltags import TagDetection, create_detector
        from ai_drone.controller import DroneController
    except ImportError as e:
        print(f"Fehler beim Importieren der Pi-Abhängigkeiten: {e}")
        print(
            "Hinweis: Bitte mit 'uv run --group raspi ./mission_drop.py' oder '.venv/bin/python mission_drop.py' starten!"
        )
        sys.exit(1)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("mission_drop")

# Konstanten für das Kamerabild
IMG_WIDTH = 1280
IMG_HEIGHT = 960
CENTER_X = IMG_WIDTH / 2.0
CENTER_Y = IMG_HEIGHT / 2.0

# States
STATE_TAKEOFF = 0
STATE_SEARCH = 1
STATE_CENTER = 2
STATE_DROP = 3
STATE_LAND = 4


class DummyDrone:
    """Mock-Controller für Dry-Run Tests ohne angeschlossenen Flight Controller."""

    def __init__(self) -> None:
        self.current_altitude: float | None = 0.5
        self.battery_voltage: float | None = 15.2
        self.flight_mode: str | None = "GUIDED"
        self.is_armed: bool = False
        self.is_flying: bool = False
        self.forward_distance: float | None = None

    def __enter__(self) -> DummyDrone:
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        pass

    def update_telemetry(self) -> None:
        pass

    def takeoff(self, target_alt: float) -> None:
        pass

    def land(self) -> None:
        pass

    def send_velocity_body(
        self, vx: float, vy: float, vz: float, yaw_rate_deg: float = 0.0
    ) -> None:
        pass


class BoundedSearchPattern:
    """Verwaltet eine expandierende Suchspirale mit strikter 2-Meter-Grenze."""

    def __init__(
        self,
        speed: float = 0.15,
        step_time: float = 2.5,
        max_radius_m: float = 2.0,
    ) -> None:
        self.speed = speed
        self.step_time = step_time
        self.max_radius_m = max_radius_m
        self.current_leg = 1
        self.leg_start_time = 0.0
        self.is_finished = False

    def reset(self) -> None:
        self.current_leg = 1
        self.leg_start_time = 0.0
        self.is_finished = False

    def get_velocity(self, now: float) -> tuple[float, float, float, float]:
        if self.is_finished:
            return (0.0, 0.0, 0.0, 0.0)

        if self.leg_start_time == 0.0:
            self.leg_start_time = now

        multiplier = (self.current_leg + 1) // 2
        leg_duration = multiplier * self.step_time
        leg_distance = multiplier * (self.speed * self.step_time)

        current_radius = leg_distance / 2.0
        if current_radius > self.max_radius_m:
            logger.warning(
                "Suchgrenze (%.1f m Radius) erreicht! Beende Suche sicherheitshalber.",
                self.max_radius_m,
            )
            self.is_finished = True
            return (0.0, 0.0, 0.0, 0.0)

        dt = now - self.leg_start_time
        if dt >= leg_duration:
            self.current_leg += 1
            self.leg_start_time = now
            multiplier = (self.current_leg + 1) // 2
            logger.info(
                "Spirale: Schenkel %d | Dauer: %.1fs | Max-Abstand: ca. %.2fm",
                self.current_leg,
                multiplier * self.step_time,
                (multiplier * self.speed * self.step_time) / 2.0,
            )

        phase = (self.current_leg - 1) % 4
        if phase == 0:
            return (self.speed, 0.0, 0.0, 0.0)
        elif phase == 1:
            return (0.0, self.speed, 0.0, 0.0)
        elif phase == 2:
            return (-self.speed, 0.0, 0.0, 0.0)
        else:
            return (0.0, -self.speed, 0.0, 0.0)


def filter_valid_tags(
    detections: list[TagDetection], target_id: int | None = None
) -> TagDetection | None:
    """Filtert und meldet gefundene Tags mit Live-Log."""
    for det in detections:
        margin_str = (
            f"{det.decision_margin:.1f}" if det.decision_margin is not None else "N/A"
        )
        hamming_str = str(det.hamming) if det.hamming is not None else "N/A"
        logger.info(
            "Tag im Bild gesehen: ID=%d | Margin=%s | Hamming=%s | Pos=(%.0f, %.0f)",
            det.tag_id,
            margin_str,
            hamming_str,
            det.center[0],
            det.center[1],
        )

        if target_id is not None and det.tag_id != target_id:
            continue
        if det.hamming is not None and det.hamming > 1:
            continue
        if det.decision_margin is not None and det.decision_margin < 10.0:
            continue
        return det
    return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Autonome AprilTag Abwurf-Mission (mit sicherem --dry-run Modus)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Sicherer Trockentest am Boden: Motoren bleiben 100%% AUS!",
    )
    parser.add_argument(
        "--family",
        type=str,
        default="tag36h11",
        help="AprilTag Familie (Standard: tag36h11)",
    )
    parser.add_argument("--takeoff-alt", type=float, default=0.5, help="Starthöhe (m)")
    parser.add_argument(
        "--max-alt", type=float, default=0.8, help="Sicherheits-Höhenlimit (m)"
    )
    parser.add_argument(
        "--max-radius",
        type=float,
        default=2.0,
        help="Maximaler Suchradius um den Startpunkt in Metern",
    )
    parser.add_argument(
        "--min-battery",
        type=float,
        default=14.4,
        help="Kritische Mindestspannung (V) für Notlandung",
    )
    parser.add_argument(
        "--search-speed", type=float, default=0.15, help="Suchgeschwindigkeit (m/s)"
    )
    parser.add_argument(
        "--center-speed",
        type=float,
        default=0.2,
        help="Max Geschwindigkeit beim Zentrieren (m/s)",
    )
    parser.add_argument(
        "--max-search-time",
        type=float,
        default=60.0,
        help="Maximalzeit für Suche vor Landung (s)",
    )
    parser.add_argument(
        "--target-id",
        type=int,
        default=None,
        help="Gezielte AprilTag-ID (Standard: beliebiges valides Tag)",
    )
    parser.add_argument(
        "--servo-pin",
        type=int,
        default=18,
        help="BCM GPIO Pin für Servo (Standard: 18, physischer Pin 12)",
    )
    parser.add_argument(
        "--servo-closed",
        type=float,
        default=-0.5,
        help="Servo-Position geschlossen (-1.0 bis 1.0)",
    )
    parser.add_argument(
        "--servo-open",
        type=float,
        default=0.5,
        help="Servo-Position offen (-1.0 bis 1.0)",
    )
    parser.add_argument(
        "--device", type=str, default=None, help="UART/USB Schnittstelle für MAVLink"
    )
    args = parser.parse_args()

    if not _is_raspberry_pi():
        logger.error(
            "Dieses Skript muss direkt auf dem Raspberry Pi der Drohne ausgeführt werden!"
        )
        return 1

    # 1. Servo initialisieren
    logger.info("Initialisiere Servo auf GPIO %d...", args.servo_pin)
    try:
        servo = Servo(
            args.servo_pin, min_pulse_width=900 / 1e6, max_pulse_width=2100 / 1e6
        )
        servo.value = args.servo_closed
    except Exception as e:
        logger.warning(
            "Hinweis: Servo konnte nicht initialisiert werden (%s). Test läuft weiter.",
            e,
        )
        servo = None

    # 2. Kamera und Detektor initialisieren
    logger.info(
        "Initialisiere Kamera und AprilTag Detector (Familie: %s)...", args.family
    )
    detector = create_detector(backend="auto", family=args.family, threads=4)
    camera = Picamera2()
    camera.configure(
        camera.create_video_configuration(
            main={"format": "YUV420", "size": (IMG_WIDTH, IMG_HEIGHT)},
            controls={"FrameRate": 15},
            buffer_count=4,
            queue=False,
        )
    )

    # 3. Begrenztes Suchmuster initialisieren (max. 2.0 m Radius)
    searcher = BoundedSearchPattern(
        speed=args.search_speed,
        step_time=2.5,
        max_radius_m=args.max_radius,
    )

    if args.dry_run:
        logger.info(
            "===================================================================="
        )
        logger.info(
            "*** DRY-RUN MODUS AKTIV: Motoren bleiben AUS, kein Arming/Flug! ***"
        )
        logger.info(
            "===================================================================="
        )

    # Controller-Instanz auswählen (echt oder Dummy bei fehlendem FC)
    drone_context = None
    if args.dry_run:
        try:
            drone_context = DroneController(
                device=args.device, max_altitude=args.max_alt
            )
        except Exception:
            logger.info(
                "Kein Flight Controller gefunden -> Verwende Dummy-Controller für Dry-Run."
            )
            drone_context = DummyDrone()
    else:
        drone_context = DroneController(device=args.device, max_altitude=args.max_alt)

    try:
        with drone_context as drone:
            state = STATE_TAKEOFF

            # Tracking- & Filter-Variablen
            search_start_time = 0.0
            stable_center_time = 0.0
            last_seen_time = 0.0
            consecutive_detections = 0
            drop_start_time = 0.0
            drop_stage = 0

            camera.start()
            logger.info("Kamera gestartet. Starte Mission...")

            # --- MISSION LOOP ---
            while True:
                # 1. Telemetrie lesen
                drone.update_telemetry()

                # 2. Sicherheitsüberwachung: Batterie-Wächter
                if (
                    not args.dry_run
                    and drone.battery_voltage is not None
                    and drone.battery_voltage > 0.0
                    and drone.battery_voltage < args.min_battery
                ):
                    logger.error(
                        "BATTERIE KRITISCH (%.2f V < %.2f V)! Leite Notlandung ein.",
                        drone.battery_voltage,
                        args.min_battery,
                    )
                    state = STATE_LAND

                # Wenn Landung abgeschlossen -> Loop beenden
                if state == STATE_LAND and (args.dry_run or not drone.is_flying):
                    logger.info("Mission erfolgreich beendet.")
                    break

                # 3. Frame von der Kamera holen (Luma-Kanal)
                request = camera.capture_request()
                try:
                    yuv = request.make_array("main")
                finally:
                    request.release()

                luma = np.ascontiguousarray(yuv[:IMG_HEIGHT, :IMG_WIDTH])

                # 4. AprilTag erkennen & filtern
                raw_detections = detector.detect(luma)
                target = filter_valid_tags(raw_detections, target_id=args.target_id)

                if target is not None:
                    consecutive_detections += 1
                    last_seen_time = time.monotonic()
                else:
                    consecutive_detections = 0

                # --- STATE MACHINE LOGIC ---
                if state == STATE_TAKEOFF:
                    if args.dry_run:
                        logger.info(
                            ">>> [DRY-RUN] Simuliere Takeoff auf %.2f m (Motoren bleiben aus)",
                            args.takeoff_alt,
                        )
                        time.sleep(1.0)
                    else:
                        logger.info(">>> STATE: TAKEOFF auf %.2f m", args.takeoff_alt)
                        drone.takeoff(target_alt=args.takeoff_alt)

                    logger.info(
                        "Takeoff abgeschlossen. Beginne Suche (Max-Radius: %.1fm)...",
                        args.max_radius,
                    )
                    state = STATE_SEARCH
                    search_start_time = time.monotonic()
                    searcher.reset()

                elif state == STATE_SEARCH:
                    now = time.monotonic()
                    elapsed = now - search_start_time

                    # Zeitlimit oder Radiuslimit erreicht -> Landen
                    if elapsed > args.max_search_time or searcher.is_finished:
                        logger.warning(
                            "Suchgrenze erreicht (Zeit: %.1fs, Fertig: %s)! Leite Landung ein.",
                            elapsed,
                            searcher.is_finished,
                        )
                        state = STATE_LAND
                        continue

                    vx, vy, vz, yaw = searcher.get_velocity(now)

                    # In Dry-Run reicht 1 Frame, im Flug 2 Frames zur Stabilisierung
                    min_consec = 1 if args.dry_run else 2
                    if consecutive_detections >= min_consec and target is not None:
                        logger.info(
                            ">>> STATE: CENTER (Tag %d stabil erfasst!)", target.tag_id
                        )
                        state = STATE_CENTER
                        if not args.dry_run:
                            drone.send_velocity_body(0, 0, 0, 0)
                        stable_center_time = 0.0
                    else:
                        if not args.dry_run:
                            drone.send_velocity_body(vx, vy, vz, yaw)

                elif state == STATE_CENTER:
                    if target is None:
                        time_lost = time.monotonic() - last_seen_time
                        if time_lost > 2.5:
                            logger.warning(
                                "Tag seit %.1fs verloren! Kehre zu SEARCH zurück.",
                                time_lost,
                            )
                            state = STATE_SEARCH
                            search_start_time = time.monotonic()
                            searcher.reset()
                        else:
                            if not args.dry_run:
                                drone.send_velocity_body(0, 0, 0, 0)
                    else:
                        offset_x = target.center[0] - CENTER_X
                        offset_y = CENTER_Y - target.center[1]

                        # Proportional-Regler
                        kp = 0.0012
                        vx = offset_y * kp
                        vy = offset_x * kp

                        vx = max(-args.center_speed, min(args.center_speed, vx))
                        vy = max(-args.center_speed, min(args.center_speed, vy))

                        if not args.dry_run:
                            drone.send_velocity_body(vx, vy, 0.0, 0.0)
                        else:
                            logger.info(
                                "[DRY-RUN CENTER] Offset: dx=%+6.1fpx, dy=%+6.1fpx | Sim-Befehl: vx=%+5.2fm/s, vy=%+5.2fm/s",
                                offset_x,
                                offset_y,
                                vx,
                                vy,
                            )

                        # Stabilitätsprüfung (< 45 Pixel Toleranz für 1.5 Sekunden)
                        if abs(offset_x) < 45 and abs(offset_y) < 45:
                            if stable_center_time == 0.0:
                                stable_center_time = time.monotonic()
                            elif time.monotonic() - stable_center_time >= 1.5:
                                logger.info(
                                    ">>> STATE: DROP (Zentrierung stabil: dx=%.1fpx, dy=%.1fpx)",
                                    offset_x,
                                    offset_y,
                                )
                                state = STATE_DROP
                                drop_start_time = time.monotonic()
                                drop_stage = 0
                                if not args.dry_run:
                                    drone.send_velocity_body(0, 0, 0, 0)
                        else:
                            stable_center_time = 0.0

                elif state == STATE_DROP:
                    if not args.dry_run:
                        drone.send_velocity_body(0, 0, 0, 0)
                    now = time.monotonic()
                    dt = now - drop_start_time

                    if drop_stage == 0:
                        if dt >= 0.4:
                            logger.info(
                                "Löse Servo physisch aus -> Position %.2f",
                                args.servo_open,
                            )
                            if servo:
                                servo.value = args.servo_open
                            drop_stage = 1
                    elif drop_stage == 1:
                        if dt >= 1.9:
                            logger.info(
                                "Schließe Servo physisch -> Position %.2f",
                                args.servo_closed,
                            )
                            if servo:
                                servo.value = args.servo_closed
                            drop_stage = 2
                    elif drop_stage == 2:
                        if dt >= 2.4:
                            logger.info("Abwurf abgeschlossen. >>> STATE: LAND")
                            state = STATE_LAND

                elif state == STATE_LAND:
                    if args.dry_run:
                        logger.info(">>> [DRY-RUN] Simuliere Landung...")
                        time.sleep(1.0)
                        break
                    else:
                        if drone.is_flying:
                            drone.land()

                time.sleep(0.01)

    except KeyboardInterrupt:
        logger.warning(
            "Mission vom Benutzer (STRG+C) abgebrochen! DroneController führt Notlandung aus."
        )
    except Exception as e:
        logger.error("Kritischer Fehler während der Mission: %s", e)
    finally:
        camera.stop()
        camera.close()
        if servo:
            servo.close()
        logger.info("Kamera und Servo sicher geschlossen.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
