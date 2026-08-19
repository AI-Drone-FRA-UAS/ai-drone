#!/usr/bin/env python3
"""Autonome Abwurf-Mission für die ai-drone mit aktiver Hinderniserkennung (MT-15).

Fliegt ein expandierendes Spiral-Suchmuster (Expanding Square Search) ab,
um von jedem beliebigen Startpunkt aus die gesamte Halle 360° abzusuchen.
Weicht Wänden automatisch aus (MT-15 ToF nach vorne), erkennt das AprilTag,
zentriert sich exakt darüber und löst den Servo (GPIO 12) aus.
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


class SearchPattern:
    """Verwaltet das Suchmuster mit Unterstützung für dynamische Hindernisausweichung.

    Unterstützt ein expandierendes Spiral-Muster (Expanding Square Search):
    Leg 1: Vorwärts (1 * T)
    Leg 2: Rechts    (1 * T)
    Leg 3: Rückwärts (2 * T)
    Leg 4: Links     (2 * T)
    Leg 5: Vorwärts (3 * T)
    Leg 6: Rechts    (3 * T)
    ...
    Wird vor der Drohne eine Wand erkannt, kann der Schenkel mit `advance_to_next_leg()`
    vorzeitig abgebrochen werden, sodass die Drohne sofort nach rechts/links abbiegt.
    """

    def __init__(
        self,
        speed: float = 0.15,
        step_time: float = 3.0,
        pattern: str = "spiral",
    ) -> None:
        self.speed = speed
        self.step_time = step_time
        self.pattern = pattern
        self.current_leg = 1
        self.leg_start_time = 0.0

    def reset(self) -> None:
        self.current_leg = 1
        self.leg_start_time = 0.0

    def advance_to_next_leg(self, now: float) -> None:
        """Schaltet bei Wandanäherung sofort auf den nächsten Schenkel (Abbiegen)."""
        self.current_leg += 1
        self.leg_start_time = now
        multiplier = (self.current_leg + 1) // 2
        logger.info(
            "Wand-Ausweichmanöver: Schalte vorzeitig auf Schenkel %d (Dauer: %.1fs)",
            self.current_leg,
            multiplier * self.step_time,
        )

    def get_velocity(self, now: float) -> tuple[float, float, float, float]:
        if self.leg_start_time == 0.0:
            self.leg_start_time = now

        if self.pattern == "spiral":
            multiplier = (self.current_leg + 1) // 2
            leg_duration = multiplier * self.step_time
            dt = now - self.leg_start_time

            if dt >= leg_duration:
                self.advance_to_next_leg(now)

            # 4 Phasen: 0=Vorwärts, 1=Rechts, 2=Rückwärts, 3=Links
            phase = (self.current_leg - 1) % 4
            if phase == 0:
                return (self.speed, 0.0, 0.0, 0.0)
            elif phase == 1:
                return (0.0, self.speed, 0.0, 0.0)
            elif phase == 2:
                return (-self.speed, 0.0, 0.0, 0.0)
            else:
                return (0.0, -self.speed, 0.0, 0.0)

        else:  # lawnmower (Schlangenmuster)
            t_forward = 8.0
            t_side = 2.0
            cycle = (t_forward + t_side) * 2
            dt = now - self.leg_start_time
            t_in = dt % cycle

            if t_in < t_forward:
                return (self.speed, 0.0, 0.0, 0.0)
            elif t_in < t_forward + t_side:
                return (0.0, self.speed, 0.0, 0.0)
            elif t_in < 2 * t_forward + t_side:
                return (-self.speed, 0.0, 0.0, 0.0)
            else:
                return (0.0, self.speed, 0.0, 0.0)


def filter_valid_tags(
    detections: list[TagDetection], target_id: int | None = None
) -> TagDetection | None:
    """Filtert Rauschen und prüft Hamming-Distanz sowie Ziel-ID."""
    for det in detections:
        # Falls spezifische ID gefordert
        if target_id is not None and det.tag_id != target_id:
            continue
        # Native AprilTag3 liefert hamming; 0 bedeutet fehlerfrei decodiert
        if det.hamming is not None and det.hamming > 0:
            continue
        # Decision Margin filtern (unter 25 ist unsicher/Rauschen)
        if det.decision_margin is not None and det.decision_margin < 25.0:
            continue
        return det
    return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Autonome AprilTag Abwurf-Mission für AI-Drone mit MT-15 Kollisionsschutz",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--takeoff-alt", type=float, default=0.6, help="Starthöhe (m)")
    parser.add_argument(
        "--max-alt", type=float, default=1.0, help="Sicherheits-Höhenlimit (m)"
    )
    parser.add_argument(
        "--min-battery",
        type=float,
        default=14.4,
        help="Kritische Mindestspannung (V) für Notlandung",
    )
    parser.add_argument(
        "--min-wall-dist",
        type=float,
        default=1.0,
        help="Mindestabstand zur Wand (m) für MT-15 ToF Sensor",
    )
    parser.add_argument(
        "--pattern",
        choices=("spiral", "lawnmower"),
        default="spiral",
        help="Suchmuster: 'spiral' (Expanding Square ab Startpunkt) oder 'lawnmower'",
    )
    parser.add_argument(
        "--step-time",
        type=float,
        default=3.0,
        help="Basis-Schenkeldauer für die Suchspirale in Sekunden",
    )
    parser.add_argument(
        "--search-speed", type=float, default=0.15, help="Suchgeschwindigkeit (m/s)"
    )
    parser.add_argument(
        "--max-search-time",
        type=float,
        default=90.0,
        help="Maximalzeit für Suche vor automatischem Abbruch (s)",
    )
    parser.add_argument(
        "--center-speed",
        type=float,
        default=0.2,
        help="Max Geschwindigkeit beim Zentrieren (m/s)",
    )
    parser.add_argument(
        "--target-id",
        type=int,
        default=None,
        help="Gezielte AprilTag-ID (Standard: beliebiges valides Tag)",
    )
    parser.add_argument(
        "--servo-pin", type=int, default=12, help="BCM GPIO Pin für Servo"
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
        logger.error("Fehler beim Initialisieren des Servos: %s", e)
        return 1

    # 2. Kamera und Detektor initialisieren
    logger.info("Initialisiere Kamera und AprilTag Detector...")
    detector = create_detector(backend="auto", threads=4)
    camera = Picamera2()
    camera.configure(
        camera.create_video_configuration(
            main={"format": "YUV420", "size": (IMG_WIDTH, IMG_HEIGHT)},
            controls={"FrameRate": 15},
            buffer_count=4,
            queue=False,
        )
    )

    # 3. Suchmuster-Generator initialisieren
    searcher = SearchPattern(
        speed=args.search_speed,
        step_time=args.step_time,
        pattern=args.pattern,
    )

    logger.info("Starte MAVLink Verbindung (STRG+C für Not-Aus)...")
    try:
        with DroneController(device=args.device, max_altitude=args.max_alt) as drone:
            state = STATE_TAKEOFF

            # Tracking- & Filter-Variablen
            search_start_time = 0.0
            stable_center_time = 0.0
            last_seen_time = 0.0
            consecutive_detections = 0
            drop_start_time = 0.0
            drop_stage = 0
            last_wall_warning = 0.0

            camera.start()
            logger.info(
                "Kamera gestartet. Modus: %s (Max-Dauer: %.1fs, Wand-Limit: %.1fm)...",
                args.pattern.upper(),
                args.max_search_time,
                args.min_wall_dist,
            )

            # --- MISSION LOOP ---
            while True:
                # 1. Telemetrie lesen
                drone.update_telemetry()

                # 2. Sicherheitsüberwachung: Batterie-Wächter
                if (
                    drone.battery_voltage is not None
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
                if state == STATE_LAND and not drone.is_flying:
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
                    logger.info(">>> STATE: TAKEOFF auf %.2f m", args.takeoff_alt)
                    drone.takeoff(target_alt=args.takeoff_alt)
                    logger.info(
                        "Takeoff abgeschlossen. Beginne Suche (%s)...", args.pattern
                    )
                    state = STATE_SEARCH
                    search_start_time = time.monotonic()
                    searcher.reset()

                elif state == STATE_SEARCH:
                    now = time.monotonic()
                    elapsed = now - search_start_time

                    # Timeout-Prüfung für die Suche
                    if elapsed > args.max_search_time:
                        logger.warning(
                            "Suchzeit-Limit (%.1fs) überschritten! Breche Mission ab und lande.",
                            args.max_search_time,
                        )
                        state = STATE_LAND
                        continue

                    vx, vy, vz, yaw = searcher.get_velocity(now)

                    # MT-15 Wand-Kollisionsüberwachung:
                    # Fliegt die Drohne vorwärts (vx > 0) und die Wand ist zu nah -> vorzeitig abbiegen!
                    if vx > 0.0 and drone.forward_distance is not None:
                        if drone.forward_distance < args.min_wall_dist:
                            if now - last_wall_warning > 1.0:
                                logger.warning(
                                    "WAND ERKANNT in %.2f m (< %.2f m)! Breche Vorwärts-Schenkel ab und biege ab.",
                                    drone.forward_distance,
                                    args.min_wall_dist,
                                )
                                last_wall_warning = now
                            searcher.advance_to_next_leg(now)
                            vx, vy, vz, yaw = searcher.get_velocity(now)

                    # Robustheits-Filter: mindestens 3 aufeinanderfolgende Frames
                    if consecutive_detections >= 3 and target is not None:
                        logger.info(
                            ">>> STATE: CENTER (Tag %d stabil erkannt!)", target.tag_id
                        )
                        state = STATE_CENTER
                        drone.send_velocity_body(0, 0, 0, 0)
                        stable_center_time = 0.0
                    else:
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
                            # Kurzzeitiges Schweben bei Frame-Verlust
                            drone.send_velocity_body(0, 0, 0, 0)
                    else:
                        # Pixel-Offset zur Bildmitte berechnen (Nadir-Mount)
                        # Bild-X (Rechts) -> Body-Y (Rechts)
                        # Bild-Y (Unten)  -> Body-X (Rückwärts, d.h. Oben ist Vorwärts)
                        offset_x = target.center[0] - CENTER_X
                        offset_y = CENTER_Y - target.center[1]

                        # Proportional-Regler
                        kp = 0.0012
                        vx = offset_y * kp
                        vy = offset_x * kp

                        # MT-15 Sicherheitsgrenze im Center-Modus:
                        # Verhindert, dass die Drohne bei der Zentrierung zu nah an eine Wand fährt
                        if (
                            vx > 0.0
                            and drone.forward_distance is not None
                            and drone.forward_distance < 0.6
                        ):
                            logger.warning(
                                "Wand zu nah (%.2fm) während Zentrierung! Begrenze Vorwärtsflug.",
                                drone.forward_distance,
                            )
                            vx = 0.0

                        # Geschwindigkeiten kappen
                        vx = max(-args.center_speed, min(args.center_speed, vx))
                        vy = max(-args.center_speed, min(args.center_speed, vy))

                        drone.send_velocity_body(vx, vy, 0.0, 0.0)

                        # Stabilitätsprüfung (< 35 Pixel Toleranz für 1.8 Sekunden)
                        if abs(offset_x) < 35 and abs(offset_y) < 35:
                            if stable_center_time == 0.0:
                                stable_center_time = time.monotonic()
                            elif time.monotonic() - stable_center_time >= 1.8:
                                logger.info(
                                    ">>> STATE: DROP (Zentrierung stabil: dx=%.1fpx, dy=%.1fpx)",
                                    offset_x,
                                    offset_y,
                                )
                                state = STATE_DROP
                                drop_start_time = time.monotonic()
                                drop_stage = 0
                                drone.send_velocity_body(0, 0, 0, 0)
                        else:
                            stable_center_time = 0.0

                elif state == STATE_DROP:
                    # Non-blocking Abwurf-Sequenz mit fortlaufender Telemetrie
                    drone.send_velocity_body(0, 0, 0, 0)
                    now = time.monotonic()
                    dt = now - drop_start_time

                    if drop_stage == 0:
                        # 0.4s Vorlauf zum Einschwingen
                        if dt >= 0.4:
                            logger.info(
                                "Löse Servo aus -> Position %.2f", args.servo_open
                            )
                            servo.value = args.servo_open
                            drop_stage = 1
                    elif drop_stage == 1:
                        # 1.5s Offenhalten für sicheren Abwurf
                        if dt >= 1.9:
                            logger.info(
                                "Schließe Servo -> Position %.2f", args.servo_closed
                            )
                            servo.value = args.servo_closed
                            drop_stage = 2
                    elif drop_stage == 2:
                        # 0.5s Nachlauf
                        if dt >= 2.4:
                            logger.info("Abwurf abgeschlossen. >>> STATE: LAND")
                            state = STATE_LAND

                elif state == STATE_LAND:
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
        servo.close()
        logger.info("Kamera und Servo sicher geschlossen.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
