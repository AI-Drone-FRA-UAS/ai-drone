"""Modul für autonomes visuelles Tracking und Personenfolge mit der IMX500 AI-Kamera.

Verbindet die Detektionen der NanoDet-Modelle (modlib) mit den Body-Frame-Geschwindigkeitsbefehlen
des DroneControllers. Enthält strikte Sicherheitswächter (Batterie, Höhe, Target-Loss-Timeout).
"""

from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING, Any, NamedTuple

if TYPE_CHECKING:
    from ai_drone.controller import DroneController

logger = logging.getLogger(__name__)

# COCO Class ID für "Person" im NanoDet Modell
PERSON_CLASS_ID = 0


class PersonTarget(NamedTuple):
    """Repräsentiert die relative Position einer erkannten Person bezüglich der Kamera."""

    distance_m: float  #: Geschätzte Distanz zur Person in Metern
    offset_x_px: float  #: Abweichung des Box-Zentrums von der Bildmitte X in Pixeln (+ ist rechts)
    offset_y_px: float  #: Abweichung des Box-Zentrums von der Bildmitte Y in Pixeln (+ ist unten)
    confidence: float  #: Konfidenzwert der Detektion (0.0 bis 1.0)
    box_area: float  #: Flächeninhalt der Bounding Box in Quadratpixeln


def get_person_target(
    detections: Any,
    frame_width: int,
    frame_height: int,
    focal_length_px: float | None = None,
    default_person_height_m: float = 1.70,
    min_confidence: float = 0.40,
) -> PersonTarget | None:
    """Extrahiert aus einer Liste von Kamera-Detektionen die relevanteste (nächste) Person.

    Schätzt die Entfernung über ein vereinfachtes Lochkameramodell anhand der Bounding-Box-Höhe.
    """
    if detections is None or len(detections) == 0:
        return None

    # Filter nach Konfidenz und Personen-Klasse (falls Attribute vorhanden sind)
    valid_targets: list[tuple[float, PersonTarget]] = []
    center_x_img = frame_width / 2.0
    center_y_img = frame_height / 2.0

    # Approximierte Brennweite für 66° HFOV falls nicht angegeben
    if focal_length_px is None:
        focal_length_px = (frame_width / 2.0) / math.tan(math.radians(66.0) / 2.0)

    # Iteriere über alle Detektionen (unterstützt modlib/numpy Record-Arrays oder Mock-Objekte)
    for i in range(len(detections)):
        det = detections[i]
        conf = float(getattr(det, "confidence", 1.0))
        cls_id = int(getattr(det, "class_id", PERSON_CLASS_ID))

        if conf < min_confidence or cls_id != PERSON_CLASS_ID:
            continue

        # Extrahiere Bounding Box Coordinaten: x1, y1, x2, y2 oder box-Attribut
        if hasattr(det, "box"):
            box = det.box
            x1, y1, x2, y2 = float(box[0]), float(box[1]), float(box[2]), float(box[3])
        elif hasattr(det, "xyxy"):
            box = det.xyxy
            x1, y1, x2, y2 = float(box[0]), float(box[1]), float(box[2]), float(box[3])
        else:
            # Fallback für direkte Indizierung [x1, y1, x2, y2, ...]
            try:
                x1, y1, x2, y2 = float(det[0]), float(det[1]), float(det[2]), float(det[3])
            except (IndexError, TypeError, ValueError):
                continue

        box_w = max(1.0, x2 - x1)
        box_h = max(1.0, y2 - y1)
        box_area = box_w * box_h

        # Zentrum der Bounding Box
        xc = x1 + box_w / 2.0
        yc = y1 + box_h / 2.0

        offset_x = xc - center_x_img
        offset_y = yc - center_y_img

        # Distanzabschätzung über die Höhe im Bild (m = (real_h * focal) / img_h)
        dist_m = (default_person_height_m * focal_length_px) / box_h
        dist_m = round(max(0.3, min(15.0, dist_m)), 2)

        target = PersonTarget(
            distance_m=dist_m,
            offset_x_px=offset_x,
            offset_y_px=offset_y,
            confidence=conf,
            box_area=box_area,
        )
        # Priorisiere die Person mit der größten Box-Fläche (nächste Person)
        valid_targets.append((box_area, target))

    if not valid_targets:
        return None

    # Sortiere nach Box-Fläche absteigend und gib das größte Target zurück
    valid_targets.sort(key=lambda item: item[0], reverse=True)
    return valid_targets[0][1]


class AutonomousFollower:
    """Regler und Sicherheitswächter für autonomes Verfolgen von Personen im Body Frame."""

    def __init__(
        self,
        drone: DroneController,
        target_dist_m: float = 2.0,
        max_vx: float = 0.3,
        max_yaw_rate_deg: float = 20.0,
        kp_dist: float = 0.4,
        kp_yaw: float = 0.25,
        lost_timeout_s: float = 3.0,
        min_battery_v: float = 14.4,
    ) -> None:
        """Initialisiert den autonomen Follower-Regler.

        Args:
            drone: Aktive Instanz des DroneControllers.
            target_dist_m: Gewünschter Halteabstand zur Person in Metern.
            max_vx: Maximale Vorwärts-/Rückwärtsgeschwindigkeit in m/s (Sicherheitsgrenze).
            max_yaw_rate_deg: Maximale Drehgeschwindigkeit in °/s.
            kp_dist: P-Regler-Verstärkung für die Distanzregelung.
            kp_yaw: P-Regler-Verstärkung für die Ausrichtungsregelung (Yaw).
            lost_timeout_s: Zeit ohne Sichtkontakt bis zum autonomen Schwebeflug/Abbruch.
            min_battery_v: Kritische Batteriespannung für automatische Notlandung.
        """
        self.drone = drone
        self.target_dist_m = target_dist_m
        self.max_vx = max_vx
        self.max_yaw_rate_deg = max_yaw_rate_deg
        self.kp_dist = kp_dist
        self.kp_yaw = kp_yaw
        self.lost_timeout_s = lost_timeout_s
        self.min_battery_v = min_battery_v

        self._last_target_time = time.monotonic()
        self._is_tracking = False

    def compute_velocity_command(
        self, target: PersonTarget | None, now: float | None = None
    ) -> tuple[float, float, float, float]:
        """Berechnet die Geschwindigkeitsbefehle (vx, vy, vz, yaw_rate_deg) basierend auf dem Target.

        Returns:
            Tuple (vx, vy, vz, yaw_rate_deg) im Body Frame.
        """
        if now is None:
            now = time.monotonic()

        if target is None:
            time_lost = now - self._last_target_time
            if time_lost >= self.lost_timeout_s and self._is_tracking:
                logger.warning(
                    "Target seit %.1f s verloren! Halte Position (Hover).", time_lost
                )
                self._is_tracking = False
            # Bei Verlust auf der Stelle schweben
            return (0.0, 0.0, 0.0, 0.0)

        self._last_target_time = now
        self._is_tracking = True

        # 1. Distanzregelung (Vorwärts / Rückwärts)
        # Wenn distance > target_dist -> vorwärts (+vx), wenn distance < target_dist -> rückwärts (-vx)
        dist_error = target.distance_m - self.target_dist_m
        vx = dist_error * self.kp_dist
        vx = max(-self.max_vx, min(self.max_vx, vx))
        # Totzone um Schwingungen zu vermeiden
        if abs(dist_error) < 0.15:
            vx = 0.0

        # 2. Ausrichtungsregelung (Yaw Rate)
        # Wenn offset_x > 0 (Person rechts) -> drehe nach rechts (+yaw_rate)
        yaw_rate = target.offset_x_px * self.kp_yaw
        yaw_rate = max(-self.max_yaw_rate_deg, min(self.max_yaw_rate_deg, yaw_rate))
        if abs(target.offset_x_px) < 15.0:
            yaw_rate = 0.0

        return (round(vx, 3), 0.0, 0.0, round(yaw_rate, 2))

    def check_safety_guardrails(self) -> None:
        """Prüft Höhe und Batterie im laufenden Betrieb. Löst bei Gefahr sofort Notlandung aus."""
        # 1. Batterie-Wächter
        if self.drone.battery_voltage is not None and self.drone.battery_voltage > 0.0:
            if self.drone.battery_voltage < self.min_battery_v:
                logger.error(
                    "BATTERIE KRITISCH (%.2f V < %.2f V)! Leite sofortige Notlandung ein.",
                    self.drone.battery_voltage,
                    self.min_battery_v,
                )
                self.drone.emergency_stop()
                raise RuntimeError("Abbruch durch Batterie-Wächter.")

        # 2. Höhen-Wächter
        if self.drone.current_altitude is not None:
            if self.drone.current_altitude > self.drone.max_altitude:
                logger.error(
                    "HÖHENLIMIT ÜBERSCHRITTEN (%.2f m > %.2f m)! Leite Notlandung ein.",
                    self.drone.current_altitude,
                    self.drone.max_altitude,
                )
                self.drone.emergency_stop()
                raise RuntimeError("Abbruch durch Höhen-Wächter.")

    def run_simulated_tracking(self, duration_s: float = 15.0) -> None:
        """Führt eine simulierte Tracking-Schleife am Schreibtisch aus (ohne Kamera).

        Erzeugt synthetische PersonTarget-Daten zur Verifikation des Steuerkreisverhaltens.
        """
        logger.info("=== Starte simulierten Autonomie-Tracking-Test (Dauer: %.1f s) ===", duration_s)
        started = time.monotonic()

        try:
            while time.monotonic() - started < duration_s:
                self.drone.update_telemetry()
                self.check_safety_guardrails()

                elapsed = time.monotonic() - started
                # Simuliere eine Person, die von 3.5m auf 1.5m zukommt und leicht hin und her pendelt
                sim_dist = 3.5 - 2.0 * (elapsed / duration_s)
                sim_offset_x = 50.0 * math.sin(elapsed * 1.5)

                target = PersonTarget(
                    distance_m=round(sim_dist, 2),
                    offset_x_px=round(sim_offset_x, 1),
                    offset_y_px=0.0,
                    confidence=0.85,
                    box_area=15000.0,
                )

                vx, vy, vz, yaw_rate = self.compute_velocity_command(target)
                logger.info(
                    "[SIM] Target: Dist=%.2fm, OffsetX=%.1fpx | Befehl: vx=%.2fm/s, yaw=%.1f°/s",
                    target.distance_m,
                    target.offset_x_px,
                    vx,
                    yaw_rate,
                )

                if self.drone.is_flying and self.drone.is_armed:
                    self.drone.send_velocity_body(vx, vy, vz, yaw_rate)

                time.sleep(0.1)

        except KeyboardInterrupt:
            logger.warning("Simuliertes Tracking manuell abgebrochen.")
        finally:
            logger.info("Beende Tracking-Schleife. Stoppe Bewegung...")
            if self.drone.is_flying:
                self.drone.send_velocity_body(0.0, 0.0, 0.0, 0.0)

    def run_live_tracking(
        self,
        confidence: float = 0.40,
        max_duration_s: float | None = None,
    ) -> None:
        """Führt das echte visuelle Tracking mit der IMX500 AI-Kamera und NanoDet durch."""
        try:
            from modlib.devices import AiCamera  # ty: ignore[unresolved-import]
            from modlib.models.zoo import (  # ty: ignore[unresolved-import]
                NanoDetPlus416x416,
            )
        except ImportError as exc:
            raise RuntimeError(
                "modlib ist nicht verfügbar. Echter Kamera-Betrieb nur auf dem Raspberry Pi möglich."
            ) from exc

        logger.info("Deploye NanoDet Modell auf der IMX500 Kamera...")
        model = NanoDetPlus416x416()
        device = AiCamera()
        device.deploy(model)

        started = time.monotonic()
        logger.info("=== Starte Live Person-Tracking ===")

        try:
            with device as stream:
                for frame in stream:
                    if max_duration_s and (time.monotonic() - started > max_duration_s):
                        logger.info("Maximale Tracking-Dauer (%.1f s) erreicht.", max_duration_s)
                        break

                    self.drone.update_telemetry()
                    self.check_safety_guardrails()

                    target = get_person_target(
                        detections=frame.detections,
                        frame_width=frame.width,
                        frame_height=frame.height,
                        min_confidence=confidence,
                    )

                    vx, vy, vz, yaw_rate = self.compute_velocity_command(target)
                    if target:
                        logger.info(
                            "Person erkannt: Dist=%.2fm, OffsetX=%.1fpx | Befehl: vx=%.2fm/s, yaw=%.1f°/s",
                            target.distance_m,
                            target.offset_x_px,
                            vx,
                            yaw_rate,
                        )
                    else:
                        logger.debug("Keine Person in Sicht. Schwebe auf der Stelle.")

                    if self.drone.is_flying and self.drone.is_armed:
                        self.drone.send_velocity_body(vx, vy, vz, yaw_rate)
        except KeyboardInterrupt:
            logger.warning("Live-Tracking manuell abgebrochen.")
        finally:
            logger.info("Beende Live-Tracking. Stoppe Bewegung...")
            if self.drone.is_flying:
                self.drone.send_velocity_body(0.0, 0.0, 0.0, 0.0)
