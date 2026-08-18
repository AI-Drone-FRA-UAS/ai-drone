"""Modulare MAVLink-Drohnensteuerung für den Raspberry Pi und lokale Testumgebungen.

Bietet die Klasse :class:`DroneController` zur sicheren Ansteuerung eines ArduPilot Copter
Flight Controllers über MAVLink. Unterstützt automatische Schnittstellenerkennung,
Non-Blocking-Telemetrie-Tracking, verifizierte Flugmoduswechsel, Schwebeflüge sowie
Body-Frame-Geschwindigkeitssteuerung für autonome Kamera-Missionen.
"""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.mavlink_devices import resolve_mavlink_endpoint

logger = logging.getLogger(__name__)


class DroneController:
    """Objektorientierter MAVLink-Controller für ArduPilot Copter.

    Implementiert Safety-Traps über einen Context-Manager (automatische Notlandung oder
    Disarming beim Verlassen des Kontextes im Flugzustand).

    Args:
        device: Pfad zum Serial-/USB-Gerät oder Netzwerk-String (z. B. 'udp:127.0.0.1:14550').
            Falls ``None``, wird automatisch nach einem passenden ArduPilot-Port gesucht.
        baud: Baudrate für die serielle Schnittstelle (Standard: 115200 für Pi UART4).
        max_altitude: Sicherheits-Höhenlimit in Metern (Standard: 0.8 m).
        target_system: MAVLink Target System ID (Standard: 1).
        target_component: MAVLink Target Component ID (Standard: 1).
    """

    def __init__(
        self,
        device: str | Path | None = None,
        baud: int = 115200,
        max_altitude: float = 0.8,
        target_system: int = 1,
        target_component: int = 1,
    ) -> None:
        self.device = self.find_device(device)
        self.baud = baud
        self.max_altitude = max_altitude
        self.target_system = target_system
        self.target_component = target_component

        self.connection: Any | None = None
        self.current_altitude: float | None = None
        self.battery_voltage: float | None = None
        self.flight_mode: str | None = None
        self.is_armed: bool = False
        self.is_flying: bool = False
        self.last_telemetry_time: float = 0.0

    @staticmethod
    def find_device(requested: str | Path | None) -> str:
        """Ermittlung der MAVLink-Schnittstelle.

        Durchsucht bei fehlender Vorgabe standardmäßige Linux-/Pi-Gerätepfade
        (z. B. ``/dev/serial0`` für Pi UART oder USB ArduPilot CDC).
        """
        try:
            device = resolve_mavlink_endpoint(
                requested,
                include_pi_uart=True,
                missing_message=(
                    "Kein ArduPilot Serial-Gerät gefunden. Bitte per --device "
                    "oder Parameter angeben."
                ),
            )
        except FileNotFoundError as error:
            if requested:
                raise
            raise RuntimeError(str(error)) from error

        if requested is None:
            logger.info("Automatische Schnittstellen-Erkennung: %s gefunden.", device)
        return device

    def __enter__(self) -> DroneController:
        self.connect()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        if exc_type is not None:
            logger.error("Ausnahme im DroneController-Kontext aufgetreten: %s", exc_val)
        try:
            if self.is_flying or self.is_armed:
                logger.warning(
                    "Safety-Trap: Verlasse Kontext im geschärften/fliegenden Zustand."
                )
                self.emergency_stop()
        finally:
            self.close()

    def connect(self) -> None:
        """Stellt die MAVLink-Verbindung her und fordert Telemetriestreams an."""
        logger.info(
            "Verbinde mit MAVLink-Schnittstelle %s (Baud: %d)...",
            self.device,
            self.baud,
        )
        self.connection = mavutil.mavlink_connection(self.device, baud=self.baud)

        logger.info("Warte auf Heartbeat vom Flight Controller...")
        self.connection.wait_heartbeat(timeout=15)
        ts = getattr(self.connection, "target_system", None)
        if isinstance(ts, int):
            self.target_system = ts
        tc = getattr(self.connection, "target_component", None)
        if isinstance(tc, int):
            self.target_component = tc
        logger.info(
            "Verbindung hergestellt. System ID: %s, Component ID: %s",
            self.target_system,
            self.target_component,
        )

        self.request_telemetry_streams()
        self.update_telemetry()

    def close(self) -> None:
        """Schließt die MAVLink-Verbindung sicher."""
        if self.connection:
            logger.info("Schließe MAVLink-Verbindung.")
            try:
                self.connection.close()
            except Exception as exc:
                logger.debug("Fehler beim Schließen der Verbindung: %s", exc)
            self.connection = None

    def request_telemetry_streams(self, rate_hz: int = 10) -> None:
        """Fordert Sensor- und Positionsdaten über MAVLink 2 (SET_MESSAGE_INTERVAL) an."""
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")

        interval_us = int(1_000_000 / max(1, rate_hz))
        message_ids = (
            mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavlink.MAVLINK_MSG_ID_RANGEFINDER,
            mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            mavlink.MAVLINK_MSG_ID_ATTITUDE,
        )

        for msg_id in message_ids:
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
        logger.info("Telemetrie-Streams angefordert (%d Hz).", rate_hz)

    def update_telemetry(self, max_messages: int = 50) -> None:
        """Liest eingehende MAVLink-Nachrichten nicht-blockierend aus und aktualisiert Attribute."""
        if not self.connection:
            return

        for _ in range(max_messages):
            msg = self.connection.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            now = time.monotonic()

            if msg_type == "HEARTBEAT":
                self.is_armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if hasattr(self.connection, "flightmode"):
                    self.flight_mode = self.connection.flightmode
            elif msg_type == "LOCAL_POSITION_NED":
                # In NED ist z nach unten gerichtet -> -z entspricht der Höhe über Startpunkt
                self.current_altitude = -float(msg.z)
                self.last_telemetry_time = now
            elif msg_type == "RANGEFINDER":
                if self.current_altitude is None:
                    self.current_altitude = float(msg.distance)
                self.last_telemetry_time = now
            elif msg_type == "DISTANCE_SENSOR":
                if self.current_altitude is None:
                    self.current_altitude = float(msg.current_distance) / 100.0
                self.last_telemetry_time = now
            elif msg_type == "SYS_STATUS":
                self.battery_voltage = float(msg.voltage_battery) / 1000.0

        # Safety Trap: Höhenwächter
        if (
            self.is_flying
            and self.current_altitude is not None
            and self.current_altitude > self.max_altitude
        ):
            logger.error(
                "Sicherheits-Limit überschritten! Höhe: %.2f m > Max: %.2f m. Leite Notlandung ein!",
                self.current_altitude,
                self.max_altitude,
            )
            self.emergency_stop()

    def wait_for_altitude(self, timeout: float = 3.0) -> float | None:
        """Wartetet blockierend bis zum Erhalt von Höhendaten."""
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.update_telemetry()
            if self.current_altitude is not None:
                return self.current_altitude
            time.sleep(0.05)
        return self.current_altitude

    def set_mode(self, mode_name: str, timeout: float = 5.0) -> None:
        """Wechselt den Flugmodus und verifiziert die Bestätigung des Flight Controllers."""
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")

        mode_mapping = self.connection.mode_mapping()
        if mode_name not in mode_mapping:
            raise ValueError(f"Flugmodus '{mode_name}' wird nicht unterstützt.")

        mode_id = mode_mapping[mode_name]
        logger.info("Sende Anforderung für Moduswechsel auf %s...", mode_name)
        self.connection.mav.set_mode_send(
            self.target_system,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.update_telemetry()
            if self.flight_mode == mode_name:
                logger.info("Modus erfolgreich auf %s gewechselt.", mode_name)
                return
            time.sleep(0.1)

        raise RuntimeError(f"Timeout beim Wechsel in Flugmodus '{mode_name}'.")

    def arm(self, timeout: float = 10.0) -> None:
        """Schärft die Motoren der Drohne im GUIDED-Modus."""
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")

        if self.flight_mode != "GUIDED":
            self.set_mode("GUIDED")

        logger.info("Sende Arming-Befehl...")
        self.connection.arducopter_arm()

        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.update_telemetry()
            if self.is_armed:
                logger.info("Drohne ist geschärft (ARMED).")
                return
            time.sleep(0.2)

        raise RuntimeError("Timeout beim Arming der Drohne.")

    def disarm(self, timeout: float = 10.0) -> None:
        """Entschärft die Motoren (DISARM)."""
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")

        logger.info("Sende Disarm-Befehl...")
        self.connection.arducopter_disarm()

        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.update_telemetry()
            if not self.is_armed:
                logger.info("Drohne ist entschärft (DISARMED).")
                self.is_flying = False
                return
            time.sleep(0.2)

        logger.warning("Disarm nach Timeout noch nicht bestätigt.")
        self.is_flying = False

    def emergency_stop(self) -> None:
        """Notfall-Abbruch: Schaltet unverzüglich in LAND oder disarmt am Boden."""
        logger.warning("=== EMERGENCY STOP AUSGELÖST ===")
        try:
            if self.connection:
                # Modus auf LAND erzwingen
                mode_mapping = self.connection.mode_mapping()
                if "LAND" in mode_mapping:
                    self.connection.mav.set_mode_send(
                        self.target_system,
                        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_mapping["LAND"],
                    )
                # Wenn am Boden oder Höhe sehr gering, zusätzlich disarmen
                if self.current_altitude is not None and self.current_altitude < 0.2:
                    self.connection.arducopter_disarm()
        except Exception as exc:
            logger.error("Fehler beim Senden des Emergency Stops: %s", exc)
        finally:
            self.is_flying = False

    def takeoff(self, target_alt: float, timeout: float = 15.0) -> None:
        """Führt einen autonomen Start im GUIDED-Modus auf die Zielhöhe durch.

        Args:
            target_alt: Zielhöhe in Metern (muss <= max_altitude sein).
            timeout: Maximal erlaubte Zeit für den Steigflug in Sekunden.
        """
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")
        if target_alt > self.max_altitude:
            raise ValueError(
                f"Zielhöhe {target_alt} m überschreitet Sicherheitslimit von {self.max_altitude} m!"
            )

        if self.flight_mode != "GUIDED":
            self.set_mode("GUIDED")
        if not self.is_armed:
            self.arm()

        logger.info("Sende Takeoff-Befehl auf %.2f m...", target_alt)
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            target_alt,
        )
        self.is_flying = True

        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.update_telemetry()

            # Prüfe, ob Telemetriestream noch aktiv ist
            if (
                self.last_telemetry_time > 0
                and (time.monotonic() - self.last_telemetry_time) > 2.0
            ):
                logger.error(
                    "Telemetrie-Abbruch während Takeoff! Leite Notlandung ein."
                )
                self.emergency_stop()
                raise RuntimeError("Telemetrie-Abbruch während des Starts.")

            if (
                self.current_altitude is not None
                and self.current_altitude >= target_alt * 0.95
            ):
                logger.info("Zielhöhe erreicht: %.2f m", self.current_altitude)
                return

            time.sleep(0.1)

        logger.error("Timeout beim Steigflug! Leite Landung ein.")
        self.emergency_stop()
        raise RuntimeError("Takeoff-Timeout überschritten.")

    def land(self, timeout: float = 20.0) -> None:
        """Wechselt in den LAND-Modus und wartet auf das Aufsetzen / Disarming."""
        logger.info("Leite Landung (LAND Modus) ein...")
        self.set_mode("LAND")

        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.update_telemetry()
            if not self.is_armed or (
                self.current_altitude is not None and self.current_altitude < 0.15
            ):
                logger.info("Landung abgeschlossen. Drohne ist am Boden.")
                self.is_flying = False
                return
            time.sleep(0.2)

        logger.warning(
            "Landung dauerte länger als %d s. Disarme sicherheitshalber.", timeout
        )
        try:
            self.disarm(timeout=3.0)
        except Exception:
            pass
        self.is_flying = False

    def send_velocity_body(
        self, vx: float, vy: float, vz: float, yaw_rate_deg: float = 0.0
    ) -> None:
        """Sendet Body-Frame Geschwindigkeits- und Gierratenbefehle an die Drohne.

        Ideale Schnittstelle für die Kamerablickrichtung (IMX500 AI Detections).

        Args:
            vx: Vorwärts-/Rückwärts-Geschwindigkeit in m/s (+ Vorwärts, - Rückwärts).
            vy: Links-/Rechts-Geschwindigkeit in m/s (+ Rechts, - Links).
            vz: Vertikale Geschwindigkeit in m/s (+ Unten/Sinken, - Oben/Steigen).
            yaw_rate_deg: Gier-Rate in Grad/Sekunde (+ Rechtsdrehung, - Linksdrehung).
        """
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")
        if not self.is_flying or not self.is_armed:
            logger.warning(
                "Ignoriere Geschwindigkeitsbefehl: Drohne ist nicht im Flug."
            )
            return

        # Bitmaske 0x05C7 (1479): Ignoriere Position (Bit 0-2), Beschleunigung (Bit 6-8)
        # und Gier-Winkel (Bit 10). Aktiviert nur vx, vy, vz (Bit 3-5) & Gier-Rate (Bit 11).
        type_mask = 0x05C7
        yaw_rate_rad = math.radians(yaw_rate_deg)

        self.connection.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0,
            0,
            0,
            vx,
            vy,
            vz,
            0,
            0,
            0,
            0,
            yaw_rate_rad,
        )
        logger.debug(
            "Velocity Body gesendet: vx=%.2f, vy=%.2f, vz=%.2f, yaw_rate=%.1f°/s",
            vx,
            vy,
            vz,
            yaw_rate_deg,
        )
