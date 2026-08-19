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
        self.rangefinder_distance: float | None = None
        self.ekf_altitude: float | None = None
        self.forward_distance: float | None = None
        self.battery_voltage: float | None = None
        self.flight_mode: str | None = None
        self.last_status_text: str | None = None
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
                self.ekf_altitude = -float(msg.z)
                if self.rangefinder_distance is None:
                    self.current_altitude = self.ekf_altitude
                self.last_telemetry_time = now
            elif msg_type == "RANGEFINDER":
                dist = float(msg.distance)
                if dist > 0.0:
                    self.rangefinder_distance = dist
                    self.current_altitude = dist
                self.last_telemetry_time = now
            elif msg_type == "DISTANCE_SENSOR":
                orientation = getattr(msg, "orientation", 25)
                dist_m = float(msg.current_distance) / 100.0
                if orientation == 0:  # MAV_SENSOR_ROTATION_NONE = Forward (0)
                    self.forward_distance = dist_m
                elif orientation == 25:  # MAV_SENSOR_ROTATION_PITCH_270 = Downward (25)
                    if dist_m > 0.0:
                        self.rangefinder_distance = dist_m
                        self.current_altitude = dist_m
                else:
                    if dist_m > 0.0:
                        self.rangefinder_distance = dist_m
                        self.current_altitude = dist_m
                self.last_telemetry_time = now
            elif msg_type == "STATUSTEXT":
                text = getattr(msg, "text", "")
                if isinstance(text, bytes):
                    text = text.decode("utf-8", errors="replace")
                text = text.strip()
                logger.warning("[Flight Controller] %s", text)
                self.last_status_text = text
                self.last_telemetry_time = now
            elif msg_type == "COMMAND_ACK":
                cmd = getattr(msg, "command", 0)
                result = getattr(msg, "result", 0)
                logger.info(
                    "[Flight Controller ACK] Command %d -> Result %d", cmd, result
                )
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

    def send_origin(
        self,
        lat: float = 50.1300,
        lon: float = 8.6900,
        alt: float = 100.0,
    ) -> None:
        """Sendet EKF Global Origin an ArduPilot für Non-GPS / Optical-Flow Navigation."""
        if not self.connection:
            return

        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        alt_int = int(alt * 1000)

        # 1. SET_GPS_GLOBAL_ORIGIN
        self.connection.mav.set_gps_global_origin_send(
            self.target_system,
            lat_int,
            lon_int,
            alt_int,
        )
        # 2. SET_HOME_POSITION
        self.connection.mav.set_home_position_send(
            self.target_system,
            lat_int,
            lon_int,
            alt_int,
            0,
            0,
            0,
            [1.0, 0.0, 0.0, 0.0],
            0,
            0,
            0,
        )
        logger.info("EKF Global Origin gesetzt (Lat: %.4f, Lon: %.4f).", lat, lon)

    def arm(
        self,
        timeout: float = 10.0,
        force: bool = False,
        mode: str = "GUIDED",
    ) -> None:
        """Schärft die Motoren der Drohne im gewählten Modus (Standard: GUIDED)."""
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")

        # Bei Non-GPS / Optical Flow immer Origin senden für Positionsschätzung
        self.send_origin()
        time.sleep(0.2)

        if self.flight_mode != mode:
            mode_map = self.connection.mode_mapping()
            # Wenn GUIDED gewünscht ist, aber GUIDED_NOGPS existiert bei reinem Optical Flow:
            target_mode = mode
            if (
                mode == "GUIDED"
                and "GUIDED_NOGPS" in mode_map
                and "GUIDED" not in mode_map
            ):
                target_mode = "GUIDED_NOGPS"
            self.set_mode(target_mode)

        logger.info(
            "Sende Arming-Befehl (Modus: %s, force=%s)...", self.flight_mode, force
        )
        if force:
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,  # 1 = Arm
                21196,  # 21196 = Force Arming (Bypass PreArm)
                0,
                0,
                0,
                0,
                0,
            )
        else:
            self.connection.arducopter_arm()

        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.update_telemetry()
            if self.is_armed:
                logger.info("Drohne ist geschärft (ARMED).")
                return
            time.sleep(0.2)

        reason = (
            f" (Meldung vom FC: '{self.last_status_text}')"
            if self.last_status_text
            else ""
        )
        raise RuntimeError(f"Timeout beim Arming der Drohne{reason}.")

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
                if self.current_altitude is not None and self.current_altitude < 0.2:
                    self.connection.arducopter_disarm()
        except Exception as exc:
            logger.error("Fehler beim Senden des Emergency Stops: %s", exc)
        finally:
            self.is_flying = False

    def set_rc_override(
        self,
        roll: int = 1500,
        pitch: int = 1500,
        throttle: int = 1500,
        yaw: int = 1500,
    ) -> None:
        """Sendet RC-Kanal-Overrides an ArduPilot (1000-2000, 1500=Neutral/Schweben)."""
        if not self.connection:
            return
        self.connection.mav.rc_channels_override_send(
            self.target_system,
            self.target_component,
            int(roll),
            int(pitch),
            int(throttle),
            int(yaw),
            0,
            0,
            0,
            0,
        )

    def clear_rc_override(self) -> None:
        """Löscht alle aktiven RC Overrides."""
        if not self.connection:
            return
        self.connection.mav.rc_channels_override_send(
            self.target_system,
            self.target_component,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def hard_emergency_kill(self) -> None:
        """Sofortiger NOT-AUS (Kill Switch): Schneidet den Motorstrom unverzüglich ab."""
        logger.warning("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        logger.warning("  SOFORT-NOT-AUS: MOTOREN WERDEN SOFORT ABGESCHALTET!  ")
        logger.warning("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        try:
            self.clear_rc_override()
            if self.connection:
                self.set_rc_override(1500, 1500, 1000, 1500)
                # Force Disarm via MAVLink (param2=21196 kill in mid-air)
                self.connection.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0,  # 0 = Disarm
                    21196,  # 21196 = Force Disarm / Kill Switch
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                self.connection.arducopter_disarm()
        except Exception as exc:
            logger.error("Fehler beim Senden des Kill-Befehls: %s", exc)
        finally:
            self.is_flying = False
            self.is_armed = False

    def takeoff(self, target_alt: float, timeout: float = 12.0) -> None:
        """Führt einen autonomen Start im ALT_HOLD Modus auf die Zielhöhe durch.

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

        # 1. In ALT_HOLD wechseln & schärfen
        if self.flight_mode != "ALT_HOLD":
            self.set_mode("ALT_HOLD")
        if not self.is_armed:
            self.arm(mode="ALT_HOLD")

        logger.info(
            "Starte Steigflug auf %.2f m im ALT_HOLD Modus (LiDAR-geführt)...",
            target_alt,
        )
        self.is_flying = True
        started = time.monotonic()

        # 2. Steigflug mit aktivem Schub (Throttle 1650)
        while time.monotonic() - started < timeout:
            self.update_telemetry()

            # Steig-Gas geben (1650 = sanfter Steigflug)
            self.set_rc_override(roll=1500, pitch=1500, throttle=1650, yaw=1500)

            # Prüfe, ob Zielhöhe erreicht ist
            if (
                self.current_altitude is not None
                and self.current_altitude >= target_alt * 0.85
            ):
                logger.info(
                    "Zielhöhe von %.2f m erreicht (Aktuell: %.2f m). Halte Position!",
                    target_alt,
                    self.current_altitude,
                )
                # Sofort Schwebegas (1500 = Höhe stabil halten)
                self.set_rc_override(roll=1500, pitch=1500, throttle=1500, yaw=1500)
                return

            time.sleep(0.05)

        logger.warning(
            "Takeoff-Zeit erreicht. Gehe in Schwebegas (1500) über (Höhe: %s m).",
            f"{self.current_altitude:.2f}"
            if self.current_altitude is not None
            else "N/A",
        )
        self.set_rc_override(roll=1500, pitch=1500, throttle=1500, yaw=1500)

    def land(self, timeout: float = 15.0) -> None:
        """Wechselt in den LAND-Modus und wartet auf das Aufsetzen / Disarming."""
        logger.info("Leite Landung (LAND Modus) ein...")
        self.set_mode("LAND")

        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.update_telemetry()
            if self.flight_mode == "ALT_HOLD":
                # Sanftes Sinken bei ALT_HOLD
                self.set_rc_override(roll=1500, pitch=1500, throttle=1350, yaw=1500)

            if not self.is_armed or (
                self.current_altitude is not None and self.current_altitude < 0.12
            ):
                logger.info("Landung abgeschlossen. Drohne ist am Boden.")
                self.clear_rc_override()
                self.is_flying = False
                return
            time.sleep(0.1)

        logger.warning("Landung abgeschlossen. Disarme sicherheitshalber.")
        self.clear_rc_override()
        try:
            self.disarm(timeout=2.0)
        except Exception:
            pass
        self.is_flying = False

    def send_velocity_body(
        self, vx: float, vy: float, vz: float, yaw_rate_deg: float = 0.0
    ) -> None:
        """Sendet Body-Frame Geschwindigkeits- und Gierratenbefehle an die Drohne.

        Args:
            vx: Vorwärts-/Rückwärts-Geschwindigkeit in m/s (+ Vorwärts, - Rückwärts).
            vy: Links-/Rechts-Geschwindigkeit in m/s (+ Rechts, - Links).
            vz: Vertikale Geschwindigkeit in m/s (+ Unten/Sinken, - Oben/Steigen).
            yaw_rate_deg: Gier-Rate in Grad/Sekunde (+ Rechtsdrehung, - Linksdrehung).
        """
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")
        if not self.is_armed:
            logger.warning(
                "Ignoriere Geschwindigkeitsbefehl: Drohne ist nicht geschärft (DISARMED)."
            )
            return

        # 1. MAVLink SET_POSITION_TARGET_LOCAL_NED
        type_mask = 0x05C7
        yaw_rate_rad = math.radians(yaw_rate_deg)
        time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF

        self.connection.mav.set_position_target_local_ned_send(
            time_boot_ms,
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

        # 2. RC Override Unterstützung für ALT_HOLD
        # Pitch: negativ ist vorwärts in RC (-vx)
        pitch_val = max(1350, min(1650, int(1500 - vx * 350)))
        # Roll: positiv ist rechts (+vy)
        roll_val = max(1350, min(1650, int(1500 + vy * 350)))
        # Throttle: 1500 neutral, 1600 steigen, 1400 sinken
        if vz < -0.10:
            thr_val = 1600
        elif vz > 0.10:
            thr_val = 1400
        else:
            thr_val = 1500
        yaw_val = max(1350, min(1650, int(1500 + yaw_rate_deg * 5)))

        self.set_rc_override(
            roll=roll_val, pitch=pitch_val, throttle=thr_val, yaw=yaw_val
        )
        logger.debug(
            "Velocity Body gesendet: vx=%.2f, vy=%.2f, vz=%.2f, yaw_rate=%.1f°/s",
            vx,
            vy,
            vz,
            yaw_rate_deg,
        )
