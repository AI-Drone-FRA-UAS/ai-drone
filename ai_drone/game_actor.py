"""GameMode Drohnen-Actor & Vektor-Mathematik.

Stellt eine intuitive, spiele-engine-artige Abstraktionsschicht zur Verfügung,
um eine ArduPilot-Drohne im ALT_HOLD-Modus (oder FLOWHOLD) über einfache
Richtungs- und Vektorbefehle anzusteuern. Beinhaltet ein aktives Failsafe-System
(Überhöhe, Übergeschwindigkeit, Schräglage, Sensor-Timeout) mit sofortigem Not-Disarm.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.mavlink_devices import resolve_mavlink_endpoint

logger = logging.getLogger(__name__)

# Throttle- & RC-Konstanten
PWM_NEUTRAL = 1500
PWM_THROTTLE_DISARM = 1000
PWM_THROTTLE_HOVER = 1500


class FailsafeException(Exception):
    """Wird ausgelöst, wenn ein Sicherheits- oder Failsafe-Wächter anschlägt."""


@dataclass
class FailsafeConfig:
    """Konfiguration für alle aktiven Sicherheits- und Failsafe-Wächter."""

    max_altitude: float = 0.80  #: Maximales Höhenlimit in Metern (80 cm)
    max_vertical_speed: float = 0.80  #: Maximale Vertikalgeschwindigkeit |vz| in m/s
    max_tilt_angle_deg: float = 25.0  #: Maximaler Neigungswinkel |Roll/Pitch| in Grad
    telemetry_timeout_s: float = 0.50  #: Timeout bei ausbleibenden LiDAR-Signalen in s
    touchdown_altitude: float = 0.05  #: Abschalthöhe für sichere Landung in m (5 cm)
    ground_altitude: float = 0.02  #: Typischer Bodenabstand am Boden in m (2 cm)
    max_accel_climb: float = 0.15  #: Maximale Beschleunigung m/s^2 für Bremskurve
    max_climb_rate: float = 0.22  #: Maximale Steiggeschwindigkeit in m/s (22 cm/s)
    target_sink_rate: float = 0.05  #: Ziel-Sinkrate bei Landung in m/s (5 cm/s)


DEFAULT_FAILSAFE_CONFIG = FailsafeConfig()


@dataclass
class Vector3:
    """3D-Vektor für intuitive Richtungs- und Geschwindigkeitsvorgaben.

    Koordinatensystem (Body-Frame der Drohne):
        x: Vorwärts (+1.0) / Rückwärts (-1.0)
        y: Rechts (+1.0) / Links (-1.0) [Strafe]
        z: Oben (+1.0) / Unten (-1.0)   [Elevation]
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: Vector3) -> Vector3:
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vector3) -> Vector3:
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> Vector3:
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> Vector3:
        return self.__mul__(scalar)

    def length(self) -> float:
        """Berechnet die euklidische Länge des Vektors."""
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalized(self) -> Vector3:
        """Gibt einen auf Länge 1 normierten Vektor zurück (oder Nullvektor)."""
        vec_len = self.length()
        if vec_len < 1e-6:
            return Vector3.zero()
        return Vector3(self.x / vec_len, self.y / vec_len, self.z / vec_len)

    def scaled(self, factor: float) -> Vector3:
        """Skaliert den Vektor um einen Faktor."""
        return self * factor

    @staticmethod
    def forward() -> Vector3:
        return Vector3(x=1.0, y=0.0, z=0.0)

    @staticmethod
    def back() -> Vector3:
        return Vector3(x=-1.0, y=0.0, z=0.0)

    @staticmethod
    def left() -> Vector3:
        return Vector3(x=0.0, y=-1.0, z=0.0)

    @staticmethod
    def right() -> Vector3:
        return Vector3(x=0.0, y=1.0, z=0.0)

    @staticmethod
    def up() -> Vector3:
        return Vector3(x=0.0, y=0.0, z=1.0)

    @staticmethod
    def down() -> Vector3:
        return Vector3(x=0.0, y=0.0, z=-1.0)

    @staticmethod
    def zero() -> Vector3:
        return Vector3(x=0.0, y=0.0, z=0.0)


class DroneGameActor:
    """Character-Controller-Abstraktion für die ArduPilot-Drohne im ALT_HOLD-Modus.

    Bietet intuitive Methoden wie `move_forward()`, `move_up()`, `rotate_yaw()`,
    `set_axis_input()` oder Vektorbewegungen `move(Vector3, speed, duration)`.

    Args:
        device: MAVLink-Schnittstelle (z. B. '/dev/serial0' oder 'udp:127.0.0.1:14550').
        baud: Baudrate für die serielle Verbindung (Standard: 115200).
        mode: Flugmodus (Standard: 'ALT_HOLD', optional 'FLOWHOLD').
        failsafe_config: Konfiguration der Failsafe-Schwellen.
        stream_rate_hz: Frequenz des Hintergrund-Streamers für RC-Overrides (Standard: 20 Hz).
        target_system: MAVLink Target System ID.
        target_component: MAVLink Target Component ID.
        auto_connect: Ob beim Initialisieren direkt die Verbindung hergestellt werden soll.
    """

    def __init__(
        self,
        device: str | Path | None = None,
        baud: int = 115200,
        mode: str = "ALT_HOLD",
        failsafe_config: FailsafeConfig | None = None,
        stream_rate_hz: int = 20,
        target_system: int = 1,
        target_component: int = 1,
        auto_connect: bool = True,
    ) -> None:
        self.requested_device = device
        self.device = self._find_device(device) if auto_connect else str(device)
        self.baud = baud
        self.target_mode = mode
        self.failsafe = failsafe_config or DEFAULT_FAILSAFE_CONFIG
        self.stream_rate_hz = max(5, min(50, stream_rate_hz))
        self.target_system = target_system
        self.target_component = target_component

        # Verbindung & Status
        self.connection: Any | None = None
        self.is_armed: bool = False
        self.is_flying: bool = False
        self.flight_mode: str | None = None
        self.battery_voltage: float | None = None

        # Telemetriedaten
        self.current_altitude: float | None = None
        self.filtered_vz: float = 0.0
        self.roll_deg: float = 0.0
        self.pitch_deg: float = 0.0
        self.yaw_deg: float = 0.0
        self.last_telemetry_time: float = 0.0
        self._last_raw_alt: float | None = None
        self._last_raw_alt_time: float = 0.0

        # Aktive RC-Override-Sollwerte (Roll, Pitch, Throttle, Yaw)
        self._target_roll: int = PWM_NEUTRAL
        self._target_pitch: int = PWM_NEUTRAL
        self._target_throttle: int = PWM_THROTTLE_DISARM
        self._target_yaw: int = PWM_NEUTRAL

        # Failsafe & Threading
        self._failsafe_triggered: bool = False
        self._failsafe_reason: str = ""
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._streamer_thread: threading.Thread | None = None

        if auto_connect:
            self.connect()

    @staticmethod
    def _find_device(requested: str | Path | None) -> str:
        """Findet automatisch verfügbare MAVLink-Schnittstellen."""
        try:
            return resolve_mavlink_endpoint(
                requested,
                include_pi_uart=True,
                missing_message="Kein MAVLink-Gerät gefunden. Bitte per --device angeben.",
            )
        except FileNotFoundError as error:
            if requested:
                raise
            raise RuntimeError(str(error)) from error

    def __enter__(self) -> DroneGameActor:
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        if exc_type is not None:
            logger.error("Ausnahme im DroneGameActor-Kontext: %s", exc_val)
        try:
            if self.is_flying or self.is_armed:
                logger.warning(
                    "Safety-Trap: DroneGameActor beendet während Flug/Arming -> Not-Disarm."
                )
                self.emergency_kill(reason="Kontext verlassen im Flugzustand")
        finally:
            self.close()

    def connect(self) -> None:
        """Stellt die MAVLink-Verbindung her und startet den Streamer-Thread."""
        logger.info("Verbinde mit MAVLink (%s, Baud: %d)...", self.device, self.baud)
        self.connection = mavutil.mavlink_connection(self.device, baud=self.baud)

        logger.info("Warte auf Heartbeat vom Flugkontroller...")
        self.connection.wait_heartbeat(timeout=15)
        ts = getattr(self.connection, "target_system", None)
        if isinstance(ts, int):
            self.target_system = ts
        tc = getattr(self.connection, "target_component", None)
        if isinstance(tc, int):
            self.target_component = tc

        logger.info(
            "Verbunden. System-ID: %s, Component-ID: %s",
            self.target_system,
            self.target_component,
        )

        self._request_telemetry_streams()
        self.update_telemetry()

        # Starte Hintergrund-Streamer & Failsafe-Wächter
        self._stop_event.clear()
        self._streamer_thread = threading.Thread(
            target=self._streamer_loop, daemon=True, name="DroneGameActor-Streamer"
        )
        self._streamer_thread.start()

    def close(self) -> None:
        """Stoppt alle Threads und schließt die MAVLink-Verbindung sicher."""
        self._stop_event.set()
        if self._streamer_thread and self._streamer_thread.is_alive():
            self._streamer_thread.join(timeout=1.0)
            self._streamer_thread = None

        if self.connection:
            try:
                self.clear_rc_override()
                self.connection.close()
            except Exception as exc:
                logger.debug("Fehler beim Schließen der Verbindung: %s", exc)
            self.connection = None

    def _request_telemetry_streams(self) -> None:
        """Fordert Sensor- und Lagedaten über MAVLink an."""
        if not self.connection:
            return

        interval_us = int(1_000_000 / self.stream_rate_hz)
        message_ids = (
            mavlink.MAVLINK_MSG_ID_RANGEFINDER,
            mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            mavlink.MAVLINK_MSG_ID_HEARTBEAT,
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

    def update_telemetry(self, max_messages: int = 40) -> None:
        """Liest anstehende MAVLink-Nachrichten nicht-blockierend aus und aktualisiert Daten."""
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
            elif msg_type == "RANGEFINDER":
                dist = float(msg.distance)
                if 0.01 <= dist <= 3.5:
                    self._process_altitude(dist, now)
            elif msg_type == "DISTANCE_SENSOR":
                orientation = getattr(msg, "orientation", 25)
                if orientation == 25:  # Downward LiDAR
                    dist = float(msg.current_distance) / 100.0
                    if 0.01 <= dist <= 3.5:
                        self._process_altitude(dist, now)
            elif msg_type == "ATTITUDE":
                self.roll_deg = math.degrees(float(msg.roll))
                self.pitch_deg = math.degrees(float(msg.pitch))
                self.yaw_deg = math.degrees(float(msg.yaw))
                self.last_telemetry_time = now
            elif msg_type == "SYS_STATUS":
                self.battery_voltage = float(msg.voltage_battery) / 1000.0

    def _process_altitude(self, raw_dist: float, now: float) -> None:
        """Verarbeitet eingehende Höhendaten und filtert die Vertikalgeschwindigkeit."""
        if self._last_raw_alt is not None and self._last_raw_alt_time > 0:
            dt = now - self._last_raw_alt_time
            if dt > 0.005:
                instant_vz = (raw_dist - self._last_raw_alt) / dt
                self.filtered_vz = 0.70 * self.filtered_vz + 0.30 * instant_vz

        self.current_altitude = raw_dist
        self._last_raw_alt = raw_dist
        self._last_raw_alt_time = now
        self.last_telemetry_time = now

    def _check_failsafes(self) -> None:
        """Prüft im Flug alle konfigurierten Sicherheits-Grenzwerte."""
        if not self.is_flying or self._failsafe_triggered:
            return

        now = time.monotonic()

        # 1. Höhenlimit-Wächter
        if (
            self.current_altitude is not None
            and self.current_altitude > self.failsafe.max_altitude
        ):
            self.trigger_failsafe(
                f"Höhenlimit überschritten! Ist: {self.current_altitude:.2f} m > Max: {self.failsafe.max_altitude:.2f} m"
            )
            return

        # 2. Vertikalgeschwindigkeits-Wächter
        if abs(self.filtered_vz) > self.failsafe.max_vertical_speed:
            self.trigger_failsafe(
                f"Vertikalgeschwindigkeit zu hoch! Ist: {self.filtered_vz:+.2f} m/s > Limit: {self.failsafe.max_vertical_speed:.2f} m/s"
            )
            return

        # 3. Schräglagen- / Überschlag-Wächter
        max_tilt = self.failsafe.max_tilt_angle_deg
        if abs(self.roll_deg) > max_tilt or abs(self.pitch_deg) > max_tilt:
            self.trigger_failsafe(
                f"Kritische Schräglage erkannt! Roll: {self.roll_deg:+.1f}°, Pitch: {self.pitch_deg:+.1f}° > Limit: {max_tilt:.1f}°"
            )
            return

        # 4. Sensor-Timeout-Wächter (LiDAR)
        if (
            self.last_telemetry_time > 0
            and (now - self.last_telemetry_time) > self.failsafe.telemetry_timeout_s
        ):
            self.trigger_failsafe(
                f"Sensor-Timeout! Kein LiDAR-Signal seit {(now - self.last_telemetry_time):.2f} s"
            )
            return

    def trigger_failsafe(self, reason: str) -> None:
        """Löst unverzüglich den Failsafe-Schutz aus (Motorstopp + Force Disarm)."""
        self._failsafe_triggered = True
        self._failsafe_reason = reason
        logger.error("[FAILSAFE] AUSGELOEST: %s", reason)
        self.emergency_kill(reason=f"Failsafe: {reason}")

    def _streamer_loop(self) -> None:
        """Hintergrund-Schleife: Sendet kontinuierlich RC-Overrides & überwacht Failsafes."""
        interval = 1.0 / self.stream_rate_hz
        while not self._stop_event.is_set():
            start_t = time.monotonic()

            self.update_telemetry()
            self._check_failsafes()

            if self.connection and not self._failsafe_triggered:
                with self._lock:
                    roll = self._target_roll
                    pitch = self._target_pitch
                    throttle = self._target_throttle
                    yaw = self._target_yaw

                try:
                    self.connection.mav.rc_channels_override_send(
                        self.target_system,
                        self.target_component,
                        roll,
                        pitch,
                        throttle,
                        yaw,
                        0,
                        0,
                        0,
                        0,
                    )
                except Exception as exc:
                    logger.debug("Fehler im RC-Streamer: %s", exc)

            elapsed = time.monotonic() - start_t
            sleep_time = max(0.001, interval - elapsed)
            time.sleep(sleep_time)

    def set_rc_target(
        self,
        roll: int = PWM_NEUTRAL,
        pitch: int = PWM_NEUTRAL,
        throttle: int = PWM_NEUTRAL,
        yaw: int = PWM_NEUTRAL,
    ) -> None:
        """Setzt die aktiven Ziel-PWM-Werte für den Hintergrund-Streamer."""
        with self._lock:
            self._target_roll = max(1000, min(2000, int(roll)))
            self._target_pitch = max(1000, min(2000, int(pitch)))
            self._target_throttle = max(1000, min(2000, int(throttle)))
            self._target_yaw = max(1000, min(2000, int(yaw)))

    def clear_rc_override(self) -> None:
        """Löscht alle RC Overrides auf dem Flight Controller."""
        if not self.connection:
            return
        try:
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
        except Exception:
            pass

    def emergency_kill(self, reason: str = "Manueller Not-Aus") -> None:
        """Sofortiger Kill-Switch: Schneidet unverzüglich den Motorstrom ab (1000 PWM + Force Disarm)."""
        logger.warning("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        logger.warning("  SOFORT-NOT-AUS: MOTORSTROM AUS & FORCE DISARM! (%s)", reason)
        logger.warning("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        self.set_rc_target(
            roll=PWM_NEUTRAL,
            pitch=PWM_NEUTRAL,
            throttle=PWM_THROTTLE_DISARM,
            yaw=PWM_NEUTRAL,
        )
        self.is_flying = False

        if self.connection:
            try:
                # 1. 1000 PWM sofort senden
                self.connection.mav.rc_channels_override_send(
                    self.target_system,
                    self.target_component,
                    PWM_NEUTRAL,
                    PWM_NEUTRAL,
                    PWM_THROTTLE_DISARM,
                    PWM_NEUTRAL,
                    0,
                    0,
                    0,
                    0,
                )
                # 2. MAVLink Force Disarm (21196)
                self.connection.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0,  # 0 = Disarm
                    21196,  # 21196 = Force Disarm
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                self.connection.arducopter_disarm()
            except Exception as exc:
                logger.error("Fehler beim Senden des Kill-Befehls: %s", exc)

        self.is_armed = False

    def set_mode(self, mode_name: str, timeout: float = 5.0) -> None:
        """Wechselt den Flugmodus und verifiziert die Bestätigung von ArduPilot."""
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")

        mode_mapping = self.connection.mode_mapping()
        if mode_name not in mode_mapping:
            if mode_name == "FLOWHOLD" and "ALT_HOLD" in mode_mapping:
                logger.info("FLOWHOLD nicht verfügbar, wechsle auf ALT_HOLD.")
                mode_name = "ALT_HOLD"
            else:
                raise ValueError(f"Modus '{mode_name}' wird nicht unterstützt!")

        mode_id = mode_mapping[mode_name]
        logger.info("Wechsle in Flugmodus %s...", mode_name)
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
            time.sleep(0.05)

        logger.warning(
            "Moduswechsel auf %s nach Timeout nicht eindeutig bestätigt.", mode_name
        )

    def takeoff(self, height_m: float = 0.50, timeout: float = 12.0) -> None:
        """Führt einen sicheren, prozeduralen Start im ALT_HOLD Modus durch."""
        if not self.connection:
            raise RuntimeError("Nicht verbunden.")
        if self._failsafe_triggered:
            raise FailsafeException(
                f"Start abgelehnt: Failsafe aktiv ({self._failsafe_reason})"
            )

        if height_m > self.failsafe.max_altitude:
            raise ValueError(
                f"Zielhöhe ({height_m:.2f} m) überschreitet Sicherheitslimit ({self.failsafe.max_altitude:.2f} m)!"
            )

        logger.info(
            "=== STARTE PROZEDURALEN TAKEOFF AUF %.2f m (Modus: %s) ===",
            height_m,
            self.target_mode,
        )

        # 1. Modus setzen & Schärfen
        self.set_mode(self.target_mode)
        self.set_rc_target(throttle=PWM_THROTTLE_DISARM)

        logger.info("Arming Motoren bei 0% Gas (1000 PWM)...")
        self.connection.arducopter_arm()

        arm_start = time.monotonic()
        while time.monotonic() - arm_start < 3.0:
            self.update_telemetry()
            if self.is_armed:
                break
            time.sleep(0.05)

        self.is_flying = True
        logger.info("Drohne ist geschärft. Steigflug beginnt...")

        # 2. Prozeduraler Steigflug mit Bremskurve
        climb_start = time.monotonic()
        while time.monotonic() - climb_start < timeout:
            if self._failsafe_triggered:
                raise FailsafeException(
                    f"Takeoff abgebrochen: Failsafe ausgelöst ({self._failsafe_reason})"
                )

            alt = self.current_altitude

            if alt is None:
                # Vorsichtiges Anfangsgas
                self.set_rc_target(throttle=1530)
            elif alt >= height_m:
                logger.info(
                    "[TARGET] Zielhoehe von %.2f m erreicht (Ist: %.2f m). Gehe in Schwebegas (1500 PWM)...",
                    height_m,
                    alt,
                )
                self.set_rc_target(throttle=PWM_THROTTLE_HOVER)
                return
            else:
                # Quadratwurzel-Bremskurve
                delta_z = max(0.0, height_m - alt)
                v_brake = math.sqrt(2.0 * self.failsafe.max_accel_climb * delta_z)
                v_cmd = min(self.failsafe.max_climb_rate, v_brake)

                # Soft Lift-off nahe am Boden
                if alt < self.failsafe.ground_altitude + 0.06:
                    ratio = max(
                        0.0,
                        min(
                            1.0,
                            (alt - self.failsafe.ground_altitude) / 0.06,
                        ),
                    )
                    v_cmd = max(0.04, v_cmd * (0.35 + 0.65 * ratio))

                error_v = v_cmd - self.filtered_vz
                if v_cmd > 0.02:
                    climb_ratio = min(
                        1.0, v_cmd / max(0.01, self.failsafe.max_climb_rate)
                    )
                    base_pwm = 1525.0 + 15.0 * climb_ratio
                    pwm_cmd = base_pwm + error_v * 50.0
                else:
                    pwm_cmd = 1500.0 + error_v * 30.0

                pwm_clamped = int(round(max(1480.0, min(1545.0, pwm_cmd))))
                self.set_rc_target(throttle=pwm_clamped)

            time.sleep(0.05)

        logger.warning("Takeoff-Timeout erreicht. Schalte auf Schwebegas (1500 PWM).")
        self.set_rc_target(throttle=PWM_THROTTLE_HOVER)

    def land(self, timeout: float = 15.0) -> None:
        """Führt einen geregelten Sinkflug durch und disarmt sicher bei Bodenkontakt."""
        logger.info(
            "=== STARTE KONTROLLIERTE LANDUNG (Abschalthöhe: %.2f m) ===",
            self.failsafe.touchdown_altitude,
        )
        sink_start = time.monotonic()
        touchdown_count = 0

        while time.monotonic() - sink_start < timeout:
            if self._failsafe_triggered:
                raise FailsafeException(
                    f"Landung durch Failsafe unterbrochen ({self._failsafe_reason})"
                )

            alt = self.current_altitude

            # Geregelter Sink-Gaswert (~1460 PWM)
            if alt is not None:
                v_cmd = -abs(self.failsafe.target_sink_rate)
                if alt <= 0.12:
                    v_cmd = max(v_cmd, -0.03)  # Bodennähe-Dämpfung

                error_v = self.filtered_vz - v_cmd
                pwm_cmd = 1465.0 - error_v * 250.0
                if alt <= 0.12:
                    pwm_cmd = max(pwm_cmd, 1475.0)
                descent_pwm = int(round(max(1430.0, min(1495.0, pwm_cmd))))
            else:
                descent_pwm = 1465

            self.set_rc_target(
                roll=PWM_NEUTRAL,
                pitch=PWM_NEUTRAL,
                throttle=descent_pwm,
                yaw=PWM_NEUTRAL,
            )

            # Touchdown-Erkennung (<= 5 cm für mindestens 2 Zyklen)
            if alt is not None and alt <= self.failsafe.touchdown_altitude:
                touchdown_count += 1
                if touchdown_count >= 2:
                    logger.info(
                        "[TOUCHDOWN] Touchdown erkannt (Hoehe: %.2f m). Schalte Motoren aus!",
                        alt,
                    )
                    break
            else:
                touchdown_count = 0

            time.sleep(0.05)

        logger.info("Motoren abschalten (1000 PWM) & Disarm...")
        self.emergency_kill(reason="Landung abgeschlossen")
        logger.info("Landung erfolgreich abgeschlossen.")

    def hover(self, duration_s: float | None = None) -> None:
        """Hält die Drohne waagerecht im Schwebeflug (Neutral-Stick 1500 PWM).

        Args:
            duration_s: Optionale Dauer in Sekunden. Falls None, wird nur der Befehl gesetzt.
        """
        self.set_rc_target(
            roll=PWM_NEUTRAL,
            pitch=PWM_NEUTRAL,
            throttle=PWM_THROTTLE_HOVER,
            yaw=PWM_NEUTRAL,
        )
        if duration_s is not None and duration_s > 0:
            start_t = time.monotonic()
            while time.monotonic() - start_t < duration_s:
                if self._failsafe_triggered:
                    raise FailsafeException(
                        f"Schwebeflug abgebrochen: {self._failsafe_reason}"
                    )
                time.sleep(0.05)

    def stop(self) -> None:
        """Stoppt alle aktiven Bewegungsbefehle und geht sofort in Schwebeflug über."""
        self.hover()

    def set_axis_input(
        self,
        forward: float = 0.0,
        strafe: float = 0.0,
        vertical: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        """Kontinuierliche Achsensteuerung für Gamepads, Tastatur (WASD) oder KI-Agents.

        Wertebereich jeweils normiert: -1.0 bis +1.0.

        Args:
            forward: Vorwärts (+1.0) / Rückwärts (-1.0) -> Steuert Pitch
            strafe: Rechts (+1.0) / Links (-1.0)       -> Steuert Roll
            vertical: Steigen (+1.0) / Sinken (-1.0)   -> Steuert Throttle
            yaw: Drehung Rechts (+1.0) / Links (-1.0)  -> Steuert Yaw
        """
        if self._failsafe_triggered:
            raise FailsafeException(
                f"Achsenbefehl abgelehnt: Failsafe aktiv ({self._failsafe_reason})"
            )

        f = max(-1.0, min(1.0, float(forward)))
        s = max(-1.0, min(1.0, float(strafe)))
        v = max(-1.0, min(1.0, float(vertical)))
        y = max(-1.0, min(1.0, float(yaw)))

        # Neigungswinkel-Begrenzung: max +/- 120 PWM (sanfte ~10-15° Neigung)
        pitch_pwm = PWM_NEUTRAL - int(round(f * 120.0))
        roll_pwm = PWM_NEUTRAL + int(round(s * 120.0))
        yaw_pwm = PWM_NEUTRAL + int(round(y * 120.0))

        # Vertikales Gas: +/- 120 PWM um 1500 (1380 bis 1620)
        throttle_pwm = PWM_THROTTLE_HOVER + int(round(v * 120.0))

        self.set_rc_target(
            roll=roll_pwm, pitch=pitch_pwm, throttle=throttle_pwm, yaw=yaw_pwm
        )

    def move_forward(self, duration_s: float = 1.0, speed: float = 0.4) -> None:
        """Fliegt für die angegebene Dauer mit gegebener Geschwindigkeit vorwärts."""
        self._execute_timed_move(forward=speed, duration_s=duration_s)

    def move_backward(self, duration_s: float = 1.0, speed: float = 0.4) -> None:
        """Fliegt für die angegebene Dauer mit gegebener Geschwindigkeit rückwärts."""
        self._execute_timed_move(forward=-speed, duration_s=duration_s)

    def move_left(self, duration_s: float = 1.0, speed: float = 0.4) -> None:
        """Fliegt für die angegebene Dauer mit gegebener Geschwindigkeit nach links (Strafe)."""
        self._execute_timed_move(strafe=-speed, duration_s=duration_s)

    def move_right(self, duration_s: float = 1.0, speed: float = 0.4) -> None:
        """Fliegt für die angegebene Dauer mit gegebener Geschwindigkeit nach rechts (Strafe)."""
        self._execute_timed_move(strafe=speed, duration_s=duration_s)

    def move_up(
        self,
        target_height_m: float | None = None,
        duration_s: float = 1.0,
        speed: float = 0.3,
    ) -> None:
        """Steigt entweder auf eine Zielhöhe oder für eine bestimmte Zeitdauer."""
        if target_height_m is not None:
            if target_height_m > self.failsafe.max_altitude:
                raise ValueError(
                    f"Zielhöhe {target_height_m:.2f} m > Max-Höhe {self.failsafe.max_altitude:.2f} m!"
                )
            start_t = time.monotonic()
            while time.monotonic() - start_t < 6.0:
                if self._failsafe_triggered:
                    raise FailsafeException(
                        f"Steigen abgebrochen: {self._failsafe_reason}"
                    )
                alt = self.current_altitude
                if alt is not None and alt >= target_height_m:
                    break
                self.set_axis_input(vertical=abs(speed))
                time.sleep(0.05)
            self.hover()
        else:
            self._execute_timed_move(vertical=abs(speed), duration_s=duration_s)

    def move_down(
        self,
        target_height_m: float | None = None,
        duration_s: float = 1.0,
        speed: float = 0.2,
    ) -> None:
        """Sinkt entweder auf eine Zielhöhe oder für eine bestimmte Zeitdauer."""
        if target_height_m is not None:
            target = max(self.failsafe.touchdown_altitude + 0.05, target_height_m)
            start_t = time.monotonic()
            while time.monotonic() - start_t < 6.0:
                if self._failsafe_triggered:
                    raise FailsafeException(
                        f"Sinken abgebrochen: {self._failsafe_reason}"
                    )
                alt = self.current_altitude
                if alt is not None and alt <= target:
                    break
                self.set_axis_input(vertical=-abs(speed))
                time.sleep(0.05)
            self.hover()
        else:
            self._execute_timed_move(vertical=-abs(speed), duration_s=duration_s)

    def rotate_yaw(self, duration_s: float = 1.0, speed: float = 0.5) -> None:
        """Dreht die Drohne um die eigene Hochachse (+ ist Rechtsdrehung, - ist Linksdrehung)."""
        self._execute_timed_move(yaw=speed, duration_s=duration_s)

    def move(
        self, vector: Vector3, duration_s: float = 1.0, speed: float = 0.4
    ) -> None:
        """Führt eine kombinierte 3D-Vektorbewegung für die angegebene Zeitdauer aus."""
        norm_v = vector.normalized().scaled(max(0.0, min(1.0, speed)))
        self._execute_timed_move(
            forward=norm_v.x,
            strafe=norm_v.y,
            vertical=norm_v.z,
            duration_s=duration_s,
        )

    def _execute_timed_move(
        self,
        forward: float = 0.0,
        strafe: float = 0.0,
        vertical: float = 0.0,
        yaw: float = 0.0,
        duration_s: float = 1.0,
    ) -> None:
        """Interne Hilfsfunktion für zeitbegrenzte Bewegungen mit anschließendem Schwebestopp."""
        self.set_axis_input(forward=forward, strafe=strafe, vertical=vertical, yaw=yaw)
        start_t = time.monotonic()
        while time.monotonic() - start_t < duration_s:
            if self._failsafe_triggered:
                raise FailsafeException(
                    f"Bewegung durch Failsafe abgebrochen ({self._failsafe_reason})"
                )
            time.sleep(0.05)
        self.hover()
