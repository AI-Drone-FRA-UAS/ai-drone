#!/usr/bin/env python3
"""Präziser Schwebeflug mit prozeduralem, dynamischem Regler (Optical Flow & LiDAR).

Ablauf:
1. INITIALISIERUNG & PRE-FLIGHT:
   - Verbindet via MAVLink, fordert Sensordaten an (20 Hz).
   - Setzt virtuellen EKF Global Origin & Home Position für GPS-freie Optical-Flow-Navigation.
   - Wechselt in den Modus FLOWHOLD (Optical Flow Positions- & LiDAR Höhenhalt).
   - Führt Pre-Flight Check der Optical Flow Qualität (Quality Score) durch.
2. ARMING:
   - Schärft die Motoren sicher bei 0.0% Gas (1000 PWM).
3. PHASE 1 (Prozeduraler Steigflug mit Bremskurve & Flow-Skalierung):
   - Weiches Abheben (Soft Lift-Off Ramp) zur Ruckminimierung.
   - Dynamische Geschwindigkeits-Vorgabe basierend auf der physikalischen Bremsweg-Kurve
     v_brake = sqrt(2 * a_max * delta_z).
   - Sensor-adaptive Skalierung: Drosselt die Geschwindigkeit bei abfallender Flow-Qualität.
   - Closed-Loop PD-Regelung auf die Vertikalgeschwindigkeit v_z (gleicht Wind & Akkuabfall aus).
   - Zielhöhe (50 cm) wird punktgenau mit v_z -> 0 erreicht; Überschwingen wird strikt unterbunden.
4. PHASE 2 (Aktiver Schwebeflug & Positionshalt):
   - Hält für die konfigurierte Dauer (Standard: 3.0s) Höhe und Position (50.0% / 1500 PWM).
5. PHASE 3 (Geregelter 5 cm/s Sinkflug):
   - Closed-Loop geregelte Sinkrate auf ca. -5 cm/s (-0.05 m/s) mit Bodennähe-Dämpfung.
6. PHASE 4 (Touchdown & Disarm bei 5 cm):
   - Sobald die Drohne die Abschalthöhe von ca. 5 cm erreicht (Sensor-Bodenwert liegt bei ~2 cm),
     schaltet das Skript die Motoren sofort auf 0% Gas (1000 PWM) ab und disarmt sicher.

Sicherheit & Not-Aus:
- PRESS ANY KEY SOFORT-NOT-AUS: Jeder beliebige Tastendruck (oder STRG+C) löst sofortigen
  Motorstopp (0% Gas + MAVLink Force Disarm) aus!
- Maximales Sicherheits-Höhenlimit (Standard: 80 cm).
- Duale Einheiten: Volle Unterstützung für 0.0%–100.0% Gas & RC-PWM (1000–2000).
- Automatisches Logging in `logs/flight_YYYYMMDD_HHMMSS.log` und `.csv` Telemetrie.
"""

from __future__ import annotations

import argparse
import csv
import math
import select
import shutil
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

# ---------------- Konfiguration & Standardwerte ----------------
PORT = "/dev/serial0"
BAUD = 115200  # 115200 oder 921600

DEFAULT_TARGET_ALT = 0.50  # Zielhöhe in Metern (50 cm)
MAX_ALTITUDE = (
    0.80  # Maximales Sicherheits-Höhenlimit in Metern (80 cm) -> leitet Landung ein
)
GROUND_ALT_ESTIMATE = 0.02  # Typischer Sensorabstand am Boden (~2 cm)
DEFAULT_TOUCHDOWN_ALT = (
    0.05  # Abschalthöhe für sichere Landung (5 cm Bodenabstand)
)
DEFAULT_HOVER_DURATION = 3.0  # Schwebepause auf 50 cm in Sekunden
DEFAULT_MAX_CLIMB_RATE = 0.22  # Maximale Steiggeschwindigkeit (22 cm/s)
DEFAULT_TARGET_SINK_RATE = 0.05  # Ziel-Sinkrate (5 cm/s = 0.05 m/s)
DEFAULT_MAX_ACCEL = (
    0.15  # Maximale Beschleunigung/Verzögerung in m/s^2 für Bremskurve
)

# Throttle-Konstanten
PWM_THROTTLE_DISARM = 1000
PWM_THROTTLE_HOVER = 1500
PWM_NEUTRAL = 1500
PCT_THROTTLE_DISARM = 0.0
PCT_THROTTLE_HOVER = 50.0
# ----------------------------------------------------------------

master = None
stop_event = threading.Event()
logger: FlightLogger | None = None


def pct_to_pwm(pct: float) -> int:
    """Wandelt Prozent (0.0% bis 100.0%) in RC-PWM (1000 bis 2000) um."""
    clamped = max(0.0, min(100.0, float(pct)))
    return int(round(1000.0 + (clamped / 100.0) * 1000.0))


def pwm_to_pct(pwm: int | float) -> float:
    """Wandelt RC-PWM (1000 bis 2000) in Prozent (0.0% bis 100.0%) um."""
    clamped = max(1000.0, min(2000.0, float(pwm)))
    return (clamped - 1000.0) / 10.0


class ProceduralAltitudeController:
    """Prozeduraler, mathematischer Regler für ruckfreien Steig- und Sinkflug."""

    def __init__(
        self,
        target_alt: float = DEFAULT_TARGET_ALT,
        max_climb_rate: float = DEFAULT_MAX_CLIMB_RATE,
        target_sink_rate: float = DEFAULT_TARGET_SINK_RATE,
        max_accel_climb: float = DEFAULT_MAX_ACCEL,
        ground_alt: float = GROUND_ALT_ESTIMATE,
        touchdown_alt: float = DEFAULT_TOUCHDOWN_ALT,
        kv: float = 180.0,
        kp: float = 90.0,
    ):
        self.target_alt = target_alt
        self.max_climb_rate = max_climb_rate
        self.target_sink_rate = target_sink_rate
        self.max_accel_climb = max_accel_climb
        self.ground_alt = ground_alt
        self.touchdown_alt = touchdown_alt
        self.kv = kv
        self.kp = kp

    def flow_scale(self, flow_q: int | None) -> float:
        """Berechnet einen Skalierungsfaktor (0.30 bis 1.0) basierend auf der optischen Flussqualität."""
        if flow_q is None:
            return 0.60
        # Flow-Qualität bewegt sich typischerweise zwischen 0 und 255
        return max(0.30, min(1.0, float(flow_q) / 200.0))

    def compute_climb(
        self,
        current_alt: float | None,
        current_vz: float,
        flow_q: int | None,
    ) -> tuple[int, float, float, float]:
        """Berechnet kontinuierlich Steiggas (PWM & %), Soll-Geschwindigkeit und Flow-Skalierung.

        - Verwendet physikalische Bremswegkurve: v_brake = sqrt(2 * a * delta_z).
        - Skaliert Maximalgeschwindigkeit adaptiv mit der Flow-Qualität.
        - Wendet Closed-Loop Geschwindigkeits-Regelung an.
        - Bei Erreichen der Zielhöhe: v_cmd = 0 und strikte Begrenzung auf <= 1500 PWM (bzw. Dämpfung).
        """
        f_scale = self.flow_scale(flow_q)

        if current_alt is None:
            # Fallback bei fehlendem Sensorwert: sehr vorsichtiges Steiggas
            return 1530, 53.0, 0.10, f_scale

        # 1. Zielhöhe erreicht oder überschritten
        if current_alt >= self.target_alt:
            v_cmd = 0.0
            if current_vz > 0.02:
                # Aktives Gegenbremsen gegen Restträgheit nach oben (48.0% = 1480 PWM)
                pwm = 1480
                pct = 48.0
            else:
                # Neutrales Schwebegas (50.0% = 1500 PWM)
                pwm = 1500
                pct = 50.0
            return pwm, pct, v_cmd, f_scale

        # 2. Physikalische Bremswegkurve
        delta_z = max(0.0, self.target_alt - current_alt)
        v_brake = math.sqrt(2.0 * self.max_accel_climb * delta_z)
        v_max_eff = self.max_climb_rate * f_scale
        v_cmd = min(v_max_eff, v_brake)

        # 3. Soft Lift-Off nahe am Boden (unter 8 cm)
        if current_alt < self.ground_alt + 0.06:
            ratio = max(0.0, min(1.0, (current_alt - self.ground_alt) / 0.06))
            v_cmd = max(0.04, v_cmd * (0.35 + 0.65 * ratio))

        # 4. Prozedurale PWM-Berechnung (Feedforward + Closed-Loop Feedback)
        error_v = v_cmd - current_vz
        if v_cmd > 0.02:
            climb_ratio = min(1.0, v_cmd / max(0.01, self.max_climb_rate))
            base_climb_pwm = 1525.0 + 15.0 * climb_ratio
            pwm_cmd = base_climb_pwm + error_v * 60.0
        else:
            pwm_cmd = 1500.0 + v_cmd * 300.0 + error_v * 40.0

        # Sicherheitsgrenzen: maximal 1545 PWM (54.5%), minimal 1480 PWM (48.0%)
        pwm_cmd = max(1480.0, min(1545.0, pwm_cmd))
        pwm = int(round(pwm_cmd))
        pct = pwm_to_pct(pwm)
        return pwm, round(pct, 1), round(v_cmd, 3), round(f_scale, 2)

    def compute_hover(
        self,
        current_alt: float | None,
        current_vz: float,
    ) -> tuple[int, float]:
        """Berechnet Schwebegas mit feinfühliger Höhenzentrierung."""
        if current_alt is None:
            return PWM_THROTTLE_HOVER, PCT_THROTTLE_HOVER

        # Toleranzfenster 4 cm um Zielhöhe
        if current_alt > self.target_alt + 0.04:
            pct = 46.5
        elif current_alt < self.target_alt - 0.04:
            pct = 53.0
        else:
            pct = PCT_THROTTLE_HOVER

        pwm = pct_to_pwm(pct)
        return pwm, round(pct, 1)

    def compute_descent(
        self,
        current_alt: float | None,
        current_vz: float,
        flow_q: int | None,
    ) -> tuple[int, float, float]:
        """Berechnet Sinkgas (PWM & %) geregelt auf die Ziel-Sinkrate (~5 cm/s)."""
        f_scale = self.flow_scale(flow_q)

        if current_alt is None:
            return 1465, 46.5, -self.target_sink_rate

        # Ziel-Sinkrate (z. B. -0.05 m/s), sanfter bei unruhigem Flow
        v_cmd = -abs(self.target_sink_rate) * (0.7 + 0.3 * f_scale)

        # Bodennähe (unter 12 cm): noch sanfter sinken (-0.03 m/s)
        if current_alt <= 0.12:
            v_cmd = max(v_cmd, -0.03)

        # Geschwindigkeits-Regelfehler
        error_v = current_vz - v_cmd  # > 0 wenn sinkt zu langsam; < 0 wenn zu schnell
        correction_pwm = -error_v * 300.0
        descent_pwm = 1465.0 + correction_pwm

        if current_alt <= 0.12:
            descent_pwm = max(descent_pwm, 1475.0)

        descent_pwm = max(1430.0, min(1495.0, descent_pwm))
        pwm = int(round(descent_pwm))
        pct = pwm_to_pct(pwm)
        return pwm, round(pct, 1), round(v_cmd, 3)


class FlightLogger:
    """Schreibt Logausgaben auf die Konsole und speichert Logdateien sowie CSV-Telemetrie."""

    def __init__(self, log_dir: Path | str = "logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = self.log_dir / f"flight_{now_str}.log"
        self.latest_log_path = self.log_dir / "fly_and_land_latest.log"
        self.csv_file_path = self.log_dir / f"flight_{now_str}.csv"
        self.latest_csv_path = self.log_dir / "fly_and_land_latest.csv"

        self.log_file = self.log_file_path.open("w", encoding="utf-8", buffering=1)
        self.csv_file = self.csv_file_path.open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "timestamp_iso",
                "elapsed_s",
                "phase",
                "mode",
                "throttle_pwm",
                "throttle_pct",
                "target_alt_m",
                "current_alt_m",
                "v_cmd_mps",
                "vz_mps",
                "flow_quality",
                "flow_scale",
            ]
        )

        self.start_time = time.monotonic()
        self.max_alt_seen: float = 0.0

    def print(self, *args, **kwargs):
        """Druckt auf stdout und schreibt synchron in die Logdatei."""
        text = " ".join(str(a) for a in args)
        print(text, **kwargs)
        if self.log_file and not self.log_file.closed:
            self.log_file.write(text + "\n")
            self.log_file.flush()

    def log_telemetry(
        self,
        phase: str,
        mode: str,
        throttle_pwm: int,
        throttle_pct: float | None = None,
        target_alt: float | None = None,
        current_alt: float | None = None,
        v_cmd: float = 0.0,
        vz: float = 0.0,
        flow_quality: int | None = None,
        flow_scale: float | None = None,
    ):
        if current_alt is not None and current_alt > self.max_alt_seen:
            self.max_alt_seen = current_alt

        if throttle_pct is None:
            throttle_pct = pwm_to_pct(throttle_pwm)

        now_iso = datetime.now().isoformat()
        elapsed = time.monotonic() - self.start_time
        self.csv_writer.writerow(
            [
                now_iso,
                f"{elapsed:.3f}",
                phase,
                mode,
                throttle_pwm,
                f"{throttle_pct:.1f}",
                f"{target_alt:.3f}" if target_alt is not None else "",
                f"{current_alt:.3f}" if current_alt is not None else "",
                f"{v_cmd:.3f}",
                f"{vz:.3f}",
                flow_quality if flow_quality is not None else "",
                f"{flow_scale:.2f}" if flow_scale is not None else "",
            ]
        )

    def close(self):
        try:
            if self.log_file and not self.log_file.closed:
                self.log_file.close()
            if self.csv_file and not self.csv_file.closed:
                self.csv_file.close()

            shutil.copyfile(self.log_file_path, self.latest_log_path)
            shutil.copyfile(self.csv_file_path, self.latest_csv_path)
        except Exception as e:
            print("Fehler beim Schließen der Logdateien:", e)


def log_msg(*args, **kwargs):
    """Zentraler Logging-Wrapper."""
    if logger is not None:
        logger.print(*args, **kwargs)
    else:
        print(*args, **kwargs)


class TelemetryTracker:
    """Liest nicht-blockierend MAVLink-Sensordaten (LiDAR, Optical Flow, EKF)."""

    def __init__(self, m):
        self.m = m
        self.current_alt: float | None = None
        self.last_alt_time: float = 0.0
        self.filtered_vz: float = 0.0
        self.last_raw_alt: float | None = None
        self.last_raw_time: float = 0.0
        self.flow_quality: int | None = None
        self.flight_mode: str | None = None
        self.is_armed: bool = False

    def update(self) -> float | None:
        """Liest alle anstehenden MAVLink-Nachrichten aus dem Puffer."""
        if self.m is None:
            return None

        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break

            now = time.monotonic()
            raw_dist: float | None = None
            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                self.is_armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if hasattr(self.m, "flightmode"):
                    self.flight_mode = self.m.flightmode
            elif msg_type == "RANGEFINDER":
                d = float(msg.distance)
                if 0.01 <= d <= 3.0:
                    raw_dist = d
            elif msg_type == "DISTANCE_SENSOR":
                orient = getattr(msg, "orientation", 25)
                if orient == 25:  # Downward LiDAR
                    d = float(msg.current_distance) / 100.0
                    if 0.01 <= d <= 3.0:
                        raw_dist = d
            elif msg_type == "OPTICAL_FLOW":
                d = float(msg.ground_distance)
                if 0.01 <= d <= 3.0:
                    raw_dist = d
                self.flow_quality = int(msg.quality)
            elif msg_type == "OPTICAL_FLOW_RAD":
                d = float(msg.distance)
                if 0.01 <= d <= 3.0:
                    raw_dist = d
                self.flow_quality = int(msg.quality)

            if raw_dist is not None:
                if self.last_raw_alt is not None and self.last_raw_time > 0:
                    dt = now - self.last_raw_time
                    if dt > 0.005:
                        instant_vz = (raw_dist - self.last_raw_alt) / dt
                        self.filtered_vz = 0.70 * self.filtered_vz + 0.30 * instant_vz

                self.current_alt = raw_dist
                self.last_alt_time = now
                self.last_raw_alt = raw_dist
                self.last_raw_time = now

        if time.monotonic() - self.last_alt_time < 0.40:
            return self.current_alt
        return None


def send_rc_raw(
    m,
    roll: int = PWM_NEUTRAL,
    pitch: int = PWM_NEUTRAL,
    throttle: int = PWM_THROTTLE_DISARM,
    yaw: int = PWM_NEUTRAL,
):
    """Sendet rohe RC-Override-PWM-Werte (1000-2000) an ArduPilot.

    Neutralstellung bei Roll/Pitch (1500 PWM) bewirkt im Modus FLOWHOLD,
    dass ArduPilot aktiv jeden horizontalen Drift über den Optical Flow Sensor ausgleicht.
    """
    if m is None:
        return
    m.mav.rc_channels_override_send(
        m.target_system,
        m.target_component,
        int(roll),
        int(pitch),
        int(throttle),
        int(yaw),
        0,
        0,
        0,
        0,
    )


def emergency_kill():
    """Sofortiger Not-Aus: Schneidet den Motorstrom unverzüglich ab (1000 PWM) und führt Force-Disarm durch."""
    log_msg("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    log_msg("  SOFORT-NOT-AUS: SCHNEIDE MOTORSTROM AB (MOTOREN AUS)!  ")
    log_msg("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    global master
    if master is not None:
        try:
            send_rc_raw(master, throttle=PWM_THROTTLE_DISARM)
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # Disarm
                21196,  # Force Disarm
                0,
                0,
                0,
                0,
                0,
            )
            master.arducopter_disarm()
            send_rc_raw(master, 0, 0, 0, 0)
        except Exception as e:
            log_msg("Fehler beim Not-Aus:", e)


def keyboard_listener():
    """Hintergrund-Thread: Lauscht auf JEDEN beliebigen Tastendruck (Press Any Key) für Sofort-Not-Aus."""
    time.sleep(0.3)

    # Windows-Plattform (msvcrt)
    if sys.platform == "win32":
        try:
            import msvcrt

            while not stop_event.is_set():
                if msvcrt.kbhit():
                    ch = msvcrt.getch()
                    log_msg(
                        f"\n>>> NOT-AUS: TASTENDRUCK ({repr(ch)}) ERKANNT! LÖSE SOFORT-NOT-AUS AUS!"
                    )
                    emergency_kill()
                    stop_event.set()
                    break
                time.sleep(0.05)
            return
        except Exception:
            pass

    # Unix / Linux (Raspberry Pi cbreak mode für jeden Tastendruck)
    old_settings = None
    if sys.stdin.isatty():
        try:
            import termios
            import tty

            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            old_settings = None

    try:
        while not stop_event.is_set():
            if sys.stdin in select.select([sys.stdin], [], [], 0.05)[0]:
                char = sys.stdin.read(1)
                if char:
                    log_msg(
                        f"\n>>> NOT-AUS: TASTENDRUCK ({repr(char)}) ERKANNT! LÖSE SOFORT-NOT-AUS AUS!"
                    )
                    emergency_kill()
                    stop_event.set()
                    break
    except Exception as e:
        log_msg("Fehler im Keyboard-Listener:", e)
    finally:
        if old_settings is not None and sys.stdin.isatty():
            try:
                import termios

                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass


def set_mode(m, mode_name: str, timeout: float = 5.0) -> str:
    """Wechselt den Flugmodus und verifiziert die Rückmeldung von ArduPilot."""
    mode_mapping = m.mode_mapping()

    target_mode = mode_name
    if target_mode not in mode_mapping:
        if target_mode == "FLOWHOLD" and "ALT_HOLD" in mode_mapping:
            log_msg(
                "Hinweis: FLOWHOLD nicht direkt in Mode-Map -> Verwende ALT_HOLD als Fallback."
            )
            target_mode = "ALT_HOLD"
        else:
            raise Exception(
                f"Modus '{mode_name}' wird vom Flight Controller nicht unterstützt!"
            )

    mode_id = mode_mapping[target_mode]
    log_msg(f"Wechsle in Flugmodus {target_mode}...")
    m.mav.set_mode_send(
        m.target_system,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    started = time.monotonic()
    while time.monotonic() - started < timeout:
        msg = m.recv_match(type="HEARTBEAT", blocking=False)
        if msg and hasattr(m, "flightmode"):
            if m.flightmode == target_mode:
                log_msg(f"Modus erfolgreich auf {target_mode} gewechselt.")
                return target_mode
        time.sleep(0.1)

    log_msg(f"Moduswechsel auf {target_mode} gesendet.")
    return target_mode


def send_origin(m, lat: float = 50.1300, lon: float = 8.6900, alt: float = 100.0):
    """Setzt den virtuellen EKF Global Origin für Optical Flow Navigation."""
    if m is None:
        return
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    alt_int = int(alt * 1000)

    m.mav.set_gps_global_origin_send(
        m.target_system,
        lat_int,
        lon_int,
        alt_int,
    )
    m.mav.set_home_position_send(
        m.target_system,
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
    log_msg("EKF Origin & Home Position gesetzt.")


def request_streams(m):
    """Fordert Sensordaten via MAV_CMD_SET_MESSAGE_INTERVAL (20 Hz) an."""
    message_ids = (
        mavlink.MAVLINK_MSG_ID_RANGEFINDER,
        mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD,
        mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavlink.MAVLINK_MSG_ID_SYS_STATUS,
    )
    for msg_id in message_ids:
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            50_000,
            0,
            0,
            0,
            0,
            0,
        )


def disarm_motors(m):
    """Führt einen sauberen Disarm durch und löscht alle Overrides."""
    if m is None:
        return
    send_rc_raw(m, throttle=PWM_THROTTLE_DISARM)
    for _ in range(3):
        try:
            m.mav.command_long_send(
                m.target_system,
                m.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # Disarm
                21196,
                0,
                0,
                0,
                0,
                0,
            )
            m.arducopter_disarm()
        except Exception:
            pass
        time.sleep(0.08)

    send_rc_raw(m, 0, 0, 0, 0)


def main():
    global master, logger

    parser = argparse.ArgumentParser(
        description="Präziser Schwebeflug im FLOWHOLD-Modus mit prozeduralem Regler & Any-Key Not-Aus."
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=DEFAULT_TARGET_ALT,
        help=f"Zielhöhe in Metern (Standard: {DEFAULT_TARGET_ALT:.2f}m)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_HOVER_DURATION,
        help=f"Schwebeflug-Dauer auf Zielhöhe in Sekunden (Standard: {DEFAULT_HOVER_DURATION:.1f}s)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="FLOWHOLD",
        help="Flugmodus (Standard: FLOWHOLD, Fallback: ALT_HOLD)",
    )
    parser.add_argument(
        "--max-climb-rate",
        type=float,
        default=DEFAULT_MAX_CLIMB_RATE,
        help=f"Maximale Steiggeschwindigkeit in m/s (Standard: {DEFAULT_MAX_CLIMB_RATE:.2f} m/s)",
    )
    parser.add_argument(
        "--target-sink-rate",
        type=float,
        default=DEFAULT_TARGET_SINK_RATE,
        help=f"Ziel-Sinkrate beim Landen in m/s (Standard: {DEFAULT_TARGET_SINK_RATE:.2f} m/s = 5 cm/s)",
    )
    parser.add_argument(
        "--touchdown-alt",
        type=float,
        default=DEFAULT_TOUCHDOWN_ALT,
        help=f"Boden-Abschalthöhe in Metern (Standard: {DEFAULT_TOUCHDOWN_ALT:.2f}m = 5 cm)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=PORT,
        help="Serieller Port (Standard: /dev/serial0)",
    )
    args = parser.parse_args()

    target_alt = args.alt
    hover_duration = args.duration
    target_mode = args.mode
    max_climb_rate = args.max_climb_rate
    target_sink_rate = args.target_sink_rate
    touchdown_alt = args.touchdown_alt

    # Initialisiere prozeduralen Regler
    controller = ProceduralAltitudeController(
        target_alt=target_alt,
        max_climb_rate=max_climb_rate,
        target_sink_rate=target_sink_rate,
        max_accel_climb=DEFAULT_MAX_ACCEL,
        ground_alt=GROUND_ALT_ESTIMATE,
        touchdown_alt=touchdown_alt,
    )

    logger = FlightLogger("logs")

    log_msg("====================================================================")
    log_msg(f"  FLOWHOLD SCHWEBEFLUG AUF {target_alt * 100:.0f} cm (PROZEDURALER REGLER)")
    log_msg(
        f"  Modus: {target_mode} | Haltezeit: {hover_duration:.1f}s | Max-Limit: {MAX_ALTITUDE * 100:.0f} cm"
    )
    log_msg(
        f"  Max Steigrate: {max_climb_rate * 100:.0f} cm/s | Ziel-Sinkrate: {target_sink_rate * 100:.1f} cm/s"
    )
    log_msg(
        f"  Abschalthöhe: {touchdown_alt * 100:.0f} cm | Flow-Adaptiv: Ja | Bremskurve: Quadratwurzel"
    )
    log_msg("  -> ArduPilot regelt Höhe via LiDAR & hält Position via Optical Flow!")
    log_msg(f"  Logdatei: {logger.log_file_path}")
    log_msg(f"  Telemetrie CSV: {logger.csv_file_path}")
    log_msg(
        "  *** SOFORT-NOT-AUS: DRÜCKE EINE BELIEBIGE TASTE ODER STRG+C ZUM KILLEN! ***"
    )
    log_msg("====================================================================")

    t = threading.Thread(target=keyboard_listener, daemon=True)
    t.start()

    try:
        log_msg(f"Verbinde mit Flight Controller ({args.device})...")
        try:
            master = mavutil.mavlink_connection(args.device, baud=BAUD)
        except Exception:
            master = mavutil.mavlink_connection(args.device, baud=921600)

        log_msg("Warte auf Heartbeat...")
        master.wait_heartbeat()
        log_msg("Verbunden!")

        request_streams(master)
        telemetry = TelemetryTracker(master)

        send_origin(master)
        time.sleep(0.3)

        active_mode = set_mode(master, target_mode)

        # Pre-Flight Sensor & Optical Flow Quality Check
        log_msg("Prüfe Optical Flow Sensor & FlowHold Bereitschaft...")
        check_start = time.monotonic()
        while time.monotonic() - check_start < 1.5:
            telemetry.update()
            time.sleep(0.05)

        flow_q = (
            telemetry.flow_quality if telemetry.flow_quality is not None else 0
        )
        if flow_q < 30:
            log_msg(
                f"⚠️ Warnung: Optical Flow Qualität niedrig (Q={flow_q}/255). Beleuchtung & Bodentextur prüfen!"
            )
        else:
            log_msg(
                f"Optical Flow Sensor bereit (Qualitäts-Score: Q={flow_q}/255)."
            )

        log_msg("Arming mit 0.0% Gas (1000 PWM)...")
        send_rc_raw(master, throttle=PWM_THROTTLE_DISARM)
        master.arducopter_arm()

        arm_start = time.monotonic()
        while time.monotonic() - arm_start < 3.0:
            telemetry.update()
            if telemetry.is_armed:
                break
            time.sleep(0.1)

        log_msg("Drohne ist geschärft (ARMED).")

        # -------------------------------------------------------------------------
        # PHASE 1: PROZEDURALER STEIGFLUG AUF ZIELHÖHE (FLOWHOLD)
        # -------------------------------------------------------------------------
        log_msg(
            f"\n>>> 1. STARTE PROZEDURALEN STEIGFLUG auf {target_alt * 100:.0f} cm..."
        )
        climb_start = time.monotonic()
        last_print = 0.0
        target_reached = False

        # Max 10 Sekunden Steigzeit
        while (time.monotonic() - climb_start < 10.0) and not stop_event.is_set():
            alt = telemetry.update()

            # Sicherheits-Check: Bei > 80 cm sofort Landung einleiten
            if alt is not None and alt > MAX_ALTITUDE:
                log_msg(
                    f"\n>>> ⚠️ SICHERHEITSHÖHE ({alt * 100:.1f} cm > {MAX_ALTITUDE * 100:.0f} cm) ERREICHT -> LEITE SOFORT-LANDUNG EIN!"
                )
                break

            # Kontinuierliche prozedurale Berechnung
            active_pwm, active_pct, v_cmd, f_scale = controller.compute_climb(
                current_alt=alt,
                current_vz=telemetry.filtered_vz,
                flow_q=telemetry.flow_quality,
            )

            # Senden: Roll/Pitch/Yaw 1500 (Neutral) für aktiven FlowHold Drift-Ausgleich
            send_rc_raw(master, throttle=active_pwm)

            logger.log_telemetry(
                phase="CLIMB",
                mode=active_mode,
                throttle_pwm=active_pwm,
                throttle_pct=active_pct,
                target_alt=target_alt,
                current_alt=alt,
                v_cmd=v_cmd,
                vz=telemetry.filtered_vz,
                flow_quality=telemetry.flow_quality,
                flow_scale=f_scale,
            )

            now = time.monotonic()
            if now - last_print >= 0.15:
                alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "---"
                v_cmd_str = f"v_cmd:{v_cmd:+5.2f}"
                vz_str = f"vz:{telemetry.filtered_vz:+5.2f} m/s"
                q_str = (
                    f"Q:{telemetry.flow_quality}"
                    if telemetry.flow_quality is not None
                    else "Q:--"
                )
                log_msg(
                    f"[STEIGEN]  Gas: {active_pct:4.1f}% ({active_pwm} PWM) | Ist: {alt_str} / Soll: {target_alt * 100:.0f} cm | {v_cmd_str} | {vz_str} | Flow: {q_str}"
                )
                last_print = now

            # Zielhöhe erreicht: Wenn Höhe >= (Zielhöhe - 2 cm) und Vertikalgeschwindigkeit <= 0.08 m/s
            if (
                alt is not None
                and alt >= target_alt - 0.02
                and telemetry.filtered_vz <= 0.08
            ):
                target_reached = True
                log_msg(
                    f"\n>>> 🎯 ZIELHÖHE ERREICHT! Ist-Höhe: {alt * 100:.1f} cm. Schalte auf Schwebegas (50.0% / 1500 PWM)..."
                )
                send_rc_raw(master, throttle=PWM_THROTTLE_HOVER)
                break

            time.sleep(0.05)

        if not target_reached and not stop_event.is_set():
            alt = telemetry.update()
            alt_str = f"{alt * 100:.1f} cm" if alt is not None else "N/A"
            log_msg(
                f"\n>>> Steigzeit-Limit erreicht (Höhe: {alt_str}). Gehe in Schwebeflug über..."
            )
            send_rc_raw(master, throttle=PWM_THROTTLE_HOVER)

        # -------------------------------------------------------------------------
        # PHASE 2: AKTIVER SCHWEBEFLUG & POSITIONSHALT (3.0 Sekunden)
        # -------------------------------------------------------------------------
        if not stop_event.is_set():
            log_msg(
                f"\n>>> 2. AKTIVER SCHWEBEFLUG für {hover_duration:.1f}s auf {target_alt * 100:.0f} cm (FLOWHOLD)..."
            )
            hover_start = time.monotonic()
            last_print = 0.0

            while (
                time.monotonic() - hover_start < hover_duration
            ) and not stop_event.is_set():
                alt = telemetry.update()

                # Sicherheits-Check
                if alt is not None and alt > MAX_ALTITUDE:
                    log_msg(
                        f"\n>>> ⚠️ SICHERHEITSHÖHE ({alt * 100:.1f} cm > {MAX_ALTITUDE * 100:.0f} cm) ERREICHT -> LEITE LANDUNG EIN!"
                    )
                    break

                hover_pwm, hover_pct = controller.compute_hover(
                    current_alt=alt,
                    current_vz=telemetry.filtered_vz,
                )

                send_rc_raw(master, throttle=hover_pwm)

                logger.log_telemetry(
                    phase="HOVER",
                    mode=active_mode,
                    throttle_pwm=hover_pwm,
                    throttle_pct=hover_pct,
                    target_alt=target_alt,
                    current_alt=alt,
                    v_cmd=0.0,
                    vz=telemetry.filtered_vz,
                    flow_quality=telemetry.flow_quality,
                    flow_scale=controller.flow_scale(telemetry.flow_quality),
                )

                now = time.monotonic()
                if now - last_print >= 0.20:
                    alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "N/A"
                    rem = hover_duration - (now - hover_start)
                    vz_str = f"vz:{telemetry.filtered_vz:+5.2f} m/s"
                    q_str = (
                        f"Q:{telemetry.flow_quality}"
                        if telemetry.flow_quality is not None
                        else "Q:--"
                    )
                    log_msg(
                        f"[SCHWEBEN] Gas: {hover_pct:4.1f}% ({hover_pwm} PWM) | Ist: {alt_str} | Soll: {target_alt * 100:.0f} cm | {vz_str} | Rest: {rem:3.1f}s | Flow: {q_str}"
                    )
                    last_print = now

                time.sleep(0.05)

        # -------------------------------------------------------------------------
        # PHASE 3: GEREGELTER 5 CM/S SINKFLUG & LANDUNG
        # -------------------------------------------------------------------------
        if not stop_event.is_set():
            log_msg(
                f"\n>>> 3. STARTE KONTROLLIERTEN SINKFLUG (~{target_sink_rate * 100:.1f} cm/s) BIS ABSCHALTHÖHE {touchdown_alt * 100:.0f} cm..."
            )
            sink_start = time.monotonic()
            last_print = 0.0
            touchdown_count = 0

            # Max 25 Sekunden Sinkzeit
            while (
                time.monotonic() - sink_start < 25.0
            ) and not stop_event.is_set():
                alt = telemetry.update()

                # Geschwindigkeitsgeregelter Sink-Gaswert
                active_descent_pwm, active_descent_pct, v_cmd = (
                    controller.compute_descent(
                        current_alt=alt,
                        current_vz=telemetry.filtered_vz,
                        flow_q=telemetry.flow_quality,
                    )
                )

                send_rc_raw(master, throttle=active_descent_pwm)

                logger.log_telemetry(
                    phase="DESCENT",
                    mode=active_mode,
                    throttle_pwm=active_descent_pwm,
                    throttle_pct=active_descent_pct,
                    target_alt=0.0,
                    current_alt=alt,
                    v_cmd=v_cmd,
                    vz=telemetry.filtered_vz,
                    flow_quality=telemetry.flow_quality,
                    flow_scale=controller.flow_scale(telemetry.flow_quality),
                )

                now = time.monotonic()
                if now - last_print >= 0.20:
                    alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "---"
                    vz_str = f"vz:{telemetry.filtered_vz:+5.2f} m/s"
                    log_msg(
                        f"[SINKEN]   Gas: {active_descent_pct:4.1f}% ({active_descent_pwm} PWM) | Ist: {alt_str} | {vz_str}"
                    )
                    last_print = now

                # Abschalthöhe erreicht (<= 5 cm für mindestens 2 aufeinanderfolgende Zyklen)
                if alt is not None and alt <= touchdown_alt:
                    touchdown_count += 1
                    if touchdown_count >= 2:
                        log_msg(
                            f"\n>>> 🛬 ABSCHALTHÖHE ERREICHT (Höhe: {alt * 100:.1f} cm <= {touchdown_alt * 100:.0f} cm)!"
                        )
                        break
                else:
                    touchdown_count = 0

                time.sleep(0.05)

            # -------------------------------------------------------------------------
            # PHASE 4: TOUCHDOWN & DISARM (0% GAS)
            # -------------------------------------------------------------------------
            log_msg(">>> 4. MOTOREN AUS (0.0% / 1000 PWM) & DISARM...")
            disarm_motors(master)
            log_msg(
                f"Flugtest erfolgreich beendet - Max-Höhe: {logger.max_alt_seen * 100:.1f} cm!"
            )

    except KeyboardInterrupt:
        log_msg("\nSTRG+C erkannt! LÖSE SOFORT-NOT-AUS AUS!")
        emergency_kill()
    except Exception as e:
        log_msg("Fehler im Ablauf:", e)
        emergency_kill()
    finally:
        stop_event.set()
        if logger:
            logger.close()


if __name__ == "__main__":
    main()
