#!/usr/bin/env python3
"""Präziser Schwebeflug & Autonomes Takeoff/Landing ohne GPS.

Highlights:
- Verwendet reale Geschwindigkeits- und Regelungsparameter von ArduPilot:
  * WPNAV_SPEED_UP = 60 cm/s (Sanfter, sicherer Indoor-Steigflug)
  * WPNAV_SPEED_DN = 40 cm/s (Kontrolliertes Sinken)
  * LAND_SPEED = 25 cm/s (Butterweiche Touchdown-Landung)
  * PILOT_SPEED_UP = 80 cm/s (Sicheres Indoor-Tempolimit)
  * MOT_THST_HOVER = 0.25 (Automatisch gelernt via MOT_HOVER_LEARN=2)
- Setzt EKF Global Origin für Non-GPS Navigation (Optical Flow MTF-01P + LiDAR).
- Autonome Landung im ArduPilot-Modus LAND mit Land-Detector Auto-Disarm.

Sicherheitsfunktionen:
- NOT-AUS (Kill Switch): Drücke jederzeit ENTER oder STRG+C (sofortiger Force-Disarm 21196).
- --dry-run Flag: Testet Verbindung, EKF-Origin und Sensor-Telemetrie ohne Motoren zu schärfen.
- Maximales Höhenlimit (--max-alt): Leitet bei Überschreitung sofortige Notlandung ein.
- Live-Telemetrieanzeige & CSV-Logging in logs/flight_YYYYMMDD_HHMMSS.csv.
"""

from __future__ import annotations

import argparse
import csv
import select
import shutil
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

# ---------------- Standard-Konfiguration ----------------
DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUD = 115200

DEFAULT_TARGET_ALT = 0.30  # Zielhöhe in Metern (30 cm)
DEFAULT_MAX_ALTITUDE = 0.60  # Sicherheits-Höhenlimit in Metern (60 cm)
DEFAULT_HOVER_DURATION = 3.0  # Schwebeflugdauer in Sekunden

# Empfohlene ArduPilot-Geschwindigkeits- und Regelungsparameter für Indoor:
INDOOR_PARAMS: dict[str, float] = {
    "WPNAV_SPEED_UP": 60.0,  # 60 cm/s Steigrate bei autonomem Takeoff
    "WPNAV_SPEED_DN": 40.0,  # 40 cm/s Sinkrate
    "WPNAV_ACCEL_Z": 100.0,  # 100 cm/s² sanfte Vertikal-Beschleunigung
    "WPNAV_ACCEL": 100.0,  # 100 cm/s² sanfte Horizontal-Beschleunigung (für Wegpunkte)
    "LAND_SPEED": 25.0,  # 25 cm/s Landegeschwindigkeit
    "PILOT_SPEED_UP": 80.0,  # 80 cm/s Maximal-Steigrate in ALT_HOLD
    "PILOT_SPEED_DN": 40.0,  # 40 cm/s Maximal-Sinkrate in ALT_HOLD
    "PILOT_ACCEL_Z": 100.0,  # 100 cm/s² feinfühlige Vertikal-Beschleunigung in ALT_HOLD
}

# Throttle-Werte für ALT_HOLD (unter Berücksichtigung von THR_DZ=100 -> Totzone 1400-1600):
PWM_THROTTLE_DISARM = 1000
PWM_THROTTLE_CLIMB = 1680  # Hebt ab mit durch PILOT_SPEED_UP limitierter Steigrate
PWM_THROTTLE_HOVER = 1500  # Neutral / Schwebegas (Soll-Steigrate 0.0 m/s)
PWM_NEUTRAL = 1500
# --------------------------------------------------------

master: Any | None = None
stop_event = threading.Event()
logger: FlightLogger | None = None


class FlightLogger:
    """Schreibt Logausgaben auf stdout und speichert Logdateien sowie CSV-Telemetrie."""

    def __init__(self, log_dir: Path | str = "logs") -> None:
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
                "target_alt_m",
                "current_alt_m",
                "vz_mps",
                "flow_quality",
                "voltage_v",
            ]
        )

        self.start_time = time.monotonic()
        self.max_alt_seen: float = 0.0

    def print(self, *args: Any, **kwargs: Any) -> None:
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
        throttle_pwm: int = 1500,
        target_alt: float | None = None,
        current_alt: float | None = None,
        vz: float = 0.0,
        flow_quality: int | None = None,
        voltage: float | None = None,
    ) -> None:
        if current_alt is not None and current_alt > self.max_alt_seen:
            self.max_alt_seen = current_alt

        now_iso = datetime.now().isoformat()
        elapsed = time.monotonic() - self.start_time
        self.csv_writer.writerow(
            [
                now_iso,
                f"{elapsed:.3f}",
                phase,
                mode,
                throttle_pwm,
                f"{target_alt:.3f}" if target_alt is not None else "",
                f"{current_alt:.3f}" if current_alt is not None else "",
                f"{vz:.3f}",
                flow_quality if flow_quality is not None else "",
                f"{voltage:.2f}" if voltage is not None else "",
            ]
        )

    def close(self) -> None:
        try:
            if self.log_file and not self.log_file.closed:
                self.log_file.close()
            if self.csv_file and not self.csv_file.closed:
                self.csv_file.close()

            shutil.copyfile(self.log_file_path, self.latest_log_path)
            shutil.copyfile(self.csv_file_path, self.latest_csv_path)
        except Exception as exc:
            print("Fehler beim Schließen der Logdateien:", exc)


def log_msg(*args: Any, **kwargs: Any) -> None:
    """Zentraler Logging-Wrapper."""
    if logger is not None:
        logger.print(*args, **kwargs)
    else:
        print(*args, **kwargs)


class TelemetryTracker:
    """Liest nicht-blockierend MAVLink-Sensordaten (LiDAR, Optical Flow, EKF, Status)."""

    def __init__(self, m: Any) -> None:
        self.m = m
        self.current_alt: float | None = None
        self.rangefinder_alt: float | None = None
        self.last_alt_time: float = 0.0
        self.filtered_vz: float = 0.0
        self.last_raw_alt: float | None = None
        self.last_raw_time: float = 0.0
        self.flow_quality: int | None = None
        self.flight_mode: str | None = None
        self.is_armed: bool = False
        self.battery_voltage: float | None = None
        self.last_status_text: str | None = None
        self.ekf_flags: int | None = None

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
                if 0.01 <= d <= 5.0:
                    self.rangefinder_alt = d
                    raw_dist = d
            elif msg_type == "DISTANCE_SENSOR":
                orient = getattr(msg, "orientation", 25)
                if orient == 25:  # Downward LiDAR
                    d = float(msg.current_distance) / 100.0
                    if 0.01 <= d <= 5.0:
                        self.rangefinder_alt = d
                        raw_dist = d
            elif msg_type == "OPTICAL_FLOW":
                d = float(msg.ground_distance)
                if 0.01 <= d <= 5.0 and self.rangefinder_alt is None:
                    raw_dist = d
                self.flow_quality = int(msg.quality)
            elif msg_type == "OPTICAL_FLOW_RAD":
                d = float(msg.distance)
                if 0.01 <= d <= 5.0 and self.rangefinder_alt is None:
                    raw_dist = d
                self.flow_quality = int(msg.quality)
            elif msg_type == "LOCAL_POSITION_NED":
                if self.rangefinder_alt is None:
                    raw_dist = -float(msg.z)
            elif msg_type == "SYS_STATUS":
                self.battery_voltage = float(msg.voltage_battery) / 1000.0
            elif msg_type == "STATUSTEXT":
                text = getattr(msg, "text", "")
                if isinstance(text, bytes):
                    text = text.decode("utf-8", errors="replace")
                text = text.strip()
                self.last_status_text = text
                log_msg(f"[FC Status] {text}")
            elif msg_type == "EKF_STATUS_REPORT":
                self.ekf_flags = getattr(msg, "flags", None)

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

        if time.monotonic() - self.last_alt_time < 0.50:
            return self.current_alt
        return None


def send_rc_raw(
    m: Any,
    roll: int = PWM_NEUTRAL,
    pitch: int = PWM_NEUTRAL,
    throttle: int = PWM_THROTTLE_DISARM,
    yaw: int = PWM_NEUTRAL,
) -> None:
    """Sendet RC-Kanal-Overrides an ArduPilot (1000-2000, 1500=Neutral)."""
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


def clear_rc_overrides(m: Any) -> None:
    """Löscht alle RC-Kanal-Overrides (gibt die Kontrolle vollständig an ArduPilot zurück)."""
    if m is None:
        return
    m.mav.rc_channels_override_send(
        m.target_system,
        m.target_component,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def emergency_kill() -> None:
    """Sofortiger Not-Aus (Kill Switch): Schneidet den Motorstrom unverzüglich ab."""
    log_msg("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    log_msg("  SOFORT-NOT-AUS: MOTOREN WERDEN SOFORT ABGESCHALTET!  ")
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
                21196,  # Force Disarm (Kill)
                0,
                0,
                0,
                0,
                0,
            )
            master.arducopter_disarm()
            clear_rc_overrides(master)
        except Exception as exc:
            log_msg("Fehler beim Not-Aus:", exc)


def keyboard_listener() -> None:
    """Hintergrund-Thread: Wartet auf Tastendruck (ENTER) für sofortigen Not-Aus."""
    time.sleep(0.3)
    while not stop_event.is_set():
        if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
            line = sys.stdin.readline()
            if line is not None:
                log_msg("\n>>> TASTENDRUCK (ENTER) ERKANNT! LÖSE NOT-AUS AUS!")
                emergency_kill()
                stop_event.set()
                break


def set_mode(m: Any, mode_name: str, timeout: float = 5.0) -> str:
    """Wechselt den Flugmodus und verifiziert die Bestätigung von ArduPilot."""
    mode_mapping = m.mode_mapping()
    target_mode = mode_name

    if target_mode not in mode_mapping:
        if target_mode == "FLOWHOLD" and "ALT_HOLD" in mode_mapping:
            log_msg(
                "Hinweis: FLOWHOLD nicht direkt in Mode-Map -> Verwende ALT_HOLD als Fallback."
            )
            target_mode = "ALT_HOLD"
        else:
            raise ValueError(
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
        if msg and hasattr(m, "flightmode") and m.flightmode == target_mode:
            log_msg(f"Modus erfolgreich auf {target_mode} gewechselt.")
            return target_mode
        time.sleep(0.1)

    log_msg(f"Moduswechsel auf {target_mode} gesendet.")
    return target_mode


def configure_indoor_params(m: Any) -> None:
    """Setzt die empfohlenen Indoor-Geschwindigkeiten direkt im Flight Controller."""
    if m is None:
        return
    log_msg("Konfiguriere ArduPilot Indoor-Geschwindigkeitsparameter...")
    for param_name, param_val in INDOOR_PARAMS.items():
        try:
            m.mav.param_set_send(
                m.target_system,
                m.target_component,
                param_name.encode("utf-8"),
                float(param_val),
                mavlink.MAV_PARAM_TYPE_REAL32,
            )
            log_msg(f"  • {param_name:15s} = {param_val:.0f} cm/s")
        except Exception as e:
            log_msg(f"  • {param_name:15s} Fehler: {e}")
        time.sleep(0.02)


def send_origin(
    m: Any, lat: float = 50.1300, lon: float = 8.6900, alt: float = 100.0
) -> None:
    """Setzt den EKF Global Origin für Non-GPS Optical Flow Navigation."""
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
    log_msg(f"EKF Global Origin gesetzt (Lat: {lat:.4f}, Lon: {lon:.4f}, Alt: {alt:.0f}m).")


def request_streams(m: Any) -> None:
    """Fordert Telemetrie via modernem MAVLink 2 Intervall (20 Hz) an."""
    message_ids = (
        mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        mavlink.MAVLINK_MSG_ID_RANGEFINDER,
        mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD,
        mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
    )
    for msg_id in message_ids:
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            50_000,  # 50.000 µs = 20 Hz
            0,
            0,
            0,
            0,
            0,
        )


def disarm_motors(m: Any, timeout: float = 3.0) -> bool:
    """Schaltet die Motoren am Boden zuverlässig und sofort ab (Force Disarm)."""
    if m is None:
        return False
    log_msg("Sende Motor-Aus-Signal (Force Disarm)...")
    send_rc_raw(m, throttle=PWM_THROTTLE_DISARM)

    start = time.monotonic()
    while time.monotonic() - start < timeout:
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # 0 = Disarm
            21196,  # 21196 = Force Disarm (Kill)
            0,
            0,
            0,
            0,
            0,
        )
        m.arducopter_disarm()

        msg = m.recv_match(type="HEARTBEAT", blocking=False)
        if msg and not (msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log_msg(">>> ✅ Motoren erfolgreich AUS & DISARMED!")
            clear_rc_overrides(m)
            return True
        time.sleep(0.1)

    log_msg("Warnung: Disarm per Heartbeat noch nicht bestätigt.")
    clear_rc_overrides(m)
    return False


def run_dry_run(master: Any, telemetry: TelemetryTracker, duration: float = 8.0) -> None:
    """Führt einen sicheren Telemetrie- und EKF-Test durch, OHNE die Motoren zu schärfen."""
    log_msg("\n====================================================================")
    log_msg(f"  DRY-RUN (SICHERHEITS-TEST): Lese Telemetrie für {duration:.1f}s")
    log_msg("  (Motoren bleiben DISARMED! Hebe die Drohne mit der Hand an...)")
    log_msg("====================================================================")

    start = time.monotonic()
    last_print = 0.0
    while time.monotonic() - start < duration and not stop_event.is_set():
        alt = telemetry.update()
        now = time.monotonic()
        if now - last_print >= 0.25:
            alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "--- cm"
            q_str = (
                f"Q:{telemetry.flow_quality:3d}"
                if telemetry.flow_quality is not None
                else "Q: --"
            )
            bat_str = (
                f"{telemetry.battery_voltage:4.2f}V"
                if telemetry.battery_voltage is not None
                else "--.-V"
            )
            ekf_str = f"EKF:0x{telemetry.ekf_flags:04x}" if telemetry.ekf_flags else "EKF: --"
            mode_str = telemetry.flight_mode or "---"
            log_msg(
                f"[DRY-RUN] Modus: {mode_str:8s} | Höhe: {alt_str} | vz: {telemetry.filtered_vz:+.2f} m/s | Flow: {q_str} | {ekf_str} | Akku: {bat_str}"
            )
            last_print = now
        time.sleep(0.04)

    log_msg(">>> ✅ DRY-RUN erfolgreich beendet. Sensoren antworten normal.")


def main() -> None:
    global master, logger

    parser = argparse.ArgumentParser(
        description="Präziser Schwebeflug & Autonomes Takeoff/Landing ohne GPS (Optical Flow + LiDAR)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="ALT_HOLD",
        choices=["ALT_HOLD", "FLOWHOLD", "GUIDED"],
        help="Flugmodus: ALT_HOLD (empfohlen für Takeoff), FLOWHOLD oder GUIDED",
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=DEFAULT_TARGET_ALT,
        help="Zielhöhe in Metern",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_HOVER_DURATION,
        help="Schwebeflug-Dauer auf Zielhöhe in Sekunden",
    )
    parser.add_argument(
        "--max-alt",
        type=float,
        default=DEFAULT_MAX_ALTITUDE,
        help="Sicherheits-Höhenlimit in Metern (löst sofortige Notlandung aus)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Nur Telemetrie & EKF testen, OHNE Motoren zu schärfen",
    )
    parser.add_argument(
        "--no-param-sync",
        action="store_true",
        help="Überspringe das automatische Setzen der Indoor-Geschwindigkeitsparameter",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=DEFAULT_PORT,
        help="Serieller Port zum Flight Controller",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help="Baudrate der seriellen Verbindung",
    )
    args = parser.parse_args()

    target_alt = args.alt
    hover_duration = args.duration
    max_alt = args.max_alt
    target_mode = args.mode
    is_dry_run = args.dry_run

    if target_alt > max_alt:
        print(
            f"Fehler: Zielhöhe ({target_alt}m) darf nicht größer als Max-Limit ({max_alt}m) sein!"
        )
        sys.exit(1)

    logger = FlightLogger("logs")

    log_msg("====================================================================")
    log_msg(f"  SCHWEBEFLUG AUF {target_alt * 100:.0f} cm (Modus: {target_mode})")
    log_msg(
        f"  (Dauer: {hover_duration:.1f}s | Limit: {max_alt * 100:.0f} cm | Dry-Run: {is_dry_run})"
    )
    log_msg("  -> Non-GPS Optical Flow (MTF-01P) & LiDAR Navigation")
    log_msg(f"  Logdatei: {logger.log_file_path}")
    log_msg(f"  Telemetrie CSV: {logger.csv_file_path}")
    log_msg("  *** NOT-AUS: Drücke jederzeit 'ENTER' oder 'STRG+C' zum Killen! ***")
    log_msg("====================================================================")

    # Not-Aus Tastatur-Thread starten
    kill_thread = threading.Thread(target=keyboard_listener, daemon=True)
    kill_thread.start()

    try:
        # 1. Verbindung herstellen
        log_msg(f"Verbinde mit Flight Controller ({args.device}, {args.baud} Baud)...")
        master = mavutil.mavlink_connection(args.device, baud=args.baud)

        log_msg("Warte auf Heartbeat vom Flight Controller...")
        master.wait_heartbeat(timeout=10)
        log_msg(
            f"Verbunden! Target System: {master.target_system}, Component: {master.target_component}"
        )

        # 2. Telemetrie-Streams & Tracker initialisieren
        request_streams(master)
        telemetry = TelemetryTracker(master)

        # 3. EKF Global Origin für Non-GPS/Optical-Flow Navigation setzen
        send_origin(master)
        time.sleep(0.3)

        # 4. Automatische Konfiguration der Indoor-Geschwindigkeiten
        if not args.no_param_sync:
            configure_indoor_params(master)
            time.sleep(0.2)

        # 5. Flugmodus setzen
        active_mode = set_mode(master, target_mode)

        # Falls Dry-Run angefordert: Nur Sensortest ausführen und beenden
        if is_dry_run:
            run_dry_run(master, telemetry, duration=10.0)
            return

        # 6. ARMING
        log_msg(f"Schärfe Motoren (Arming) im Modus {active_mode}...")
        send_rc_raw(master, throttle=PWM_THROTTLE_DISARM)
        master.arducopter_arm()

        arm_start = time.monotonic()
        while time.monotonic() - arm_start < 4.0:
            telemetry.update()
            if telemetry.is_armed:
                break
            time.sleep(0.1)

        if not telemetry.is_armed:
            reason = (
                f" (Meldung: '{telemetry.last_status_text}')"
                if telemetry.last_status_text
                else ""
            )
            log_msg(f"Arming fehlgeschlagen{reason}! Breche ab.")
            return

        log_msg(">>> Drohne ist geschärft (ARMED).")
        time.sleep(0.3)

        # ------------------------------------------------------------------
        # PHASE 1: GEREGELTER STEIGFLUG (Takeoff)
        # ------------------------------------------------------------------
        if active_mode == "GUIDED":
            log_msg(
                f"\n>>> 1. STARTE AUTONOMEN TAKEOFF auf {target_alt * 100:.0f} cm (MAV_CMD_NAV_TAKEOFF)..."
            )
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
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
        else:
            log_msg(
                f"\n>>> 1. STARTE STEIGFLUG auf {target_alt * 100:.0f} cm (Geregelt via PILOT_SPEED_UP=80 cm/s)..."
            )
            send_rc_raw(master, throttle=PWM_THROTTLE_CLIMB)

        climb_start = time.monotonic()
        last_print = 0.0
        target_reached = False

        while (time.monotonic() - climb_start < 8.0) and not stop_event.is_set():
            alt = telemetry.update()
            now = time.monotonic()

            if active_mode != "GUIDED":
                send_rc_raw(master, throttle=PWM_THROTTLE_CLIMB)

            logger.log_telemetry(
                phase="CLIMB",
                mode=active_mode,
                throttle_pwm=PWM_THROTTLE_CLIMB if active_mode != "GUIDED" else 1500,
                target_alt=target_alt,
                current_alt=alt,
                vz=telemetry.filtered_vz,
                flow_quality=telemetry.flow_quality,
                voltage=telemetry.battery_voltage,
            )

            if now - last_print >= 0.15:
                alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "---"
                q_str = (
                    f"Q:{telemetry.flow_quality}"
                    if telemetry.flow_quality is not None
                    else "Q:--"
                )
                log_msg(
                    f"[STEIGEN]  Ist: {alt_str} / Soll: {target_alt * 100:.0f} cm | vz: {telemetry.filtered_vz:+.2f} m/s | Flow: {q_str}"
                )
                last_print = now

            # Sicherheits-Check
            if alt is not None and alt > max_alt:
                log_msg(
                    f"\n>>> ⚠️ SICHERHEITSHÖHE ({alt * 100:.1f} cm > {max_alt * 100:.0f} cm) ERREICHT -> LEITE LANDUNG EIN!"
                )
                break

            # Zielhöhe erreicht (>= 85% von target_alt)
            if alt is not None and alt >= target_alt * 0.85:
                target_reached = True
                log_msg(
                    f"\n>>> 🎯 ZIELHÖHE ERREICHT! Ist-Höhe: {alt * 100:.1f} cm. Schalte auf Schwebegas (1500 PWM)..."
                )
                send_rc_raw(master, throttle=PWM_THROTTLE_HOVER)
                break

            time.sleep(0.04)

        if not target_reached and not stop_event.is_set():
            send_rc_raw(master, throttle=PWM_THROTTLE_HOVER)

        # ------------------------------------------------------------------
        # PHASE 2: SCHWEBEFLUG (Position & Altitude Hold)
        # ------------------------------------------------------------------
        if not stop_event.is_set():
            log_msg(
                f"\n>>> 2. AKTIVER SCHWEBEFLUG für {hover_duration:.1f}s auf {target_alt * 100:.0f} cm..."
            )
            hover_start = time.monotonic()
            last_print = 0.0

            while (
                time.monotonic() - hover_start < hover_duration
            ) and not stop_event.is_set():
                alt = telemetry.update()

                if active_mode != "GUIDED":
                    send_rc_raw(master, throttle=PWM_THROTTLE_HOVER)

                logger.log_telemetry(
                    phase="HOVER",
                    mode=active_mode,
                    throttle_pwm=PWM_THROTTLE_HOVER,
                    target_alt=target_alt,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    flow_quality=telemetry.flow_quality,
                    voltage=telemetry.battery_voltage,
                )

                # Sicherheits-Check
                if alt is not None and alt > max_alt:
                    log_msg(
                        f"\n>>> ⚠️ SICHERHEITSHÖHE ({alt * 100:.1f} cm > {max_alt * 100:.0f} cm) ERREICHT -> LEITE LANDUNG EIN!"
                    )
                    break

                now = time.monotonic()
                if now - last_print >= 0.20:
                    alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "N/A"
                    rem = hover_duration - (now - hover_start)
                    q_str = (
                        f"Q:{telemetry.flow_quality}"
                        if telemetry.flow_quality is not None
                        else "Q:--"
                    )
                    bat_str = (
                        f"{telemetry.battery_voltage:.2f}V"
                        if telemetry.battery_voltage is not None
                        else "--V"
                    )
                    log_msg(
                        f"[SCHWEBEN] Ist: {alt_str} | Soll: {target_alt * 100:.0f} cm | Rest: {rem:3.1f}s | Flow: {q_str} | Akku: {bat_str}"
                    )
                    last_print = now

                time.sleep(0.04)

        # ------------------------------------------------------------------
        # PHASE 3: AUTONOME LANDUNG (ArduPilot Modus LAND mit LAND_SPEED=25 cm/s)
        # ------------------------------------------------------------------
        if not stop_event.is_set():
            log_msg("\n>>> 3. STARTE LANDUNG: Wechsle in ArduPilot-Modus 'LAND'...")
            clear_rc_overrides(master)
            set_mode(master, "LAND")

            sink_start = time.monotonic()
            last_print = 0.0

            # Max 15 Sekunden Landezeit
            while (time.monotonic() - sink_start < 15.0) and not stop_event.is_set():
                alt = telemetry.update()

                logger.log_telemetry(
                    phase="LANDING",
                    mode=telemetry.flight_mode or "LAND",
                    throttle_pwm=0,
                    target_alt=0.0,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    flow_quality=telemetry.flow_quality,
                    voltage=telemetry.battery_voltage,
                )

                now = time.monotonic()
                if now - last_print >= 0.20:
                    alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "---"
                    vz_str = f"{telemetry.filtered_vz:+.2f} m/s"
                    log_msg(
                        f"[LANDEN]   Modus: {telemetry.flight_mode or 'LAND'} | Ist: {alt_str} | vz: {vz_str} | Status: Sinkflug..."
                    )
                    last_print = now

                # Touchdown Erkennung:
                # 1. ArduPilot Land-Detector hat Motoren disarmed
                if not telemetry.is_armed:
                    log_msg("\n>>> 🛬 BODENKONTAKT & ARDUPILOT AUTO-DISARM ERKANNT!")
                    break

                # 2. LiDAR meldet Aufsetzen (< 4 cm)
                if alt is not None and alt <= 0.04:
                    log_msg(
                        f"\n>>> 🛬 BODENKONTAKT ERKANNT (Höhe: {alt * 100:.1f} cm) -> Disarme!"
                    )
                    break

                time.sleep(0.04)

            log_msg(">>> 4. MOTOREN DISARMEN (Sicherheits-Check)...")
            disarm_motors(master)
            log_msg(
                f"Flugtest erfolgreich beendet! Max-Höhe: {logger.max_alt_seen * 100:.1f} cm."
            )

    except KeyboardInterrupt:
        log_msg("\nSTRG+C erkannt! LÖSE SOFORT-NOT-AUS AUS!")
        emergency_kill()
    except Exception as exc:
        log_msg("Fehler im Ablauf:", exc)
        emergency_kill()
    finally:
        stop_event.set()
        if master is not None:
            try:
                disarm_motors(master, timeout=1.0)
            except Exception:
                pass
        if logger:
            logger.close()


if __name__ == "__main__":
    main()
