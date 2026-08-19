#!/usr/bin/env python3
"""Präziser 50 cm Schwebeflug mit aktiver PID-Höhenregelung & ultra-sanftem Sinkflug.

Ablauf:
1. PHASE 1 (Abheben): Erkennt beim Abheben (~8 cm, Boden=2 cm) das exakte Schwebegas (z. B. 39-42%).
2. PHASE 2 (Sanfter Steigflug): Steigt geregelt mit ~8 cm/s sanft und überschwingfrei auf 50 cm.
3. PHASE 3 (Präzisions-Schweben): Hält die Höhe für exakt 3.0 Sekunden aktiv auf 50 cm (PID-Regler).
4. PHASE 4 (Ultra-sanfter Sinkflug): Sinkt mit 2.5 cm/s geregelt zu Boden.
5. PHASE 5 (Bodenkontakt & Disarm): Schaltet am Boden sofort auf 0% Schub und disarmt.

Sicherheit & Logging:
- SOFORT-NOT-AUS per ENTER-Taste oder STRG+C (0% Gas + Force Disarm).
- Sicherheits-Höhenlimit (Standard: 80 cm).
- Automatisches Logging in `logs/flight_YYYYMMDD_HHMMSS.log` und `.csv` Telemetrie.
"""

from __future__ import annotations

import csv
import select
import shutil
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

# ---------------- Konfiguration ----------------
PORT = "/dev/serial0"
BAUD = 115200  # 115200 oder 921600

DEFAULT_TARGET_ALT = 0.50  # Zielhöhe in Metern (50 cm)
MAX_ALTITUDE = 0.80  # Sicherheits-Höhenlimit in Metern (80 cm)
DEFAULT_HOVER_DURATION = 3.0  # Schwebepause auf 50 cm in Sekunden
DEFAULT_CLIMB_SPEED = 0.08  # Steiggeschwindigkeit: 0.08 m/s (8 cm/s)
DEFAULT_DESCENT_SPEED = 0.025  # Sinkgeschwindigkeit: 0.025 m/s (2.5 cm/s)
LIFTOFF_ALT = 0.08  # Abhebe-Schwelle in Metern (8 cm; Bodenwert ist ~2 cm)
MAX_RAMP_THROTTLE = 52.0  # Maximales Schublimit für Abhebe-Suche

# Trimmung gegen Drift (50.0% = Neutral / 1500 PWM)
# > 50.0% zieht nach hinten (Pitch Up), < 50.0% zieht nach vorne (Pitch Down)
DEFAULT_PITCH_TRIM = 51.8  # Standard-Gegensteuerung gegen Vorwärtsdrift
DEFAULT_ROLL_TRIM = 50.0  # Standard-Roll (50% = Mitte)

# PID-Regler Parameter für Höhenhaltung um hover_throttle
KP_ALT = 14.0  # Proportional (% Schub pro Meter Höhenabweichung)
KI_ALT = 1.5  # Integral (% Schub pro Meter*Sekunde)
KD_ALT = 4.0  # Dämpfung (% Schub pro m/s Vertikalgeschwindigkeit)
MAX_I = 3.5  # Maximaler I-Term Anteil (+/- 3.5%)
MAX_CORRECTION = 7.0  # Maximale Schub-Abweichung vom Schwebegas (+/- 7.0%)
# ------------------------------------------------

master = None
stop_event = threading.Event()
logger: FlightLogger | None = None


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
                "throttle_pct",
                "target_alt_m",
                "current_alt_m",
                "vz_mps",
                "p_term",
                "i_term",
                "d_term",
                "total_correction",
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
        throttle_pct: float,
        target_alt: float | None = None,
        current_alt: float | None = None,
        vz: float = 0.0,
        p_term: float = 0.0,
        i_term: float = 0.0,
        d_term: float = 0.0,
        total_correction: float = 0.0,
    ):
        if current_alt is not None and current_alt > self.max_alt_seen:
            self.max_alt_seen = current_alt

        now_iso = datetime.now().isoformat()
        elapsed = time.monotonic() - self.start_time
        self.csv_writer.writerow(
            [
                now_iso,
                f"{elapsed:.3f}",
                phase,
                f"{throttle_pct:.2f}",
                f"{target_alt:.3f}" if target_alt is not None else "",
                f"{current_alt:.3f}" if current_alt is not None else "",
                f"{vz:.3f}",
                f"{p_term:.2f}",
                f"{i_term:.2f}",
                f"{d_term:.2f}",
                f"{total_correction:.2f}",
            ]
        )

    def close(self):
        try:
            if self.log_file and not self.log_file.closed:
                self.log_file.close()
            if self.csv_file and not self.csv_file.closed:
                self.csv_file.close()

            # Kopiere auf latest-Dateien für schnellen Abruf
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
    """Liest nicht-blockierend MAVLink-Sensordaten und berechnet Höhe + Vertikalgeschwindigkeit."""

    def __init__(self, m):
        self.m = m
        self.current_alt: float | None = None
        self.last_alt_time: float = 0.0
        self.filtered_vz: float = 0.0
        self.last_raw_alt: float | None = None
        self.last_raw_time: float = 0.0

    def update(self) -> float | None:
        """Liest alle anstehenden MAVLink-Nachrichten aus dem Puffer."""
        if self.m is None:
            return None

        while True:
            msg = self.m.recv_match(
                type=[
                    "RANGEFINDER",
                    "DISTANCE_SENSOR",
                    "OPTICAL_FLOW",
                    "OPTICAL_FLOW_RAD",
                ],
                blocking=False,
            )
            if msg is None:
                break

            now = time.monotonic()
            raw_dist: float | None = None
            msg_type = msg.get_type()

            if msg_type == "RANGEFINDER":
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
            elif msg_type == "OPTICAL_FLOW_RAD":
                d = float(msg.distance)
                if 0.01 <= d <= 3.0:
                    raw_dist = d

            if raw_dist is not None:
                if self.last_raw_alt is not None and self.last_raw_time > 0:
                    dt = now - self.last_raw_time
                    if dt > 0.005:
                        instant_vz = (raw_dist - self.last_raw_alt) / dt
                        # Tiefpass-Filter für die Vertikalgeschwindigkeit
                        self.filtered_vz = 0.70 * self.filtered_vz + 0.30 * instant_vz

                self.current_alt = raw_dist
                self.last_alt_time = now
                self.last_raw_alt = raw_dist
                self.last_raw_time = now

        # Daten gelten als aktuell, wenn innerhalb der letzten 400ms empfangen
        if time.monotonic() - self.last_alt_time < 0.40:
            return self.current_alt
        return None


class AltitudePID:
    """PID-Höhenregler mit Anti-Windup um das Schwebegas."""

    def __init__(
        self,
        kp: float = KP_ALT,
        ki: float = KI_ALT,
        kd: float = KD_ALT,
        max_i: float = MAX_I,
        max_correction: float = MAX_CORRECTION,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_i = max_i
        self.max_correction = max_correction
        self.integral = 0.0
        self.last_time = time.monotonic()
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.total_correction = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_time = time.monotonic()
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.total_correction = 0.0

    def compute(
        self,
        target_alt: float,
        current_alt: float | None,
        vz: float,
        hover_throttle: float,
    ) -> float:
        if current_alt is None:
            return hover_throttle

        now = time.monotonic()
        dt = max(0.005, min(0.15, now - self.last_time))
        self.last_time = now

        # error > 0: Drohne ist tiefer als Soll -> Schub erhöhen
        # vz > 0: Drohne steigt -> Schub dämpfen
        error = target_alt - current_alt

        # Integriere nur in der Luft (> 4 cm)
        if current_alt > 0.04:
            self.integral += error * dt
            self.integral = max(-self.max_i, min(self.max_i, self.integral))
        else:
            self.integral = 0.0

        self.p_term = self.kp * error
        self.i_term = self.ki * self.integral
        self.d_term = -self.kd * vz

        total = self.p_term + self.i_term + self.d_term
        self.total_correction = max(
            -self.max_correction, min(self.max_correction, total)
        )
        cmd = hover_throttle + self.total_correction
        return max(0.0, min(100.0, cmd))


def percent_to_pwm(pct: float) -> int:
    """Wandelt 0-100% Schub linear in RC-PWM (1000-2000) um."""
    clamped = max(0.0, min(100.0, pct))
    return int(1000 + clamped * 10.0)


def send_rc_percent(
    m,
    roll_pct: float = DEFAULT_ROLL_TRIM,
    pitch_pct: float = DEFAULT_PITCH_TRIM,
    throttle_pct: float = 0.0,
    yaw_pct: float = 50.0,
):
    """Sendet Steuerbefehle in Prozent (Roll/Pitch/Yaw 50% = Neutral/Mitte)."""
    if m is None:
        return
    r_pwm = percent_to_pwm(roll_pct)
    p_pwm = percent_to_pwm(pitch_pct)
    t_pwm = percent_to_pwm(throttle_pct)
    y_pwm = percent_to_pwm(yaw_pct)
    m.mav.rc_channels_override_send(
        m.target_system,
        m.target_component,
        r_pwm,
        p_pwm,
        t_pwm,
        y_pwm,
        0,
        0,
        0,
        0,
    )


def emergency_kill():
    """Sofortiger Not-Aus: Schneidet den Motorstrom unverzüglich ab (0%)."""
    log_msg("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    log_msg("  SOFORT-NOT-AUS: SCHNEIDE MOTORSTROM AB (0% SCHUB)!  ")
    log_msg("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    global master
    if master is not None:
        try:
            send_rc_percent(master, throttle_pct=0.0)
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
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as e:
            log_msg("Fehler beim Not-Aus:", e)


def keyboard_listener():
    """Hintergrund-Thread: Wartet auf ENTER-Taste für Not-Aus."""
    time.sleep(0.5)
    while not stop_event.is_set():
        if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
            line = sys.stdin.readline()
            if line is not None:
                log_msg("\n>>> TASTENDRUCK (ENTER) ERKANNT! LÖSE NOT-AUS AUS!")
                emergency_kill()
                stop_event.set()
                break


def set_mode(m, mode_name: str):
    """Wechselt den Flugmodus auf dem Flight Controller."""
    mode_mapping = m.mode_mapping()
    if mode_name not in mode_mapping:
        raise Exception(f"Modus '{mode_name}' wird nicht unterstützt!")
    mode_id = mode_mapping[mode_name]
    log_msg(f"Wechsle in Modus {mode_name}...")
    m.mav.set_mode_send(
        m.target_system,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    time.sleep(0.8)


def request_streams(m):
    """Fordert Sensordaten via MAV_CMD_SET_MESSAGE_INTERVAL (20 Hz) an."""
    message_ids = (
        mavlink.MAVLINK_MSG_ID_RANGEFINDER,
        mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD,
    )
    for msg_id in message_ids:
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            50_000,  # 50,000 us = 20 Hz
            0,
            0,
            0,
            0,
            0,
        )
    m.mav.request_data_stream_send(
        m.target_system, m.target_component, mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1
    )
    m.mav.request_data_stream_send(
        m.target_system,
        m.target_component,
        mavlink.MAV_DATA_STREAM_RAW_SENSORS,
        20,
        1,
    )


def disarm_motors(m):
    """Führt einen sauberen Disarm durch und löscht alle Overrides."""
    if m is None:
        return
    send_rc_percent(m, throttle_pct=0.0)
    for _ in range(3):
        try:
            m.mav.command_long_send(
                m.target_system,
                m.target_component,
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
            m.arducopter_disarm()
        except Exception:
            pass
        time.sleep(0.08)

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


def main():
    global master, logger

    import argparse

    parser = argparse.ArgumentParser(
        description="Präziser 50 cm Schwebeflug mit PID-Höhenregelung & Trimmung"
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=DEFAULT_TARGET_ALT,
        help="Zielhöhe in Metern (Standard: 0.50m)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_HOVER_DURATION,
        help="Schwebeflug-Dauer auf Zielhöhe in Sekunden (Standard: 3.0s)",
    )
    parser.add_argument(
        "--pitch-trim",
        type=float,
        default=DEFAULT_PITCH_TRIM,
        help="Pitch-Trimmung in Prozent (>50% zieht nach hinten, <50% nach vorne, Standard: 51.8%%)",
    )
    parser.add_argument(
        "--roll-trim",
        type=float,
        default=DEFAULT_ROLL_TRIM,
        help="Roll-Trimmung in Prozent (>50% rechts, <50% links, Standard: 50.0%%)",
    )
    parser.add_argument(
        "--climb-speed",
        type=float,
        default=DEFAULT_CLIMB_SPEED,
        help="Steiggeschwindigkeit in m/s (Standard: 0.08 m/s = 8 cm/s)",
    )
    parser.add_argument(
        "--descent-speed",
        type=float,
        default=DEFAULT_DESCENT_SPEED,
        help="Sinkgeschwindigkeit in m/s (Standard: 0.025 m/s = 2.5 cm/s)",
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
    pitch_trim = args.pitch_trim
    roll_trim = args.roll_trim
    climb_speed = args.climb_speed
    descent_speed = args.descent_speed

    logger = FlightLogger("logs")

    log_msg("====================================================================")
    log_msg(f"  PRÄZISER SCHWEBEFLUG AUF {target_alt * 100:.0f} cm (PID-Geregelt)")
    log_msg(
        f"  (Haltezeit: {hover_duration:.1f}s | Sinkflug: {descent_speed * 100:.1f} cm/s | Max: {MAX_ALTITUDE * 100:.0f} cm)"
    )
    log_msg(f"  Trimmung: Pitch={pitch_trim:.1f}% (>50=hinten), Roll={roll_trim:.1f}%")
    log_msg(f"  Logdatei: {logger.log_file_path}")
    log_msg(f"  Telemetrie CSV: {logger.csv_file_path}")
    log_msg("  *** NOT-AUS: Drücke jederzeit 'ENTER' oder 'STRG+C' zum Killen! ***")
    log_msg("====================================================================")

    # Not-Aus Listener starten
    t = threading.Thread(target=keyboard_listener, daemon=True)
    t.start()

    try:
        # 1. Verbinden
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
        pid = AltitudePID()

        # 2. Modus auf STABILIZE
        log_msg("Setze Modus auf STABILIZE...")
        set_mode(master, "STABILIZE")

        # 3. Schärfen mit 0% Gas
        log_msg("Arming mit 0% Gas...")
        send_rc_percent(
            master,
            roll_pct=roll_trim,
            pitch_pct=pitch_trim,
            throttle_pct=0.0,
        )
        master.arducopter_arm()
        log_msg("Drohne ist geschärft (ARMED).")
        time.sleep(1.0)

        # 4. PHASE 1: SANFTES HOCHFAHREN BIS ZUM ABHEBEN (~8 cm)
        log_msg("\n>>> 1. STARTE SCHUB-RAMPE BIS ZUM ABHEBEN...")
        thr_pct = 15.0
        hover_throttle = 0.0
        liftoff_detected = False

        while thr_pct <= MAX_RAMP_THROTTLE and not stop_event.is_set():
            thr_pct += 0.4
            send_rc_percent(
                master,
                roll_pct=roll_trim,
                pitch_pct=pitch_trim,
                throttle_pct=thr_pct,
            )

            alt = telemetry.update()
            alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "Am Boden"
            log_msg(f"[SUCHE SCHWEBEDRUCK] Schub: {thr_pct:5.1f}% | Höhe: {alt_str}")

            logger.log_telemetry(
                phase="SEARCH_HOVER",
                throttle_pct=thr_pct,
                target_alt=LIFTOFF_ALT,
                current_alt=alt,
                vz=telemetry.filtered_vz,
            )

            # Sicherheitslimit-Check
            if alt is not None and alt > MAX_ALTITUDE:
                log_msg(f"\nWARNUNG: Maximalhöhe ({alt * 100:.1f} cm) überschritten!")
                emergency_kill()
                return

            if alt is not None and alt >= LIFTOFF_ALT:
                hover_throttle = thr_pct
                liftoff_detected = True
                log_msg(
                    f"\n>>> 🚀 ABHEBEN ERKANNT BEI {hover_throttle:.1f}% SCHUB! Starte geregelten Steigflug..."
                )
                break

            time.sleep(0.08)

        if not liftoff_detected or hover_throttle == 0.0:
            log_msg(
                f"\nFEHLER: Drohne konnte bis {MAX_RAMP_THROTTLE:.0f}% Schub nicht abheben. Breche sicher ab."
            )
            disarm_motors(master)
            return

        # 5. PHASE 2: GEREGELTER STEIGFLUG BIS ZIELHÖHE (PID-Regler, kein Überschwingen)
        if not stop_event.is_set():
            log_msg(
                f"\n>>> 2. SANFTER GEREGELTER STEIGFLUG bis {target_alt * 100:.0f} cm ({climb_speed * 100:.0f} cm/s)..."
            )
            pid.reset()
            climb_start = time.monotonic()
            current_alt = telemetry.update()
            start_alt = current_alt if current_alt is not None else LIFTOFF_ALT
            last_print = 0.0

            while (time.monotonic() - climb_start < 8.0) and not stop_event.is_set():
                elapsed = time.monotonic() - climb_start
                ramp_alt = min(target_alt, start_alt + (elapsed * climb_speed))

                alt = telemetry.update()

                # Sicherheitslimit-Check
                if alt is not None and alt > MAX_ALTITUDE:
                    log_msg(
                        f"\nWARNUNG: Maximalhöhe ({alt * 100:.1f} cm) überschritten!"
                    )
                    emergency_kill()
                    return

                active_thr = pid.compute(
                    target_alt=ramp_alt,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    hover_throttle=hover_throttle,
                )
                send_rc_percent(
                    master,
                    roll_pct=roll_trim,
                    pitch_pct=pitch_trim,
                    throttle_pct=active_thr,
                )

                logger.log_telemetry(
                    phase="CLIMB",
                    throttle_pct=active_thr,
                    target_alt=ramp_alt,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    p_term=pid.p_term,
                    i_term=pid.i_term,
                    d_term=pid.d_term,
                    total_correction=pid.total_correction,
                )

                now = time.monotonic()
                if now - last_print >= 0.15:
                    alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "---"
                    log_msg(
                        f"[STEIGEN]  Schub: {active_thr:5.1f}% | Ist: {alt_str} | Soll: {ramp_alt * 100:5.1f} cm"
                    )
                    last_print = now

                # Zielhöhe erreicht und eingeschwungen
                if (
                    ramp_alt >= target_alt
                    and alt is not None
                    and abs(alt - target_alt) <= 0.03
                ):
                    log_msg(
                        f"\n>>> 🎯 {target_alt * 100:.0f} CM ERREICHT & EINGESCHWUNGEN! Höhe: {alt * 100:.1f} cm"
                    )
                    break

                time.sleep(0.05)

        # 6. PHASE 3: PRÄZISIONS-SCHWEBEFLUG AUF EXAKT ZIELHÖHE
        if not stop_event.is_set():
            log_msg(
                f"\n>>> 3. AKTIVES SCHWEBEN für {hover_duration:.1f}s auf exakt {target_alt * 100:.0f} cm..."
            )
            hover_start = time.monotonic()
            last_print = 0.0

            while (
                time.monotonic() - hover_start < hover_duration
            ) and not stop_event.is_set():
                alt = telemetry.update()

                # Sicherheitslimit-Check
                if alt is not None and alt > MAX_ALTITUDE:
                    log_msg(
                        f"\nWARNUNG: Maximalhöhe ({alt * 100:.1f} cm) überschritten!"
                    )
                    emergency_kill()
                    return

                active_thr = pid.compute(
                    target_alt=target_alt,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    hover_throttle=hover_throttle,
                )
                send_rc_percent(
                    master,
                    roll_pct=roll_trim,
                    pitch_pct=pitch_trim,
                    throttle_pct=active_thr,
                )

                logger.log_telemetry(
                    phase="HOVER",
                    throttle_pct=active_thr,
                    target_alt=target_alt,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    p_term=pid.p_term,
                    i_term=pid.i_term,
                    d_term=pid.d_term,
                    total_correction=pid.total_correction,
                )

                now = time.monotonic()
                if now - last_print >= 0.20:
                    alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "N/A"
                    rem = hover_duration - (now - hover_start)
                    log_msg(
                        f"[SCHWEBEN] Schub: {active_thr:5.1f}% | Ist: {alt_str} | Soll: {target_alt * 100:5.1f} cm | Rest: {rem:3.1f}s"
                    )
                    last_print = now

                time.sleep(0.05)

        # 7. PHASE 4: GEREGELTER ULTRA-SANFTER SINKFLUG
        if not stop_event.is_set():
            log_msg(
                f"\n>>> 4. STARTE ULTRA-SANFTEN SINKFLUG ({descent_speed * 100:.1f} cm/s)..."
            )
            pid.reset()
            sink_start = time.monotonic()
            current_alt = telemetry.update()
            start_alt = current_alt if current_alt is not None else target_alt
            last_print = 0.0
            touchdown_count = 0

            # Max 35s Sinkflug
            while (time.monotonic() - sink_start < 35.0) and not stop_event.is_set():
                elapsed = time.monotonic() - sink_start
                desired_alt = max(0.02, start_alt - (elapsed * descent_speed))

                alt = telemetry.update()

                active_thr = pid.compute(
                    target_alt=desired_alt,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    hover_throttle=hover_throttle,
                )
                send_rc_percent(
                    master,
                    roll_pct=roll_trim,
                    pitch_pct=pitch_trim,
                    throttle_pct=active_thr,
                )

                logger.log_telemetry(
                    phase="DESCENT",
                    throttle_pct=active_thr,
                    target_alt=desired_alt,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    p_term=pid.p_term,
                    i_term=pid.i_term,
                    d_term=pid.d_term,
                    total_correction=pid.total_correction,
                )

                now = time.monotonic()
                if now - last_print >= 0.20:
                    alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "---"
                    log_msg(
                        f"[SINKEN]   Schub: {active_thr:5.1f}% | Ist: {alt_str} | Soll: {desired_alt * 100:5.1f} cm"
                    )
                    last_print = now

                # Aufsetzen am Boden (< 4.5 cm bei Sollhöhe < 12 cm)
                if alt is not None and alt <= 0.045 and desired_alt <= 0.12:
                    touchdown_count += 1
                    if touchdown_count >= 3:
                        log_msg(
                            f"\n>>> 🛬 BUTTERWEICHES AUFSETZEN ERKANNT (Höhe: {alt * 100:.1f} cm)!"
                        )
                        break
                else:
                    touchdown_count = 0

                time.sleep(0.05)

            # 8. PHASE 5: MOTOREN AUS & DISARM
            log_msg(">>> 5. MOTOREN AUS & DISARM...")
            disarm_motors(master)
            log_msg(
                f"Flugtest erfolgreich beendet - Max-Höhe: {logger.max_alt_seen * 100:.1f} cm, Schwebegas: {hover_throttle:.1f}%!"
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
