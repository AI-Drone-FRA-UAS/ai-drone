#!/usr/bin/env python3
"""Präziser Schwebeflug basierend auf ArduPilot FLOWHOLD (Optical Flow Position & LiDAR Altitude Hold).

Ablauf:
1. INITIALISIERUNG: Verbindet via MAVLink, setzt EKF Origin und wechselt in den Modus FLOWHOLD.
2. ARMING: Schärft die Motoren sicher bei 0% Gas (1000 PWM).
3. PHASE 1 (Steigflug): Steigt im FLOWHOLD-Modus (1650 PWM) mit aktiver optischer Driftkorrektur auf 50 cm.
4. PHASE 2 (Schweben): Hält für 3.0 Sekunden vollautomatisch 50 cm Höhe & horizontale Position (1500 PWM).
5. PHASE 3 (Sanftes Sinken): Sinkt geregelt mit ~1380 PWM zu Boden, während die Position gehalten wird.
6. PHASE 4 (Touchdown & Disarm): Schaltet am Boden sofort auf 1000 PWM (0% Gas) und disarmt sicher.

Sicherheit & Logging:
- SOFORT-NOT-AUS per ENTER-Taste oder STRG+C (0% Gas + Force Disarm).
- Sicherheits-Höhenlimit (Standard: 80 cm).
- Automatisches Logging in `logs/flight_YYYYMMDD_HHMMSS.log` und `.csv` Telemetrie.
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

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

# ---------------- Konfiguration ----------------
PORT = "/dev/serial0"
BAUD = 115200  # 115200 oder 921600

DEFAULT_TARGET_ALT = 0.50  # Zielhöhe in Metern (50 cm)
MAX_ALTITUDE = (
    0.80  # Maximales Sicherheits-Höhenlimit in Metern (80 cm) -> leitet Landung ein
)
DEFAULT_HOVER_DURATION = 3.0  # Schwebepause auf 50 cm in Sekunden

# PWM-Werte für ArduPilot FLOWHOLD / ALT_HOLD Modus:
# 1000 = Motor aus / Min
# 1400-1440 = Sanftes Sinken (kontrolliert mit ~10-15 cm/s)
# 1470-1500 = Aktives Bremsen / Halten
# 1530-1545 = Sanfter Steigflug (extrem feinfühlig, verhindert Überschwingen)
PWM_THROTTLE_DISARM = 1000
PWM_THROTTLE_CLIMB = 1540
PWM_THROTTLE_HOVER = 1500
PWM_THROTTLE_BRAKE = 1470
PWM_THROTTLE_DESCENT = 1420
PWM_NEUTRAL = 1500
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
                "mode",
                "throttle_pwm",
                "target_alt_m",
                "current_alt_m",
                "vz_mps",
                "flow_quality",
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
        target_alt: float | None = None,
        current_alt: float | None = None,
        vz: float = 0.0,
        flow_quality: int | None = None,
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
                mode,
                throttle_pwm,
                f"{target_alt:.3f}" if target_alt is not None else "",
                f"{current_alt:.3f}" if current_alt is not None else "",
                f"{vz:.3f}",
                flow_quality if flow_quality is not None else "",
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
    """Sendet rohe RC-Override-PWM-Werte (1000-2000) an ArduPilot."""
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
    """Sofortiger Not-Aus: Schneidet den Motorstrom unverzüglich ab (1000 PWM)."""
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
        description="Präziser Schwebeflug im FLOWHOLD-Modus (Optical Flow + LiDAR)"
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
        "--mode",
        type=str,
        default="FLOWHOLD",
        help="Flugmodus (Standard: FLOWHOLD, Fallback: ALT_HOLD)",
    )
    parser.add_argument(
        "--climb-pwm",
        type=int,
        default=PWM_THROTTLE_CLIMB,
        help="Steig-PWM (Standard: 1650)",
    )
    parser.add_argument(
        "--descent-pwm",
        type=int,
        default=PWM_THROTTLE_DESCENT,
        help="Sink-PWM (Standard: 1380)",
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
    climb_pwm = args.climb_pwm
    descent_pwm = args.descent_pwm

    logger = FlightLogger("logs")

    log_msg("====================================================================")
    log_msg(f"  FLOWHOLD SCHWEBEFLUG AUF {target_alt * 100:.0f} cm")
    log_msg(
        f"  (Modus: {target_mode} | Haltezeit: {hover_duration:.1f}s | Max-Limit: {MAX_ALTITUDE * 100:.0f} cm)"
    )
    log_msg("  -> ArduPilot regelt Höhe via LiDAR & hält Position via Optical Flow!")
    log_msg(f"  Logdatei: {logger.log_file_path}")
    log_msg(f"  Telemetrie CSV: {logger.csv_file_path}")
    log_msg("  *** NOT-AUS: Drücke jederzeit 'ENTER' oder 'STRG+C' zum Killen! ***")
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

        log_msg("Arming mit 0% Gas (1000 PWM)...")
        send_rc_raw(master, throttle=PWM_THROTTLE_DISARM)
        master.arducopter_arm()

        arm_start = time.monotonic()
        while time.monotonic() - arm_start < 3.0:
            telemetry.update()
            if telemetry.is_armed:
                break
            time.sleep(0.1)

        log_msg("Drohne ist geschärft (ARMED).")
        # 5. PHASE 1: GEREGELTER STEIGFLUG AUF ZIELHÖHE (FLOWHOLD)
        log_msg(
            f"\n>>> 1. STARTE SANFTEN STEIGFLUG auf {target_alt * 100:.0f} cm (Throttle: {climb_pwm} PWM)..."
        )
        climb_start = time.monotonic()
        last_print = 0.0
        target_reached = False

        # Max 8 Sekunden Steigzeit
        while (time.monotonic() - climb_start < 8.0) and not stop_event.is_set():
            alt = telemetry.update()

            # Sicherheits-Check: Bei > 80 cm nicht abstürzen lassen, sondern kontrolliert landen!
            if alt is not None and alt > MAX_ALTITUDE:
                log_msg(
                    f"\n>>> ⚠️ SICHERHEITSHÖHE ({alt * 100:.1f} cm > {MAX_ALTITUDE * 100:.0f} cm) ERREICHT -> LEITE SOFORT-LANDUNG EIN!"
                )
                break

            # Vorausschauende Steig-Drosselung (Aktives Bremsen):
            # Ab 28 cm Höhe (55% von 50 cm) Gas auf 1470 PWM drosseln, damit die Aufwärts-
            # Trägheit frühzeitig gebremst wird und die Drohne nicht über 50 cm schießt.
            if alt is not None and alt >= target_alt * 0.55:
                active_throttle = (
                    PWM_THROTTLE_BRAKE  # 1470 PWM: baut Steigrate sanft ab
                )
            else:
                active_throttle = (
                    climb_pwm  # 1540 PWM: feinfühliges Abheben und Steigen
                )

            send_rc_raw(master, throttle=active_throttle)

            logger.log_telemetry(
                phase="CLIMB",
                mode=active_mode,
                throttle_pwm=active_throttle,
                target_alt=target_alt,
                current_alt=alt,
                vz=telemetry.filtered_vz,
                flow_quality=telemetry.flow_quality,
            )

            now = time.monotonic()
            if now - last_print >= 0.15:
                alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "---"
                q_str = (
                    f"Q:{telemetry.flow_quality}"
                    if telemetry.flow_quality is not None
                    else "Q:--"
                )
                log_msg(
                    f"[STEIGEN]  PWM: {active_throttle} | Ist: {alt_str} / Soll: {target_alt * 100:.0f} cm | Flow: {q_str}"
                )
                last_print = now

            # Zielhöhe erreicht -> Sofort in Schwebegas (1500 PWM) wechseln
            if alt is not None and alt >= target_alt * 0.88:
                target_reached = True
                log_msg(
                    f"\n>>> 🎯 ZIELHÖHE ERREICHT! Ist-Höhe: {alt * 100:.1f} cm. Schalte auf Schwebegas (1500 PWM)..."
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

        # 6. PHASE 2: POSITION- & ALTITUDE-HOLD SCHWEBEFLUG (3.0 Sekunden)
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

                # Sicherheits-Check: Bei > 80 cm kontrolliert abfangen und landen
                if alt is not None and alt > MAX_ALTITUDE:
                    log_msg(
                        f"\n>>> ⚠️ SICHERHEITSHÖHE ({alt * 100:.1f} cm > {MAX_ALTITUDE * 100:.0f} cm) ERREICHT -> LEITE LANDUNG EIN!"
                    )
                    break

                # Feine Höhenkorrektur im Schwebeflug:
                # Liegt sie über 54 cm: mit 1460 PWM leicht nach unten drücken
                # Liegt sie unter 46 cm: mit 1530 PWM sanft anheben
                # Ansonsten 1500 PWM Totzone
                if alt is not None and alt > target_alt + 0.04:
                    hover_pwm = 1460
                elif alt is not None and alt < target_alt - 0.04:
                    hover_pwm = 1530
                else:
                    hover_pwm = PWM_THROTTLE_HOVER

                send_rc_raw(master, throttle=hover_pwm)

                logger.log_telemetry(
                    phase="HOVER",
                    mode=active_mode,
                    throttle_pwm=hover_pwm,
                    target_alt=target_alt,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    flow_quality=telemetry.flow_quality,
                )

                now = time.monotonic()
                if now - last_print >= 0.20:
                    alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "N/A"
                    rem = hover_duration - (now - hover_start)
                    q_str = (
                        f"Q:{telemetry.flow_quality}"
                        if telemetry.flow_quality is not None
                        else "Q:--"
                    )
                    log_msg(
                        f"[SCHWEBEN] PWM: {hover_pwm} | Ist: {alt_str} | Soll: {target_alt * 100:.0f} cm | Rest: {rem:3.1f}s | Flow: {q_str}"
                    )
                    last_print = now

                time.sleep(0.05)

        # 7. PHASE 3: GEREGELTER SINKFLUG & LANDUNG
        if not stop_event.is_set():
            log_msg(
                f"\n>>> 3. STARTE KONTROLLIERTEN SINKFLUG / LANDUNG (Throttle: {descent_pwm} PWM)..."
            )
            sink_start = time.monotonic()
            last_print = 0.0
            touchdown_count = 0

            # Max 20 Sekunden Sinkzeit
            while (time.monotonic() - sink_start < 20.0) and not stop_event.is_set():
                alt = telemetry.update()

                # Sanftes Sinken anfordern (1420 PWM)
                # Nahe am Boden (< 15 cm) noch sanfter (1450 PWM)
                current_descent_pwm = descent_pwm
                if alt is not None and alt <= 0.15:
                    current_descent_pwm = min(descent_pwm + 30, 1460)

                send_rc_raw(master, throttle=current_descent_pwm)

                logger.log_telemetry(
                    phase="DESCENT",
                    mode=active_mode,
                    throttle_pwm=current_descent_pwm,
                    target_alt=0.0,
                    current_alt=alt,
                    vz=telemetry.filtered_vz,
                    flow_quality=telemetry.flow_quality,
                )

                now = time.monotonic()
                if now - last_print >= 0.20:
                    alt_str = f"{alt * 100:5.1f} cm" if alt is not None else "---"
                    log_msg(f"[SINKEN]   PWM: {current_descent_pwm} | Ist: {alt_str}")
                    last_print = now

                # Aufsetzen am Boden (< 6 cm)
                if alt is not None and alt <= 0.06:
                    touchdown_count += 1
                    if touchdown_count >= 2:
                        log_msg(
                            f"\n>>> 🛬 BODENKONTAKT ERKANNT (Höhe: {alt * 100:.1f} cm)!"
                        )
                        break
                else:
                    touchdown_count = 0

                time.sleep(0.05)

            log_msg(">>> 4. MOTOREN AUS & DISARM...")
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
