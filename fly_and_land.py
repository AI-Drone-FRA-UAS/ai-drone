#!/usr/bin/env python3
"""Linearer Schubkurven-Test im STABILIZE-Modus (10% - 38% Direkt-Gas).

Im STABILIZE-Modus steuert der Prozent-Wert direkt die Motoren (1:1 stufenlos).
Sanfte Rampe: 10% -> 15% -> 20% -> 25% -> 30% -> max 38%.
Sobald der MTF-01P Laser-LiDAR das Abheben (18 cm) erkennt, hält sie kurz
und senkt das Gas sanft wieder auf 0% ab mit sofortigem Disarm.
Mit SOFORT-NOT-AUS per ENTER-Taste oder STRG+C.
"""

from __future__ import annotations

import select
import sys
import threading
import time

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

# ---------------- Konfiguration ----------------
PORT = "/dev/serial0"
BAUD = 115200  # 115200 oder 921600

LIFTOFF_ALT = 0.18  # Abhebe-Schwelle in Metern (18 cm)
MAX_ALTITUDE = 0.50  # Maximales Sicherheitslimit (50 cm)
MAX_THROTTLE = 38.0  # Maximales Gaslimit (38% für 4S LiPo)
# ------------------------------------------------

master = None
stop_event = threading.Event()


def percent_to_pwm(pct: float) -> int:
    """Wandelt 0-100% Schub linear in RC-PWM (1000-2000) um."""
    clamped = max(0.0, min(100.0, pct))
    return int(1000 + clamped * 10.0)


def send_rc_percent(
    m,
    roll_pct: float = 50.0,
    pitch_pct: float = 50.0,
    throttle_pct: float = 0.0,
    yaw_pct: float = 50.0,
):
    """Sendet Steuerbefehle in Prozent (Roll/Pitch/Yaw 50% = Mitte)."""
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
    print("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("  SOFORT-NOT-AUS: SCHNEIDE MOTORSTROM AB (0% SCHUB)!  ")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
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
        except Exception as e:
            print("Fehler beim Not-Aus:", e)


def keyboard_listener():
    """Hintergrund-Thread: Wartet auf ENTER-Taste für Not-Aus."""
    # 0.5s Pause, damit Start-Enter im Terminal nicht als Not-Aus gewertet wird
    time.sleep(0.5)
    while not stop_event.is_set():
        if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
            line = sys.stdin.readline()
            if line is not None:
                print(">>> TASTENDRUCK (ENTER) ERKANNT! LÖSE NOT-AUS AUS!")
                emergency_kill()
                stop_event.set()
                break


def set_mode(m, mode_name: str):
    mode_mapping = m.mode_mapping()
    if mode_name not in mode_mapping:
        raise Exception(f"Modus '{mode_name}' wird nicht unterstützt!")
    mode_id = mode_mapping[mode_name]
    print(f"Wechsle in Modus {mode_name}...")
    m.mav.set_mode_send(
        m.target_system,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    time.sleep(0.8)


def request_streams(m):
    """Fordert Sensordaten mit 10 Hz an."""
    m.mav.request_data_stream_send(
        m.target_system, m.target_component, mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1
    )
    m.mav.request_data_stream_send(
        m.target_system, m.target_component, mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10, 1
    )


def get_current_altitude(m) -> float | None:
    """Liest die echte Höhe direkt vom MTF-01P Laser-LiDAR."""
    deadline = time.monotonic() + 0.04
    latest_lidar = None

    while time.monotonic() < deadline:
        msg = m.recv_match(type=["RANGEFINDER", "DISTANCE_SENSOR"], blocking=False)
        if msg is None:
            break

        msg_type = msg.get_type()
        if msg_type == "RANGEFINDER":
            dist = float(msg.distance)
            if 0.01 <= dist <= 3.0:
                latest_lidar = dist
        elif msg_type == "DISTANCE_SENSOR":
            orient = getattr(msg, "orientation", 25)
            if orient == 25:  # Downward
                dist = float(msg.current_distance) / 100.0
                if 0.01 <= dist <= 3.0:
                    latest_lidar = dist

    return latest_lidar


def main():
    global master

    print("====================================================================")
    print("  LINEARER SCHUB-TEST IM STABILIZE-MODUS (10% bis max 38%)")
    print("  Stufenlose 1:1 Gassteuerung ohne Totzone!")
    print("  *** NOT-AUS: Drücke jederzeit 'ENTER' oder 'STRG+C' zum Killen! ***")
    print("====================================================================")

    # Not-Aus Listener starten
    t = threading.Thread(target=keyboard_listener, daemon=True)
    t.start()

    try:
        # 1. Verbinden
        print("Verbinde mit Flight Controller...")
        try:
            master = mavutil.mavlink_connection(PORT, baud=BAUD)
        except Exception:
            master = mavutil.mavlink_connection(PORT, baud=921600)

        print("Warte auf Heartbeat...")
        master.wait_heartbeat()
        print("Verbunden!")

        request_streams(master)

        # 2. Modus auf STABILIZE
        print("Setze Modus auf STABILIZE...")
        set_mode(master, "STABILIZE")

        # 3. Schärfen mit 0% Gas
        print("Arming mit 0% Gas...")
        send_rc_percent(master, throttle_pct=0.0)
        master.arducopter_arm()
        print("Drohne ist geschärft (ARMED).")
        time.sleep(1.0)

        # 4. SANFTE LINEARE SCHUB-RAMPE (10% bis max 38%)
        print("\n>>> 1. STARTE SANFTE LINEARE SCHUB-KURVE (10% -> 38%)...")
        thr_pct = 10.0
        liftoff_detected = False

        while thr_pct <= MAX_THROTTLE and not stop_event.is_set():
            # Erhöhe Gas alle 0.1s um +0.5% (sehr sanfter, gleichmäßiger Anstieg)
            thr_pct += 0.5
            send_rc_percent(master, throttle_pct=thr_pct)

            alt = get_current_altitude(master)
            alt_str = f"{alt:.2f} m" if alt is not None else "Am Boden"

            print(f"[GAS-KURVE]  Schub: {thr_pct:5.1f}% | Höhe (LiDAR): {alt_str}")

            # Prüfe, ob Drohne leicht abhebt (LiDAR >= 18 cm)
            if alt is not None and alt >= LIFTOFF_ALT:
                liftoff_detected = True
                print(
                    f"\n>>> 🚀 LEICHTES ABHEBEN BEI {thr_pct:.1f}% SCHUB ERKANNT! Höhe: {alt:.2f} m"
                )
                break

            if alt is not None and alt > MAX_ALTITUDE:
                print(f"WARNUNG: Höhe ({alt:.2f} m) erreicht! Stoppe Steigen.")
                break

            time.sleep(0.1)

        # 5. KURZES SCHWEBEN (2.0 Sekunden)
        if liftoff_detected and not stop_event.is_set():
            print("\n>>> 2. HALTE DIESES GAS für 2.0 Sekunden...")
            hover_start = time.monotonic()
            while time.monotonic() - hover_start < 2.0 and not stop_event.is_set():
                send_rc_percent(master, throttle_pct=thr_pct)
                alt = get_current_altitude(master)
                alt_str = f"{alt:.2f} m" if alt is not None else "N/A"
                print(f"[SCHWEBEN]    Schub: {thr_pct:5.1f}% | Höhe: {alt_str}")
                time.sleep(0.4)

        # 6. SANFTES ABSENKEN (Schub langsam auf 0% runterfahren)
        if not stop_event.is_set():
            print("\n>>> 3. SENKE GAS SANFT WIEDER AB...")
            sink_thr = thr_pct
            while sink_thr >= 0.0:
                sink_thr -= 1.0
                send_rc_percent(master, throttle_pct=max(0.0, sink_thr))

                alt = get_current_altitude(master)
                alt_str = f"{alt:.2f} m" if alt is not None else "Am Boden"
                print(
                    f"[GAS RUNTER]  Schub: {max(0.0, sink_thr):5.1f}% | Höhe: {alt_str}"
                )
                time.sleep(0.08)

            # 7. Dauerhafter Force Disarm & Motoren aus
            print("\n>>> 4. DISARM (Schneide Motorstrom komplett ab)...")
            send_rc_percent(master, throttle_pct=0.0)
            for _ in range(3):
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
                time.sleep(0.1)

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
            print("Motoren vollständig gestoppt (DISARMED)!")

    except KeyboardInterrupt:
        print("\nSTRG+C erkannt! LÖSE SOFORT-NOT-AUS AUS!")
        emergency_kill()
    except Exception as e:
        print("Fehler im Ablauf:", e)
        emergency_kill()
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
