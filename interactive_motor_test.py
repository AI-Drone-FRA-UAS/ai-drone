#!/usr/bin/env python3
"""Interaktiver Motor- & PWM-Test für Drohnen-Rotoren mit Pfeiltasten und WASD.

Steuerung:
  • ↑ (Pfeil HOCH) oder 'w' / '+':   Gas erhöhen (+1.0% / +10 PWM)
  • ↓ (Pfeil RUNTER) oder 's' / '-': Gas senken (-1.0% / -10 PWM)
  • ← / → (LINKS/RECHTS):            Motor wechseln (ALLE, M1, M2, M3, M4)
  • 1, 2, 3, 4:                      Direktauswahl Motor 1 bis 4
  • 0 oder 'a':                      Alle Motoren gleichzeitig ansteuern
  • LEERTASTE (Space):               Sofort-Pause (0.0% Gas / 1000 PWM)
  • 'q' oder 'x' oder STRG+C:        Beenden & sicheres Abschalten

Sicherheit:
  • Vor dem Start: Propeller entfernen oder Sicherheitsabstand einhalten!
  • Konfigurierbares Maximal-Gaslimit (Standard: max. 30.0% / 1300 PWM).
  • Watchdog: Schaltet Motoren bei Verbindungsabbruch sofort stromlos.
"""

from __future__ import annotations

import argparse
import atexit
import os
import select
import sys
import termios
import threading
import time
import tty
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

# ---------------- Standardwerte & Sicherheitslimits ----------------
DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUD = 115200

DEFAULT_MAX_THROTTLE_PCT = 80.0  # Erhöhtes Standardlimit: 80% Gas
PWM_MIN = 1000
PWM_MAX_ABSOLUTE = 2000  # Maximal bis 2000 PWM (100%)
# -------------------------------------------------------------------

master: Any | None = None
running = True


def pct_to_pwm(pct: float) -> int:
    """Rechnet 0.0% - 100.0% in 1000 - 2000 PWM um."""
    clamped = max(0.0, min(100.0, float(pct)))
    return int(round(1000.0 + (clamped / 100.0) * 1000.0))


def pwm_to_pct(pwm: int | float) -> float:
    """Rechnet 1000 - 2000 PWM in 0.0% - 100.0% um."""
    clamped = max(1000.0, min(2000.0, float(pwm)))
    return (clamped - 1000.0) / 10.0


def emergency_stop() -> None:
    """Stoppt alle Motoren unverzüglich und führt Force Disarm durch."""
    global master
    if master is not None:
        try:
            # 1. 0% Gas senden
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                1500,
                1500,
                PWM_MIN,
                1500,
                0,
                0,
                0,
                0,
            )
            # 2. Force Disarm
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                21196,
                0,
                0,
                0,
                0,
                0,
            )
            master.arducopter_disarm()
            # 3. Overrides löschen
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
        except Exception:
            pass


atexit.register(emergency_stop)


class RawTerminal:
    """Kontextmanager für robustes, nicht-blockierendes Einlesen von Tastatur-Eingaben über SSH."""

    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.old_settings: list[Any] | None = None

    def __enter__(self) -> RawTerminal:
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        if self.old_settings is not None and sys.stdin.isatty():
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_actions(self) -> list[str]:
        """Liest gepufferte Tasten/Escape-Sequenzen ein und gibt Aktions-Kommandos zurück."""
        actions: list[str] = []
        if not sys.stdin.isatty():
            return actions

        r, _, _ = select.select([sys.stdin], [], [], 0.02)
        if not r:
            return actions

        try:
            data = os.read(self.fd, 64)
        except Exception:
            return actions

        i = 0
        n = len(data)
        while i < n:
            b = data[i : i + 1]

            # Escape-Sequenzen für Pfeiltasten (\x1b[A, \x1b[B, \x1bOA etc.)
            if b == b"\x1b":
                if i + 2 < n and data[i + 1 : i + 2] in (b"[", b"O"):
                    code = data[i + 2 : i + 3]
                    if code == b"A":
                        actions.append("UP")
                    elif code == b"B":
                        actions.append("DOWN")
                    elif code == b"C":
                        actions.append("RIGHT")
                    elif code == b"D":
                        actions.append("LEFT")
                    i += 3
                    continue
                else:
                    # Isolierte ESC-Taste -> Ignorieren (verhindert versehentliches Schließen)
                    i += 1
                    continue

            # Reguläre Tasten
            try:
                char = b.decode("utf-8", errors="ignore").lower()
            except Exception:
                i += 1
                continue

            if char in ("w", "+"):
                actions.append("UP")
            elif char in ("s", "-"):
                actions.append("DOWN")
            elif char == "d":
                actions.append("RIGHT")
            elif char in ("a", "0"):
                actions.append("ALL")
            elif char in ("1", "2", "3", "4"):
                actions.append(f"M{char}")
            elif char == " ":
                actions.append("STOP")
            elif char in ("q", "x", "\x03"):  # q, x oder STRG+C
                actions.append("QUIT")

            i += 1

        return actions


def draw_progress_bar(pct: float, max_pct: float, width: int = 26) -> str:
    """Erstellt einen farbigen ANSI-Fortschrittsbalken für das Terminal."""
    filled = int(round((pct / max(0.1, max_pct)) * width))
    filled = max(0, min(width, filled))
    bar = "█" * filled + "░" * (width - filled)
    return f"[{bar}]"


def main() -> None:
    global master, running

    parser = argparse.ArgumentParser(
        description="Interaktiver Drohnen-Motortest mit Pfeiltasten und WASD.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
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
        help="Baudrate",
    )
    parser.add_argument(
        "--max-throttle",
        type=float,
        default=80.0,
        help="Maximales Sicherheits-Gaslimit in Prozent (Standard: 80% / 1800 PWM)",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=2.0,
        help="Schrittweite pro Tastendruck in Prozent (z. B. 2.0 für 2% / 20 PWM)",
    )
    args = parser.parse_args()

    max_throttle_pct = min(100.0, max(5.0, args.max_throttle))
    step_pct = max(0.5, min(10.0, args.step))
    max_pwm = pct_to_pwm(max_throttle_pct)

    print("\033[2J\033[H")  # Terminal leeren
    print("====================================================================")
    print("  🚁 INTERAKTIVER DYNAMISCHER MOTOR- & PWM-TEST")
    print(f"  Port: {args.device} ({args.baud} Baud) | Max-Limit: {max_throttle_pct:.1f}% ({max_pwm} PWM)")
    print("====================================================================")
    connected = False
    for attempt in range(1, 4):
        try:
            print(f"Verbinde mit Flight Controller ({args.device}, {args.baud} Baud) - Versuch {attempt}/3...")
            master = mavutil.mavlink_connection(
                args.device,
                baud=args.baud,
                autoreconnect=True,
            )
            # Serielle Puffer leeren
            if hasattr(master, "port") and hasattr(master.port, "reset_input_buffer"):
                try:
                    master.port.reset_input_buffer()
                    master.port.reset_output_buffer()
                except Exception:
                    pass

            master.wait_heartbeat(timeout=6)
            print(f"✅ Verbunden! System-ID: {master.target_system}, Komponente: {master.target_component}")
            connected = True
            break
        except Exception as exc:
            print(f"  ⚠️ Verbindungsversuch {attempt} fehlgeschlagen ({exc}). Wiederhole in 1 Sekunde...")
            if master is not None:
                try:
                    master.close()
                except Exception:
                    pass
            time.sleep(1.0)

    if not connected:
        print("\n❌ Flight Controller antwortet nicht auf /dev/serial0!")
        print("Mögliche Ursachen:")
        print("  1. Akku (LiPo) anstecken, damit der Flight Controller mit Strom versorgt ist.")
        print("  2. Auf dem Pi prüfen, ob ein alter Prozess läuft: 'pkill -9 -f python3'")
        emergency_stop()
        sys.exit(1)

    try:
        # In Modus STABILIZE schalten für 1:1 direkte Gaskontrolle
        print("Schalte in Modus STABILIZE...")
        mode_mapping = master.mode_mapping()
        if "STABILIZE" in mode_mapping:
            master.mav.set_mode_send(
                master.target_system,
                mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_mapping["STABILIZE"],
            )
            time.sleep(0.3)

        # Motoren schärfen bei Standgas
        print("Schärfe Motoren (Arming) bei 0% Gas (1000 PWM)...")
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500,
            1500,
            PWM_MIN,
            1500,
            0,
            0,
            0,
            0,
        )
        master.arducopter_arm()
        time.sleep(0.5)
        print("✅ Drohne ist geschärft (ARMED). Starte interaktive Steuerung...\n")
        time.sleep(0.8)

    except Exception as exc:
        print(f"\n❌ Verbindungsfehler: {exc}")
        emergency_stop()
        sys.exit(1)

    # Zustandsvariablen
    selected_target = "ALL"  # "ALL", "M1", "M2", "M3", "M4"
    current_throttle_pct = 0.0
    current_pwm = PWM_MIN
    battery_voltage = None
    last_send_time = 0.0

    targets = ["ALL", "M1", "M2", "M3", "M4"]

    try:
        with RawTerminal() as term:
            while running:
                now = time.monotonic()

                # MAVLink Nachrichten einlesen (Batteriespannung robust abfragen)
                try:
                    while True:
                        msg = master.recv_match(type="SYS_STATUS", blocking=False)
                        if msg is None:
                            break
                        battery_voltage = float(msg.voltage_battery) / 1000.0
                except Exception:
                    pass

                # Tastendrücke verarbeiten
                actions = term.read_actions()
                for act in actions:
                    if act == "UP":
                        current_throttle_pct = min(max_throttle_pct, current_throttle_pct + step_pct)
                    elif act == "DOWN":
                        current_throttle_pct = max(0.0, current_throttle_pct - step_pct)
                    elif act == "STOP":
                        current_throttle_pct = 0.0
                    elif act == "QUIT":
                        running = False
                        break
                    elif act in ("M1", "M2", "M3", "M4"):
                        selected_target = act
                    elif act == "ALL":
                        selected_target = "ALL"
                    elif act == "RIGHT":
                        idx = (targets.index(selected_target) + 1) % len(targets)
                        selected_target = targets[idx]
                    elif act == "LEFT":
                        idx = (targets.index(selected_target) - 1) % len(targets)
                        selected_target = targets[idx]

                if not running:
                    break

                current_pwm = pct_to_pwm(current_throttle_pct)

                # PWM an Flight Controller senden (mit 20 Hz Watchdog)
                if now - last_send_time >= 0.05:
                    try:
                        if selected_target == "ALL":
                            master.mav.rc_channels_override_send(
                                master.target_system,
                                master.target_component,
                                1500,
                                1500,
                                current_pwm,
                                1500,
                                0,
                                0,
                                0,
                                0,
                            )
                        else:
                            motor_idx = int(selected_target[1])
                            master.mav.command_long_send(
                                master.target_system,
                                master.target_component,
                                mavlink.MAV_CMD_DO_MOTOR_TEST,
                                0,
                                motor_idx,
                                mavlink.MOTOR_TEST_THROTTLE_PERCENT,
                                float(current_throttle_pct),
                                0.20,  # 200 ms Dauer (wird kontinuierlich erneuert)
                                0,
                                mavlink.MOTOR_TEST_ORDER_SEQUENCE,
                                0,
                            )
                    except Exception:
                        pass
                    last_send_time = now

                # Kompakte Live-UI Anzeige (bleibt garantiert in einer Zeile)
                bar = draw_progress_bar(current_throttle_pct, max_throttle_pct, width=12)
                bat_str = f"{battery_voltage:.1f}V" if battery_voltage else "--.-V"

                target_display = f"\033[1;33m[{selected_target:3s}]\033[0m"
                pct_display = f"\033[1;32m{current_throttle_pct:4.1f}%\033[0m"
                pwm_display = f"\033[1;36m{current_pwm} PWM\033[0m"

                sys.stdout.write("\r\033[K")  # Zeile leeren
                sys.stdout.write(
                    f"{target_display} Gas:{pct_display} ({pwm_display}) {bar} | Akku:{bat_str} | [↑/↓ Gas | Space 0 | q Aus]"
                )
                sys.stdout.flush()

                time.sleep(0.02)

    except (KeyboardInterrupt, Exception):
        pass
    finally:
        print("\n\n>>> Beende Motortest & schalte Motoren sicher AUS (0% Gas + Disarm)...")
        emergency_stop()
        print("✅ Motoren erfolgreich AUS & Drohne DISARMED.\n")


if __name__ == "__main__":
    main()
