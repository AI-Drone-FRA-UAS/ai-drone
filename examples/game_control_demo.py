"""Interaktives Demo- & Test-Skript für die GameMode Drohnensteuerung.

Unterstützt zwei Betriebsmodi:
1. Automatische Choreografie-Sequenz (Start -> Vorwärts -> Hover -> Drehung -> Landung)
2. Interaktive Terminal-Tastatursteuerung (WASD / Space / Shift / Q / E / X)
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Any
from unittest.mock import MagicMock

from ai_drone.game_actor import (
    DEFAULT_FAILSAFE_CONFIG,
    DroneGameActor,
    FailsafeConfig,
    Vector3,
)

_keyboard_stop_event = threading.Event()


def create_mock_actor(failsafe_cfg: FailsafeConfig) -> DroneGameActor:
    """Erstellt eine gemockte Drohnen-Instanz für Dry-Run Tests ohne echte Hardware."""
    actor = DroneGameActor(
        device="udp:127.0.0.1:14550",
        failsafe_config=failsafe_cfg,
        auto_connect=False,
    )
    mock_conn = MagicMock()
    mock_conn.mode_mapping.return_value = {"ALT_HOLD": 2, "FLOWHOLD": 22, "LAND": 9}
    mock_conn.target_system = 1
    mock_conn.target_component = 1
    mock_conn.flightmode = "ALT_HOLD"
    mock_conn.recv_match.return_value = None

    actor.connection = mock_conn
    actor.flight_mode = "ALT_HOLD"
    actor.is_armed = False
    actor.current_altitude = 0.02
    actor.last_telemetry_time = time.monotonic()

    # Simuliere Arming & Höhengewinn beim Takeoff
    orig_arm = mock_conn.arducopter_arm

    def mock_arm(*_args: Any, **_kwargs: Any) -> None:
        actor.is_armed = True
        actor.last_telemetry_time = time.monotonic()

    orig_arm.side_effect = mock_arm

    # Starte Streamer
    actor._stop_event.clear()
    actor._streamer_thread = threading.Thread(
        target=actor._streamer_loop, daemon=True, name="Mock-Streamer"
    )
    actor._streamer_thread.start()

    # Simulierter Höhenanstieg
    def altitude_sim() -> None:
        while not actor._stop_event.is_set():
            actor.last_telemetry_time = time.monotonic()
            if actor.is_flying and actor.current_altitude is not None:
                # Reagiere auf Throttle
                thr = actor._target_throttle
                if thr > 1500:
                    actor.current_altitude = min(0.55, actor.current_altitude + 0.03)
                elif thr < 1490:
                    actor.current_altitude = max(0.02, actor.current_altitude - 0.03)
            time.sleep(0.05)

    sim_thread = threading.Thread(target=altitude_sim, daemon=True)
    sim_thread.start()

    return actor


def run_choreography(drone: DroneGameActor, target_alt: float = 0.50) -> None:
    """Führt eine programmierte Choreografie-Sequenz aus."""
    print("\n=========================================================")
    print("  STARTE AUTOMATISCHE CHOREOGRAFIE-SEQUENZ")
    print("=========================================================")
    print(f"1. Start (Takeoff) auf {target_alt * 100:.0f} cm...")
    drone.takeoff(height_m=target_alt)

    print("2. Schwebeflug für 1.5 Sekunden...")
    drone.hover(duration_s=1.5)

    print("3. Fliege 1.0 Sekunde Vorwärts...")
    drone.move_forward(duration_s=1.0, speed=0.35)

    print("4. Schwebeflug für 1.0 Sekunde...")
    drone.hover(duration_s=1.0)

    print("5. Drehung um 90° nach Rechts (Yaw)...")
    drone.rotate_yaw(duration_s=1.0, speed=0.40)

    print("6. Kombinierte Vektorbewegung (Diagonal: Vorwärts + Links)...")
    drone.move(vector=Vector3.forward() + Vector3.left(), duration_s=1.0, speed=0.35)

    print("7. Schwebeflug vor Landung...")
    drone.hover(duration_s=1.5)

    print("8. Automatische Landung (Touchdown & Disarm)...")
    drone.land()
    print("\n[OK] Choreografie erfolgreich abgeschlossen!")


def run_interactive_keyboard(drone: DroneGameActor) -> None:
    """Startet den interaktiven WASD-Tastatur-Controller."""
    print("\n=========================================================")
    print("  INTERAKTIVE TASTATUR-STEUERUNG (GAME MODE)")
    print("=========================================================")
    print("  Tastenbelegung:")
    print("    [W / S]           : Vorwaerts / Rueckwaerts (Pitch)")
    print("    [A / D]           : Strafe Links / Rechts (Roll)")
    print("    [R]               : Steigen (Throttle Up)")
    print("    [F / C]           : Sinken (Throttle Down)")
    print("    [Q / E]           : Drehen Links / Rechts (Yaw)")
    print("    [X]               : Sofort-Schweben (Neutral 1500 PWM)")
    print("    [T]               : Starten (Takeoff auf 50 cm)")
    print("    [L]               : Landen")
    print("    [LEERTASTE / K / ESC] : 🚨 NOT-AUS (Sofort-Disarm & Motorstopp)")
    print("=========================================================")
    print("Druecke [T] zum Starten, [LEERTASTE] fuer NOT-AUS...\n")

    # Windows Tastenerkennung
    if sys.platform == "win32":
        import msvcrt

        try:
            while not _keyboard_stop_event.is_set():
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode("utf-8", errors="ignore").lower()
                    if key in (" ", "\x1b", "k"):  # Leertaste, ESC oder K
                        print("\n>>> NOT-AUS / BEENDEN GEDRUECKT (LEERTASTE/K/ESC)!")
                        drone.emergency_kill(reason="Tastatur-Not-Aus (Leertaste)")
                        break
                    elif key == "t":
                        print("\n>>> TAKEOFF...")
                        drone.takeoff(height_m=0.50)
                    elif key == "l":
                        print("\n>>> LANDUNG...")
                        drone.land()
                    elif key == "w":
                        drone.set_axis_input(forward=0.4)
                        print(" [VORWAERTS] ", end="\r", flush=True)
                    elif key == "s":
                        drone.set_axis_input(forward=-0.4)
                        print(" [RUECKWAERTS] ", end="\r", flush=True)
                    elif key == "a":
                        drone.set_axis_input(strafe=-0.4)
                        print(" [LINKS] ", end="\r", flush=True)
                    elif key == "d":
                        drone.set_axis_input(strafe=0.4)
                        print(" [RECHTS] ", end="\r", flush=True)
                    elif key == "r":
                        drone.set_axis_input(vertical=0.3)
                        print(" [STEIGEN] ", end="\r", flush=True)
                    elif key in ("f", "c", "y"):
                        drone.set_axis_input(vertical=-0.3)
                        print(" [SINKEN] ", end="\r", flush=True)
                    elif key == "q":
                        drone.set_axis_input(yaw=-0.4)
                        print(" [DREHEN LINKS] ", end="\r", flush=True)
                    elif key == "e":
                        drone.set_axis_input(yaw=0.4)
                        print(" [DREHEN RECHTS] ", end="\r", flush=True)
                    elif key == "x":
                        drone.hover()
                        print(" [SCHWEBEN/STOP] ", end="\r", flush=True)

                time.sleep(0.04)
        except Exception as e:
            print("Fehler in Tastatursteuerung:", e)
            drone.emergency_kill()

    else:
        # Linux / Pi cbreak Mode
        import select
        import termios
        import tty

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not _keyboard_stop_event.is_set():
                if select.select([sys.stdin], [], [], 0.04)[0]:
                    char = sys.stdin.read(1).lower()
                    if char in (" ", "\x1b", "k"):  # Leertaste, ESC oder K
                        print("\n>>> NOT-AUS / BEENDEN GEDRUECKT (LEERTASTE/K/ESC)!")
                        drone.emergency_kill(reason="Tastatur-Not-Aus (Leertaste)")
                        break
                    elif char == "t":
                        drone.takeoff(height_m=0.50)
                    elif char == "l":
                        drone.land()
                    elif char == "w":
                        drone.set_axis_input(forward=0.4)
                    elif char == "s":
                        drone.set_axis_input(forward=-0.4)
                    elif char == "a":
                        drone.set_axis_input(strafe=-0.4)
                    elif char == "d":
                        drone.set_axis_input(strafe=0.4)
                    elif char == "r":
                        drone.set_axis_input(vertical=0.3)
                    elif char in ("f", "c", "y"):
                        drone.set_axis_input(vertical=-0.3)
                    elif char == "q":
                        drone.set_axis_input(yaw=-0.4)
                    elif char == "e":
                        drone.set_axis_input(yaw=0.4)
                    elif char == "x":
                        drone.hover()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="GameMode Drohnensteuerung: Interaktives Demo & Choreografie."
    )
    parser.add_argument(
        "--demo",
        choices=["choreo", "keyboard"],
        default="choreo",
        help="Demomodus: 'choreo' (automatische Sequenz) oder 'keyboard' (WASD Steuerung)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="/dev/serial0",
        help="MAVLink-Schnittstelle (z. B. /dev/serial0 oder udp:127.0.0.1:14550)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baudrate (Standard: 115200)",
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=0.50,
        help="Ziel-Schwebhöhe in Metern (Standard: 0.50 m)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Führt die Demo als Mock/Simulation ohne echte MAVLink-Hardware aus.",
    )
    args = parser.parse_args()

    cfg = DEFAULT_FAILSAFE_CONFIG

    if args.dry_run:
        print(">>> Starte GameMode im DRY-RUN / SIMULATIONS-Modus...")
        drone = create_mock_actor(cfg)
    else:
        drone = DroneGameActor(
            device=args.device,
            baud=args.baud,
            mode="ALT_HOLD",
            failsafe_config=cfg,
            auto_connect=True,
        )

    with drone:
        if args.demo == "choreo":
            run_choreography(drone, target_alt=args.alt)
        else:
            run_interactive_keyboard(drone)


if __name__ == "__main__":
    main()
