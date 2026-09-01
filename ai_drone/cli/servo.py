"""Manual control and calibration tool for 9g micro servo (e.g. SG90 / MS18-F)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

WIRING_DIAGRAM = """
[Raspberry Pi Zero 2 W Header (40 Pins)]
=========================================
  Pin 2 / Pin 4:  5V Power (Red Wire)   -------> [Servo VCC] (WARNING: External 5V recommended under load)
  Pin 6 / Pin 14: GND (Brown/Black)     -------> [Servo GND]
  Pin 12:         BCM GPIO 18 (Yellow/Orange) -> [Servo Signal] (Default in mission_drop.py)
  -- ALTERNATIV --
  Pin 32:         BCM GPIO 12 (Yellow/Orange) -> [Servo Signal]

WARNING ON POWER:
The SG90/MS18-F servo can draw 400mA-1600mA (stall current). Running it directly
from the Pi's 5V rail can cause brownouts / sudden reboots under mechanical resistance.
For payload drops or continuous movement, power the servo from an external 5V source
with grounds connected together (common GND).
"""

PIN_DESCRIPTIONS = {
    18: "Pin 12 (BCM 18, PWM0 - Default in mission_drop.py)",
    12: "Pin 32 (BCM 12, PWM0)",
    13: "Pin 33 (BCM 13, PWM1)",
    19: "Pin 35 (BCM 19, PWM1)",
}


def _is_raspberry_pi() -> bool:
    try:
        model = Path("/proc/device-tree/model").read_text()
    except (FileNotFoundError, PermissionError):
        return False
    return "raspberry pi" in model.lower()


class MockServo:
    """Mock servo implementation for simulation and off-Pi testing."""

    def __init__(
        self,
        pin: int,
        *,
        min_pulse_width: float = 0.0009,
        max_pulse_width: float = 0.0021,
    ) -> None:
        self.pin = pin
        self.min_pulse_width = min_pulse_width
        self.max_pulse_width = max_pulse_width
        self._value: float | None = 0.0
        self.is_closed = False

    @property
    def value(self) -> float | None:
        return self._value

    @value.setter
    def value(self, val: float | None) -> None:
        self._value = val

    def close(self) -> None:
        self.is_closed = True


def _pulse_to_value(
    pulse_us: int | float,
    *,
    min_us: int = 900,
    max_us: int = 2100,
) -> float:
    """Convert pulse width in microseconds to normalized float in [-1.0, 1.0]."""
    if pulse_us == 1500:
        return 0.0
    if pulse_us > 1500:
        if max_us == 1500:
            return 0.0
        return float((pulse_us - 1500) / (max_us - 1500))
    if min_us == 1500:
        return 0.0
    return float((pulse_us - 1500) / (1500 - min_us))


def _value_to_pulse(
    value: float,
    *,
    min_us: int = 900,
    max_us: int = 2100,
) -> int:
    """Convert normalized float in [-1.0, 1.0] to pulse width in microseconds."""
    if value >= 0:
        return int(round(1500 + value * (max_us - 1500)))
    return int(round(1500 + value * (1500 - min_us)))


def _pulse_us(value: float, *, min_us: int = 900, max_us: int = 2100) -> int:
    """Compatibility alias for _value_to_pulse."""
    return _value_to_pulse(value, min_us=min_us, max_us=max_us)


def _pulse_to_angle(
    pulse_us: int | float,
    *,
    min_us: int = 900,
    max_us: int = 2100,
) -> float:
    """Convert pulse width to degrees relative to center (nominally -60° to +60°)."""
    val = _pulse_to_value(pulse_us, min_us=min_us, max_us=max_us)
    return val * 60.0


def _target_value_from_input(
    value: str,
    *,
    min_us: int = 900,
    max_us: int = 2100,
    allow_extended: bool = False,
) -> float:
    """Parse a user input string into a normalized servo value."""
    command = value.strip().lower()

    if command.endswith("us"):
        pulse_us = int(command.removesuffix("us").strip())
        low_limit = 500 if allow_extended else min_us
        high_limit = 2500 if allow_extended else max_us
        if not low_limit <= pulse_us <= high_limit:
            raise ValueError(
                f"Pulsbreite muss zwischen {low_limit}µs und {high_limit}µs liegen (eingegeben: {pulse_us}µs)"
            )
        return _pulse_to_value(pulse_us, min_us=min_us, max_us=max_us)

    if command.endswith("deg") or command.endswith("°"):
        cleaned = command.removesuffix("deg").removesuffix("°").strip()
        degrees = float(cleaned)
        max_deg = 90.0 if allow_extended else 60.0
        if not -max_deg <= degrees <= max_deg:
            raise ValueError(
                f"Winkel muss zwischen {-max_deg}° und +{max_deg}° liegen (eingegeben: {degrees}°)"
            )
        return degrees / 60.0

    target = float(command)
    max_val = 1.66 if allow_extended else 1.0
    if not -max_val <= target <= max_val:
        raise ValueError(
            f"Wert muss zwischen {-max_val:.2f} und +{max_val:.2f} liegen (eingegeben: {target})"
        )
    return target


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Manuelle Steuerung und Kalibrierung des Servomotors (SG90 / MS18-F).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--pin",
        type=int,
        default=18,
        help="BCM GPIO Pin (Standard: 18 = Physischer Pin 12; alternativ z.B. 12 = Physischer Pin 32)",
    )
    parser.add_argument(
        "--min-us",
        type=int,
        default=900,
        help="Sichere minimale Pulsbreite in Mikrosekunden (Standard: 900)",
    )
    parser.add_argument(
        "--max-us",
        type=int,
        default=2100,
        help="Sichere maximale Pulsbreite in Mikrosekunden (Standard: 2100)",
    )
    parser.add_argument(
        "--mode",
        choices=["manual", "interactive", "sweep", "center"],
        default="manual",
        help="Betriebsmodus: manual/interactive (interaktive Kalibrierung), sweep (Schwenktest), center (Neutralstellung)",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=10,
        help="Schrittweite für Feintuning in Mikrosekunden (Standard: 10µs)",
    )
    parser.add_argument(
        "--sweep-delay",
        type=float,
        default=0.015,
        help="Verzögerung zwischen Sweep-Schritten in Sekunden",
    )
    parser.add_argument(
        "--sweep-step",
        type=float,
        default=0.02,
        help="Schrittgröße im Sweep-Modus (-1.0 bis 1.0)",
    )
    parser.add_argument(
        "--allow-extended",
        action="store_true",
        help="Erlaube erweiterten Bereich (500µs - 2500µs). VORSICHT: Kann Servo mechanisch beschädigen!",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Simulation / Mock-Modus erzwingen (nützlich für Tests auf dem Laptop ohne Pi)",
    )
    return parser


def _print_calibration_report(
    *,
    pin: int,
    min_us: int,
    max_us: int,
    calib_min_us: int | None,
    calib_max_us: int | None,
    calib_closed_us: int | None,
    calib_open_us: int | None,
) -> None:
    pin_desc = PIN_DESCRIPTIONS.get(pin, f"BCM GPIO {pin}")
    print("\n" + "=" * 64)
    print("           SERVO KALIBRIERUNGS-ERGEBNIS (ZUSAMMENFASSUNG)")
    print("=" * 64)
    print(f"GPIO Pin:              {pin_desc}")
    print(f"Konfigurierte Limits:  {min_us} µs bis {max_us} µs (Neutral: 1500 µs)")
    print("-" * 64)

    print("Ermittelte Positionen:")
    if calib_min_us is not None:
        min_val = _pulse_to_value(calib_min_us, min_us=min_us, max_us=max_us)
        min_deg = _pulse_to_angle(calib_min_us, min_us=min_us, max_us=max_us)
        print(f"  * Minimum (MIN):     {calib_min_us:4d} µs  (Wert: {min_val:+.2f}, {min_deg:+.1f}°)")
    else:
        print(f"  * Minimum (MIN):     [Nicht markiert - Standard: {min_us} µs / -1.00]")

    print("  * Neutral (CENTER):  1500 µs  (Wert:  0.00,   0.0°)")

    if calib_max_us is not None:
        max_val = _pulse_to_value(calib_max_us, min_us=min_us, max_us=max_us)
        max_deg = _pulse_to_angle(calib_max_us, min_us=min_us, max_us=max_us)
        print(f"  * Maximum (MAX):     {calib_max_us:4d} µs  (Wert: {max_val:+.2f}, {max_deg:+.1f}°)")
    else:
        print(f"  * Maximum (MAX):     [Nicht markiert - Standard: {max_us} µs / +1.00]")

    if calib_closed_us is not None:
        c_val = _pulse_to_value(calib_closed_us, min_us=min_us, max_us=max_us)
        print(f"  * Abwurf GESCHLOSSEN: {calib_closed_us:4d} µs  (Wert: {c_val:+.2f})")

    if calib_open_us is not None:
        o_val = _pulse_to_value(calib_open_us, min_us=min_us, max_us=max_us)
        print(f"  * Abwurf OFFEN:       {calib_open_us:4d} µs  (Wert: {o_val:+.2f})")

    print("-" * 64)
    print("Kopierbare Parameter für 'mission_drop.py':")
    closed_arg = (
        f"{_pulse_to_value(calib_closed_us, min_us=min_us, max_us=max_us):.2f}"
        if calib_closed_us is not None
        else "-0.50"
    )
    open_arg = (
        f"{_pulse_to_value(calib_open_us, min_us=min_us, max_us=max_us):.2f}"
        if calib_open_us is not None
        else "0.50"
    )
    print(f"  --servo-pin {pin} --servo-closed {closed_arg} --servo-open {open_arg}")
    print()
    print("Beispiel-Befehl für Testlauf:")
    print(f"  uv run ./mission_drop.py --dry-run --servo-pin {pin} --servo-closed {closed_arg} --servo-open {open_arg}")
    print("=" * 64 + "\n")


def _print_interactive_help() -> None:
    print(
        """
--------------------------------------------------------------------------------
VERFÜGBARE BEFEHLE:
  + / -          : Schritt um Feintuning-Schrittweite (Standard: 10µs)
  ++ / --        : Schritt um 50 µs
  +++ / ---      : Schritt um 100 µs
  +<N> / -<N>    : Schritt um N µs (z.B. +20, -30)
  <Zahl>         : Direktwert in µs (z.B. 1200, 1800) ODER float (-1.0 bis 1.0)
  <Winkel>deg    : Winkel in Grad (z.B. -45deg, 30deg, 0deg)
  center / c / 0 : Neutralstellung (1500 µs / 0.0)

POSITIONEN MARKIEREN & SPEICHERN:
  set min        : Aktuelle Position als MINIMUM speichern
  set max        : Aktuelle Position als MAXIMUM speichern
  set closed     : Aktuelle Position als GESCHLOSSEN (Abwurf) speichern
  set open       : Aktuelle Position als OFFEN (Abwurf) speichern

TESTEN & FUNKTIONEN:
  test           : Testfahrt zwischen Geschlossen/Offen bzw. Min/Max
  sweep          : Sanfter Schwenktest zwischen Min und Max
  min / max      : Zu gespeichertem Min / Max fahren
  closed / open  : Zu gespeichertem Geschlossen / Offen fahren
  release / off  : PWM-Signal abschalten (verhindert Brummen & Überhitzung)
  hold / on      : PWM-Signal wieder aktivieren
  step <N>       : Schrittweite auf N µs ändern (z.B. 'step 25')
  status / show  : Aktuelle Kalibrierungsdaten anzeigen
  help / ?       : Diese Hilfe anzeigen
  exit / q       : Programm beenden und Zusammenfassung ausgeben
--------------------------------------------------------------------------------
"""
    )


def _init_servo(pin: int, min_us: int, max_us: int, simulate: bool) -> tuple[Any, bool]:
    is_pi = _is_raspberry_pi()
    if simulate or not is_pi:
        if not is_pi:
            print("INFO: Kein Raspberry Pi erkannt -> Starte im Simulations-/Mock-Modus.")
            print("      (Für echte Hardware bitte direkt auf dem Pi oder via drone-deploy ausführen)")
        return MockServo(pin, min_pulse_width=min_us / 1e6, max_pulse_width=max_us / 1e6), True

    try:
        from gpiozero import Servo  # ty: ignore[unresolved-import]
    except ImportError:
        print("WARNUNG: gpiozero nicht gefunden. Installiere mit: sudo apt install python3-gpiozero", file=sys.stderr)
        print("         Wechsle in Simulations-Modus.", file=sys.stderr)
        return MockServo(pin, min_pulse_width=min_us / 1e6, max_pulse_width=max_us / 1e6), True

    servo = Servo(pin, min_pulse_width=min_us / 1e6, max_pulse_width=max_us / 1e6)
    return servo, False


def run_interactive(
    servo: Any,
    *,
    pin: int,
    min_us: int,
    max_us: int,
    step_us: int,
    allow_extended: bool,
    is_mock: bool,
) -> int:
    current_us = 1500
    current_step = step_us
    is_holding = True

    calib_min_us: int | None = None
    calib_max_us: int | None = None
    calib_closed_us: int | None = None
    calib_open_us: int | None = None

    low_bound = 500 if allow_extended else min_us
    high_bound = 2500 if allow_extended else max_us

    # Initial move to center
    servo.value = 0.0

    pin_desc = PIN_DESCRIPTIONS.get(pin, f"BCM GPIO {pin}")
    print("\n" + "=" * 70)
    print("          AI-DRONE SERVO MANUELLE STEUERUNG & KALIBRIERUNG")
    print("=" * 70)
    print(f"Hardware-Pin:      {pin_desc}")
    print(f"Sicherer Bereich:  {min_us} µs bis {max_us} µs (Center: 1500 µs)")
    if allow_extended:
        print("HINWEIS:           ERWEITERTER BEREICH AKTIV (500µs - 2500µs)!")
    if is_mock:
        print("MODUS:             SIMULATION / MOCK (Keine echte Hardware angeschlossen)")
    print(f"Standard-Schritt:  {current_step} µs")
    print("Tipp:              Tippe '?' oder 'help' für Befehle, 'exit' zum Beenden.")
    print("=" * 70 + "\n")

    while True:
        val = _pulse_to_value(current_us, min_us=min_us, max_us=max_us)
        deg = _pulse_to_angle(current_us, min_us=min_us, max_us=max_us)
        state_str = "HOLD" if is_holding else "IDLE/OFF"

        prompt = f"servo [{current_us:4d}µs | {val:+.2f} | {deg:+5.1f}° | {state_str}]> "

        try:
            raw = input(prompt).strip()
        except (KeyboardInterrupt, EOFError):
            print("\nAbbruch durch Benutzer.")
            break

        if not raw:
            continue

        cmd = raw.lower()

        if cmd in {"exit", "quit", "q", "bye"}:
            break

        if cmd in {"?", "help", "hilfe"}:
            _print_interactive_help()
            continue

        if cmd in {"status", "show", "report", "info"}:
            _print_calibration_report(
                pin=pin,
                min_us=min_us,
                max_us=max_us,
                calib_min_us=calib_min_us,
                calib_max_us=calib_max_us,
                calib_closed_us=calib_closed_us,
                calib_open_us=calib_open_us,
            )
            continue

        if cmd.startswith("step"):
            parts = cmd.split()
            if len(parts) > 1:
                try:
                    new_step = int(parts[1].removesuffix("us"))
                    if new_step <= 0:
                        raise ValueError
                    current_step = new_step
                    print(f"-> Schrittweite auf {current_step} µs gesetzt.")
                except ValueError:
                    print("Fehler: Ungültige Schrittweite. Beispiel: 'step 25'")
            else:
                print(f"Aktuelle Schrittweite: {current_step} µs")
            continue

        # Calibration marking commands
        if cmd in {"set min", "mark min", "save min"}:
            calib_min_us = current_us
            print(f"✓ MINIMUM markiert bei: {current_us} µs (Wert: {val:+.2f}, Winkel: {deg:+.1f}°)")
            continue

        if cmd in {"set max", "mark max", "save max"}:
            calib_max_us = current_us
            print(f"✓ MAXIMUM markiert bei: {current_us} µs (Wert: {val:+.2f}, Winkel: {deg:+.1f}°)")
            continue

        if cmd in {"set closed", "mark closed", "save closed"}:
            calib_closed_us = current_us
            print(f"✓ GESCHLOSSEN (Drop Closed) markiert bei: {current_us} µs (Wert: {val:+.2f})")
            continue

        if cmd in {"set open", "mark open", "save open"}:
            calib_open_us = current_us
            print(f"✓ OFFEN (Drop Open) markiert bei: {current_us} µs (Wert: {val:+.2f})")
            continue

        # Signal hold / release
        if cmd in {"release", "off", "idle", "relax", "aus"}:
            servo.value = None
            is_holding = False
            print("-> PWM-Signal abgeschaltet (Servo stromlos / entlastet).")
            continue

        if cmd in {"hold", "on", "ein", "an"}:
            servo.value = val
            is_holding = True
            print(f"-> PWM-Signal aktiviert (Halte {current_us} µs).")
            continue

        # Direct navigation presets
        if cmd in {"c", "center", "mitte", "0"}:
            target_us = 1500
        elif cmd == "min":
            target_us = calib_min_us if calib_min_us is not None else min_us
            print(f"-> Fahre zu MIN ({target_us} µs)...")
        elif cmd == "max":
            target_us = calib_max_us if calib_max_us is not None else max_us
            print(f"-> Fahre zu MAX ({target_us} µs)...")
        elif cmd == "closed":
            if calib_closed_us is None:
                print("Hinweis: Position 'closed' noch nicht markiert (nutze 'set closed'). Verwende -0.50 (1200µs).")
                target_us = 1200
            else:
                target_us = calib_closed_us
                print(f"-> Fahre zu GESCHLOSSEN ({target_us} µs)...")
        elif cmd == "open":
            if calib_open_us is None:
                print("Hinweis: Position 'open' noch nicht markiert (nutze 'set open'). Verwende +0.50 (1800µs).")
                target_us = 1800
            else:
                target_us = calib_open_us
                print(f"-> Fahre zu OFFEN ({target_us} µs)...")
        elif cmd in {"test", "test-drop"}:
            # Test drop movement
            p_closed = calib_closed_us if calib_closed_us is not None else _value_to_pulse(-0.5, min_us=min_us, max_us=max_us)
            p_open = calib_open_us if calib_open_us is not None else _value_to_pulse(0.5, min_us=min_us, max_us=max_us)
            print(f"\n[TESTLAUF] 1. Fahre auf GESCHLOSSEN ({p_closed} µs)...")
            servo.value = _pulse_to_value(p_closed, min_us=min_us, max_us=max_us)
            time.sleep(1.2)
            print(f"[TESTLAUF] 2. Fahre auf OFFEN / ABWURF ({p_open} µs)...")
            servo.value = _pulse_to_value(p_open, min_us=min_us, max_us=max_us)
            time.sleep(1.2)
            print(f"[TESTLAUF] 3. Zurück auf GESCHLOSSEN ({p_closed} µs)...")
            servo.value = _pulse_to_value(p_closed, min_us=min_us, max_us=max_us)
            time.sleep(0.8)
            current_us = p_closed
            is_holding = True
            print("[TESTLAUF] Testlauf abgeschlossen!\n")
            continue
        elif cmd == "sweep":
            s_min = calib_min_us if calib_min_us is not None else min_us
            s_max = calib_max_us if calib_max_us is not None else max_us
            print(f"\n[SWEEP] Schwenke zwischen {s_min} µs und {s_max} µs (2 Zyklen)...")
            for _ in range(2):
                # Min to Max
                for u in range(s_min, s_max + 1, 20):
                    servo.value = _pulse_to_value(u, min_us=min_us, max_us=max_us)
                    time.sleep(0.015)
                time.sleep(0.3)
                # Max to Min
                for u in range(s_max, s_min - 1, -20):
                    servo.value = _pulse_to_value(u, min_us=min_us, max_us=max_us)
                    time.sleep(0.015)
                time.sleep(0.3)
            # Return to center
            servo.value = 0.0
            current_us = 1500
            is_holding = True
            print("[SWEEP] Schwenk beendet. Zurück auf Neutral (1500 µs).\n")
            continue
        # Relative step inputs
        elif cmd == "+":
            target_us = current_us + current_step
        elif cmd == "-":
            target_us = current_us - current_step
        elif cmd == "++":
            target_us = current_us + 50
        elif cmd == "--":
            target_us = current_us - 50
        elif cmd == "+++":
            target_us = current_us + 100
        elif cmd == "---":
            target_us = current_us - 100
        elif (cmd.startswith("+") or cmd.startswith("-")) and len(cmd) > 1:
            try:
                num_str = cmd[1:].removesuffix("us")
                if "." in num_str:
                    rel_val = float(cmd)
                    delta_us = int(rel_val * (max_us - 1500))
                    target_us = current_us + delta_us
                else:
                    delta_us = int(cmd.removesuffix("us"))
                    target_us = current_us + delta_us
            except ValueError:
                print(f"Fehler: Unbekannter relativer Befehl '{raw}'")
                continue
        # Direct inputs (numeric, us, deg)
        else:
            try:
                # If raw integer like '1200'
                if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
                    num = int(raw)
                    if 400 <= num <= 2600:
                        target_us = num
                    else:
                        target_val = float(raw)
                        target_us = _value_to_pulse(target_val, min_us=min_us, max_us=max_us)
                else:
                    target_val = _target_value_from_input(
                        raw,
                        min_us=min_us,
                        max_us=max_us,
                        allow_extended=allow_extended,
                    )
                    target_us = _value_to_pulse(target_val, min_us=min_us, max_us=max_us)
            except ValueError as err:
                print(f"Eingabefehler: {err}")
                print("Tippe '?' für Hilfe.")
                continue

        # Check limits
        if target_us < low_bound:
            print(f"WARNUNG: Untergrenze erreicht ({low_bound} µs)!")
            target_us = low_bound
        elif target_us > high_bound:
            print(f"WARNUNG: Obergrenze erreicht ({high_bound} µs)!")
            target_us = high_bound

        current_us = target_us
        target_val = _pulse_to_value(current_us, min_us=min_us, max_us=max_us)
        servo.value = target_val
        is_holding = True

    # Output final summary report upon exiting
    _print_calibration_report(
        pin=pin,
        min_us=min_us,
        max_us=max_us,
        calib_min_us=calib_min_us,
        calib_max_us=calib_max_us,
        calib_closed_us=calib_closed_us,
        calib_open_us=calib_open_us,
    )
    return 0


def main() -> int:
    args = _parser().parse_args()

    servo, is_mock = _init_servo(
        pin=args.pin,
        min_us=args.min_us,
        max_us=args.max_us,
        simulate=args.simulate,
    )

    pin_desc = PIN_DESCRIPTIONS.get(args.pin, f"BCM GPIO {args.pin}")
    print(f"Aktiver Pin: {pin_desc}")
    print(f"Puls-Bereich: {args.min_us}µs bis {args.max_us}µs")
    print(f"Modus: {args.mode.upper()}")
    print(WIRING_DIAGRAM)

    try:
        if args.mode == "center":
            print("Bewege Servo in Neutralstellung (1500 µs). Drücke Strg+C zum Beenden.")
            servo.value = 0.0
            while True:
                time.sleep(1)

        elif args.mode == "sweep":
            print(f"Schwenke Servo zwischen {args.min_us}µs und {args.max_us}µs. Drücke Strg+C zum Stoppen.")
            value = 0.0
            direction = 1
            while True:
                servo.value = value
                pulse_us = _value_to_pulse(value, min_us=args.min_us, max_us=args.max_us)
                deg = _pulse_to_angle(pulse_us, min_us=args.min_us, max_us=args.max_us)
                sys.stdout.write(f"\rPosition: {value:+6.2f} | Puls: {pulse_us:4d}µs | Winkel: {deg:+5.1f}°   ")
                sys.stdout.flush()

                value += direction * args.sweep_step
                if value >= 1.0:
                    value = 1.0
                    direction = -1
                elif value <= -1.0:
                    value = -1.0
                    direction = 1
                time.sleep(args.sweep_delay)

        else:
            # manual or interactive mode
            run_interactive(
                servo,
                pin=args.pin,
                min_us=args.min_us,
                max_us=args.max_us,
                step_us=args.step,
                allow_extended=args.allow_extended,
                is_mock=is_mock,
            )

    except KeyboardInterrupt:
        print("\nAbgebrochen.")
    finally:
        try:
            servo.value = None
            servo.close()
        except Exception:
            pass
        print("Servo-Signal abgeschaltet und GPIO freigegeben.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
