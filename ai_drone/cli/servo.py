"""Control a 9g micro servo on Raspberry Pi GPIO 12."""

from __future__ import annotations

import argparse
import math
import sys
import time

from ai_drone.platform import is_raspberry_pi

ACTUATION_CONFIRMATION = "SERVO_CLEAR"
SERVO_GPIO_PIN = 12
ABSOLUTE_MIN_PULSE_US = 750
ABSOLUTE_MAX_PULSE_US = 2250

WIRING_DIAGRAM = """
[Raspberry Pi Zero 2 WH Header (40 Pins)]
=========================================
  Regulated external 5V -------------> [Servo VCC]
  Supply GND + Pi Pin 6 (GND) -------> [Servo GND / common ground]
  Pin 32: BCM GPIO 12 (Yellow/Orange) -> [Servo Signal]

WARNING ON POWER:
The SG90 servo can draw 400mA-1600mA. Running it directly from the Raspberry Pi's
5V pin can cause voltage drops and sudden Pi reboots/brownouts under load.
Use a suitably rated regulated 5V supply and connect its ground to Pi ground.
Do not use the Pi header as servo power unless the complete shared 5V power
budget, wiring, protection, and transient behavior have first been validated.
"""


def _target_value_from_input(
    value: str,
    *,
    min_us: int,
    max_us: int,
) -> float:
    command = value.strip().lower()
    if command.endswith("us"):
        pulse_us = int(command.removesuffix("us").strip())
        if not min_us <= pulse_us <= max_us:
            raise ValueError(f"pulse width must be between {min_us} and {max_us}us")
        if pulse_us == 1500:
            return 0.0
        if pulse_us > 1500:
            return (pulse_us - 1500) / (max_us - 1500)
        return (pulse_us - 1500) / (1500 - min_us)

    if command.endswith("deg"):
        degrees = float(command.removesuffix("deg").strip())
        if not math.isfinite(degrees) or not -60.0 <= degrees <= 60.0:
            raise ValueError("angle must be between -60 and +60 degrees")
        return degrees / 60.0

    target = float(command)
    if not math.isfinite(target) or not -1.0 <= target <= 1.0:
        raise ValueError("value must be between -1.0 and 1.0")
    return target


def _pulse_us(value: float, *, min_us: int, max_us: int) -> int:
    if value >= 0:
        return int(1500 + value * (max_us - 1500))
    return int(1500 + value * (1500 - min_us))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control a 9g micro servo motor using BCM GPIO 12 on Raspberry Pi."
    )
    parser.add_argument(
        "--pin",
        type=int,
        choices=[SERVO_GPIO_PIN],
        default=SERVO_GPIO_PIN,
        help="fixed BCM GPIO pin (12, physical pin 32)",
    )
    parser.add_argument(
        "--min-us",
        type=int,
        default=900,
        help="Minimum pulse width in microseconds (default: 900)",
    )
    parser.add_argument(
        "--max-us",
        type=int,
        default=2100,
        help="Maximum pulse width in microseconds (default: 2100)",
    )
    parser.add_argument(
        "--mode",
        choices=["sweep", "manual", "center"],
        required=True,
        help="Command mode: sweep (continuous), manual (interactive CLI), center (hold neutral)",
    )
    parser.add_argument(
        "--confirm-actuation",
        choices=[ACTUATION_CONFIRMATION],
        required=True,
        help=(f"Required physical-safety acknowledgement: {ACTUATION_CONFIRMATION}"),
    )
    parser.add_argument(
        "--sweep-delay",
        type=float,
        default=0.01,
        help="Delay in seconds between sweep steps (default: 0.01)",
    )
    parser.add_argument(
        "--sweep-step",
        type=float,
        default=0.02,
        help="Step size for sweeping between -1.0 and 1.0 (default: 0.02)",
    )
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if not args.min_us < 1500 < args.max_us:
        parser.error("pulse geometry must satisfy --min-us < 1500 < --max-us")
    if args.min_us < ABSOLUTE_MIN_PULSE_US:
        parser.error(
            f"--min-us must be at least the absolute software limit "
            f"{ABSOLUTE_MIN_PULSE_US}"
        )
    if args.max_us > ABSOLUTE_MAX_PULSE_US:
        parser.error(
            f"--max-us must be at most the absolute software limit "
            f"{ABSOLUTE_MAX_PULSE_US}"
        )
    if not math.isfinite(args.sweep_delay) or args.sweep_delay < 0:
        parser.error("--sweep-delay must be a finite, non-negative number")
    if not math.isfinite(args.sweep_step) or not 0 < args.sweep_step <= 1:
        parser.error("--sweep-step must be finite and greater than 0 and at most 1")


def main(arguments: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(arguments)
    _validate_args(parser, args)

    if not is_raspberry_pi():
        print("ERROR: This command must be run on a Raspberry Pi.", file=sys.stderr)
        print(
            "Run it via 'uv run drone-deploy --servo --mode <mode> "
            f"--confirm-actuation {ACTUATION_CONFIRMATION}' from the laptop.",
            file=sys.stderr,
        )
        print(f"Wiring details for reference:\n{WIRING_DIAGRAM}", file=sys.stderr)
        return 1

    try:
        from gpiozero import Servo  # ty: ignore[unresolved-import]
    except ImportError:
        print("ERROR: gpiozero is not installed on this Pi.", file=sys.stderr)
        print("Install it with: sudo apt install python3-gpiozero", file=sys.stderr)
        return 1

    try:
        servo = Servo(
            args.pin,
            min_pulse_width=args.min_us / 1_000_000.0,
            max_pulse_width=args.max_us / 1_000_000.0,
            initial_value=None,
        )
    except Exception as error:
        print(
            f"ERROR: Failed to initialize Servo on GPIO {args.pin}: {error}",
            file=sys.stderr,
        )
        return 1

    print(
        f"Active pin: BCM GPIO {args.pin} (physical pin 32)\n"
        f"Pulse range: {args.min_us}us to {args.max_us}us\n"
        f"Mode: {args.mode.upper()}\n"
        f"{WIRING_DIAGRAM}"
    )

    try:
        if args.mode == "center":
            print("Moving servo to center position. Press Ctrl-C to release.")
            servo.value = 0.0
            while True:
                time.sleep(1)

        if args.mode == "sweep":
            print("Sweeping servo between min and max. Press Ctrl-C to stop.")
            value = 0.0
            direction = 1
            while True:
                servo.value = value
                pulse_us = _pulse_us(value, min_us=args.min_us, max_us=args.max_us)
                sys.stdout.write(f"\rPosition: {value:6.2f} | Pulse: {pulse_us:4d}us")
                sys.stdout.flush()

                value += direction * args.sweep_step
                if value >= 1.0:
                    value = 1.0
                    direction = -1
                elif value <= -1.0:
                    value = -1.0
                    direction = 1
                time.sleep(args.sweep_delay)

        if args.mode == "manual":
            print("Enter values like 0.0, 1500us, -30deg, or exit.")
            while True:
                try:
                    raw_value = input("servo> ")
                except (KeyboardInterrupt, EOFError):
                    print()
                    break
                if raw_value.strip().lower() in {"exit", "quit", "q"}:
                    break
                try:
                    value = _target_value_from_input(
                        raw_value,
                        min_us=args.min_us,
                        max_us=args.max_us,
                    )
                except ValueError as error:
                    print(f"Error: {error}")
                    continue

                pulse_us = _pulse_us(value, min_us=args.min_us, max_us=args.max_us)
                print(f"Moving to value={value:.2f}, pulse={pulse_us}us")
                servo.value = value
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        servo.close()
        print("Servo released.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
