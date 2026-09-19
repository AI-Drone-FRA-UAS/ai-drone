"""Control a 9g micro servo on Raspberry Pi GPIO 12."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any

from ai_drone.mount import (
    ABSOLUTE_MAX_PULSE_US,
    ABSOLUTE_MIN_PULSE_US,
    DEFAULT_MAX_PULSE_US,
    DEFAULT_MIN_PULSE_US,
    SERVO_GPIO_PIN,
    ServoProcessLock,
    create_servo,
    parse_servo_input,
    pulse_us,
)
from ai_drone.platform import is_raspberry_pi

ACTUATION_CONFIRMATION = "SERVO_CLEAR"

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
        default=DEFAULT_MIN_PULSE_US,
        help="Minimum pulse width in microseconds (default: 900)",
    )
    parser.add_argument(
        "--max-us",
        type=int,
        default=DEFAULT_MAX_PULSE_US,
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


def _center(servo: Any, args: argparse.Namespace) -> None:
    midpoint = pulse_us(0.0, min_us=args.min_us, max_us=args.max_us)
    print(f"Moving to the configured midpoint ({midpoint}us). Press Ctrl-C to release.")
    servo.value = 0.0
    while True:
        time.sleep(1)


def _sweep(servo: Any, args: argparse.Namespace) -> None:
    print("Sweeping servo between min and max. Press Ctrl-C to stop.")
    value = 0.0
    direction = 1
    while True:
        servo.value = value
        width_us = pulse_us(value, min_us=args.min_us, max_us=args.max_us)
        sys.stdout.write(f"\rPosition: {value:6.2f} | Pulse: {width_us:4d}us")
        sys.stdout.flush()

        value += direction * args.sweep_step
        if value >= 1.0:
            value = 1.0
            direction = -1
        elif value <= -1.0:
            value = -1.0
            direction = 1
        time.sleep(args.sweep_delay)


def _manual(servo: Any, args: argparse.Namespace) -> None:
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
            value = parse_servo_input(
                raw_value,
                min_us=args.min_us,
                max_us=args.max_us,
            )
        except ValueError as error:
            print(f"Error: {error}")
            continue

        width_us = pulse_us(value, min_us=args.min_us, max_us=args.max_us)
        print(f"Moving to value={value:.2f}, pulse={width_us}us")
        servo.value = value


def main(arguments: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(arguments)
    _validate_args(parser, args)

    if not is_raspberry_pi():
        print("ERROR: This command must be run on a Raspberry Pi.", file=sys.stderr)
        print(
            "Run it via 'uv run python scripts/servo.py --mode <mode> "
            f"--confirm-actuation {ACTUATION_CONFIRMATION}' from the laptop.",
            file=sys.stderr,
        )
        print(f"Wiring details for reference:\n{WIRING_DIAGRAM}", file=sys.stderr)
        return 1

    process_lock = None
    try:
        process_lock = ServoProcessLock()
        servo = create_servo(args.pin, min_us=args.min_us, max_us=args.max_us)
    except BaseException as error:
        if process_lock is not None:
            process_lock.close()
        if isinstance(error, KeyboardInterrupt):
            return 130
        if not isinstance(error, Exception):
            raise
        print(
            f"ERROR: Failed to initialize Servo on GPIO {args.pin}: {error}",
            file=sys.stderr,
        )
        return 1

    try:
        print(
            f"Active pin: BCM GPIO {args.pin} (physical pin 32)\n"
            f"Pulse range: {args.min_us}us to {args.max_us}us\n"
            f"Mode: {args.mode.upper()}\n"
            f"{WIRING_DIAGRAM}"
        )

        {"center": _center, "sweep": _sweep, "manual": _manual}[args.mode](servo, args)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            servo.close()
        finally:
            process_lock.close()
        print("Servo released.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
