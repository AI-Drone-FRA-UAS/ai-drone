#!/usr/bin/env python3
"""Test script for a 9g micro servo (e.g., SG90 or MS18-F) on a Raspberry Pi.

Allows controlling the servo connected to BCM GPIO 12 (physical pin 32).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Setup wiring and safety instructions
WIRING_DIAGRAM = """
[Raspberry Pi Zero 2 WH Header (40 Pins)]
=========================================
  Pin 2:  5V Power (Red Wire) -------> [Servo VCC] (WARNING: see below)
  Pin 6:  GND (Brown Wire) ---------> [Servo GND]
  Pin 32: BCM GPIO 12 (Yellow/Orange) -> [Servo Signal]

WARNING ON POWER:
The SG90 servo can draw 400mA-1600mA. Running it directly from the Raspberry Pi's
5V pin can cause voltage drops and sudden Pi reboots/brownouts under load.
For light testing without load, the Pi's 5V pin is usually fine, but for any load,
use an external 5V power supply and connect its ground to the Pi's ground (GND).
"""


def _is_raspberry_pi() -> bool:
    try:
        model = Path("/proc/device-tree/model").read_text()
    except (FileNotFoundError, PermissionError):
        return False
    return "raspberry pi" in model.lower()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Control a 9g micro servo motor using BCM GPIO 12 on Raspberry Pi."
    )
    parser.add_argument(
        "--pin",
        type=int,
        default=12,
        help="BCM GPIO pin number (default: 12, physical pin 32)",
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
        default="sweep",
        help="Command mode: sweep (continuous), manual (interactive CLI), center (hold neutral)",
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
    args = parser.parse_args()

    if not _is_raspberry_pi():
        print("ERROR: This script must be run on a Raspberry Pi.", file=sys.stderr)
        print(
            "Please copy this script to the Pi or run it via the deploy script.",
            file=sys.stderr,
        )
        print(f"Wiring details for reference:\n{WIRING_DIAGRAM}", file=sys.stderr)
        return 1

    # Import gpiozero here so it only fails if run on non-Pi platforms
    try:
        from gpiozero import Servo  # ty: ignore[unresolved-import]
    except ImportError:
        print("ERROR: gpiozero is not installed on this Pi.", file=sys.stderr)
        print(
            "Please install it using: sudo apt install python3-gpiozero",
            file=sys.stderr,
        )
        print(
            "Or if using a virtual environment: uv pip install gpiozero",
            file=sys.stderr,
        )
        return 1

    # Try importing rich for colored formatting, fallback if not present
    try:
        from rich.console import Console  # ty: ignore[unresolved-import]
        from rich.panel import Panel  # ty: ignore[unresolved-import]

        console = Console()
        has_rich = True
    except ImportError:
        console = None
        has_rich = False

    # Convert microseconds to seconds for gpiozero
    min_pw = args.min_us / 1_000_000.0
    max_pw = args.max_us / 1_000_000.0

    try:
        servo = Servo(args.pin, min_pulse_width=min_pw, max_pulse_width=max_pw)
    except Exception as e:
        print(
            f"ERROR: Failed to initialize Servo on GPIO {args.pin}: {e}",
            file=sys.stderr,
        )
        return 1

    def print_msg(msg: str, is_header: bool = False, style: str = ""):
        if has_rich and console:
            if is_header:
                console.print(
                    Panel(msg, title="Servo Control System", border_style="cyan")
                )
            else:
                console.print(msg, style=style)
        else:
            if is_header:
                print("=" * 60)
                print(msg)
                print("=" * 60)
            else:
                print(msg)

    header_text = (
        f"Active pin: BCM GPIO {args.pin} (Physical Pin 32)\n"
        f"Pulse range: {args.min_us}µs to {args.max_us}µs (Neutral: 1500µs)\n"
        f"Mode: {args.mode.upper()}\n"
        f"{WIRING_DIAGRAM}"
    )
    print_msg(header_text, is_header=True)

    try:
        if args.mode == "center":
            print_msg("Moving servo to center position (0.0 / 1500µs)...")
            servo.value = 0.0
            print_msg(
                "Holding position. Press Ctrl-C to release servo and exit.",
                style="bold yellow",
            )
            while True:
                time.sleep(1)

        elif args.mode == "sweep":
            print_msg(
                "Starting sweep mode. Sweeping servo between min (-1.0) and max (1.0)."
            )
            print_msg("Press Ctrl-C to stop.", style="bold yellow")

            val = 0.0
            direction = 1

            while True:
                servo.value = val
                pos_percent = int((val + 1.0) * 50)
                bar = "#" * (pos_percent // 4) + "-" * (25 - (pos_percent // 4))

                if val >= 0:
                    us = int(1500 + val * (args.max_us - 1500))
                else:
                    us = int(1500 + val * (1500 - args.min_us))

                status_line = f"\rPosition: {val:6.2f} | Pulse: {us:4d}µs | [{bar}]"
                sys.stdout.write(status_line)
                sys.stdout.flush()

                val += direction * args.sweep_step
                if val >= 1.0:
                    val = 1.0
                    direction = -1
                elif val <= -1.0:
                    val = -1.0
                    direction = 1

                time.sleep(args.sweep_delay)

        elif args.mode == "manual":
            print_msg("Manual / Interactive Mode.", style="bold green")
            print_msg("You can enter values in three formats:")
            print_msg("  1. Value: float between -1.0 (min) and 1.0 (max)")
            print_msg(
                "  2. Angle: integer followed by 'deg', e.g., '30deg' or '-45deg' (maps -60 to +60 degrees to -1.0 to 1.0)"
            )
            print_msg(
                "  3. Pulse: integer followed by 'us', e.g., '1200us' or '1800us' (maps min_us to max_us)"
            )
            print_msg("Type 'exit', 'quit', or press Ctrl-C to quit.\n")

            while True:
                try:
                    inp = (
                        input(
                            "Enter command (e.g., '0.0', '1500us', '-30deg', 'exit'): "
                        )
                        .strip()
                        .lower()
                    )
                except (KeyboardInterrupt, EOFError):
                    print()
                    break

                if inp in ("exit", "quit", "q"):
                    break
                if not inp:
                    continue

                target_value = None
                try:
                    if inp.endswith("us"):
                        us_val = int(inp.replace("us", "").strip())
                        if us_val < args.min_us or us_val > args.max_us:
                            print_msg(
                                f"Error: Pulse width must be between {args.min_us} and {args.max_us} microseconds.",
                                style="red",
                            )
                            continue
                        if us_val == 1500:
                            target_value = 0.0
                        elif us_val > 1500:
                            target_value = (us_val - 1500) / (args.max_us - 1500)
                        else:
                            target_value = (us_val - 1500) / (1500 - args.min_us)
                    elif inp.endswith("deg"):
                        deg_val = float(inp.replace("deg", "").strip())
                        if deg_val < -60.0 or deg_val > 60.0:
                            print_msg(
                                "Error: Angle must be between -60 and +60 degrees.",
                                style="red",
                            )
                            continue
                        target_value = deg_val / 60.0
                    else:
                        val = float(inp)
                        if val < -1.0 or val > 1.0:
                            print_msg(
                                "Error: Value must be between -1.0 and 1.0.",
                                style="red",
                            )
                            continue
                        target_value = val
                except ValueError:
                    print_msg(
                        "Error: Could not parse input. Try '0.5', '1800us', or '-20deg'.",
                        style="red",
                    )
                    continue

                if target_value is not None:
                    if target_value >= 0:
                        us = int(1500 + target_value * (args.max_us - 1500))
                    else:
                        us = int(1500 + target_value * (1500 - args.min_us))
                    deg = target_value * 60.0

                    print_msg(
                        f"Moving to: value={target_value:.2f} | angle={deg:+.1f}° | pulse={us}µs",
                        style="green",
                    )
                    servo.value = target_value

    except KeyboardInterrupt:
        print_msg("\nInterrupted by user.", style="yellow")
    finally:
        print_msg("Releasing servo pin...")
        servo.close()
        print_msg("Done. Goodbye!", style="bold green")

    return 0


if __name__ == "__main__":
    sys.exit(main())
