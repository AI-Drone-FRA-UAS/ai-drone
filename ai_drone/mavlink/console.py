"""Launch the project's MAVProxy console with safe connection defaults."""

from __future__ import annotations

import argparse
import os
import shutil
from collections.abc import Sequence
from pathlib import Path
from typing import NoReturn

from ai_drone.mavlink.devices import (
    STABLE_FLIGHT_CONTROLLER_DEVICE,
    find_serial_device,
)

STABLE_DEVICE = STABLE_FLIGHT_CONTROLLER_DEVICE
DEFAULT_BAUD = 115200


def _find_device(requested: str | None) -> Path:
    try:
        return find_serial_device(
            requested,
            prefer_stable=True,
            include_pi_uart=False,
            stable_device=STABLE_DEVICE,
            missing_message=(
                "No ArduPilot USB serial device found. Connect the flight "
                "controller or pass --device /dev/..."
            ),
        )
    except FileNotFoundError as error:
        raise SystemExit(str(error)) from error


def _command(arguments: Sequence[str] | None = None) -> list[str]:
    parser = argparse.ArgumentParser(
        description="Open MAVProxy for the connected ArduPilot flight controller."
    )
    parser.add_argument(
        "--device",
        help="serial device (default: stable ArduPilot path or /dev/ttyACM*)",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args, mavproxy_arguments = parser.parse_known_args(arguments)

    if args.baud <= 0:
        parser.error("--baud must be greater than zero")

    executable = shutil.which("mavproxy.py")
    if executable is None:
        raise SystemExit("mavproxy.py is unavailable; run 'uv sync' first.")

    device = _find_device(args.device)
    return [
        executable,
        f"--master={device}",
        f"--baudrate={args.baud}",
        *mavproxy_arguments,
    ]


def main() -> NoReturn:
    command = _command()
    print(
        f"Opening MAVProxy on {command[1].removeprefix('--master=')} "
        f"at {command[2].removeprefix('--baudrate=')} baud...",
        flush=True,
    )
    os.execv(command[0], command)
