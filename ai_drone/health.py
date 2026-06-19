"""Verify the developer USB and Raspberry Pi UART MAVLink connections."""

from __future__ import annotations

import argparse
import base64
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.console import DEFAULT_BAUD, _find_device

DEFAULT_PI_HOST = "seb@seb-is-pm"
DEFAULT_PI_DEVICE = "/dev/serial0"
DEFAULT_TIMEOUT = 10.0

_REMOTE_CHECK = """
from pymavlink import mavutil
import sys

device = sys.argv[1]
baud = int(sys.argv[2])
timeout = float(sys.argv[3])

connection = mavutil.mavlink_connection(
    device,
    baud=baud,
    source_system=254,
    source_component=191,
)
heartbeat = connection.wait_heartbeat(timeout=timeout)
if heartbeat is None:
    connection.close()
    print(f"FAIL Pi UART: no heartbeat on {device}", flush=True)
    raise SystemExit(2)

armed = bool(
    heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
)
connection.mav.param_request_read_send(
    connection.target_system,
    connection.target_component,
    b"SYSID_THISMAV",
    -1,
)

parameter = None
while True:
    message = connection.recv_match(
        type="PARAM_VALUE",
        blocking=True,
        timeout=timeout,
    )
    if message is None:
        break
    if message.param_id == "SYSID_THISMAV":
        parameter = message
        break

connection.close()
if parameter is None:
    print(
        f"FAIL Pi UART: heartbeat received but SYSID_THISMAV did not respond",
        flush=True,
    )
    raise SystemExit(3)

print(
    "PASS Pi UART: "
    f"device={device} "
    f"system={connection.target_system} "
    f"component={connection.target_component} "
    f"SYSID_THISMAV={parameter.param_value:g} "
    f"armed={armed}",
    flush=True,
)
"""


@dataclass(frozen=True)
class LinkResult:
    label: str
    device: str
    system: int
    component: int
    system_id: int
    armed: bool


def check_local_link(
    device: Path,
    baud: int = DEFAULT_BAUD,
    timeout: float = DEFAULT_TIMEOUT,
) -> LinkResult:
    """Require a heartbeat and parameter response over one local serial link."""
    connection = mavutil.mavlink_connection(
        str(device),
        baud=baud,
        source_system=255,
        source_component=190,
    )
    try:
        heartbeat = connection.wait_heartbeat(timeout=timeout)
        if heartbeat is None:
            raise RuntimeError(f"no heartbeat on {device}")

        armed = bool(heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        connection.mav.param_request_read_send(
            connection.target_system,
            connection.target_component,
            b"SYSID_THISMAV",
            -1,
        )

        while True:
            message = connection.recv_match(
                type="PARAM_VALUE",
                blocking=True,
                timeout=timeout,
            )
            if message is None:
                raise RuntimeError(
                    "heartbeat received but SYSID_THISMAV did not respond"
                )
            if message.param_id == "SYSID_THISMAV":
                return LinkResult(
                    label="Developer USB",
                    device=str(device),
                    system=connection.target_system,
                    component=connection.target_component,
                    system_id=int(message.param_value),
                    armed=armed,
                )
    finally:
        connection.close()


def _remote_command(device: str, baud: int, timeout: float) -> str:
    encoded = base64.b64encode(_REMOTE_CHECK.encode()).decode()
    return (
        ".venv/bin/python -c "
        f"'import base64;exec(base64.b64decode(\"{encoded}\"))' "
        f"{device} {baud} {timeout}"
    )


def check_pi_link(
    host: str,
    device: str = DEFAULT_PI_DEVICE,
    baud: int = DEFAULT_BAUD,
    timeout: float = DEFAULT_TIMEOUT,
) -> bool:
    """Run the same read-only round-trip check through the Pi over SSH."""
    command = _remote_command(device, baud, timeout)
    completed = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=8", host, f"cd ~/ai-drone && {command}"],
        check=False,
    )
    return completed.returncode == 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Check MAVLink through developer USB and Raspberry Pi /dev/serial0."
        )
    )
    parser.add_argument("--usb-device", help="developer-machine FC serial device")
    parser.add_argument("--pi-host", default=DEFAULT_PI_HOST)
    parser.add_argument("--pi-device", default=DEFAULT_PI_DEVICE)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument("--usb-only", action="store_true")
    parser.add_argument("--pi-only", action="store_true")
    return parser


def run(arguments: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(arguments)
    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")
    if args.usb_only and args.pi_only:
        parser.error("--usb-only and --pi-only cannot be used together")

    checks_passed = True
    if not args.pi_only:
        try:
            device = _find_device(args.usb_device)
            result = check_local_link(device, args.baud, args.timeout)
        except (OSError, RuntimeError) as error:
            print(f"FAIL Developer USB: {error}", flush=True)
            checks_passed = False
        else:
            print(
                "PASS Developer USB: "
                f"device={result.device} "
                f"system={result.system} "
                f"component={result.component} "
                f"SYSID_THISMAV={result.system_id} "
                f"armed={result.armed}",
                flush=True,
            )

    if not args.usb_only:
        if not check_pi_link(args.pi_host, args.pi_device, args.baud, args.timeout):
            checks_passed = False

    if checks_passed:
        print("PASS All requested MAVLink connections are working.", flush=True)
        return 0

    print("FAIL One or more MAVLink connections failed.", flush=True)
    return 1


def main() -> None:
    raise SystemExit(run())
