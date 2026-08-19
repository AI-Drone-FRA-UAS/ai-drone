"""Verify the developer USB and Raspberry Pi UART MAVLink connections."""

from __future__ import annotations

import argparse
import math
import os
import shlex
import subprocess
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from pymavlink import mavutil

from ai_drone.console import DEFAULT_BAUD
from ai_drone.mavlink_devices import find_serial_device
from ai_drone.mavlink_safety import (
    heartbeat_is_armed,
    is_vehicle_message,
    require_fresh_disarmed_heartbeat,
)
from ai_drone.pi_targets import (
    DEFAULT_DIRECT_SSH_CONFIG,
    DEFAULT_PI_HOSTNAME,
    DEFAULT_PI_USERNAME,
    resolve_connection_target,
    split_ssh_target,
    ssh_base_command,
)

DEFAULT_PI_HOST = f"{DEFAULT_PI_USERNAME}@{DEFAULT_PI_HOSTNAME}"
DEFAULT_PI_DEVICE = "/dev/serial0"
DEFAULT_TIMEOUT = 10.0


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
    *,
    label: str = "Developer USB",
) -> LinkResult:
    """Require a heartbeat and parameter response over one local serial link."""
    if baud <= 0:
        raise ValueError("baud must be greater than zero")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and greater than zero")
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
        if heartbeat_is_armed(heartbeat):
            raise RuntimeError(f"vehicle is ARMED on {device}")

        target_system = int(connection.target_system)
        target_component = int(connection.target_component)
        connection.mav.param_request_read_send(
            target_system,
            target_component,
            b"SYSID_THISMAV",
            -1,
        )

        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            message = connection.recv_match(
                type=["PARAM_VALUE", "HEARTBEAT"],
                blocking=True,
                timeout=remaining,
            )
            if message is None:
                break
            if not is_vehicle_message(
                message, system_id=target_system, component_id=target_component
            ):
                continue
            if message.get_type() == "HEARTBEAT":
                if heartbeat_is_armed(message):
                    raise RuntimeError(f"vehicle became ARMED on {device}")
                continue
            if message.param_id == "SYSID_THISMAV":
                final_timeout = deadline - time.monotonic()
                if final_timeout <= 0:
                    raise TimeoutError("no time remained for a final heartbeat")
                require_fresh_disarmed_heartbeat(
                    connection,
                    system_id=target_system,
                    component_id=target_component,
                    timeout=final_timeout,
                )
                return LinkResult(
                    label=label,
                    device=str(device),
                    system=target_system,
                    component=target_component,
                    system_id=int(message.param_value),
                    armed=False,
                )
        raise RuntimeError("heartbeat received but SYSID_THISMAV did not respond")
    finally:
        connection.close()


def _remote_command(device: str, baud: int, timeout: float) -> str:
    return shlex.join(
        [
            ".venv/bin/python",
            "-m",
            "ai_drone.health",
            "--usb-only",
            "--usb-device",
            device,
            "--baud",
            str(baud),
            "--timeout",
            str(timeout),
            "--local-label",
            "Pi UART",
        ]
    )


def check_pi_link(
    host: str,
    device: str = DEFAULT_PI_DEVICE,
    baud: int = DEFAULT_BAUD,
    timeout: float = DEFAULT_TIMEOUT,
    *,
    ssh_config: str | None = DEFAULT_DIRECT_SSH_CONFIG,
) -> bool:
    """Run the same read-only round-trip check through the Pi over SSH."""
    command = _remote_command(device, baud, timeout)
    completed = subprocess.run(
        [
            *ssh_base_command(ssh_config),
            "-o",
            "ConnectTimeout=8",
            host,
            f"cd ~/ai-drone && {command}",
        ],
        check=False,
    )
    return completed.returncode == 0


def _parser(
    *,
    ssh_config: str | None,
    pi_host: str = DEFAULT_PI_HOST,
) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Check MAVLink through developer USB and Raspberry Pi /dev/serial0."
        )
    )
    parser.add_argument("--usb-device", help="developer-machine FC serial device")
    parser.add_argument("--pi-host", default=pi_host)
    parser.add_argument("--pi-device", default=DEFAULT_PI_DEVICE)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument("--ssh-config", default=ssh_config)
    parser.add_argument("--usb-only", action="store_true")
    parser.add_argument("--pi-only", action="store_true")
    parser.add_argument(
        "--local-label", default="Developer USB", help=argparse.SUPPRESS
    )
    return parser


def run(
    arguments: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
) -> int:
    values = os.environ if environ is None else environ
    connection_target = resolve_connection_target(values)
    explicit_host = values.get("PI_HOST")
    if explicit_host:
        _, pi_user, pi_address = split_ssh_target(
            explicit_host,
            connection_target.pi_user,
        )
        default_pi_host = f"{pi_user}@{pi_address}"
    else:
        default_pi_host = f"{connection_target.pi_user}@{connection_target.pi_hostname}"
    parser = _parser(
        ssh_config=connection_target.ssh_config,
        pi_host=default_pi_host,
    )
    args = parser.parse_args(arguments)
    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and greater than zero")
    if args.usb_only and args.pi_only:
        parser.error("--usb-only and --pi-only cannot be used together")

    checks_passed = True
    if not args.pi_only:
        try:
            device = find_serial_device(
                args.usb_device,
                prefer_stable=True,
                include_pi_uart=False,
                missing_message=(
                    "No ArduPilot USB serial device found. Connect the flight "
                    "controller or pass --usb-device /dev/..."
                ),
            )
            result = check_local_link(
                device,
                args.baud,
                args.timeout,
                label=args.local_label,
            )
        except (OSError, RuntimeError, FileNotFoundError) as error:
            print(f"FAIL Developer USB: {error}", flush=True)
            checks_passed = False
        else:
            print(
                f"PASS {result.label}: "
                f"device={result.device} "
                f"system={result.system} "
                f"component={result.component} "
                f"SYSID_THISMAV={result.system_id} "
                f"armed={result.armed}",
                flush=True,
            )

    if not args.usb_only and not check_pi_link(
        args.pi_host,
        args.pi_device,
        args.baud,
        args.timeout,
        ssh_config=args.ssh_config,
    ):
        checks_passed = False

    if checks_passed:
        print("PASS All requested MAVLink connections are working.", flush=True)
        return 0

    print("FAIL One or more MAVLink connections failed.", flush=True)
    return 1


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
