"""Connect to the Pi over Tailscale, its Wi-Fi hotspot, or USB."""

from __future__ import annotations

import argparse
import platform
import shlex
import subprocess
from collections.abc import Mapping, Sequence

from ai_drone.link import usb_ssh, wifi
from ai_drone.link.targets import (
    DEFAULT_DIRECT_SSH_CONFIG,
    ConnectionTarget,
    ping_command,
    resolve_connection_target,
    ssh_base_command,
    wait_for_ping,
)

# The AP is either up (answers immediately) or absent — do not wait 3 minutes.
AP_PING_TIMEOUT = 15
TAILSCALE_CONNECT_TIMEOUT = 5


def _print_command(command: Sequence[str]) -> None:
    print(f"  {shlex.join(command)}", flush=True)


def interactive_ssh_command(
    target: str,
    ssh_config: str | None = DEFAULT_DIRECT_SSH_CONFIG,
) -> list[str]:
    return [*ssh_base_command(ssh_config), "-t", target]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Connect to the Pi (Tailscale -> Pi Wi-Fi AP -> USB cable)."
    )
    parser.add_argument(
        "--transport",
        choices=("auto", "tailscale", "hotspot", "usb"),
        default="auto",
        help="connection to use; auto tries Tailscale, hotspot, then USB",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the commands each transport would run without connecting",
    )
    return parser


# --- Transport 1: Tailscale ---------------------------------------------------


def tailscale_probe_command(
    ssh_target: str,
    ssh_config: str | None = DEFAULT_DIRECT_SSH_CONFIG,
) -> list[str]:
    """A non-interactive reachability check that also proves key auth works."""

    return [
        *ssh_base_command(ssh_config),
        "-o",
        "BatchMode=yes",
        "-o",
        f"ConnectTimeout={TAILSCALE_CONNECT_TIMEOUT}",
        "-o",
        "StrictHostKeyChecking=accept-new",
        ssh_target,
        "true",
    ]


def connect_tailscale(
    ssh_target: str,
    *,
    ssh_config: str | None = DEFAULT_DIRECT_SSH_CONFIG,
    dry_run: bool,
) -> bool:
    probe = tailscale_probe_command(ssh_target, ssh_config)
    interactive = interactive_ssh_command(ssh_target, ssh_config)
    if dry_run:
        print("1. Tailscale:", flush=True)
        _print_command(probe)
        _print_command(interactive)
        return False

    print(f"Trying Tailscale ({ssh_target})...", flush=True)
    if subprocess.run(probe, check=False).returncode != 0:
        print("  Pi is not reachable over Tailscale.", flush=True)
        return False
    print("  Reachable. Opening a shell.", flush=True)
    subprocess.run(interactive, check=True)
    return True


# --- Transport 2: the Pi's own Wi-Fi AP ---------------------------------------


def _restore_wifi(previous: str | None, system: str, device: str | None) -> None:
    if not previous:
        return
    print(f"  Restoring previous Wi-Fi network ({previous})...", flush=True)
    subprocess.run(wifi.join_command(previous, system, device), check=False)


def connect_wifi_ap(target: ConnectionTarget, system: str, *, dry_run: bool) -> bool:
    ssid = target.ap_ssid
    ap_target = f"{target.pi_user}@{target.ap_ip}"
    device = None if dry_run else wifi.wifi_device(system)
    join = wifi.join_command(ssid, system, device)
    interactive = interactive_ssh_command(ap_target, target.ssh_config)

    if dry_run:
        print("2. Pi Wi-Fi AP:", flush=True)
        _print_command(join)
        _print_command(ping_command(target.ap_ip, system))
        _print_command(interactive)
        return False

    print(f"Trying the Pi's Wi-Fi AP ({ssid})...", flush=True)
    if not wifi.ap_available(ssid, system):
        print(f"  {ssid} is not broadcasting.", flush=True)
        return False

    previous = wifi.current_ssid(system, device)
    print(f"  Joining {ssid}...", flush=True)
    if subprocess.run(join, check=False).returncode != 0:
        print(
            f"  Could not join {ssid}. If it has never been saved on this "
            f"machine, run once:\n"
            f"    nmcli dev wifi connect {ssid} password <PSK>",
            flush=True,
        )
        _restore_wifi(previous, system, device)
        return False

    if not wait_for_ping(target.ap_ip, AP_PING_TIMEOUT, system):
        print(f"  Joined {ssid} but {target.ap_ip} did not answer.", flush=True)
        _restore_wifi(previous, system, device)
        return False

    print(f"  Pi answers at {target.ap_ip}. Opening a shell.", flush=True)
    try:
        subprocess.run(interactive, check=True)
    finally:
        _restore_wifi(previous, system, device)
    return True


# --- Transport 3: USB cable ---------------------------------------------------


def connect_usb(
    *, environ: Mapping[str, str] | None = None, dry_run: bool = False
) -> bool:
    argv: list[str] = []
    if dry_run:
        argv.append("--dry-run")
    return usb_ssh.run(argv, environ=environ) == 0


# --- Orchestration ------------------------------------------------------------


def _tailscale_target(target: ConnectionTarget) -> str:
    return f"{target.pi_user}@{target.pi_hostname}"


def run(
    arguments: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
) -> int:
    args = _parser().parse_args(arguments)
    system = platform.system()
    target = resolve_connection_target(environ)
    tailscale_target = _tailscale_target(target)

    if args.dry_run and args.transport == "auto":
        print("drone-connect would try, in order:", flush=True)
        connect_tailscale(
            tailscale_target,
            ssh_config=target.ssh_config,
            dry_run=True,
        )
        connect_wifi_ap(target, system, dry_run=True)
        print("3. USB cable:", flush=True)
        usb_ssh.run(["--dry-run"], environ=environ)
        return 0

    if args.transport in {"auto", "tailscale"} and connect_tailscale(
        tailscale_target,
        ssh_config=target.ssh_config,
        dry_run=args.dry_run,
    ):
        return 0
    if args.transport == "tailscale":
        return 0 if args.dry_run else 1
    if args.transport in {"auto", "hotspot"} and connect_wifi_ap(
        target, system, dry_run=args.dry_run
    ):
        return 0
    if args.transport == "hotspot":
        return 0 if args.dry_run else 1
    if args.transport == "usb":
        return 0 if connect_usb(environ=environ, dry_run=args.dry_run) else 1
    print("Falling back to the USB cable...", flush=True)
    return 0 if connect_usb(environ=environ) else 1


def main() -> None:
    raise SystemExit(run())
