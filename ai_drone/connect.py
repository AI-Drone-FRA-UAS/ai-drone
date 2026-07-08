"""Connect to the Pi over Tailscale, its own Wi-Fi AP, or a USB cable.

Two entry points share the same three transports and priority order:

* ``autoconnect`` (:func:`run_auto`) tries them in order — Tailscale, then the
  Pi's ``AI-Drone-Zero`` AP, then the USB gadget — and stops at the first that
  connects. This mirrors how the Pi is expected to come up: it prefers a known
  Wi-Fi (so it is on Tailscale), self-hosts its AP only when offline, and the
  cable is the always-there last resort.
* ``manuconnect`` (:func:`run_manual`) shows the same three as a menu and runs
  the one the user picks.

This module owns transport selection. The Wi-Fi command builders live in
``pi_wifi`` and the USB link setup lives in ``pi_usb_ssh``.
"""

from __future__ import annotations

import argparse
import platform
import shlex
import subprocess
import time
from collections.abc import Mapping, Sequence

from ai_drone import pi_usb_ssh, pi_wifi
from ai_drone.pi_targets import (
    DEFAULT_PI_HOTSPOT_IP,
    ConnectionTarget,
    ping_command,
    resolve_connection_target,
)

# The AP is either up (answers immediately) or absent — do not wait 3 minutes.
AP_PING_TIMEOUT = 15
TAILSCALE_CONNECT_TIMEOUT = 5


def _print_command(command: Sequence[str]) -> None:
    print(f"  {shlex.join(command)}", flush=True)


def interactive_ssh_command(target: str) -> list[str]:
    return ["ssh", "-t", target]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Connect to the Pi (Tailscale -> Pi Wi-Fi AP -> USB cable)."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the commands each transport would run without connecting",
    )
    return parser


# --- Transport 1: Tailscale ---------------------------------------------------


def tailscale_probe_command(ssh_target: str) -> list[str]:
    """A non-interactive reachability check that also proves key auth works."""

    return [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        f"ConnectTimeout={TAILSCALE_CONNECT_TIMEOUT}",
        "-o",
        "StrictHostKeyChecking=accept-new",
        ssh_target,
        "true",
    ]


def connect_tailscale(ssh_target: str, *, dry_run: bool) -> bool:
    probe = tailscale_probe_command(ssh_target)
    interactive = interactive_ssh_command(ssh_target)
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
    subprocess.run(pi_wifi.join_command(previous, system, device), check=False)


def wait_for_ping(
    host: str,
    timeout_seconds: int,
    system: str,
    *,
    dry_run: bool,
) -> bool:
    command = ping_command(host, system)
    if dry_run:
        _print_command(command)
        return True

    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        completed = subprocess.run(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if completed.returncode == 0:
            return True
        time.sleep(3)
    return False


def connect_wifi_ap(target: ConnectionTarget, system: str, *, dry_run: bool) -> bool:
    ssid = target.ap_ssid
    ap_target = f"{target.pi_user}@{target.ap_ip}"
    device = None if dry_run else pi_wifi.wifi_device(system)
    join = pi_wifi.join_command(ssid, system, device)
    interactive = interactive_ssh_command(ap_target)

    if dry_run:
        print("2. Pi Wi-Fi AP:", flush=True)
        _print_command(join)
        _print_command(ping_command(target.ap_ip, system))
        _print_command(interactive)
        return False

    print(f"Trying the Pi's Wi-Fi AP ({ssid})...", flush=True)
    if not pi_wifi.ap_available(ssid, system):
        print(f"  {ssid} is not broadcasting.", flush=True)
        return False

    previous = pi_wifi.current_ssid(system, device)
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

    if not wait_for_ping(target.ap_ip, AP_PING_TIMEOUT, system, dry_run=False):
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
    argv = ["--usb-only"]
    if dry_run:
        argv.append("--dry-run")
    return pi_usb_ssh.run(argv, environ=environ) == 0


# --- Orchestration ------------------------------------------------------------


def _tailscale_target(target: ConnectionTarget) -> str:
    return f"{target.pi_user}@{target.pi_hostname}"


def _host_from_ssh_target(target: str) -> str:
    return target.rsplit("@", 1)[-1]


def network_ssh_targets(
    pi_user: str,
    pi_hostname: str,
    pi_usb_ip: str,
    explicit_targets: Sequence[str] | None = None,
) -> list[str]:
    candidates = [
        *(explicit_targets or ()),
        f"{pi_user}@{pi_hostname}.local",
        f"{pi_user}@{pi_hostname}",
        f"{pi_user}@{DEFAULT_PI_HOTSPOT_IP}",
        f"{pi_user}@{pi_usb_ip}",
    ]

    unique: list[str] = []
    for candidate in candidates:
        if candidate and candidate not in unique:
            unique.append(candidate)
    return unique


def _host_responds(host: str, system: str) -> bool:
    completed = subprocess.run(
        ping_command(host, system),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return completed.returncode == 0


def find_reachable_network_target(
    targets: Sequence[str],
    system: str,
    *,
    timeout_seconds: int = 0,
) -> str | None:
    deadline = time.monotonic() + timeout_seconds
    while True:
        for target in targets:
            if _host_responds(_host_from_ssh_target(target), system):
                return target
        if timeout_seconds <= 0 or time.monotonic() >= deadline:
            return None
        time.sleep(2)


def print_network_targets(targets: Sequence[str], system: str) -> None:
    print("Would try existing Wi-Fi/hotspot/network SSH targets first:", flush=True)
    for target in targets:
        _print_command(ping_command(_host_from_ssh_target(target), system))
        _print_command(interactive_ssh_command(target))


def try_network_ssh(
    targets: Sequence[str],
    system: str,
    *,
    timeout_seconds: int = 0,
    dry_run: bool,
) -> bool:
    if dry_run:
        print_network_targets(targets, system)
        return False

    print("Checking existing Wi-Fi/hotspot/network SSH targets...", flush=True)
    target = find_reachable_network_target(
        targets,
        system,
        timeout_seconds=timeout_seconds,
    )
    if target is None:
        return False

    print(f"Pi is reachable at {_host_from_ssh_target(target)}.", flush=True)
    command = interactive_ssh_command(target)
    _print_command(command)
    subprocess.run(command, check=True)
    return True


def run_legacy_usb_entrypoint(
    args: argparse.Namespace,
    defaults: ConnectionTarget,
    system: str,
) -> int:
    targets = network_ssh_targets(
        args.pi_user,
        args.pi_hostname,
        args.pi_ip,
        args.ssh_target,
    )

    connected = try_network_ssh(
        targets,
        system,
        timeout_seconds=args.timeout if args.network_only else 0,
        dry_run=args.dry_run,
    )
    if connected:
        return 0
    if args.network_only:
        if args.dry_run:
            return 0
        print("No existing Wi-Fi/hotspot/network SSH target responded.", flush=True)
        return 1
    if not args.dry_run:
        print("No existing network target responded; falling back to USB.", flush=True)

    return pi_usb_ssh.run_usb_transport(args, defaults, system)


def run_auto(
    arguments: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
) -> int:
    args = _parser().parse_args(arguments)
    system = platform.system()
    target = resolve_connection_target(environ)
    tailscale_target = _tailscale_target(target)

    if args.dry_run:
        print("autoconnect would try, in order:", flush=True)
        connect_tailscale(tailscale_target, dry_run=True)
        connect_wifi_ap(target, system, dry_run=True)
        print("3. USB cable:", flush=True)
        pi_usb_ssh.run(["--usb-only", "--dry-run"], environ=environ)
        return 0

    if connect_tailscale(tailscale_target, dry_run=False):
        return 0
    if connect_wifi_ap(target, system, dry_run=False):
        return 0
    print("Falling back to the USB cable...", flush=True)
    return 0 if connect_usb(environ=environ) else 1


def _prompt_choice(prompt: str = "Enter 1, 2, or 3: ") -> str:
    while True:
        choice = input(prompt).strip()
        if choice in {"1", "2", "3"}:
            return choice
        print("Please enter 1, 2, or 3.", flush=True)


def run_manual(
    arguments: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
) -> int:
    args = _parser().parse_args(arguments)
    system = platform.system()
    target = resolve_connection_target(environ)
    tailscale_target = _tailscale_target(target)

    print("How do you want to connect to the Pi?", flush=True)
    print(f"  1) Tailscale     (ssh {tailscale_target})", flush=True)
    print(f"  2) Pi Wi-Fi AP   ({target.ap_ssid} -> {target.ap_ip})", flush=True)
    print("  3) USB cable", flush=True)
    choice = _prompt_choice()

    if choice == "1":
        return 0 if connect_tailscale(tailscale_target, dry_run=args.dry_run) else 1
    if choice == "2":
        return 0 if connect_wifi_ap(target, system, dry_run=args.dry_run) else 1
    return 0 if connect_usb(environ=environ, dry_run=args.dry_run) else 1


def auto_main() -> None:
    raise SystemExit(run_auto())


def manual_main() -> None:
    raise SystemExit(run_manual())
