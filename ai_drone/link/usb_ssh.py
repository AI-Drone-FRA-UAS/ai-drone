"""Configure the Pi USB gadget network link and open SSH."""

from __future__ import annotations

import argparse
import ipaddress
import platform
import re
import shlex
import subprocess
from collections.abc import Mapping, Sequence
from pathlib import Path

from ai_drone.pi_targets import (
    DEFAULT_DIRECT_SSH_CONFIG,
    ConnectionTarget,
    ping_command,
    resolve_connection_target,
    ssh_base_command,
    wait_for_ping,
)

MTU = "1412"
_SAFE_INTERFACE = re.compile(r"[^\x00-\x1f\x7f]{1,128}\Z")


def _parser(target: ConnectionTarget) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Configure the Pi USB Ethernet link and open SSH."
    )
    parser.add_argument("--pi-ip", default=target.pi_ip)
    parser.add_argument("--host-ip", default=target.host_ip)
    parser.add_argument("--pi-user", default=target.pi_user)
    parser.add_argument(
        "--usb-iface",
        default=target.usb_iface,
        help=(
            "explicit USB gadget interface to configure; required for live setup "
            "to avoid modifying an unrelated USB network adapter"
        ),
    )
    parser.add_argument("--timeout", type=int, default=target.timeout_seconds)
    parser.add_argument("--ssh-config", default=target.ssh_config)
    parser.add_argument("--dry-run", action="store_true")
    return parser


def linux_config_commands(iface: str, host_ip: str) -> list[list[str]]:
    return [
        ["sudo", "ip", "link", "set", iface, "up"],
        ["sudo", "ip", "link", "set", "dev", iface, "mtu", MTU],
        ["sudo", "ip", "addr", "add", f"{host_ip}/24", "dev", iface],
    ]


def darwin_config_commands(iface: str, host_ip: str) -> list[list[str]]:
    return [
        ["sudo", "ifconfig", iface, "up"],
        ["sudo", "ifconfig", iface, "mtu", MTU],
        ["sudo", "ifconfig", iface, host_ip, "netmask", "255.255.255.0"],
    ]


def _ps_quote(value: str) -> str:
    return "'" + value.replace("'", "''") + "'"


def powershell_command(script: str) -> list[str]:
    return [
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-Command",
        script,
    ]


def windows_config_commands(iface: str, host_ip: str) -> list[list[str]]:
    quoted_iface = _ps_quote(iface)
    quoted_ip = _ps_quote(host_ip)
    script = (
        f"$alias = {quoted_iface}; "
        f"$ip = {quoted_ip}; "
        "Set-NetIPInterface -InterfaceAlias $alias -Dhcp Disabled "
        "-ErrorAction SilentlyContinue; "
        "$current = Get-NetIPAddress -InterfaceAlias $alias "
        "-AddressFamily IPv4 -ErrorAction SilentlyContinue | "
        "Where-Object { $_.IPAddress -eq $ip }; "
        "if (-not $current) { "
        "Get-NetIPAddress -InterfaceAlias $alias -AddressFamily IPv4 "
        "-ErrorAction SilentlyContinue | "
        "Where-Object { $_.PrefixOrigin -ne 'WellKnown' } | "
        "Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue; "
        "New-NetIPAddress -InterfaceAlias $alias -IPAddress $ip "
        "-PrefixLength 24 -ErrorAction Stop | Out-Null "
        "}"
    )
    return [
        powershell_command(script),
        [
            "netsh",
            "interface",
            "ipv4",
            "set",
            "subinterface",
            iface,
            f"mtu={MTU}",
            "store=active",
        ],
    ]


def windows_find_script() -> str:
    return (
        "$pattern = 'RNDIS|Remote NDIS|USB Ethernet|Ethernet Gadget|CDC'; "
        "$matches = Get-NetAdapter | "
        "Where-Object { "
        "$_.InterfaceDescription -match $pattern -or $_.Name -match $pattern "
        "}; "
        "$up = $matches | Where-Object { $_.Status -eq 'Up' } | "
        "Select-Object -First 1; "
        "if ($up) { $up.Name; exit 0 }; "
        "$fallback = $matches | Where-Object { $_.Status -ne 'Disabled' } | "
        "Select-Object -First 1; "
        "if ($fallback) { $fallback.Name }"
    )


def windows_find_command() -> list[str]:
    return powershell_command(windows_find_script())


def is_linux_usb_netdev(iface: str) -> bool:
    if iface == "lo":
        return False
    if iface.startswith(("wlan", "wl", "docker", "virbr", "veth")):
        return False
    if iface.startswith("br-"):
        return False

    device = Path("/sys/class/net") / iface / "device"
    try:
        dev_path = device.resolve(strict=True)
    except FileNotFoundError:
        return False
    return "/usb" in str(dev_path)


def find_linux_usb_iface() -> str | None:
    for path in Path("/sys/class/net").glob("*"):
        if is_linux_usb_netdev(path.name):
            return path.name
    return None


def find_darwin_usb_iface() -> str | None:
    completed = subprocess.run(
        ["ifconfig", "-u"],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        return None

    for line in completed.stdout.splitlines():
        if not line or line[0].isspace() or ":" not in line:
            continue
        iface = line.split(":", 1)[0]
        if iface == "lo0" or iface.startswith(
            ("bridge", "awdl", "llw", "utun", "ap", "anpi")
        ):
            continue
        if iface == "en0":
            continue
        details = subprocess.run(
            ["ifconfig", iface],
            check=False,
            capture_output=True,
            text=True,
        )
        if "status: inactive" not in details.stdout:
            return iface
    return None


def find_windows_usb_iface() -> str | None:
    completed = subprocess.run(
        windows_find_command(),
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        return None
    iface = completed.stdout.strip()
    return iface or None


def find_usb_iface(system: str) -> str | None:
    if system == "Darwin":
        return find_darwin_usb_iface()
    if system == "Windows":
        return find_windows_usb_iface()
    return find_linux_usb_iface()


def _run_capture(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, check=False, capture_output=True, text=True)


def iface_exists(iface: str, system: str) -> bool:
    if system == "Darwin":
        return _run_capture(["ifconfig", iface]).returncode == 0
    if system == "Windows":
        script = f"Get-NetAdapter -Name {_ps_quote(iface)} -ErrorAction Stop"
        return _run_capture(powershell_command(script)).returncode == 0
    return (Path("/sys/class/net") / iface).is_dir()


def has_host_ip(iface: str, host_ip: str, system: str) -> bool:
    if system == "Darwin":
        completed = _run_capture(["ifconfig", iface])
        return host_ip in completed.stdout
    if system == "Windows":
        script = (
            f"Get-NetIPAddress -InterfaceAlias {_ps_quote(iface)} "
            f"-IPAddress {_ps_quote(host_ip)} "
            "-AddressFamily IPv4 -ErrorAction SilentlyContinue"
        )
        completed = _run_capture(powershell_command(script))
        return completed.returncode == 0 and bool(completed.stdout.strip())

    completed = _run_capture(["ip", "-4", "addr", "show", "dev", iface])
    return f"{host_ip}/24" in completed.stdout


def config_commands(iface: str, host_ip: str, system: str) -> list[list[str]]:
    if system == "Darwin":
        return darwin_config_commands(iface, host_ip)
    if system == "Windows":
        return windows_config_commands(iface, host_ip)
    return linux_config_commands(iface, host_ip)


def ssh_command(
    pi_user: str,
    pi_ip: str,
    ssh_config: str | None = DEFAULT_DIRECT_SSH_CONFIG,
) -> list[str]:
    remote = (
        "sudo ip link set dev usb0 mtu 1412 2>/dev/null || "
        "sudo ifconfig usb0 mtu 1412 2>/dev/null || true; "
        "exec $SHELL --login"
    )
    return [*ssh_base_command(ssh_config), "-t", f"{pi_user}@{pi_ip}", remote]


def _print_command(command: Sequence[str]) -> None:
    print(f"  {shlex.join(command)}", flush=True)


def _run(command: Sequence[str], *, dry_run: bool) -> None:
    _print_command(command)
    if not dry_run:
        subprocess.run(command, check=True)


def run_usb_transport(
    args: argparse.Namespace,
    defaults: ConnectionTarget,
    system: str,
) -> int:
    print("Waiting for Pi USB Ethernet interface...", flush=True)
    print(f"Target: {args.pi_user}@{args.pi_ip}", flush=True)
    print(
        f"Use a data-capable USB cable from the laptop to the {defaults.port_hint}.",
        flush=True,
    )

    if not args.usb_iface:
        candidate = None if args.dry_run else find_usb_iface(system)
        print(
            "Refusing to reconfigure an auto-detected network adapter. Pass "
            "--usb-iface after verifying the Pi USB gadget interface.",
            flush=True,
        )
        if candidate:
            print(
                f"Unverified candidate: {candidate}. Inspect it, then rerun with "
                f"--usb-iface {shlex.quote(candidate)}.",
                flush=True,
            )
        return 1
    if not args.dry_run and not iface_exists(args.usb_iface, system):
        print(f"USB interface does not exist: {args.usb_iface}", flush=True)
        return 1
    iface = args.usb_iface

    print(f"Found USB network interface: {iface}", flush=True)
    print(f"Configuring laptop side as {args.host_ip}/24...", flush=True)
    if args.dry_run or not has_host_ip(iface, args.host_ip, system):
        for command in config_commands(iface, args.host_ip, system):
            _run(command, dry_run=args.dry_run)

    print(f"Waiting for Pi at {args.pi_ip}...", flush=True)
    reachable = True
    if args.dry_run:
        _print_command(ping_command(args.pi_ip, system))
    else:
        reachable = wait_for_ping(args.pi_ip, args.timeout, system)
    if not reachable:
        print(f"Timed out waiting for {args.pi_ip}.", flush=True)
        print(f"Try: ping {args.pi_ip}", flush=True)
        retry = ssh_command(args.pi_user, args.pi_ip, args.ssh_config)
        print(f"Try: {shlex.join(retry[:-1])}", flush=True)
        return 1

    print("Pi answers ping.", flush=True)
    print(
        "Connecting with SSH. Use the Pi password you set while flashing.", flush=True
    )
    _run(
        ssh_command(args.pi_user, args.pi_ip, args.ssh_config),
        dry_run=args.dry_run,
    )
    return 0


def run(
    arguments: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
) -> int:
    defaults = resolve_connection_target(environ)
    parser = _parser(defaults)
    args = parser.parse_args(arguments)
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")
    try:
        pi_ip = ipaddress.IPv4Address(args.pi_ip)
        host_ip = ipaddress.IPv4Address(args.host_ip)
    except ipaddress.AddressValueError as error:
        parser.error(f"--pi-ip and --host-ip must be valid IPv4 addresses: {error}")
    if pi_ip == host_ip:
        parser.error("--pi-ip and --host-ip must be distinct")
    if not pi_ip.is_private or not host_ip.is_private:
        parser.error("--pi-ip and --host-ip must be private addresses")
    if ipaddress.IPv4Network(f"{pi_ip}/24", strict=False) != ipaddress.IPv4Network(
        f"{host_ip}/24", strict=False
    ):
        parser.error("--pi-ip and --host-ip must be in the same /24 subnet")
    if args.usb_iface is not None and (
        _SAFE_INTERFACE.fullmatch(args.usb_iface) is None
        or args.usb_iface.startswith("-")
    ):
        parser.error("--usb-iface contains unsafe characters or is too long")
    return run_usb_transport(args, defaults, platform.system())


def main() -> None:
    raise SystemExit(run())
