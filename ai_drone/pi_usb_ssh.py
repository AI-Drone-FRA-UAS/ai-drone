"""Configure the Pi USB gadget network link and open SSH."""

from __future__ import annotations

import argparse
import platform
import shlex
import subprocess
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

from ai_drone.pi_targets import (
    DEFAULT_PI_HOTSPOT_IP,
    UsbTarget,
    ping_command,
    resolve_usb_target,
)

MTU = "1412"


def _parser(target: UsbTarget) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "SSH into the Pi over Wi-Fi/hotspot when reachable, otherwise bring "
            "up the USB Ethernet link."
        )
    )
    parser.add_argument("--pi-ip", default=target.pi_ip)
    parser.add_argument("--host-ip", default=target.host_ip)
    parser.add_argument("--pi-user", default=target.pi_user)
    parser.add_argument("--pi-hostname", default=target.pi_hostname)
    parser.add_argument("--usb-iface", default=target.usb_iface)
    parser.add_argument("--timeout", type=int, default=target.timeout_seconds)
    parser.add_argument(
        "--ssh-target",
        action="append",
        help="explicit SSH target to try before auto-detected targets; can repeat",
    )
    parser.add_argument(
        "--network-only",
        action="store_true",
        help="only try existing Wi-Fi/hotspot/network SSH targets",
    )
    parser.add_argument(
        "--usb-only",
        action="store_true",
        help="skip Wi-Fi/hotspot checks and configure the USB link immediately",
    )
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


def ssh_command(pi_user: str, pi_ip: str) -> list[str]:
    remote = (
        "sudo ip link set dev usb0 mtu 1412 2>/dev/null || "
        "sudo ifconfig usb0 mtu 1412 2>/dev/null || true; "
        "exec $SHELL --login"
    )
    return ["ssh", "-t", f"{pi_user}@{pi_ip}", remote]


def plain_ssh_command(target: str) -> list[str]:
    return ["ssh", "-t", target]


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


def _print_command(command: Sequence[str]) -> None:
    print(f"  {shlex.join(command)}", flush=True)


def _run(command: Sequence[str], *, dry_run: bool) -> None:
    _print_command(command)
    if not dry_run:
        subprocess.run(command, check=True)


def wait_for_iface(
    timeout_seconds: int,
    system: str,
    *,
    dry_run: bool,
) -> str | None:
    if dry_run:
        print("  would auto-detect a USB/RNDIS Ethernet interface", flush=True)
        return None

    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        iface = find_usb_iface(system)
        if iface:
            return iface
        time.sleep(2)
    return None


def wait_for_ping(
    pi_ip: str,
    timeout_seconds: int,
    system: str,
    *,
    dry_run: bool,
) -> bool:
    command = ping_command(pi_ip, system)
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
        _print_command(plain_ssh_command(target))


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
    _run(plain_ssh_command(target), dry_run=False)
    return True


def run(
    arguments: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
) -> int:
    defaults = resolve_usb_target(environ)
    parser = _parser(defaults)
    args = parser.parse_args(arguments)
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")
    if args.network_only and args.usb_only:
        parser.error("--network-only and --usb-only cannot be used together")

    system = platform.system()
    targets = network_ssh_targets(
        args.pi_user,
        args.pi_hostname,
        args.pi_ip,
        args.ssh_target,
    )

    if not args.usb_only:
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
            print(
                "No existing network target responded; falling back to USB.", flush=True
            )

    if args.dry_run:
        print("If none of those respond, would configure USB Ethernet:", flush=True)

    print("Waiting for Pi USB Ethernet interface...", flush=True)
    print(f"Target: {args.pi_hostname}", flush=True)
    print(
        f"Use a data-capable USB cable from the laptop to the {defaults.port_hint}.",
        flush=True,
    )

    if args.usb_iface:
        if not args.dry_run and not iface_exists(args.usb_iface, system):
            print(f"USB interface does not exist: {args.usb_iface}", flush=True)
            return 1
        iface = args.usb_iface
    else:
        iface = wait_for_iface(args.timeout, system, dry_run=args.dry_run)
        if iface is None and args.dry_run:
            iface = "<auto-detected-interface>"

    if not iface:
        print("Timed out waiting for a USB Ethernet interface.", flush=True)
        print("Check the cable, Pi USB port, and Pi boot status.", flush=True)
        if system == "Windows":
            print(
                "If Windows found the adapter under a custom name, rerun with "
                '--usb-iface "Adapter Name".',
                flush=True,
            )
        return 1

    print(f"Found USB network interface: {iface}", flush=True)
    print(f"Configuring laptop side as {args.host_ip}/24...", flush=True)
    if args.dry_run or not has_host_ip(iface, args.host_ip, system):
        for command in config_commands(iface, args.host_ip, system):
            _run(command, dry_run=args.dry_run)

    print(f"Waiting for Pi at {args.pi_ip}...", flush=True)
    if not wait_for_ping(args.pi_ip, args.timeout, system, dry_run=args.dry_run):
        print(f"Timed out waiting for {args.pi_ip}.", flush=True)
        print(f"Try: ping {args.pi_ip}", flush=True)
        print(f"Try: ssh {args.pi_user}@{args.pi_ip}", flush=True)
        return 1

    print("Pi answers ping.", flush=True)
    print(
        "Connecting with SSH. Use the Pi password you set while flashing.", flush=True
    )
    _run(ssh_command(args.pi_user, args.pi_ip), dry_run=args.dry_run)
    return 0


def main() -> None:
    raise SystemExit(run())
