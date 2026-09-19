"""Shared Raspberry Pi connection defaults and environment resolution."""

from __future__ import annotations

import os
import platform
import subprocess
import time
from collections.abc import Mapping
from dataclasses import dataclass

from ai_drone.settings import load_settings

DEFAULT_PI_HOSTNAME = "seb-is-pm"
DEFAULT_PI_USERNAME = "seb"
DEFAULT_PI_USB_IP = "192.168.7.2"
DEFAULT_PI_HOTSPOT_IP = "192.168.4.1"
DEFAULT_HOST_USB_IP = "192.168.7.1"
DEFAULT_PI_USB_PORT_HINT = "Pi Zero 2 WH micro-USB port labeled USB, not PWR IN"
DEFAULT_PI_AP_SSID = "AI-Drone-Zero"
DEFAULT_DIRECT_SSH_CONFIG = None
REMOTE_UV = 'PATH="$HOME/.local/bin:$PATH" uv'


@dataclass(frozen=True)
class DeployTarget:
    """Resolved SSH/deploy settings for the Raspberry Pi."""

    ssh_target: str
    user: str
    address: str
    project_dir: str
    ssh_config: str | None


@dataclass(frozen=True)
class ConnectionTarget:
    """Resolved settings for all Raspberry Pi connection transports."""

    pi_ip: str
    host_ip: str
    pi_user: str
    pi_hostname: str
    port_hint: str
    usb_iface: str | None
    timeout_seconds: int
    ap_ssid: str
    ap_ip: str
    ssh_config: str | None


def _env(environ: Mapping[str, str] | None) -> Mapping[str, str]:
    return os.environ if environ is None else environ


def ssh_base_command(ssh_config: str | None) -> list[str]:
    """Build an OpenSSH command prefix with an explicit optional config file."""

    command = ["ssh"]
    if ssh_config:
        command.extend(["-F", ssh_config])
    return command


def ping_command(host: str, system: str | None = None) -> list[str]:
    """Return a one-shot ping command for the current host platform."""

    current_system = platform.system() if system is None else system
    if current_system == "Windows":
        return ["ping", "-n", "1", "-w", "1000", host]
    return ["ping", "-c", "1", "-W", "1", host]


def ping_host(host: str, system: str | None = None) -> bool:
    completed = subprocess.run(
        ping_command(host, system),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return completed.returncode == 0


def wait_for_ping(
    host: str,
    timeout_seconds: float,
    system: str | None = None,
    *,
    interval_seconds: float = 3.0,
) -> bool:
    """Wait up to ``timeout_seconds`` for a host to answer one ping."""

    if timeout_seconds <= 0 or interval_seconds <= 0:
        raise ValueError("ping timeout and interval must be greater than zero")
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if ping_host(host, system):
            return True
        time.sleep(min(interval_seconds, max(0.0, deadline - time.monotonic())))
    return False


def split_ssh_target(host: str, default_user: str) -> tuple[str, str, str]:
    """Return (ssh_target, user, address) for host values with or without user."""

    if "@" not in host:
        return f"{default_user}@{host}", default_user, host

    user, address = host.split("@", 1)
    user = user or default_user
    return f"{user}@{address}", user, address


def preferred_pi_addresses(target: ConnectionTarget) -> tuple[str, ...]:
    """Return Pi addresses in the common connection priority order.

    USB is intentionally omitted until the caller has supplied a verified
    interface name.  Merely answering at the USB subnet address is not enough
    evidence that reconfiguring a host network adapter is safe.
    """

    candidates = [target.pi_hostname, target.ap_ip]
    if target.usb_iface:
        candidates.append(target.pi_ip)
    return tuple(dict.fromkeys(candidates))


def resolve_deploy_target(
    environ: Mapping[str, str] | None = None,
) -> DeployTarget:
    values = _env(environ)
    settings = load_settings(environ=values).connection
    _, default_user, default_address = split_ssh_target(
        settings.host, DEFAULT_PI_USERNAME
    )
    host = values.get("PI_HOST") or values.get("PI_HOSTNAME") or default_address
    ssh_target, user, address = split_ssh_target(
        host, values.get("PI_USER", default_user)
    )
    return DeployTarget(
        ssh_target=ssh_target,
        user=user,
        address=address,
        project_dir=values.get("PI_DIR")
        or settings.project_dir
        or f"/home/{user}/ai-drone",
        ssh_config=values.get("SSH_CONFIG", settings.ssh_config) or None,
    )


def resolve_connection_target(
    environ: Mapping[str, str] | None = None,
) -> ConnectionTarget:
    values = _env(environ)
    target = resolve_deploy_target(values)
    timeout = int(values.get("TIMEOUT_SECONDS", "180"))
    return ConnectionTarget(
        pi_ip=values.get("PI_IP", DEFAULT_PI_USB_IP),
        host_ip=values.get("HOST_IP", DEFAULT_HOST_USB_IP),
        pi_user=target.user,
        pi_hostname=target.address,
        port_hint=values.get("PI_USB_PORT_HINT", DEFAULT_PI_USB_PORT_HINT),
        usb_iface=values.get("USB_IFACE") or None,
        timeout_seconds=timeout,
        ap_ssid=values.get("PI_AP_SSID", DEFAULT_PI_AP_SSID),
        ap_ip=values.get("PI_AP_IP", DEFAULT_PI_HOTSPOT_IP),
        ssh_config=target.ssh_config,
    )
