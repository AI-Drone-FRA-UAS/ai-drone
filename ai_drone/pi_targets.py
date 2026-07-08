"""Shared Raspberry Pi connection defaults and environment resolution."""

from __future__ import annotations

import os
import platform
import subprocess
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path


DEFAULT_PI_HOSTNAME = "seb-is-pm"
DEFAULT_PI_USERNAME = "seb"
DEFAULT_PI_USB_IP = "192.168.7.2"
DEFAULT_PI_HOTSPOT_IP = "192.168.4.1"
DEFAULT_HOST_USB_IP = "192.168.7.1"
DEFAULT_PI_USB_PORT_HINT = "Pi Zero 2 WH micro-USB port labeled USB, not PWR IN"
DEFAULT_PI_AP_SSID = "AI-Drone-Zero"


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


def _env(environ: Mapping[str, str] | None) -> Mapping[str, str]:
    return os.environ if environ is None else environ


def _default_ssh_config(environ: Mapping[str, str]) -> str | None:
    if "SSH_CONFIG" in environ:
        return environ["SSH_CONFIG"] or None

    home = environ.get("HOME")
    path = (
        Path(home).expanduser() / ".ssh" / "config"
        if home
        else Path.home() / ".ssh" / "config"
    )
    return str(path) if path.is_file() else None


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


def split_ssh_target(host: str, default_user: str) -> tuple[str, str, str]:
    """Return (ssh_target, user, address) for host values with or without user."""

    if "@" not in host:
        return host, default_user, host

    user, address = host.split("@", 1)
    return host, user or default_user, address


def resolve_deploy_target(
    environ: Mapping[str, str] | None = None,
    *,
    ping: Callable[[str], bool] | None = None,
    system: str | None = None,
) -> DeployTarget:
    values = _env(environ)
    pi_user = values.get("PI_USER", DEFAULT_PI_USERNAME)
    probe = ping if ping is not None else lambda host: ping_host(host, system)

    explicit_host = values.get("PI_HOST")
    if explicit_host:
        ssh_target, user, address = split_ssh_target(explicit_host, pi_user)
    else:
        if probe(DEFAULT_PI_USB_IP):
            address = DEFAULT_PI_USB_IP
        elif probe(DEFAULT_PI_HOSTNAME):
            address = DEFAULT_PI_HOSTNAME
        else:
            address = DEFAULT_PI_USB_IP
        user = pi_user
        ssh_target = f"{user}@{address}"

    project_dir = values.get("PI_DIR", f"/home/{user}/ai-drone")
    return DeployTarget(
        ssh_target=ssh_target,
        user=user,
        address=address,
        project_dir=project_dir,
        ssh_config=_default_ssh_config(values),
    )


UsbTarget = ConnectionTarget


def resolve_connection_target(
    environ: Mapping[str, str] | None = None,
) -> ConnectionTarget:
    values = _env(environ)
    timeout = int(values.get("TIMEOUT_SECONDS", "180"))
    return ConnectionTarget(
        pi_ip=values.get("PI_IP", DEFAULT_PI_USB_IP),
        host_ip=values.get("HOST_IP", DEFAULT_HOST_USB_IP),
        pi_user=values.get("PI_USER", DEFAULT_PI_USERNAME),
        pi_hostname=values.get("PI_HOSTNAME", DEFAULT_PI_HOSTNAME),
        port_hint=values.get("PI_USB_PORT_HINT", DEFAULT_PI_USB_PORT_HINT),
        usb_iface=values.get("USB_IFACE") or None,
        timeout_seconds=timeout,
        ap_ssid=values.get("PI_AP_SSID", DEFAULT_PI_AP_SSID),
        ap_ip=values.get("PI_AP_IP", DEFAULT_PI_HOTSPOT_IP),
    )


def resolve_usb_target(
    environ: Mapping[str, str] | None = None,
) -> ConnectionTarget:
    return resolve_connection_target(environ)
