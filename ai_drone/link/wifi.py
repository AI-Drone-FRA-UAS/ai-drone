"""Cross-platform helpers for joining the Pi's own Wi-Fi AP (``AI-Drone-Zero``).

The connect flow never handles the Wi-Fi PSK: each OS command below joins a
*previously saved* profile (Linux ``nmcli con up``, macOS
``networksetup -setairportnetwork`` using the keychain, Windows
``netsh wlan connect`` using a stored WLAN profile). Save it once with, e.g.::

    nmcli dev wifi connect AI-Drone-Zero password <PSK>

Functions come in two flavours, matching ``link.usb_ssh``: pure command builders
(``*_command``) that are trivial to unit test, and thin read-only runners
(``ap_available`` / ``current_ssid`` / ``wifi_device``) that shell out.
"""

from __future__ import annotations

import subprocess
from collections.abc import Sequence


def _run_capture(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, check=False, capture_output=True, text=True)


# --- Wi-Fi interface (only macOS needs an explicit device name) ---------------


def wifi_device_command(system: str) -> list[str] | None:
    if system == "Darwin":
        return ["networksetup", "-listallhardwareports"]
    return None


def parse_darwin_wifi_device(output: str) -> str | None:
    """Pull the ``enX`` device that follows the ``Wi-Fi`` hardware port block."""

    lines = output.splitlines()
    for index, line in enumerate(lines):
        if "Wi-Fi" in line or "AirPort" in line:
            for follow in lines[index + 1 : index + 4]:
                stripped = follow.strip()
                if stripped.startswith("Device:"):
                    return stripped.split(":", 1)[1].strip() or None
    return None


def wifi_device(system: str) -> str | None:
    command = wifi_device_command(system)
    if command is None:
        return None
    completed = _run_capture(command)
    if completed.returncode != 0:
        return None
    return parse_darwin_wifi_device(completed.stdout)


# --- Scan: is the AP visible right now? ---------------------------------------


def scan_command(system: str) -> list[str] | None:
    if system == "Darwin":
        # Modern macOS has no reliable scriptable scan; skip and just try to join.
        return None
    if system == "Windows":
        return ["netsh", "wlan", "show", "networks"]
    return ["nmcli", "-t", "-f", "SSID", "dev", "wifi", "list", "--rescan", "yes"]


def ssid_in_scan_output(ssid: str, output: str, system: str) -> bool:
    if system == "Windows":
        # Lines look like: "SSID 3 : AI-Drone-Zero"
        return any(
            line.split(":", 1)[1].strip() == ssid
            for line in output.splitlines()
            if line.strip().startswith("SSID") and ":" in line
        )
    # nmcli -t escapes ':' inside fields as '\:'; a plain SSID compares directly.
    return any(line.strip() == ssid for line in output.splitlines())


def ap_available(ssid: str, system: str) -> bool:
    """Whether the AP is broadcasting. Optimistic (``True``) when unscannable."""

    command = scan_command(system)
    if command is None:
        return True
    completed = _run_capture(command)
    if completed.returncode != 0:
        return True
    return ssid_in_scan_output(ssid, completed.stdout, system)


# --- Current SSID (so a failed join can be rolled back) -----------------------


def current_ssid_command(system: str, device: str | None = None) -> list[str]:
    if system == "Darwin":
        return ["networksetup", "-getairportnetwork", device or "en0"]
    if system == "Windows":
        return ["netsh", "wlan", "show", "interfaces"]
    return ["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"]


def parse_current_ssid(output: str, system: str) -> str | None:
    if system == "Darwin":
        # "Current Wi-Fi Network: <SSID>"
        for line in output.splitlines():
            if ":" in line:
                _, value = line.split(":", 1)
                value = value.strip()
                if value and "not associated" not in value.lower():
                    return value
        return None
    if system == "Windows":
        for line in output.splitlines():
            stripped = line.strip()
            if stripped.startswith("SSID") and not stripped.startswith("BSSID"):
                return stripped.split(":", 1)[1].strip() or None
        return None
    for line in output.splitlines():
        if line.startswith("yes:"):
            return line.split(":", 1)[1] or None
    return None


def current_ssid(system: str, device: str | None = None) -> str | None:
    completed = _run_capture(current_ssid_command(system, device))
    if completed.returncode != 0:
        return None
    return parse_current_ssid(completed.stdout, system)


# --- Join a known SSID --------------------------------------------------------


def join_command(ssid: str, system: str, device: str | None = None) -> list[str]:
    if system == "Darwin":
        return ["networksetup", "-setairportnetwork", device or "en0", ssid]
    if system == "Windows":
        return ["netsh", "wlan", "connect", f"name={ssid}", f"ssid={ssid}"]
    return ["nmcli", "con", "up", ssid]
