"""Read-only checks for an existing Linux serial owner before opening a port."""

from __future__ import annotations

import stat
import subprocess
import sys
from pathlib import Path

from ai_drone.mavlink.devices import is_network_endpoint


class SerialDeviceBusyError(RuntimeError):
    """A visible existing process owns the selected serial device."""


def _inspect(command: list[str]) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command, capture_output=True, text=True, timeout=3, check=False
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RuntimeError(
            "Cannot inspect serial ownership; port was not opened"
        ) from error


def require_available_serial(endpoint: str, *, on_pi: bool = False) -> None:
    """Refuse known owners without opening, changing, or stopping the device.

    fuser can inspect the normal same-user recorder without sudo. This is an
    existing-owner check, not an atomic lock against every possible client or
    a claim to see descriptors hidden by another user's permissions.
    Explicit network endpoints have separate socket semantics and are skipped.
    """
    if is_network_endpoint(endpoint):
        return
    if not sys.platform.startswith("linux"):
        raise RuntimeError("Serial-owner inspection currently requires Linux")
    device = Path(endpoint).resolve(strict=True)
    if not stat.S_ISCHR(device.stat().st_mode):
        raise RuntimeError("Bench checks require a live serial device, not a data file")
    if on_pi:
        state = _inspect(
            [
                "systemctl",
                "show",
                "ai-drone-walk.service",
                "--property=LoadState",
                "--property=ActiveState",
            ]
        )
        properties = dict(
            line.split("=", 1) for line in state.stdout.splitlines() if "=" in line
        )
        absent = properties.get("LoadState") == "not-found" and state.returncode in (
            0,
            1,
        )
        if not absent and (state.returncode or properties.get("LoadState") != "loaded"):
            raise RuntimeError(
                "Cannot inspect recorder service; serial port was not opened"
            )
        if properties.get("ActiveState") not in ("inactive", "failed"):
            raise RuntimeError(
                "Recorder is running or its state is unknown; wait for drone-walk to finish"
            )
    # The resolved path is absolute and cannot be parsed as an option. Some
    # supported host fuser implementations do not accept a -- separator.
    owners = _inspect(["fuser", str(device)])
    if owners.returncode == 0:
        raise SerialDeviceBusyError(
            "Serial device is busy; close the other serial client first"
        )
    if owners.returncode != 1 or owners.stdout.strip() or owners.stderr.strip():
        raise RuntimeError(
            "Cannot establish that the serial device is idle; port was not opened"
        )
