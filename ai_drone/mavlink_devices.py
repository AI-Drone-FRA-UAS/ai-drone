"""Shared MAVLink serial device discovery helpers."""

from __future__ import annotations

from pathlib import Path

STABLE_FLIGHT_CONTROLLER_DEVICE = Path(
    "/dev/serial/by-id/usb-ArduPilot_FlywooF745_200023000451333436353531-if00"
)
NETWORK_ENDPOINT_PREFIXES = ("udp:", "tcp:", "tcpin:", "udpout:", "tcpout:")


def is_network_endpoint(value: str) -> bool:
    """Whether *value* is a pymavlink network endpoint instead of a path."""

    return value.startswith(NETWORK_ENDPOINT_PREFIXES)


def serial_candidates(
    *,
    prefer_stable: bool = False,
    include_pi_uart: bool = True,
    stable_device: Path = STABLE_FLIGHT_CONTROLLER_DEVICE,
) -> list[Path]:
    """Return candidate serial devices in the preferred order."""

    candidates: list[Path] = []
    if prefer_stable:
        candidates.append(stable_device)
    candidates.extend(sorted(Path("/dev/serial/by-id").glob("*ArduPilot*")))
    if prefer_stable:
        candidates.append(Path("/dev/ttyACM0"))
    candidates.extend(sorted(Path("/dev").glob("ttyACM*")))
    if include_pi_uart:
        candidates.append(Path("/dev/serial0"))

    unique: list[Path] = []
    for candidate in candidates:
        if candidate not in unique:
            unique.append(candidate)
    return unique


def find_serial_device(
    requested: str | Path | None,
    *,
    prefer_stable: bool = False,
    include_pi_uart: bool = True,
    stable_device: Path = STABLE_FLIGHT_CONTROLLER_DEVICE,
    missing_message: str | None = None,
) -> Path:
    """Resolve a requested serial device or find the first existing candidate."""

    if requested:
        device = Path(requested)
        if not device.exists():
            raise FileNotFoundError(f"Serial device does not exist: {device}")
        return device

    for device in serial_candidates(
        prefer_stable=prefer_stable,
        include_pi_uart=include_pi_uart,
        stable_device=stable_device,
    ):
        if device.exists():
            return device

    if missing_message is not None:
        raise FileNotFoundError(missing_message)
    raise FileNotFoundError("No MAVLink serial device found. Use --device /dev/...")


def resolve_mavlink_endpoint(
    requested: str | Path | None,
    *,
    prefer_stable: bool = False,
    include_pi_uart: bool = True,
    stable_device: Path = STABLE_FLIGHT_CONTROLLER_DEVICE,
    missing_message: str | None = None,
) -> str:
    """Resolve either a pymavlink network endpoint or a serial device path."""

    if requested:
        value = str(requested)
        if is_network_endpoint(value):
            return value
    return str(
        find_serial_device(
            requested,
            prefer_stable=prefer_stable,
            include_pi_uart=include_pi_uart,
            stable_device=stable_device,
            missing_message=missing_message,
        )
    )
