"""Typed local defaults; command-line options take precedence."""

from __future__ import annotations

import ipaddress
import math
import os
import re
import tomllib
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass, fields
from pathlib import Path


@dataclass(frozen=True)
class ConnectionSettings:
    host: str = "seb@seb-is-pm"
    project_dir: str | None = None
    ssh_config: str | None = None


@dataclass(frozen=True)
class RecordingSettings:
    storage_reserve_mib: float = 256
    storage_warning_mib: float = 1024
    storage_stop_mib: float = 16
    storage_check_interval: float = 2


@dataclass(frozen=True)
class TransferSettings:
    destination: str | None = None


@dataclass(frozen=True)
class OperatorSettings:
    endpoints: tuple[str, ...] = ()
    token_file: str | None = None
    interval: float = 1
    timeout: float = 5
    request_timeout: float = 1
    ssh_host: str | None = None
    remote_port: int = 18787


@dataclass(frozen=True)
class RuntimeSettings:
    socket: str = "/run/ai-drone/vehicle.sock"
    status: str = "/run/ai-drone/status.json"
    device: str = "/dev/serial0"
    baud: int = 115200
    network_profiles: str = "/etc/ai-drone/network-profiles.json"


@dataclass(frozen=True)
class Settings:
    connection: ConnectionSettings = ConnectionSettings()
    recording: RecordingSettings = RecordingSettings()
    transfer: TransferSettings = TransferSettings()
    operator: OperatorSettings = OperatorSettings()
    runtime: RuntimeSettings = RuntimeSettings()


_COMMAND_SETTINGS: ContextVar[Settings | None] = ContextVar(
    "command_settings", default=None
)


@contextmanager
def use_settings(settings: Settings) -> Iterator[None]:
    """Inject one immutable configuration into a command and its legacy adapters."""
    token = _COMMAND_SETTINGS.set(settings)
    try:
        yield
    finally:
        _COMMAND_SETTINGS.reset(token)


def validate_operator_forward(host: str | None, port: int) -> None:
    if (
        host is not None
        and re.fullmatch(
            r"(?:[A-Za-z0-9_][A-Za-z0-9_.-]*@)?[A-Za-z0-9_][A-Za-z0-9_.:-]*", host
        )
        is None
    ):
        raise ValueError("operator SSH host must be a hostname or user@hostname")
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("operator remote port must be an integer between 1 and 65535")


def _section(
    document: dict,
    name: str,
    defaults: ConnectionSettings
    | RecordingSettings
    | TransferSettings
    | OperatorSettings
    | RuntimeSettings,
) -> dict:
    values = document.get(name, {})
    if not isinstance(values, dict):
        raise ValueError(f"{name} must be a TOML table")
    known = {field.name for field in fields(type(defaults))}
    if values.keys() - known:
        raise ValueError(
            f"unknown {name} setting: {', '.join(sorted(values.keys() - known))}"
        )
    for key, value in values.items():
        default = getattr(defaults, key)
        if isinstance(default, tuple):
            valid = isinstance(value, list) and all(
                isinstance(item, str)
                and item.strip()
                and not any(character in item for character in "\0\r\n")
                for item in value
            )
            if valid:
                values[key] = tuple(value)
        elif isinstance(default, (int, float)):
            valid = type(value) in (int, float) and math.isfinite(value) and value > 0
        else:
            valid = (
                isinstance(value, str)
                and bool(value.strip())
                and not any(character in value for character in "\0\r\n")
            )
        if not valid:
            raise ValueError(f"invalid {name}.{key}")
    return values


def load_settings(
    path: Path | None = None, *, environ: Mapping[str, str] | None = None
) -> Settings:
    if path is None and (current := _COMMAND_SETTINGS.get()) is not None:
        return current
    environment = os.environ if environ is None else environ
    selected = path or environment.get("AI_DRONE_CONFIG")
    source = Path(selected).expanduser() if selected else Path("drone.toml")
    try:
        with source.open("rb") as handle:
            document = tomllib.load(handle)
    except FileNotFoundError:
        if selected:
            raise ValueError(f"configuration file does not exist: {source}") from None
        return Settings()
    except tomllib.TOMLDecodeError as error:
        raise ValueError(f"invalid TOML configuration: {source}") from error
    if document.keys() - {"connection", "recording", "transfer", "operator", "runtime"}:
        raise ValueError("unknown configuration section")
    result = Settings(
        ConnectionSettings(**_section(document, "connection", ConnectionSettings())),
        RecordingSettings(**_section(document, "recording", RecordingSettings())),
        TransferSettings(**_section(document, "transfer", TransferSettings())),
        OperatorSettings(**_section(document, "operator", OperatorSettings())),
        RuntimeSettings(**_section(document, "runtime", RuntimeSettings())),
    )
    recording = result.recording
    if (
        not 0
        < recording.storage_stop_mib
        < recording.storage_reserve_mib
        <= recording.storage_warning_mib
    ):
        raise ValueError("storage thresholds require 0 < stop < reserve <= warning")
    _validate_operator_settings(result.operator)
    if (
        type(result.runtime.baud) is not int
        or not 1 <= result.runtime.baud <= 4_000_000
    ):
        raise ValueError("runtime.baud must be an integer between 1 and 4000000")
    return result


def _validate_operator_settings(operator: OperatorSettings) -> None:
    """Validate heartbeat timing, paired credentials and endpoint origins together."""
    validate_operator_forward(operator.ssh_host, operator.remote_port)
    if not (
        0.2 <= operator.interval <= 5
        and 1 <= operator.timeout <= 30
        and operator.timeout >= 2 * operator.interval
        and 0.1 <= operator.request_timeout <= min(5, operator.timeout / 2)
        and len(operator.endpoints) <= 4
    ):
        raise ValueError(
            "operator heartbeat requires 1-30s timeout, at least two polls, and at most four endpoints"
        )
    if bool(operator.endpoints) != bool(operator.token_file):
        raise ValueError(
            "operator.endpoints and operator.token_file must be configured together"
        )
    from urllib.parse import urlsplit

    for endpoint in operator.endpoints:
        address = urlsplit(endpoint)
        if (
            address.scheme not in {"http", "https"}
            or not address.hostname
            or address.username is not None
            or address.password is not None
            or address.query
            or address.fragment
            or address.path not in {"", "/"}
        ):
            raise ValueError(
                "operator endpoints must be HTTP(S) origins without credentials or paths"
            )
        try:
            _ = address.port
            if address.port == 0:
                raise ValueError("port zero is not an operator endpoint")
            ipaddress.ip_address(address.hostname)
        except ValueError as error:
            raise ValueError(
                "operator endpoint requires a numeric IP and valid port"
            ) from error
