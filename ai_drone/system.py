"""Subprocess representations; callers retain operation-specific safety policy."""

from __future__ import annotations

import argparse
import shlex
import shutil
import signal
import subprocess
import threading
from collections.abc import Callable, Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from types import FrameType


@dataclass(frozen=True)
class UnitState:
    load: str
    active: str
    enabled: str
    returncode: int

    @property
    def absent(self) -> bool:
        return self.load == "not-found" and self.returncode in (0, 1)


def unit_state(result: subprocess.CompletedProcess[str]) -> UnitState:
    properties = dict(
        line.split("=", 1) for line in result.stdout.splitlines() if "=" in line
    )
    return UnitState(
        properties.get("LoadState", "unknown"),
        properties.get("ActiveState", "unknown"),
        properties.get("UnitFileState", "unknown"),
        result.returncode,
    )


def run(
    command: Sequence[str],
    *,
    dry_run: bool = False,
    capture: bool = False,
    timeout: float | None = None,
) -> subprocess.CompletedProcess[str]:
    print(f"  {shlex.join(command)}", flush=True)
    if dry_run:
        return subprocess.CompletedProcess(list(command), 0, "", "")
    return subprocess.run(
        command, check=True, capture_output=capture, text=True, timeout=timeout
    )


def require_uv() -> str:
    executable = shutil.which("uv")
    if executable is None:
        raise FileNotFoundError("uv is required for Python execution")
    return executable


def namespace_to_flags(
    args: argparse.Namespace,
    *,
    exclude: frozenset[str] = frozenset(),
    aliases: Mapping[str, str] | None = None,
    serializers: Mapping[str, Callable[[object], str]] | None = None,
) -> list[str]:
    flags: list[str] = []
    for name, value in vars(args).items():
        if name in exclude or value is None:
            continue
        option = (aliases or {}).get(name, f"--{name.replace('_', '-')}")
        if isinstance(value, bool):
            if value:
                flags.append(option)
        elif name in (serializers or {}):
            flags.extend((option, (serializers or {})[name](value)))
        else:
            for item in value if isinstance(value, list) else [value]:
                flags.extend((option, str(item)))
    return flags


def systemd_run_command(
    unit: str,
    command: Sequence[str],
    *,
    uid: int,
    gid: int,
    directory: Path,
    environment: Mapping[str, str],
    properties: Mapping[str, str],
) -> list[str]:
    return [
        "sudo",
        "-n",
        "systemd-run",
        f"--unit={unit}",
        "--collect",
        "--service-type=exec",
        "--expand-environment=no",
        f"--uid={uid}",
        f"--gid={gid}",
        f"--working-directory={directory}",
        *(f"--setenv={key}={value}" for key, value in environment.items()),
        *(f"--property={key}={value}" for key, value in properties.items()),
        *command,
    ]


SignalHandler = Callable[[int, FrameType | None], object] | int | None


@contextmanager
def handled_signals(handlers: Mapping[int, SignalHandler]) -> Iterator[None]:
    """Install an explicit policy on the main thread, restoring nested handlers."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    previous = {}
    try:
        for number, handler in handlers.items():
            previous[number] = signal.signal(number, handler)
        yield
    finally:
        for number, handler in reversed(previous.items()):
            signal.signal(number, handler)
