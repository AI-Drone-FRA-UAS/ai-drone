"""Apply an uploaded runtime on the Pi after disarmed maintenance checks."""

from __future__ import annotations

import contextlib
import os
import re
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from ai_drone.link.deploy import (
    _is_excluded,
    _iter_sync_paths,
    _manifest_paths,
    _validate_runtime_source,
)
from ai_drone.runtime_status import RuntimeStatus
from ai_drone.system import handled_signals, unit_state


@dataclass(frozen=True)
class Prepared:
    restart_socket: str | None = None


@dataclass(frozen=True)
class Stopping:
    restart_socket: str


@dataclass(frozen=True)
class BackedUp:
    restart_socket: str | None


@dataclass(frozen=True)
class Installing:
    restart_socket: str | None


@dataclass(frozen=True)
class Installed:
    restart_socket: str | None


@dataclass(frozen=True)
class Restored:
    restart_socket: str | None


DeployState = Prepared | Stopping | BackedUp | Installing | Installed | Restored


def _mutable_paths(root: Path, *, environment: bool) -> list[Path]:
    """List update-owned paths, without descending into protected local state."""
    paths = []
    for current, directories, files in os.walk(root, followlinks=False):
        parent = Path(current)

        def included(name: str, parent: Path = parent) -> bool:
            relative = (parent / name).relative_to(root)
            return (environment and relative.parts[0] == ".venv") or not _is_excluded(
                relative
            )

        directories[:] = [name for name in directories if included(name)]
        paths.extend(parent / name for name in [*directories, *files] if included(name))
    return sorted(paths, key=lambda path: (len(path.parts), path.as_posix()))


def _copy_paths(source: Path, destination: Path, paths: Sequence[Path]) -> None:
    for path in paths:
        target = destination / path.relative_to(source)
        if path.is_dir() and not path.is_symlink():
            target.mkdir(parents=True, exist_ok=True)
            shutil.copystat(path, target, follow_symlinks=False)
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target, follow_symlinks=False)


def _remove_mutable(root: Path, *, environment: bool) -> None:
    for path in reversed(_mutable_paths(root, environment=environment)):
        if path.is_dir() and not path.is_symlink():
            # Protected state can live inside an otherwise source-owned directory.
            with contextlib.suppress(OSError):
                path.rmdir()
        else:
            path.unlink()


def _runtime_service_state(*, timeout: float = 10) -> str:
    from ai_drone.cli.power import RUNTIME_UNIT

    result = subprocess.run(
        ["systemctl", "show", RUNTIME_UNIT, "--property=LoadState,ActiveState"],
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    state = unit_state(result)
    if state.absent:
        return "inactive"
    if (
        result.returncode
        or state.load != "loaded"
        or state.active not in {"active", "inactive", "failed"}
    ):
        raise RuntimeError(
            "runtime service state is unknown or changing; refusing deploy"
        )
    return state.active


def _runtime_service(action: str) -> None:
    from ai_drone.cli.power import RUNTIME_UNIT

    subprocess.run(
        ["sudo", "-n", "systemctl", action, RUNTIME_UNIT],
        check=True,
        timeout=30,
    )


def _runtime_ready(status: object, now: float) -> bool:
    try:
        parsed = RuntimeStatus.parse(status)
    except ValueError:
        return False
    return parsed.healthy_after_restart(now)


def _wait_runtime_ready(socket: str, *, timeout: float = 12) -> None:
    """Read-only after restart: a new client or armed vehicle must remain untouched."""
    from ai_drone.mavlink.remote import runtime_request

    deadline = time.monotonic() + timeout
    while (remaining := deadline - time.monotonic()) > 0:
        try:
            status = runtime_request(
                socket, {"status": True}, timeout=min(1, remaining)
            )
            remaining = deadline - time.monotonic()
            if (
                remaining > 0
                and _runtime_service_state(timeout=min(1, remaining)) == "active"
                and _runtime_ready(status, time.monotonic())
            ):
                return
        except (OSError, RuntimeError, ValueError, subprocess.SubprocessError):
            pass
        time.sleep(min(0.2, max(0, deadline - time.monotonic())))
    raise RuntimeError(
        "restarted runtime did not report healthy FC telemetry; "
        "previous source/environment retained for recovery"
    )


def _environment_interpreter(project: Path) -> tuple[str, bool, str]:
    """Preserve the installed minor version, independently of developer defaults.

    New Pi installs stay on the qualified system 3.13 route. Candidate 3.14
    environments need a reviewed native-artifact preservation/install contract;
    rebuilding or exactly syncing them would discard separately built bindings.
    """
    config = project / ".venv/pyvenv.cfg"
    if not config.exists():
        if (project / ".venv").exists():
            raise RuntimeError("existing environment has no interpreter metadata")
        return "/usr/bin/python3.13", True, "3.13"
    values = dict(
        (key.strip(), value.strip())
        for line in config.read_text().splitlines()
        if "=" in line
        for key, value in [line.split("=", 1)]
    )
    version = values.get("version_info", values.get("version", ""))
    if not re.fullmatch(r"3\.(?:11|12|13|14)\.\d+(?:\.final\.0)?", version):
        raise RuntimeError(
            "existing environment Python version is unknown or unsupported"
        )
    if version.startswith("3.14."):
        raise RuntimeError(
            "Python 3.14 deployment is blocked pending a reviewed native-binding "
            "artifact manifest and installation contract; preserve the separate "
            "candidate environment and keep the working Python 3.13 runtime"
        )
    if values.get("include-system-site-packages") == "true":
        return ".venv/bin/python", False, ".".join(version.split(".")[:2])
    executable = values.get("executable", "")
    if not Path(executable).is_absolute():
        raise RuntimeError("existing environment base interpreter is unknown")
    return executable, True, ".".join(version.split(".")[:2])


def _install_local(project: Path, *, offline: bool) -> None:
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError("uv is required on the Pi")
    interpreter, rebuild, minor = _environment_interpreter(project)
    if rebuild:
        subprocess.run(
            [
                uv,
                "venv",
                "--clear",
                "--python",
                interpreter,
                "--system-site-packages",
                ".venv",
            ],
            cwd=project,
            check=True,
        )
    subprocess.run(
        [
            uv,
            "sync",
            "--locked",
            "--python",
            ".venv/bin/python",
            "--no-dev",
            "--group",
            "raspi",
            *(["--offline"] if offline else []),
        ],
        cwd=project,
        check=True,
    )
    _validate_runtime_interpreter(project, uv, minor)


def _validate_runtime_interpreter(project: Path, uv: str, minor: str) -> None:
    """Import native dependencies without constructing hardware objects.

    A stale CPython extension or a free-threaded candidate must fail the source
    and environment transaction before the service can be restarted.
    """
    subprocess.run(
        [
            uv,
            "run",
            "--no-project",
            "--no-config",
            "--offline",
            "--python",
            str(project / ".venv/bin/python"),
            "python",
            "-I",
            "-c",
            "import sys, sysconfig; "
            "assert sys.version_info[:2] == tuple(map(int, sys.argv[1].split('.'))); "
            "assert not sysconfig.get_config_var('Py_GIL_DISABLED'); "
            "import numpy, PIL, pymavlink, cv2, picamera2, libcamera, pykms, "
            "gpiozero, lgpio, prctl, apriltag",
            minor,
        ],
        cwd=project,
        check=True,
    )


def _staged_source_paths(stage: Path, project: Path, deployment_id: str) -> list[Path]:
    """Validate the uploaded source and target before any maintenance action."""
    from ai_drone.platform import is_raspberry_pi

    if not is_raspberry_pi():
        raise RuntimeError("deployment maintenance requires the Raspberry Pi")
    if (
        stage.is_symlink()
        or stat.S_IMODE(stage.stat().st_mode) != 0o700
        or stage.stat().st_uid != os.geteuid()
    ):
        raise RuntimeError("deployment staging directory is not private and owned")
    _manifest_paths(stage, deployment_id)
    source_paths = _iter_sync_paths(stage)
    _validate_runtime_source(stage, source_paths)
    if (
        project.resolve() != project
        or not project.is_dir()
        or (project / ".venv").is_symlink()
    ):
        raise RuntimeError(
            "deployment requires an existing real project directory and local environment"
        )
    return source_paths


def _apply_staged_update(
    stage: Path, project: Path, deployment_id: str, *, offline: bool
) -> None:
    """Run from uploaded code, so an old installation need not know this protocol."""
    from ai_drone.cli.power import _guard_idle, _pi_fc, _pi_snapshot, _require_disarmed
    from ai_drone.mavlink.remote import runtime_request

    source_paths = _staged_source_paths(stage, project, deployment_id)
    initial = _pi_snapshot()
    _guard_idle(initial)
    _require_disarmed(_pi_fc(initial))
    runtime = initial.get("runtime")
    active = _runtime_service_state() == "active"
    if active != (runtime is not None) or (
        runtime and runtime.get("maintenance") is not False
    ):
        raise RuntimeError(
            "runtime service/socket mismatch or existing maintenance; refusing deploy"
        )
    old_paths = _mutable_paths(project, environment=True)
    required = sum(
        path.lstat().st_size
        for path in [*old_paths, *source_paths]
        if not path.is_dir()
    )
    if shutil.disk_usage(project.parent).free < required + 64 * 1024 * 1024:
        raise RuntimeError(
            "insufficient free space for source/environment rollback and update"
        )
    backup = Path(tempfile.mkdtemp(prefix=".ai-drone-rollback-", dir=project.parent))
    state: DeployState = Prepared()
    try:
        if runtime is not None:
            response = runtime_request(runtime["socket"], {"maintenance": True})
            if response.get("maintenance") is not True:
                raise RuntimeError("runtime refused deployment maintenance")
            state = Stopping(runtime["socket"])
            _runtime_service("stop")
        quiet = _pi_snapshot()
        _guard_idle(quiet)
        _require_disarmed(_pi_fc(quiet))
        _copy_paths(project, backup, old_paths)
        state = BackedUp(state.restart_socket)
        # Copying the environment can take time; do not reuse its earlier heartbeat.
        final = _pi_snapshot()
        _guard_idle(final)
        _require_disarmed(_pi_fc(final))
        state = Installing(state.restart_socket)
        _remove_mutable(project, environment=False)
        _copy_paths(stage, project, source_paths)
        _install_local(project, offline=offline)
        state = Installed(state.restart_socket)
    except BaseException:
        if isinstance(state, Installing):
            _remove_mutable(project, environment=True)
            _copy_paths(backup, project, _mutable_paths(backup, environment=True))
            state = Restored(state.restart_socket)
        raise
    finally:
        _finish_deployment(state, backup)


def _finish_deployment(state: DeployState, backup: Path) -> None:
    from ai_drone.mavlink.remote import runtime_request

    restarted = state.restart_socket is None
    try:
        if state.restart_socket is not None and not isinstance(state, Installing):
            _runtime_service("start")
            _wait_runtime_ready(state.restart_socket)
            restarted = True
        if state.restart_socket is not None and not isinstance(state, Installed):
            # A failed stop can leave the original process alive and latched.
            with contextlib.suppress(OSError, RuntimeError, TimeoutError):
                runtime_request(state.restart_socket, {"maintenance": False})
    finally:
        if (isinstance(state, Installed) and restarted) or isinstance(
            state, Prepared | Stopping | BackedUp
        ):
            shutil.rmtree(backup)
        else:
            print(f"Previous source/environment retained at {backup}", file=sys.stderr)


def _transaction_entry(
    stage: str, project: str, deployment_id: str, *, offline: bool
) -> None:
    import fcntl

    def interrupted(signum: int, _frame: object) -> None:
        raise RuntimeError(f"deployment interrupted by signal {signum}")

    lock = Path(project).parent / f".{Path(project).name}.deploy.lock"
    descriptor = os.open(lock, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        with handled_signals(
            dict.fromkeys((signal.SIGTERM, signal.SIGINT, signal.SIGHUP), interrupted)
        ):
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            _apply_staged_update(
                Path(stage), Path(project), deployment_id, offline=offline
            )
    finally:
        os.close(descriptor)
        shutil.rmtree(stage)
