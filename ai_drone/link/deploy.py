"""Deploy the project to the Raspberry Pi from Linux, macOS, or Windows."""

from __future__ import annotations

import argparse
import io
import json
import os
import re
import secrets
import shlex
import subprocess
import tarfile
import tempfile
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from ai_drone.link.targets import (
    DeployTarget,
    remote_python_command,
    resolve_deploy_target,
    ssh_base_command,
)
from ai_drone.system import print_command
from ai_drone.system import run as run_command

REPO_ROOT = Path(__file__).resolve().parents[2]
MANIFEST_NAME = ".__ai_drone_manifest"
SENTINEL_NAME = ".__ai_drone_deploy_sentinel"
MANIFEST_FORMAT = "ai-drone-runtime-v1"
SENTINEL_PREFIX = "ai-drone-deploy-v1:"

# Keep the deployed tree deliberately smaller than the development checkout.
# These are the only source paths needed to install and operate the Pi runtime.
RUNTIME_TREES = ("ai_drone", "native-runtime")
RUNTIME_FILES = frozenset(
    {
        "README.md",  # Referenced by pyproject.toml during package builds.
        "drone.example.toml",
        "firmware/FlywooF745-nogps-loiter.manifest.json",
        "firmware/FlywooF745-nogps-loiter-extra.hwdef",
        "pyproject.toml",
        "scripts/network.py",
        "scripts/transfer.py",
        "scripts/power.py",
        "scripts/setup_runtime.py",
        "scripts/motor_test.py",
        "scripts/servo.py",
        "scripts/mount.py",
        "scripts/usb_ssh.py",
        "scripts/tag_mount_capture.py",
        "scripts/verify_ardupilot_firmware.py",
        "scripts/pi-safe-upgrade.sh",
        "scripts/setup-pi-hotspot.sh",
        "scripts/setup-pi-power-resilience.sh",
        "scripts/usb0-static.service",
        "uv.lock",
    }
)
REQUIRED_RUNTIME_PATHS = frozenset(
    {"README.md", "ai_drone", "ai_drone/__init__.py", "pyproject.toml", "uv.lock"}
)

# These paths are never uploaded or removed from an existing deployment.  In
# particular, artifacts and the virtual environment are state, not source.
EXCLUDE_NAMES = frozenset(
    {
        ".git",
        ".venv",
        ".ruff_cache",
        ".pytest_cache",
        "artifacts",
        "state",
        "drone.toml",
        "__pycache__",
        "aitrios-rpi-sample-apps",
        "Drone-Handbook.pdf",
        "raspios-lite",
    }
)

_SAFE_REMOTE_COMPONENT = re.compile(r"[A-Za-z0-9._-]+\Z")
_SAFE_REMOTE_USER = re.compile(r"[a-z_][a-z0-9_-]{0,31}\Z")
_FORBIDDEN_REMOTE_ROOTS = frozenset(
    {"bin", "boot", "dev", "etc", "lib", "lib64", "proc", "run", "sbin", "sys", "usr"}
)


@dataclass(frozen=True)
class DeployPlan:
    target: DeployTarget
    dry_run: bool
    offline: bool = False
    native_payload: Path | None = None


def _validated_remote_project_dir(target: DeployTarget) -> str:
    """Return a canonical, conservatively safe POSIX deployment directory."""

    if _SAFE_REMOTE_USER.fullmatch(target.user) is None:
        raise ValueError(f"refusing unsafe Pi user name {target.user!r}")

    raw = target.project_dir
    if not raw or "\x00" in raw or "\\" in raw:
        raise ValueError("PI_DIR must be a non-empty absolute POSIX path")

    # A trailing slash is harmless and common in environment configuration;
    # all other normalization differences indicate repeated separators or dot
    # components that should not reach a recursive delete operation.
    candidate = raw.rstrip("/") or "/"
    path = PurePosixPath(candidate)
    canonical = path.as_posix()
    if not path.is_absolute() or candidate != canonical:
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: use a canonical absolute path"
        )

    target_home = PurePosixPath(
        "/root" if target.user == "root" else f"/home/{target.user}"
    )
    if path == target_home:
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: it is the target user's home directory"
        )

    parts = path.parts[1:]
    if len(parts) < 2:
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: deployment paths must not be root "
            "or a top-level directory"
        )
    if any(
        part in {".", ".."} or _SAFE_REMOTE_COMPONENT.fullmatch(part) is None
        for part in parts
    ):
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: path components may contain only "
            "letters, digits, '.', '_' and '-'"
        )

    _validate_deployment_location(parts, target.user, raw)
    return canonical


def _validate_deployment_location(parts: tuple[str, ...], user: str, raw: str) -> None:
    """Keep a canonical target within permitted, dedicated storage locations."""
    first = parts[0]
    if first in _FORBIDDEN_REMOTE_ROOTS:
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: deployment below /{first} is not allowed"
        )
    if first == "home" and (user == "root" or parts[1] != user):
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: /home deployments must stay below "
            f"/home/{user}"
        )
    if first in {"home", "Users"} and len(parts) < 3:
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: a user's home directory is not a "
            "deployment target"
        )
    if first == "root" and user != "root":
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: only root may deploy below /root"
        )
    if first == "var" and len(parts) < 3:
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: /var deployment paths need a "
            "dedicated child directory"
        )
    if first in {"home", "Users", "root"} and parts[-1].startswith("."):
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: hidden home paths are not deployment "
            "targets"
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sync the runtime and install dependencies on the Raspberry Pi."
    )
    parser.add_argument("--dry-run", action="store_true", help="print commands only")
    parser.add_argument(
        "--offline", action="store_true", help="install from the Pi's uv cache only"
    )
    parser.add_argument(
        "--native-payload",
        type=Path,
        help="explicit reviewed source/native-wheel payload (always installs offline)",
    )
    return parser


def build_plan(
    arguments: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
) -> DeployPlan:
    parser = _parser()
    args = parser.parse_args(arguments)
    target = resolve_deploy_target(environ)
    _validated_remote_project_dir(target)
    return DeployPlan(
        target=target,
        dry_run=args.dry_run,
        offline=args.offline,
        native_payload=args.native_payload,
    )


def remote_command(
    target: DeployTarget, command: str, *, tty: bool = False
) -> list[str]:
    ssh_command = ssh_base_command(target.ssh_config)
    if tty:
        ssh_command.append("-t")
    return [*ssh_command, target.ssh_target, command]


def remote_preflight_command(plan: DeployPlan) -> list[str]:
    """Build a read-only check for the remote deployment directory."""

    target = plan.target
    project_dir = _validated_remote_project_dir(target)
    script = f"""\
project_dir={shlex.quote(project_dir)}
resolved="$(realpath -m -- "$project_dir")" || {{
    echo "Refusing deploy: realpath could not inspect PI_DIR" >&2
    exit 2
}}
if [ "$resolved" != "$project_dir" ]; then
    echo "Refusing deploy: PI_DIR or an ancestor resolves through a symlink" >&2
    exit 2
fi
if [ -e "$project_dir" ] || [ -L "$project_dir" ]; then
    if [ ! -d "$project_dir" ]; then
        echo "Refusing deploy: existing PI_DIR is not a directory" >&2
        exit 2
    fi
    expected_uid="$(id -u -- {shlex.quote(target.user)})" || {{
        echo "Refusing deploy: target user does not exist" >&2
        exit 2
    }}
    actual_uid="$(stat -c %u -- "$project_dir")" || {{
        echo "Refusing deploy: cannot inspect PI_DIR ownership" >&2
        exit 2
    }}
    if [ "$actual_uid" != "$expected_uid" ]; then
        echo "Refusing deploy: existing PI_DIR has the wrong owner" >&2
        exit 2
    fi
fi
"""
    return remote_command(target, f"sh -c {shlex.quote(script)}")


def _is_secret_path(relative: Path | PurePosixPath | str) -> bool:
    parts = PurePosixPath(relative).parts
    return any(part == ".env" or part.startswith(".env.") for part in parts)


def _is_excluded(relative: Path) -> bool:
    return (
        _is_secret_path(relative)
        or relative.suffix.lower() in {".wav", ".mp3", ".ogg", ".flac", ".m4a"}
        or any(
            part in EXCLUDE_NAMES or part.startswith("drone-tone-handover-backup-")
            for part in relative.parts
        )
    )


def _is_runtime_path(relative: Path | PurePosixPath | str) -> bool:
    value = PurePosixPath(relative).as_posix()
    if value in RUNTIME_FILES:
        return True
    if any(value == tree or value.startswith(f"{tree}/") for tree in RUNTIME_TREES):
        return True
    # Explicit files need their parent directories in both transport payloads.
    return any(runtime_file.startswith(f"{value}/") for runtime_file in RUNTIME_FILES)


def _iter_sync_paths(repo_root: Path) -> list[Path]:
    paths: list[Path] = []
    for root, dirs, files in os.walk(repo_root):
        root_path = Path(root)
        rel_root = root_path.relative_to(repo_root)
        dirs[:] = [
            dirname
            for dirname in dirs
            if _is_runtime_path(rel_root / dirname)
            and not _is_excluded(rel_root / dirname)
        ]
        for dirname in dirs:
            paths.append(root_path / dirname)
        for filename in files:
            path = root_path / filename
            relative = path.relative_to(repo_root)
            if _is_runtime_path(relative) and not _is_excluded(relative):
                paths.append(path)
    return sorted(paths)


def _validate_runtime_source(repo_root: Path, sync_paths: Sequence[Path]) -> None:
    relative_paths = {path.relative_to(repo_root).as_posix() for path in sync_paths}
    missing = sorted(REQUIRED_RUNTIME_PATHS - relative_paths)
    if missing:
        raise ValueError(
            "deployment source is missing required runtime paths: " + ", ".join(missing)
        )

    unsupported = [
        path.relative_to(repo_root).as_posix()
        for path in sync_paths
        if path.is_symlink() or not (path.is_file() or path.is_dir())
    ]
    if unsupported:
        raise ValueError(
            "deployment payload contains symlinks or special files: "
            + ", ".join(sorted(unsupported))
        )


def _validated_deployment_id(deployment_id: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9_-]{16,128}", deployment_id):
        raise ValueError("deployment id must be 16-128 URL-safe characters")
    return deployment_id


def _manifest_bytes(
    sync_paths: Sequence[Path], repo_root: Path, deployment_id: str
) -> bytes:
    document = {
        "deployment_id": deployment_id,
        "format": MANIFEST_FORMAT,
        "paths": [path.relative_to(repo_root).as_posix() for path in sync_paths],
    }
    return (json.dumps(document, separators=(",", ":"), sort_keys=True) + "\n").encode()


def _add_archive_bytes(tar: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name)
    info.mode = 0o600
    info.size = len(data)
    tar.addfile(info, fileobj=io.BytesIO(data))


def _create_sync_archive(repo_root: Path, deployment_id: str) -> Path:
    deployment_id = _validated_deployment_id(deployment_id)
    sync_paths = _iter_sync_paths(repo_root)
    _validate_runtime_source(repo_root, sync_paths)
    with tempfile.NamedTemporaryFile(delete=False, suffix=".tar.gz") as temporary:
        archive = Path(temporary.name)

    try:
        with tarfile.open(archive, "w:gz") as tar:
            for path in sync_paths:
                tar.add(
                    path,
                    arcname=path.relative_to(repo_root).as_posix(),
                    recursive=False,
                )
            _add_archive_bytes(
                tar,
                MANIFEST_NAME,
                _manifest_bytes(sync_paths, repo_root, deployment_id),
            )
            _add_archive_bytes(
                tar,
                SENTINEL_NAME,
                f"{SENTINEL_PREFIX}{deployment_id}\n".encode(),
            )
    except BaseException:
        archive.unlink(missing_ok=True)
        raise

    return archive


class DeploymentPayloadError(RuntimeError):
    """The uploaded payload is missing or inconsistent with its manifest."""


def _read_deployment_file(path: Path, maximum_size: int) -> str:
    if path.is_symlink() or not path.is_file():
        raise DeploymentPayloadError(f"{path.name} is missing or is not a regular file")
    try:
        if path.stat().st_size > maximum_size:
            raise DeploymentPayloadError(f"{path.name} is unexpectedly large")
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise DeploymentPayloadError(f"cannot read {path.name}: {error}") from error


def _manifest_listing(root: Path, deployment_id: str) -> list[object]:
    sentinel_path = root / SENTINEL_NAME
    sentinel = _read_deployment_file(sentinel_path, 512)
    if sentinel != f"{SENTINEL_PREFIX}{deployment_id}\n":
        raise DeploymentPayloadError("deployment sentinel does not match this upload")

    manifest_path = root / MANIFEST_NAME
    manifest_text = _read_deployment_file(manifest_path, 1_000_000)
    try:
        document = json.loads(manifest_text)
    except (TypeError, ValueError) as error:
        raise DeploymentPayloadError(f"manifest is not valid JSON: {error}") from error
    if not isinstance(document, dict):
        raise DeploymentPayloadError("manifest root is not an object")
    if document.get("format") != MANIFEST_FORMAT:
        raise DeploymentPayloadError("manifest format is unsupported")
    if document.get("deployment_id") != deployment_id:
        raise DeploymentPayloadError("manifest does not match this upload")

    listed = document.get("paths")
    if not isinstance(listed, list) or not listed:
        raise DeploymentPayloadError("manifest path list is missing or empty")
    return listed


def _validated_manifest_path(relative: object) -> str:
    if not isinstance(relative, str):
        raise DeploymentPayloadError("manifest contains a non-string path")
    path = PurePosixPath(relative)
    if (
        not path.parts
        or path.is_absolute()
        or path.as_posix() != relative
        or any(part in {"", ".", ".."} for part in path.parts)
    ):
        raise DeploymentPayloadError(f"manifest contains unsafe path {relative!r}")
    if relative in {MANIFEST_NAME, SENTINEL_NAME}:
        raise DeploymentPayloadError("manifest contains deployment metadata")
    if _is_excluded(Path(relative)) or not _is_runtime_path(relative):
        raise DeploymentPayloadError(f"manifest contains non-runtime path {relative!r}")
    return relative


def _manifest_paths(root: Path, deployment_id: str) -> set[str]:
    listed = _manifest_listing(root, deployment_id)
    allowed = {_validated_manifest_path(relative) for relative in listed}

    if len(allowed) != len(listed):
        raise DeploymentPayloadError("manifest contains duplicate paths")
    missing_required = sorted(REQUIRED_RUNTIME_PATHS - allowed)
    if missing_required:
        raise DeploymentPayloadError(
            "manifest omits required paths: " + ", ".join(missing_required)
        )

    missing_payload = []
    for relative in sorted(allowed):
        path = root.joinpath(*PurePosixPath(relative).parts)
        if path.is_symlink() or not path.exists():
            missing_payload.append(relative)
    if missing_payload:
        raise DeploymentPayloadError(
            "uploaded payload is missing paths: " + ", ".join(missing_payload)
        )
    return allowed


def _run_remote_preflight(plan: DeployPlan) -> None:
    print("Checking remote deployment directory safety ...", flush=True)
    run_command(remote_preflight_command(plan), dry_run=plan.dry_run)


def _deploy_transaction(plan: DeployPlan, repo_root: Path = REPO_ROOT) -> None:
    if plan.native_payload is not None:
        from ai_drone.link.native import validate_payload

        repo_root = plan.native_payload.resolve()
        validate_payload(repo_root)
    elif (repo_root / "native-runtime").exists():
        raise ValueError("native runtime requires explicit --native-payload selection")
    _validate_runtime_source(repo_root, _iter_sync_paths(repo_root))
    _run_remote_preflight(plan)
    deployment_id = secrets.token_hex(16)
    stage = f"/tmp/ai-drone-deploy-{deployment_id}"
    project = _validated_remote_project_dir(plan.target)
    upload = remote_command(
        plan.target,
        f"umask 077; mkdir -- {shlex.quote(stage)} && tar -xzf - -C {shlex.quote(stage)}",
    )
    payload = (
        "from ai_drone.cli.deploy import _transaction_entry;"
        f"_transaction_entry({stage!r}, {project!r}, {deployment_id!r}, "
        f"offline={plan.offline!r}, native={plan.native_payload is not None!r})"
    )
    command = remote_python_command(
        plan.target,
        ["-P", "-c", payload],
        uv_flags=[
            "--no-project",
            "--no-config",
            "--offline",
            "--python",
            project + "/.venv/bin/python",
        ],
        environment={"PYTHONPATH": stage},
    )
    print(
        "Stage runtime; verify disarmed idle state; back up source/environment; update and restart.",
        flush=True,
    )
    print_command(upload)
    print_command(command)
    if plan.dry_run:
        return
    archive = _create_sync_archive(repo_root, deployment_id)
    try:
        with archive.open("rb") as source:
            subprocess.run(upload, stdin=source, check=True)
        subprocess.run(command, check=True)
    finally:
        archive.unlink(missing_ok=True)


def run(arguments: Sequence[str] | None = None) -> int:
    plan = build_plan(arguments)
    _deploy_transaction(plan)
    print("Dry run complete." if plan.dry_run else "Runtime updated.", flush=True)
    return 0


def main() -> None:
    raise SystemExit(run())
