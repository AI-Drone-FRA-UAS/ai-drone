"""Deploy the project to the Raspberry Pi from Linux, macOS, or Windows."""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import platform
import re
import secrets
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from ai_drone.pi_targets import DeployTarget, resolve_deploy_target

REPO_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_NAME = ".__ai_drone_manifest"
SENTINEL_NAME = ".__ai_drone_deploy_sentinel"
MANIFEST_FORMAT = "ai-drone-runtime-v1"
SENTINEL_PREFIX = "ai-drone-deploy-v1:"

# Keep the deployed tree deliberately smaller than the development checkout.
# These are the only source paths needed to install and operate the Pi runtime.
RUNTIME_TREES = ("ai_drone",)
RUNTIME_FILES = frozenset(
    {
        "README.md",  # Referenced by pyproject.toml during package builds.
        "pyproject.toml",
        "scripts/pi-safe-upgrade.sh",
        "scripts/setup-pi-dual-network.sh",
        "scripts/setup-pi-hotspot.sh",
        "scripts/setup-pi-power-resilience.sh",
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
    mode: str | None
    extra_args: tuple[str, ...]
    dry_run: bool
    sync_method: str


def ssh_base_command(target: DeployTarget) -> list[str]:
    command = ["ssh"]
    if target.ssh_config:
        command.extend(["-F", target.ssh_config])
    return command


def choose_sync_method(
    system: str | None = None,
    *,
    rsync_path: str | None = None,
) -> str:
    current_system = platform.system() if system is None else system
    if current_system != "Windows" and rsync_path:
        return "rsync"
    return "tar"


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

    first = parts[0]
    if first in _FORBIDDEN_REMOTE_ROOTS:
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: deployment below /{first} is not allowed"
        )
    if first == "home" and (target.user == "root" or parts[1] != target.user):
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: /home deployments must stay below "
            f"/home/{target.user}"
        )
    if first in {"home", "Users"} and len(parts) < 3:
        raise ValueError(
            f"refusing unsafe PI_DIR {raw!r}: a user's home directory is not a "
            "deployment target"
        )
    if first == "root" and target.user != "root":
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
    return canonical


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sync ai-drone to the Raspberry Pi and optionally run a task."
    )
    parser.add_argument("--picam", action="store_true", help="start the IMX500 stream")
    parser.add_argument(
        "--apriltag", action="store_true", help="start safe AprilTag detection"
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="record camera and all available disarmed sensor telemetry",
    )
    parser.add_argument(
        "--lidar", action="store_true", help="sample lidar over Pi UART"
    )
    parser.add_argument(
        "--servo", action="store_true", help="start the SG90 servo test"
    )
    parser.add_argument(
        "--motor-test",
        action="store_true",
        help="run the guarded, propeller-free ArduPilot bench motor test",
    )
    parser.add_argument("--ssh", action="store_true", help="open a shell on the Pi")
    parser.add_argument("--dry-run", action="store_true", help="print commands only")
    return parser


def build_plan(
    arguments: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
    system: str | None = None,
    ping: Callable[[str], bool] | None = None,
    rsync_path: str | None = None,
) -> DeployPlan:
    parser = _parser()
    args, extra_args = parser.parse_known_args(arguments)
    modes = [
        name
        for name in (
            "picam",
            "apriltag",
            "record",
            "lidar",
            "servo",
            "motor_test",
            "ssh",
        )
        if getattr(args, name)
    ]
    if len(modes) > 1:
        parser.error(
            "--picam, --apriltag, --record, --lidar, --servo, --motor-test, "
            "and --ssh are mutually exclusive"
        )
    if not modes and extra_args:
        parser.error(
            "extra command options require --picam, --apriltag, --record, --lidar, "
            "--servo, or --motor-test"
        )

    target = resolve_deploy_target(environ, ping=ping, system=system)
    _validated_remote_project_dir(target)
    sync_method = choose_sync_method(
        system,
        rsync_path=shutil.which("rsync") if rsync_path is None else rsync_path,
    )
    return DeployPlan(
        target=target,
        mode=modes[0] if modes else None,
        extra_args=tuple(extra_args),
        dry_run=args.dry_run,
        sync_method=sync_method,
    )


def remote_command(
    target: DeployTarget, command: str, *, tty: bool = False
) -> list[str]:
    ssh_command = ssh_base_command(target)
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
    return _is_secret_path(relative) or any(
        part in EXCLUDE_NAMES for part in relative.parts
    )


def _is_runtime_path(relative: Path | PurePosixPath | str) -> bool:
    value = PurePosixPath(relative).as_posix()
    if value in RUNTIME_FILES:
        return True
    if any(value == tree or value.startswith(f"{tree}/") for tree in RUNTIME_TREES):
        return True
    # Explicit files need their parent directories in both transport payloads.
    return any(runtime_file.startswith(f"{value}/") for runtime_file in RUNTIME_FILES)


def _rsync_filter_rules() -> list[str]:
    rules: list[str] = []
    for name in sorted(EXCLUDE_NAMES):
        rules.extend(
            (
                f"protect {name}",
                f"protect {name}/***",
                f"hide {name}",
                f"hide {name}/***",
            )
        )
    rules.extend(
        (
            "protect .env",
            "protect .env.*",
            "hide .env",
            "hide .env.*",
        )
    )
    for tree in RUNTIME_TREES:
        rules.extend((f"show /{tree}/", f"show /{tree}/***"))
    parent_dirs = {
        parent.as_posix()
        for runtime_file in RUNTIME_FILES
        for parent in PurePosixPath(runtime_file).parents
        if parent.as_posix() != "."
    }
    rules.extend(f"show /{parent}/" for parent in sorted(parent_dirs))
    rules.extend(f"show /{runtime_file}" for runtime_file in sorted(RUNTIME_FILES))
    rules.append("hide /***")
    return rules


def rsync_command(plan: DeployPlan, repo_root: Path = REPO_ROOT) -> list[str]:
    project_dir = _validated_remote_project_dir(plan.target)
    ssh_transport = shlex.join(ssh_base_command(plan.target))
    command = ["rsync", "-az", "--delete", "-e", ssh_transport]
    for rule in _rsync_filter_rules():
        command.extend(["--filter", rule])
    command.extend([f"{repo_root}/", f"{plan.target.ssh_target}:{project_dir}/"])
    return command


def install_command(plan: DeployPlan) -> list[str]:
    target = plan.target
    project_dir = _validated_remote_project_dir(target)
    setup = (
        "if [ ! -f .venv/pyvenv.cfg ] || ! grep -qx "
        '"include-system-site-packages = true" .venv/pyvenv.cfg; then '
        "uv venv --clear --python /usr/bin/python3 --system-site-packages .venv; "
        "fi; "
        "uv sync --locked --python .venv/bin/python --no-dev --group raspi"
    )
    target_home = "/root" if target.user == "root" else f"/home/{target.user}"
    path_prefix = f"PATH={target_home}/.local/bin:$PATH"
    command = (
        f"cd {shlex.quote(project_dir)} && {path_prefix} sh -lc {shlex.quote(setup)}"
    )
    return remote_command(target, command)


def mode_command(plan: DeployPlan) -> list[str] | None:
    target = plan.target
    project_dir = _validated_remote_project_dir(target)
    extra = " ".join(shlex.quote(argument) for argument in plan.extra_args)
    suffix = f" {extra}" if extra else ""

    if plan.mode == "picam":
        command = (
            f"cd {shlex.quote(project_dir)} && "
            f".venv/bin/python -m ai_drone.cli.picam{suffix}"
        )
        return remote_command(target, command, tty=True)
    if plan.mode == "apriltag":
        command = (
            f"cd {shlex.quote(project_dir)} && "
            f".venv/bin/python -m ai_drone.cli.apriltag{suffix}"
        )
        return remote_command(target, command, tty=True)
    if plan.mode == "record":
        command = (
            f"cd {shlex.quote(project_dir)} && "
            f".venv/bin/python -m ai_drone.cli.record{suffix}"
        )
        return remote_command(target, command, tty=True)
    if plan.mode == "lidar":
        command = (
            f"cd {shlex.quote(project_dir)} && "
            f".venv/bin/python -m ai_drone.cli.lidar --device /dev/serial0{suffix}"
        )
        return remote_command(target, command, tty=True)
    if plan.mode == "servo":
        command = (
            f"cd {shlex.quote(project_dir)} && "
            f".venv/bin/python -m ai_drone.cli.servo{suffix}"
        )
        return remote_command(target, command, tty=True)
    if plan.mode == "motor_test":
        command = (
            f"cd {shlex.quote(project_dir)} && "
            f".venv/bin/python -m ai_drone.cli.motor_test{suffix}"
        )
        return remote_command(target, command, tty=True)
    if plan.mode == "ssh":
        command = f"cd {shlex.quote(project_dir)} && exec $SHELL --login"
        return remote_command(target, command, tty=True)
    return None


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


class DeploymentCleanupError(RuntimeError):
    """The uploaded metadata is not safe enough to authorize deletion."""


def _read_deployment_file(path: Path, maximum_size: int) -> str:
    if path.is_symlink() or not path.is_file():
        raise DeploymentCleanupError(f"{path.name} is missing or is not a regular file")
    try:
        if path.stat().st_size > maximum_size:
            raise DeploymentCleanupError(f"{path.name} is unexpectedly large")
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise DeploymentCleanupError(f"cannot read {path.name}: {error}") from error


def _manifest_listing(root: Path, deployment_id: str) -> list[object]:
    sentinel_path = root / SENTINEL_NAME
    sentinel = _read_deployment_file(sentinel_path, 512)
    if sentinel != f"{SENTINEL_PREFIX}{deployment_id}\n":
        raise DeploymentCleanupError("deployment sentinel does not match this upload")

    manifest_path = root / MANIFEST_NAME
    manifest_text = _read_deployment_file(manifest_path, 1_000_000)
    try:
        document = json.loads(manifest_text)
    except (TypeError, ValueError) as error:
        raise DeploymentCleanupError(f"manifest is not valid JSON: {error}") from error
    if not isinstance(document, dict):
        raise DeploymentCleanupError("manifest root is not an object")
    if document.get("format") != MANIFEST_FORMAT:
        raise DeploymentCleanupError("manifest format is unsupported")
    if document.get("deployment_id") != deployment_id:
        raise DeploymentCleanupError("manifest does not match this upload")

    listed = document.get("paths")
    if not isinstance(listed, list) or not listed:
        raise DeploymentCleanupError("manifest path list is missing or empty")
    return listed


def _validated_manifest_path(relative: object) -> str:
    if not isinstance(relative, str):
        raise DeploymentCleanupError("manifest contains a non-string path")
    path = PurePosixPath(relative)
    if (
        not path.parts
        or path.is_absolute()
        or path.as_posix() != relative
        or any(part in {"", ".", ".."} for part in path.parts)
    ):
        raise DeploymentCleanupError(f"manifest contains unsafe path {relative!r}")
    if relative in {MANIFEST_NAME, SENTINEL_NAME}:
        raise DeploymentCleanupError("manifest contains deployment metadata")
    if _is_excluded(Path(relative)) or not _is_runtime_path(relative):
        raise DeploymentCleanupError(f"manifest contains non-runtime path {relative!r}")
    return relative


def _manifest_paths(root: Path, deployment_id: str) -> set[str]:
    listed = _manifest_listing(root, deployment_id)
    allowed = {_validated_manifest_path(relative) for relative in listed}

    if len(allowed) != len(listed):
        raise DeploymentCleanupError("manifest contains duplicate paths")
    missing_required = sorted(REQUIRED_RUNTIME_PATHS - allowed)
    if missing_required:
        raise DeploymentCleanupError(
            "manifest omits required paths: " + ", ".join(missing_required)
        )

    missing_payload = []
    for relative in sorted(allowed):
        path = root.joinpath(*PurePosixPath(relative).parts)
        if path.is_symlink() or not path.exists():
            missing_payload.append(relative)
    if missing_payload:
        raise DeploymentCleanupError(
            "uploaded payload is missing paths: " + ", ".join(missing_payload)
        )
    return allowed


def _cleanup_deployment(deployment_id: str, root: Path | None = None) -> None:
    deployment_id = _validated_deployment_id(deployment_id)
    deployment_root = Path.cwd() if root is None else root
    allowed = _manifest_paths(deployment_root, deployment_id)
    metadata = {MANIFEST_NAME, SENTINEL_NAME}

    for path in sorted(
        deployment_root.rglob("*"),
        key=lambda item: (len(item.parts), item.as_posix()),
        reverse=True,
    ):
        relative = path.relative_to(deployment_root).as_posix()
        if relative in metadata or _is_excluded(Path(relative)) or relative in allowed:
            continue
        if path.is_dir() and not path.is_symlink():
            with contextlib.suppress(OSError):
                path.rmdir()
        else:
            path.unlink(missing_ok=True)

    (deployment_root / MANIFEST_NAME).unlink()
    (deployment_root / SENTINEL_NAME).unlink()


def _cleanup_entry(deployment_id: str, root: Path | None = None) -> int:
    try:
        _cleanup_deployment(deployment_id, root)
    except (DeploymentCleanupError, ValueError) as error:
        print(f"Refusing deployment cleanup: {error}", file=sys.stderr)
        return 2
    return 0


def _cleanup_script(deployment_id: str) -> str:
    deployment_id = _validated_deployment_id(deployment_id)
    payload = (
        "from ai_drone.deploy import _cleanup_entry;"
        f"raise SystemExit(_cleanup_entry({deployment_id!r}))"
    )
    return f"python3 -c {shlex.quote(payload)}"


def _print_command(command: Sequence[str]) -> None:
    print(f"  {shlex.join(command)}", flush=True)


def _run(command: Sequence[str], *, dry_run: bool) -> None:
    _print_command(command)
    if not dry_run:
        subprocess.run(command, check=True)


def _run_remote_preflight(plan: DeployPlan) -> None:
    print("Checking remote deployment directory safety ...", flush=True)
    _run(remote_preflight_command(plan), dry_run=plan.dry_run)


def _sync_with_rsync(plan: DeployPlan, repo_root: Path) -> None:
    project_dir = _validated_remote_project_dir(plan.target)
    _validate_runtime_source(repo_root, _iter_sync_paths(repo_root))
    _run_remote_preflight(plan)
    print(
        f"Syncing to {plan.target.ssh_target}:{project_dir}/ ...",
        flush=True,
    )
    _run(rsync_command(plan, repo_root), dry_run=plan.dry_run)


def _sync_with_tar(plan: DeployPlan, repo_root: Path) -> None:
    target = plan.target
    project_dir = _validated_remote_project_dir(target)
    _validate_runtime_source(repo_root, _iter_sync_paths(repo_root))
    _run_remote_preflight(plan)
    deployment_id = secrets.token_hex(16)
    extract = (
        f"mkdir -p {shlex.quote(project_dir)} && "
        f"tar -xzf - -C {shlex.quote(project_dir)}"
    )
    cleanup = f"cd {shlex.quote(project_dir)} && {_cleanup_script(deployment_id)}"
    extract_command = remote_command(target, extract)
    cleanup_command = remote_command(target, cleanup)

    print(
        f"Syncing to {target.ssh_target}:{project_dir}/ using SSH tar stream ...",
        flush=True,
    )
    print(f"  python tar.gz stream | {shlex.join(extract_command)}", flush=True)
    _print_command(cleanup_command)
    if plan.dry_run:
        return

    archive = _create_sync_archive(repo_root, deployment_id)
    try:
        with archive.open("rb") as handle:
            subprocess.run(extract_command, stdin=handle, check=True)
        subprocess.run(cleanup_command, check=True)
    finally:
        archive.unlink(missing_ok=True)


def sync_project(plan: DeployPlan, repo_root: Path = REPO_ROOT) -> None:
    _validated_remote_project_dir(plan.target)
    if plan.sync_method == "rsync":
        _sync_with_rsync(plan, repo_root)
    else:
        _sync_with_tar(plan, repo_root)
    print("Sync done.", flush=True)


def run(arguments: Sequence[str] | None = None) -> int:
    plan = build_plan(arguments)
    sync_project(plan)

    print("Installing raspi dependencies on the Pi ...", flush=True)
    _run(install_command(plan), dry_run=plan.dry_run)
    print("Dependencies installed.", flush=True)

    command = mode_command(plan)
    if command is None:
        print(
            "Done. Use --picam, --apriltag, --record, --lidar, --servo, "
            "--motor-test, or --ssh.",
            flush=True,
        )
        return 0

    if plan.mode == "picam":
        print("Starting the IMX500 AI stream on the Pi ...", flush=True)
        print(f"Open http://{plan.target.address}:8080/ in your browser.", flush=True)
        print("Press Ctrl-C to stop.", flush=True)
    elif plan.mode == "apriltag":
        print("Starting safe AprilTag detection on the Pi ...", flush=True)
        print("This mode never arms, moves, or actuates the servo.", flush=True)
    elif plan.mode == "record":
        print(
            "Recording camera and disarmed sensor telemetry on the Pi ...", flush=True
        )
        print("This mode never arms, changes mode, or actuates anything.", flush=True)
    elif plan.mode == "lidar":
        print("Sampling MTF-01P through the Pi flight-controller link ...", flush=True)
    elif plan.mode == "servo":
        print("Starting the SG90 servo test on the Pi ...", flush=True)
    elif plan.mode == "motor_test":
        print("Starting the guarded ArduPilot bench motor test ...", flush=True)
        print("PROPELLERS MUST BE REMOVED and the vehicle secured.", flush=True)
    elif plan.mode == "ssh":
        print("Opening shell on the Pi ...", flush=True)

    _run(command, dry_run=plan.dry_run)
    return 0


def main() -> None:
    raise SystemExit(run())
