"""Deploy the project to the Raspberry Pi from Linux, macOS, or Windows."""

from __future__ import annotations

import argparse
import base64
import io
import os
import platform
import shlex
import shutil
import subprocess
import tarfile
import tempfile
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from ai_drone.pi_targets import DeployTarget, resolve_deploy_target

REPO_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_NAME = ".__ai_drone_manifest"
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


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sync ai-drone to the Raspberry Pi and optionally run a task."
    )
    parser.add_argument("--picam", action="store_true", help="start the IMX500 stream")
    parser.add_argument(
        "--lidar", action="store_true", help="sample lidar over Pi UART"
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
    modes = [name for name in ("picam", "lidar", "ssh") if getattr(args, name)]
    if len(modes) > 1:
        parser.error("--picam, --lidar, and --ssh are mutually exclusive")
    if not modes and extra_args:
        parser.error("extra command options require --picam or --lidar")

    target = resolve_deploy_target(environ, ping=ping, system=system)
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


def rsync_command(plan: DeployPlan, repo_root: Path = REPO_ROOT) -> list[str]:
    ssh_transport = shlex.join(ssh_base_command(plan.target))
    command = ["rsync", "-az", "--delete", "-e", ssh_transport]
    for name in EXCLUDE_NAMES:
        command.extend(["--exclude", name])
    command.extend(
        [f"{repo_root}/", f"{plan.target.ssh_target}:{plan.target.project_dir}/"]
    )
    return command


def install_command(plan: DeployPlan) -> list[str]:
    target = plan.target
    setup = (
        "if [ ! -f .venv/pyvenv.cfg ] || ! grep -qx "
        '"include-system-site-packages = true" .venv/pyvenv.cfg; then '
        "uv venv --clear --python /usr/bin/python3 --system-site-packages .venv; "
        "fi; "
        "uv sync --locked --python .venv/bin/python --no-dev --group raspi"
    )
    path_prefix = f"PATH=/home/{target.user}/.local/bin:$PATH"
    command = (
        f"cd {shlex.quote(target.project_dir)} && "
        f"{path_prefix} sh -lc {shlex.quote(setup)}"
    )
    return remote_command(target, command)


def mode_command(plan: DeployPlan) -> list[str] | None:
    target = plan.target
    extra = " ".join(shlex.quote(argument) for argument in plan.extra_args)
    suffix = f" {extra}" if extra else ""

    if plan.mode == "picam":
        command = f"cd {shlex.quote(target.project_dir)} && .venv/bin/python test_picam.py{suffix}"
        return remote_command(target, command, tty=True)
    if plan.mode == "lidar":
        command = (
            f"cd {shlex.quote(target.project_dir)} && "
            f".venv/bin/python test_lidar.py --device /dev/serial0{suffix}"
        )
        return remote_command(target, command, tty=True)
    if plan.mode == "ssh":
        command = f"cd {shlex.quote(target.project_dir)} && exec $SHELL --login"
        return remote_command(target, command, tty=True)
    return None


def _is_excluded(relative: Path) -> bool:
    return any(part in EXCLUDE_NAMES for part in relative.parts)


def _iter_sync_paths(repo_root: Path) -> list[Path]:
    paths: list[Path] = []
    for root, dirs, files in os.walk(repo_root):
        root_path = Path(root)
        rel_root = root_path.relative_to(repo_root)
        dirs[:] = [dirname for dirname in dirs if not _is_excluded(rel_root / dirname)]
        for dirname in dirs:
            paths.append(root_path / dirname)
        for filename in files:
            path = root_path / filename
            if not _is_excluded(path.relative_to(repo_root)):
                paths.append(path)
    return sorted(paths)


def _create_sync_archive(repo_root: Path) -> Path:
    sync_paths = _iter_sync_paths(repo_root)
    manifest = "\n".join(path.relative_to(repo_root).as_posix() for path in sync_paths)
    temp = tempfile.NamedTemporaryFile(delete=False, suffix=".tar.gz")
    archive = Path(temp.name)
    temp.close()

    with tarfile.open(archive, "w:gz") as tar:
        for path in sync_paths:
            tar.add(
                path,
                arcname=path.relative_to(repo_root).as_posix(),
                recursive=False,
            )
        data = manifest.encode()
        info = tarfile.TarInfo(MANIFEST_NAME)
        info.size = len(data)
        tar.addfile(info, fileobj=io.BytesIO(data))

    return archive


def _cleanup_script() -> str:
    code = f"""
from pathlib import Path

root = Path.cwd()
manifest_path = root / {MANIFEST_NAME!r}
allowed = set(manifest_path.read_text().splitlines())
protected = {sorted(EXCLUDE_NAMES)!r}

def is_protected(relative):
    return any(part in protected for part in relative.split("/"))

for path in sorted(root.rglob("*"), key=lambda item: len(item.parts), reverse=True):
    relative = path.relative_to(root).as_posix()
    if relative == {MANIFEST_NAME!r} or is_protected(relative) or relative in allowed:
        continue
    if path.is_dir() and not path.is_symlink():
        try:
            path.rmdir()
        except OSError:
            pass
    else:
        path.unlink(missing_ok=True)

manifest_path.unlink(missing_ok=True)
"""
    encoded = base64.b64encode(code.encode()).decode()
    payload = f"import base64;exec(base64.b64decode({encoded!r}))"
    return f"python3 -c {shlex.quote(payload)}"


def _print_command(command: Sequence[str]) -> None:
    print(f"  {shlex.join(command)}", flush=True)


def _run(command: Sequence[str], *, dry_run: bool) -> None:
    _print_command(command)
    if not dry_run:
        subprocess.run(command, check=True)


def _sync_with_rsync(plan: DeployPlan, repo_root: Path) -> None:
    print(
        f"Syncing to {plan.target.ssh_target}:{plan.target.project_dir}/ ...",
        flush=True,
    )
    _run(rsync_command(plan, repo_root), dry_run=plan.dry_run)


def _sync_with_tar(plan: DeployPlan, repo_root: Path) -> None:
    target = plan.target
    extract = (
        f"mkdir -p {shlex.quote(target.project_dir)} && "
        f"tar -xzf - -C {shlex.quote(target.project_dir)}"
    )
    cleanup = f"cd {shlex.quote(target.project_dir)} && {_cleanup_script()}"
    extract_command = remote_command(target, extract)
    cleanup_command = remote_command(target, cleanup)

    print(
        f"Syncing to {target.ssh_target}:{target.project_dir}/ using SSH tar stream ...",
        flush=True,
    )
    print(f"  python tar.gz stream | {shlex.join(extract_command)}", flush=True)
    _print_command(cleanup_command)
    if plan.dry_run:
        return

    archive = _create_sync_archive(repo_root)
    try:
        with archive.open("rb") as handle:
            subprocess.run(extract_command, stdin=handle, check=True)
        subprocess.run(cleanup_command, check=True)
    finally:
        archive.unlink(missing_ok=True)


def sync_project(plan: DeployPlan, repo_root: Path = REPO_ROOT) -> None:
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
        print("Done. Use --picam, --lidar, or --ssh.", flush=True)
        return 0

    if plan.mode == "picam":
        print("Starting the IMX500 AI stream on the Pi ...", flush=True)
        print(f"Open http://{plan.target.address}:8080/ in your browser.", flush=True)
        print("Press Ctrl-C to stop.", flush=True)
    elif plan.mode == "lidar":
        print("Sampling MTF-01P through the Pi flight-controller link ...", flush=True)
    elif plan.mode == "ssh":
        print("Opening shell on the Pi ...", flush=True)

    _run(command, dry_run=plan.dry_run)
    return 0


def main() -> None:
    raise SystemExit(run())
