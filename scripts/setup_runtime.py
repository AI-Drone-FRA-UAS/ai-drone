"""Review, install, or revert the Pi's shared FC service without reconnecting Wi-Fi."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import time
import uuid
from contextlib import suppress
from dataclasses import asdict, dataclass
from pathlib import Path

from ai_drone.durability import atomic_write_text, fsync_directory
from ai_drone.mavlink.remote import runtime_request
from ai_drone.network import read_link, read_profiles
from ai_drone.platform import is_raspberry_pi
from ai_drone.runtime_status import RuntimeStatus
from ai_drone.settings import Settings, load_settings
from ai_drone.system import unit_state

UNIT = "ai-drone-runtime.service"
LEGACY = "ai-drone-network.service"
UNIT_PATH = Path("/etc/systemd/system") / UNIT
LOCKS_CONFIG = Path("/etc/tmpfiles.d/ai-drone-locks.conf")
LOCKS_DIRECTORY = Path("/run/ai-drone-locks")
BACKUPS = Path("/var/backups/ai-drone")
NM_PATHS = (
    Path("/etc/NetworkManager/system-connections"),
    Path("/run/NetworkManager/system-connections"),
    Path("/usr/lib/NetworkManager/system-connections"),
)


@dataclass(frozen=True)
class Plan:
    project: Path
    config: Path
    user: str
    uv: Path
    settings: Settings


def _absolute(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute() or any(char in value for char in "\0\r\n"):
        raise argparse.ArgumentTypeError(
            "an absolute path without newlines is required"
        )
    return path


def _quote(value: str | Path) -> str:
    return (
        '"'
        + str(value).replace("\\", "\\\\").replace('"', '\\"').replace("%", "%%")
        + '"'
    )


def unit_text(plan: Plan) -> str:
    command = [
        str(plan.uv),
        "run",
        "--no-sync",
        "--python",
        str(plan.project / ".venv/bin/python"),
        "python",
        "-m",
        "ai_drone.cli.main",
        "--config",
        str(plan.config),
        "runtime",
        "serve",
    ]
    return "\n".join(
        (
            "# Managed by scripts/setup_runtime.py; backups are in /var/backups/ai-drone.",
            "[Unit]",
            "Description=Shared drone FC telemetry and grounded Wi-Fi access",
            "After=NetworkManager.service",
            "Wants=NetworkManager.service",
            "",
            "[Service]",
            "Type=simple",
            f"User={plan.user}",
            f"WorkingDirectory={_quote(plan.project)}",
            "RuntimeDirectory=ai-drone",
            "RuntimeDirectoryMode=0700",
            "UMask=0077",
            "ExecStart="
            + " ".join(_quote(value).replace("$", "$$") for value in command),
            "Restart=on-failure",
            "RestartSec=3",
            "TimeoutStopSec=15",
            "",
            "[Install]",
            "WantedBy=multi-user.target",
            "",
        )
    )


def run(command: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
        env={**os.environ, "LC_ALL": "C"},
    )
    if check and result.returncode:
        # Do not print subprocess output: NetworkManager profiles may contain secrets.
        raise RuntimeError(
            f"{command[0]} {command[1]} failed (exit {result.returncode})"
        )
    return result


def _nmcli(arguments: list[str]) -> str:
    return run(["nmcli", *arguments]).stdout.strip()


def _service_state(unit: str) -> dict[str, str]:
    result = run(
        ["systemctl", "show", unit, "--property=LoadState,ActiveState,UnitFileState"],
        check=False,
    )
    state = unit_state(result)
    if state.load == "not-found":
        return {"load": "not-found", "active": "inactive", "enabled": "disabled"}
    if (
        result.returncode
        or state.load != "loaded"
        or state.active not in {"active", "inactive", "failed"}
        or state.enabled not in {"enabled", "disabled", "static"}
    ):
        raise RuntimeError(
            f"cannot safely inspect {unit}; masked or changing units require manual review"
        )
    return {
        "load": state.load,
        "active": state.active,
        "enabled": state.enabled,
    }


def _fresh_disarmed(status: dict) -> None:
    if not RuntimeStatus.parse(status).disarmed_at_receipt():
        raise RuntimeError("fresh selected-FC disarmed telemetry is required")
    if status.get("control_client") or status.get("network_busy"):
        raise RuntimeError(
            "control or network operation is active; setup requires an idle runtime"
        )
    if type(status.get("clients")) is not int or status["clients"] != 1:
        raise RuntimeError("recording or another runtime client is active")


def require_disarmed(plan: Plan) -> None:
    """Use an existing owner, otherwise receive passively from a confirmed idle UART."""
    if (
        Path(plan.settings.runtime.socket).exists()
        or _service_state(UNIT)["active"] == "active"
    ):
        _fresh_disarmed(runtime_request(plan.settings.runtime.socket, {"status": True}))
        return
    from ai_drone.mavlink.connection import open_ardupilot_connection
    from ai_drone.mavlink.devices import is_network_endpoint
    from ai_drone.mavlink.ownership import require_available_serial
    from ai_drone.mavlink.safety import heartbeat_is_armed, require_ardupilot_heartbeat

    runtime = plan.settings.runtime
    if is_network_endpoint(runtime.device):
        raise RuntimeError(
            "installation requires the physical Pi FC, not a network endpoint"
        )
    require_available_serial(runtime.device, on_pi=True)
    connection = open_ardupilot_connection(runtime.device, baud=runtime.baud)
    try:
        for _ in range(2):
            heartbeat = require_ardupilot_heartbeat(
                connection, system_id=1, component_id=1, timeout=3
            )
            if heartbeat_is_armed(heartbeat):
                raise RuntimeError("FC is armed; setup refused")
    finally:
        connection.close()


def eligibility(profiles: dict, previous: dict | None) -> dict:
    old = {} if previous is None else previous.get("profiles", {})
    if previous is not None and (
        previous.get("schema") != 1
        or not isinstance(old, dict)
        or any(
            not isinstance(value, dict)
            or type(value.get("autoconnect")) is not bool
            or value.get("mode") not in {"", "infrastructure", "ap", "adhoc", "mesh"}
            for value in old.values()
        )
    ):
        raise ValueError("existing network eligibility manifest is invalid")
    merged = {
        identifier: {
            **profile,
            "autoconnect": profile["autoconnect"]
            or (
                old.get(identifier, {}).get("autoconnect") is True
                and (old[identifier].get("mode") or "infrastructure")
                == (profile["mode"] or "infrastructure")
            ),
        }
        for identifier, profile in profiles.items()
    }
    if not any(
        profile["autoconnect"] and profile["mode"] in {"", "infrastructure"}
        for profile in merged.values()
    ):
        raise ValueError("at least one saved Wi-Fi client must be eligible for startup")
    return {"schema": 1, "profiles": merged}


def _profiles() -> dict:
    return {
        profile.uuid: {
            key: value
            for key, value in asdict(profile).items()
            if key not in {"uuid", "reachable"}
        }
        for profile in read_profiles(run=_nmcli)
    }


def _write(path: Path, content: str, mode: int) -> None:
    if path.is_symlink():
        raise ValueError(f"refusing symlink destination: {path}")
    atomic_write_text(path, content)
    path.chmod(mode)
    with path.open("rb") as handle:
        os.fsync(handle.fileno())


def _private_directory(path: Path, *, create: bool = True) -> None:
    if create:
        path.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = path.lstat()
    if (
        not stat.S_ISDIR(info.st_mode)
        or info.st_uid != 0
        or stat.S_IMODE(info.st_mode) != 0o700
    ):
        raise PermissionError(f"private root-owned directory required: {path}")


def _backup_file(path: Path, destination: Path) -> dict:
    if path.is_symlink():
        raise ValueError(f"refusing symlink file: {path}")
    if not path.exists():
        return {"path": str(path), "exists": False}
    info = path.stat()
    if not stat.S_ISREG(info.st_mode):
        raise ValueError(f"regular file required: {path}")
    destination.write_bytes(path.read_bytes())
    destination.chmod(0o600)
    return {
        "path": str(path),
        "exists": True,
        "mode": stat.S_IMODE(info.st_mode),
        "uid": info.st_uid,
        "gid": info.st_gid,
    }


def backup(plan: Plan, profiles: dict, states: dict) -> tuple[Path, dict]:
    _private_directory(BACKUPS)
    directory = BACKUPS / (time.strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:8])
    _private_directory(directory)
    metadata = {
        "schema": 1,
        "project": str(plan.project),
        "config": str(plan.config),
        "user": plan.user,
        "profiles": profiles,
        "services": states,
        "files": {
            "runtime.service": _backup_file(UNIT_PATH, directory / "runtime.service"),
            "locks.conf": _backup_file(LOCKS_CONFIG, directory / "locks.conf"),
            "profiles.json": _backup_file(
                Path(plan.settings.runtime.network_profiles),
                directory / "profiles.json",
            ),
        },
        "device_autoconnect": _nmcli(
            ["-g", "GENERAL.AUTOCONNECT", "device", "show", "wlan0"]
        ),
    }
    if metadata["device_autoconnect"] not in {"yes", "no"}:
        raise ValueError("cannot inspect Wi-Fi device autoconnect")
    # Full keyfiles are private recovery evidence; rollback never rewrites credentials.
    for index, source in enumerate(NM_PATHS):
        if source.exists():
            if source.is_symlink() or any(
                path.is_symlink() for path in source.rglob("*")
            ):
                raise ValueError(
                    "NetworkManager keyfile snapshots cannot contain symlinks"
                )
            shutil.copytree(source, directory / f"nm-profiles-{index}")
    _write(directory / "backup.json", json.dumps(metadata, indent=2) + "\n", 0o600)
    for path in directory.rglob("*"):
        path.chmod(0o700 if path.is_dir() else 0o600)
        if path.is_file():
            with path.open("rb") as handle:
                os.fsync(handle.fileno())
    for path in sorted(
        (path for path in directory.rglob("*") if path.is_dir()),
        key=lambda path: len(path.parts),
        reverse=True,
    ):
        fsync_directory(path)
    fsync_directory(directory)
    fsync_directory(BACKUPS)
    return directory, metadata


def restore(directory: Path, metadata: dict) -> None:
    """Restore only flags and our files; never activate a network or reload keyfiles."""
    errors = []

    def attempt(function, *args):
        try:
            function(*args)
        except Exception as error:
            errors.append(str(error))

    current = _service_state(UNIT)
    if current["active"] == "active":
        run(["systemctl", "stop", UNIT])
    if current["load"] != "not-found":
        attempt(run, ["systemctl", "disable", UNIT])
    for identifier, profile in metadata["profiles"].items():
        attempt(
            _nmcli,
            [
                "connection",
                "modify",
                "uuid",
                identifier,
                "connection.autoconnect",
                "yes" if profile["autoconnect"] else "no",
            ],
        )
    for name, info in metadata["files"].items():
        path = Path(info["path"])
        if info["exists"]:
            attempt(_restore_file, path, directory / name, info)
        else:
            attempt(path.unlink, True)
    attempt(run, ["systemctl", "daemon-reload"])
    _restore_services(metadata["services"], attempt)
    attempt(
        _nmcli,
        ["device", "set", "wlan0", "autoconnect", metadata["device_autoconnect"]],
    )
    if errors:
        raise RuntimeError(
            f"rollback incomplete; retain {directory}: {'; '.join(errors)}"
        )


def _restore_services(states: dict, attempt) -> None:
    """Restore enablement and only restart the previously active shared runtime."""
    for unit, state in states.items():
        if state["load"] != "not-found" and state["enabled"] != "static":
            attempt(
                run,
                [
                    "systemctl",
                    "enable" if state["enabled"] == "enabled" else "disable",
                    unit,
                ],
            )
        if unit == UNIT and state["active"] == "active":
            attempt(run, ["systemctl", "start", unit])


def _restore_file(path: Path, source: Path, info: dict) -> None:
    _write(path, source.read_text(), info["mode"])
    os.chown(path, info["uid"], info["gid"])


def safe_restore(plan: Plan, directory: Path, metadata: dict) -> None:
    """Recheck ownership after startup: a new client may have begun meanwhile."""
    active = _service_state(UNIT)["active"] == "active"
    if Path(plan.settings.runtime.socket).exists() and not active:
        raise RuntimeError(
            "stop the manually launched runtime before reverting its service"
        )
    if active:
        runtime_request(plan.settings.runtime.socket, {"maintenance": True})
        try:
            run(["systemctl", "stop", UNIT])
        except BaseException:
            with suppress(Exception):
                runtime_request(plan.settings.runtime.socket, {"maintenance": False})
            raise
    else:
        require_disarmed(plan)
    restore(directory, metadata)


def _verify(plan: Plan, active: str) -> None:
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        try:
            status = runtime_request(
                plan.settings.runtime.socket, {"status": True}, timeout=2
            )
            _fresh_disarmed(status)
        except (OSError, RuntimeError, ValueError):
            time.sleep(0.2)
            continue
        if read_link(run=_nmcli) != (active, True, False):
            raise RuntimeError("active Wi-Fi client changed during installation")
        if status.get("network_error"):
            raise RuntimeError("runtime started with a network policy error")
        return
    raise RuntimeError("new runtime did not report fresh disarmed telemetry")


def _write_network_policy(manifest: Path, profiles: dict, selected: dict) -> None:
    """Disable saved-profile autoconnect, retaining original eligibility on disk."""
    for identifier in profiles:
        _nmcli(
            [
                "connection",
                "modify",
                "uuid",
                identifier,
                "connection.autoconnect",
                "no",
            ]
        )
    if not manifest.parent.exists():
        manifest.parent.mkdir(parents=True, mode=0o755)
        manifest.parent.chmod(0o755)
    _write(manifest, json.dumps(selected, indent=2) + "\n", 0o644)


def _account_ids(user: str) -> tuple[int, int]:
    import pwd

    account = pwd.getpwnam(user)
    return account.pw_uid, account.pw_gid


def provision_locks(plan: Plan) -> None:
    """Keep GPIO exclusion in one boot-lifetime directory, independent of service restarts."""
    uid, gid = _account_ids(plan.user)
    if LOCKS_DIRECTORY.exists() or LOCKS_DIRECTORY.is_symlink():
        current = LOCKS_DIRECTORY.lstat()
        if (
            not stat.S_ISDIR(current.st_mode)
            or current.st_uid != uid
            or current.st_gid != gid
            or stat.S_IMODE(current.st_mode) != 0o700
        ):
            raise PermissionError(
                "servo lock directory has unexpected ownership or mode"
            )
    _write(
        LOCKS_CONFIG,
        f"# GPIO exclusion survives ai-drone-runtime restarts.\nd {LOCKS_DIRECTORY} 0700 {uid} {gid} -\n",
        0o644,
    )
    run(["systemd-tmpfiles", "--create", str(LOCKS_CONFIG)])


def install(plan: Plan) -> Path:
    require_disarmed(plan)
    profiles = _profiles()
    active, connected, activating = read_link(run=_nmcli)
    if (
        not connected
        or activating
        or active is None
        or active not in profiles
        or profiles[active]["mode"] not in {"", "infrastructure"}
    ):
        raise RuntimeError("keep a saved Wi-Fi client connected before installation")
    manifest = Path(plan.settings.runtime.network_profiles)
    previous = json.loads(manifest.read_text()) if manifest.exists() else None
    selected = eligibility(profiles, previous)
    states = {unit: _service_state(unit) for unit in (UNIT, LEGACY)}
    if (
        Path(plan.settings.runtime.socket).exists()
        and states[UNIT]["active"] != "active"
    ):
        raise RuntimeError(
            "stop the manually launched runtime before installing its service"
        )
    directory, metadata = backup(plan, profiles, states)
    print(f"Private backup: {directory}", flush=True)
    maintenance = states[UNIT]["active"] == "active"
    if maintenance:
        runtime_request(plan.settings.runtime.socket, {"maintenance": True})
    else:
        require_disarmed(plan)
    try:
        if maintenance:
            run(["systemctl", "stop", UNIT])
            maintenance = False
        # The legacy oneshot has no ExecStop; disabling it leaves the current route intact.
        if states[LEGACY]["load"] != "not-found":
            run(["systemctl", "disable", LEGACY])
        _write_network_policy(manifest, profiles, selected)
        provision_locks(plan)
        _write(UNIT_PATH, unit_text(plan), 0o644)
        run(["systemctl", "daemon-reload"])
        run(["systemctl", "enable", UNIT])
        run(["systemctl", "start", UNIT])
        _verify(plan, active)
    except BaseException as error:
        if maintenance:
            with suppress(Exception):
                runtime_request(plan.settings.runtime.socket, {"maintenance": False})
            # Stopping the old owner failed before flags/files changed. Do not
            # continue modifying its network policy while it may still be running.
            raise
        try:
            safe_restore(plan, directory, metadata)
        except Exception as rollback_error:
            raise RuntimeError(
                f"installation failed ({error}); rollback withheld or incomplete, retain {directory}: {rollback_error}"
            ) from rollback_error
        raise
    return directory


def _validate_apply(plan: Plan) -> None:
    if os.geteuid() != 0 or not is_raspberry_pi():
        raise RuntimeError("apply/revert requires root on the Raspberry Pi")
    import pwd

    account = pwd.getpwnam(plan.user)
    if account.pw_uid == 0:
        raise ValueError("runtime must use a normal user")
    if (
        not plan.project.is_dir()
        or not plan.uv.is_file()
        or not os.access(plan.uv, os.X_OK)
    ):
        raise ValueError("project directory and executable uv must already exist")
    for path in (plan.config, plan.project / "pyproject.toml"):
        if not path.is_file():
            raise ValueError(f"required file is missing: {path}")
    runtime = plan.settings.runtime
    if any(
        Path(path).parent != Path("/run/ai-drone")
        for path in (runtime.socket, runtime.status)
    ):
        raise ValueError("installed runtime socket/status must be inside /run/ai-drone")
    if not Path(runtime.network_profiles).is_absolute():
        raise ValueError("runtime.network_profiles must be absolute")
    _validate_service_access(plan, Path(account.pw_dir))


def _validate_service_access(plan: Plan, home: Path) -> None:
    """Check files and the service entry point as its normal operating user."""
    # Fail before mutating if normal service execution cannot read its code/config.
    run(["runuser", "-u", plan.user, "--", "test", "-r", str(plan.config)])
    run(["runuser", "-u", plan.user, "--", "test", "-x", str(plan.uv)])
    parent = Path(plan.settings.runtime.network_profiles).parent
    while not parent.exists():
        parent = parent.parent
    run(["runuser", "-u", plan.user, "--", "test", "-x", str(parent)])
    if token_name := plan.settings.operator.token_file:
        from ai_drone.operator import read_token

        token = (
            home / token_name[2:]
            if token_name.startswith("~/")
            else Path(token_name).expanduser()
        )
        if not token.is_absolute():
            token = plan.project / token
        read_token(token)
        run(["runuser", "-u", plan.user, "--", "test", "-r", str(token)])
    run(
        [
            "runuser",
            "-u",
            plan.user,
            "--",
            str(plan.uv),
            "run",
            "--no-sync",
            "--directory",
            str(plan.project),
            "--python",
            str(plan.project / ".venv/bin/python"),
            "python",
            "-m",
            "ai_drone.cli.main",
            "--config",
            str(plan.config),
            "runtime",
            "--help",
        ]
    )


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=_absolute, required=True)
    parser.add_argument("--config", type=_absolute, required=True)
    parser.add_argument("--user", default="seb")
    parser.add_argument(
        "--uv", type=_absolute, help="uv executable (default /home/USER/.local/bin/uv)"
    )
    parser.add_argument(
        "--apply", action="store_true", help="perform the displayed install or rollback"
    )
    parser.add_argument(
        "--revert", type=_absolute, help="private backup directory to restore"
    )
    args = parser.parse_args(arguments)
    try:
        if not re.fullmatch(r"[a-z_][a-z0-9_-]*", args.user):
            raise ValueError("invalid normal service user")
        plan = Plan(
            args.project,
            args.config,
            args.user,
            args.uv or Path(f"/home/{args.user}/.local/bin/uv"),
            load_settings(args.config),
        )
        if not args.apply:
            print(
                f"DRY RUN: {'restore ' + str(args.revert) if args.revert else 'install ' + UNIT}"
            )
            print(
                "Require idle, fresh disarmed FC; back up privately; preserve the connected Wi-Fi client."
            )
            print(
                "Restore original Wi-Fi startup flags, service files, and enablement."
                if args.revert
                else "Disable saved Wi-Fi autoconnect and legacy boot selector; service chooses saved clients only while disarmed."
            )
            print(
                "No flight, recording, AP activation, disconnect, or reboot is requested. Add --apply to execute."
            )
            if not args.revert:
                print(
                    f"Provision {LOCKS_DIRECTORY} via {LOCKS_CONFIG}; keep GPIO lock files across service restarts."
                )
                print(unit_text(plan))
            return 0
        _validate_apply(plan)
        if args.revert:
            _private_directory(args.revert, create=False)
            metadata = json.loads((args.revert / "backup.json").read_text())
            if metadata.get("schema") != 1 or (
                metadata.get("project"),
                metadata.get("config"),
                metadata.get("user"),
            ) != (str(plan.project), str(plan.config), plan.user):
                raise ValueError(
                    "backup does not match this project, configuration, and user"
                )
            expected = {str(UNIT_PATH), plan.settings.runtime.network_profiles}
            expected_names = {"runtime.service", "profiles.json"}
            # Older restoration bundles predate independent servo-lock provisioning.
            if "locks.conf" in metadata["files"]:
                expected.add(str(LOCKS_CONFIG))
                expected_names.add("locks.conf")
            if (
                set(metadata["files"]) != expected_names
                or {info["path"] for info in metadata["files"].values()} != expected
                or set(metadata["services"]) != {UNIT, LEGACY}
            ):
                raise ValueError("backup contains unexpected service files")
            safe_restore(plan, args.revert, metadata)
            print(f"Restored flags and service files from {args.revert}")
        else:
            directory = install(plan)
            print(
                f"Installed {UNIT}; active Wi-Fi retained. Revert using --revert {directory} --apply."
            )
        return 0
    except (
        OSError,
        ValueError,
        RuntimeError,
        KeyError,
        subprocess.SubprocessError,
    ) as error:
        print(f"Runtime setup failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
