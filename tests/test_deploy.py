from __future__ import annotations

import json
import shlex
import shutil
import subprocess
import tarfile
import time
from functools import partial
from pathlib import Path

import pytest

from ai_drone.cli import deploy as deploy_pi
from ai_drone.link import deploy
from ai_drone.link.targets import DeployTarget

DEPLOYMENT_ID = "0123456789abcdef0123456789abcdef"


def _target(
    project_dir: str = "/home/seb/ai-drone", *, user: str = "seb"
) -> DeployTarget:
    return DeployTarget(
        ssh_target=f"{user}@drone",
        user=user,
        address="drone",
        project_dir=project_dir,
        ssh_config=None,
    )


def _plan(
    project_dir: str = "/home/seb/ai-drone",
    *,
    user: str = "seb",
    dry_run: bool = False,
) -> deploy.DeployPlan:
    return deploy.DeployPlan(
        target=_target(project_dir, user=user),
        dry_run=dry_run,
    )


def _write_file(root: Path, relative: str, content: str = "content\n") -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    return path


def _write_runtime_source(root: Path) -> None:
    _write_file(root, "README.md", "# ai-drone\n")
    _write_file(root, "drone.example.toml", "[runtime]\n")
    _write_file(root, "pyproject.toml", "[project]\nname = 'ai-drone'\n")
    _write_file(root, "uv.lock", "version = 1\n")
    _write_file(root, "ai_drone/__init__.py", "")
    _write_file(root, "ai_drone/runtime.py", "VALUE = 1\n")
    _write_file(root, "scripts/network.py", "# network helper\n")
    _write_file(root, "scripts/usb0-static.service", "[Service]\n")
    _write_file(root, "scripts/setup-pi-hotspot.sh", "#!/bin/sh\n")


def _write_deployment_metadata(root: Path, deployment_id: str = DEPLOYMENT_ID) -> None:
    sync_paths = deploy._iter_sync_paths(root)
    (root / deploy.MANIFEST_NAME).write_bytes(
        deploy._manifest_bytes(sync_paths, root, deployment_id)
    )
    (root / deploy.SENTINEL_NAME).write_text(
        f"{deploy.SENTINEL_PREFIX}{deployment_id}\n"
    )


@pytest.mark.parametrize(
    "project_dir",
    [
        "",
        ".",
        "ai-drone",
        "/",
        "/home",
        "/home/seb",
        "/home/other-user",
        "/home/other-user/ai-drone",
        "/home/seb/.config",
        "/home/seb/.ssh",
        "/root",
        "/root/ai-drone",
        "/root/.ssh",
        "/srv",
        "/var/lib",
        "/etc/ai-drone",
        "/usr/local/ai-drone",
        "/home/seb/../secrets",
        "/home//seb/ai-drone",
        "/home/seb/ai drone",
        "C:\\Users\\seb\\ai-drone",
    ],
)
def test_rejects_dangerous_remote_project_directories(project_dir: str) -> None:
    with pytest.raises(ValueError, match="PI_DIR"):
        deploy._validated_remote_project_dir(_target(project_dir))


@pytest.mark.parametrize(
    ("project_dir", "expected"),
    [
        ("/home/seb/ai-drone", "/home/seb/ai-drone"),
        ("/home/seb/projects/ai-drone/", "/home/seb/projects/ai-drone"),
        ("/srv/ai-drone", "/srv/ai-drone"),
        ("/opt/drone", "/opt/drone"),
        ("/var/lib/ai-drone", "/var/lib/ai-drone"),
        ("/workspace/ai-drone", "/workspace/ai-drone"),
    ],
)
def test_accepts_dedicated_remote_project_directories(
    project_dir: str, expected: str
) -> None:
    assert deploy._validated_remote_project_dir(_target(project_dir)) == expected


def test_root_may_use_a_dedicated_directory_below_root() -> None:
    target = _target("/root/ai-drone", user="root")

    assert deploy._validated_remote_project_dir(target) == "/root/ai-drone"


def test_invalid_remote_directory_is_rejected_before_sync(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    called = False

    def unexpected_run(*_args: object, **_kwargs: object) -> None:
        nonlocal called
        called = True

    monkeypatch.setattr(deploy.subprocess, "run", unexpected_run)

    with pytest.raises(ValueError, match="PI_DIR"):
        deploy._deploy_transaction(_plan("/home/seb"))

    assert called is False


def test_remote_preflight_checks_realpath_type_and_owner() -> None:
    command = deploy.remote_preflight_command(_plan())

    assert command[:2] == ["ssh", "seb@drone"]
    script = command[-1]
    assert 'realpath -m -- "$project_dir"' in script
    assert '[ "$resolved" != "$project_dir" ]' in script
    assert '[ -e "$project_dir" ] || [ -L "$project_dir" ]' in script
    assert '[ ! -d "$project_dir" ]' in script
    assert "id -u -- seb" in script
    assert 'stat -c %u -- "$project_dir"' in script
    assert '[ "$actual_uid" != "$expected_uid" ]' in script
    shell_command = shlex.split(script)
    assert shell_command[:2] == ["sh", "-c"]
    if shutil.which("sh") is not None:
        subprocess.run(["sh", "-n", "-c", shell_command[2]], check=True)


def test_rejects_unsafe_remote_user_before_building_shell_commands() -> None:
    target = DeployTarget(
        ssh_target="pilot@drone",
        user="pilot;touch-pwned",
        address="drone",
        project_dir="/home/pilot/ai-drone",
        ssh_config=None,
    )

    with pytest.raises(ValueError, match="user name"):
        deploy._validated_remote_project_dir(target)


def test_runtime_payload_is_allowlisted_and_omits_secrets(tmp_path: Path) -> None:
    _write_runtime_source(tmp_path)
    _write_file(tmp_path, ".env", "ROOT_SECRET=1\n")
    _write_file(tmp_path, ".env.production", "ROOT_SECRET=2\n")
    _write_file(tmp_path, "drone.toml", "local settings\n")
    _write_file(tmp_path, "ai_drone/.env", "PACKAGE_SECRET=1\n")
    _write_file(tmp_path, "ai_drone/.env.local", "PACKAGE_SECRET=2\n")
    _write_file(tmp_path, "ai_drone/__pycache__/runtime.pyc")
    _write_file(tmp_path, "docs/HANDOFF.md")
    _write_file(tmp_path, "tests/test_runtime.py")
    _write_file(tmp_path, "scripts/not-deployed.sh")
    _write_file(tmp_path, "requirements-raspi.txt")
    _write_file(tmp_path, "artifacts/recording.bin")

    paths = {
        path.relative_to(tmp_path).as_posix()
        for path in deploy._iter_sync_paths(tmp_path)
    }

    assert paths >= deploy.REQUIRED_RUNTIME_PATHS
    assert "ai_drone/runtime.py" in paths
    assert "drone.example.toml" in paths
    assert "scripts/network.py" in paths
    assert "scripts/usb0-static.service" in paths
    assert "scripts/setup-pi-hotspot.sh" in paths
    assert not any(".env" in part for path in paths for part in Path(path).parts)
    assert "ai_drone/__pycache__" not in paths
    assert "docs" not in paths
    assert "tests" not in paths
    assert "scripts/not-deployed.sh" not in paths
    assert "requirements-raspi.txt" not in paths
    assert "artifacts" not in paths
    assert "drone.toml" not in paths


def test_tar_archive_contains_only_runtime_payload_and_bound_metadata(
    tmp_path: Path,
) -> None:
    _write_runtime_source(tmp_path)
    _write_file(tmp_path, "ai_drone/.env.production", "SECRET=1\n")
    _write_file(tmp_path, "docs/private-notes.md")
    _write_file(tmp_path, "scripts/not-deployed.sh")

    archive = deploy._create_sync_archive(tmp_path, DEPLOYMENT_ID)
    try:
        with tarfile.open(archive, "r:gz") as tar:
            names = set(tar.getnames())
            manifest_member = tar.extractfile(deploy.MANIFEST_NAME)
            sentinel_member = tar.extractfile(deploy.SENTINEL_NAME)
            assert manifest_member is not None
            assert sentinel_member is not None
            manifest = json.loads(manifest_member.read())
            sentinel = sentinel_member.read().decode()
    finally:
        archive.unlink(missing_ok=True)

    assert names >= deploy.REQUIRED_RUNTIME_PATHS
    assert "scripts/network.py" in names
    assert "drone.example.toml" in names
    assert "scripts/usb0-static.service" in names
    assert "ai_drone/.env.production" not in names
    assert "docs/private-notes.md" not in names
    assert "scripts/not-deployed.sh" not in names
    assert manifest["format"] == deploy.MANIFEST_FORMAT
    assert manifest["deployment_id"] == DEPLOYMENT_ID
    assert set(manifest["paths"]) == names - {
        deploy.MANIFEST_NAME,
        deploy.SENTINEL_NAME,
    }
    assert sentinel == f"{deploy.SENTINEL_PREFIX}{DEPLOYMENT_ID}\n"


@pytest.mark.parametrize(
    "failure",
    [
        "missing_manifest",
        "missing_sentinel",
        "mismatched_manifest",
        "mismatched_sentinel",
        "malformed_manifest",
        "incomplete_manifest",
    ],
)
def test_staged_payload_refuses_invalid_or_missing_metadata(
    tmp_path: Path, failure: str
) -> None:
    _write_runtime_source(tmp_path)
    _write_deployment_metadata(tmp_path)
    stale = _write_file(tmp_path, "docs/must-survive.md")

    if failure == "missing_manifest":
        (tmp_path / deploy.MANIFEST_NAME).unlink()
    elif failure == "missing_sentinel":
        (tmp_path / deploy.SENTINEL_NAME).unlink()
    elif failure == "mismatched_sentinel":
        (tmp_path / deploy.SENTINEL_NAME).write_text(
            f"{deploy.SENTINEL_PREFIX}{'f' * 32}\n"
        )
    elif failure == "malformed_manifest":
        (tmp_path / deploy.MANIFEST_NAME).write_text("not json\n")
    else:
        manifest_path = tmp_path / deploy.MANIFEST_NAME
        manifest = json.loads(manifest_path.read_text())
        if failure == "mismatched_manifest":
            manifest["deployment_id"] = "f" * 32
        else:
            manifest["paths"].remove("pyproject.toml")
        manifest_path.write_text(json.dumps(manifest))

    with pytest.raises(deploy.DeploymentPayloadError):
        deploy._manifest_paths(tmp_path, DEPLOYMENT_ID)
    assert stale.is_file()
    assert (tmp_path / "ai_drone/runtime.py").is_file()


def test_staged_payload_rejects_unsafe_manifest_path(tmp_path: Path) -> None:
    _write_runtime_source(tmp_path)
    _write_deployment_metadata(tmp_path)
    stale = _write_file(tmp_path, "docs/must-survive.md")
    manifest_path = tmp_path / deploy.MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text())
    manifest["paths"].append("../outside")
    manifest_path.write_text(json.dumps(manifest))

    with pytest.raises(deploy.DeploymentPayloadError, match="unsafe path"):
        deploy._manifest_paths(tmp_path, DEPLOYMENT_ID)
    assert stale.is_file()


def test_repo_root_points_at_the_actual_repository() -> None:
    """REPO_ROOT is derived from module depth, so moving deploy.py must fail here."""
    assert (deploy.REPO_ROOT / "pyproject.toml").is_file()
    assert (deploy.REPO_ROOT / "ai_drone" / "__init__.py").is_file()
    assert Path(deploy.__file__).resolve().parents[1] == deploy.REPO_ROOT / "ai_drone"


@pytest.fixture
def maintenance_update(tmp_path, monkeypatch):
    from ai_drone import platform
    from ai_drone.cli import power
    from ai_drone.mavlink import remote

    source = tmp_path / "new-source"
    stage = tmp_path / "stage"
    project = tmp_path / "installed-old-version"
    _write_runtime_source(source)
    _write_file(source, "scripts/setup_runtime.py", "# new installer\n")
    stage.mkdir(mode=0o700)
    archive = deploy._create_sync_archive(source, DEPLOYMENT_ID)
    with tarfile.open(archive) as bundle:
        bundle.extractall(stage, filter="data")
    archive.unlink()
    _write_runtime_source(project)
    _write_file(project, "ai_drone/runtime.py", "OLD_VERSION = 1\n")
    _write_file(project, "legacy.py", "old source\n")
    _write_file(project, ".venv/lib/dependency.py", "old dependency\n")
    _write_file(project, ".venv/pyvenv.cfg", "old config\n")
    _write_file(project, ".env", "credentials\n")
    _write_file(project, "ai_drone/.env", "nested credentials\n")
    _write_file(project, "drone.toml", "per-machine configuration\n")
    _write_file(project, "artifacts/recording.bin", "camera recording\n")
    _write_file(project, "drone-music-currently-from-00m17s.wav", "tone recording\n")
    _write_file(project, "media/34_fuller_less_ambience.wav", "tone reference\n")
    _write_file(
        project, "drone-tone-handover-backup-20260909.tar.gz", "recovery archive\n"
    )
    _write_file(
        project, "drone-tone-handover-backup-20260909.sha256", "archive checksum\n"
    )
    _write_file(project, "state/2026-09-09/startup-tone.md", "ESC recovery evidence\n")
    runtime = {
        "status": "disarmed",
        "armed": False,
        "fresh": True,
        "source_known": True,
        "system_id": 1,
        "component_id": 1,
        "updated_monotonic": power.time.monotonic(),
        "heartbeat_age_s": 0.1,
        "control_client": None,
        "network_busy": False,
        "clients": 1,
        "maintenance": False,
        "closed": False,
        "error": None,
        "network_error": None,
        "socket": "/run/ai-drone/vehicle.sock",
    }
    state: dict = {
        "running": True,
        "events": [],
        "runtime": runtime,
        "project": project,
        "stage": stage,
    }

    def snapshot():
        state["events"].append("snapshot")
        return {
            "complete": True,
            "packages": [],
            "package_database_ok": True,
            "hardware": (
                [{"device": "/dev/ttyAMA0", "runtime": True}]
                if state["running"]
                else []
            ),
            "walk": {"ActiveState": "inactive"},
            "runtime": state["runtime"] if state["running"] else None,
        }

    def service(action):
        state["events"].append(action)
        state["running"] = action == "start"

    def request(_socket, request, **_kwargs):
        state["events"].append(request)
        return state["runtime"] if request == {"status": True} else request

    monkeypatch.setattr(platform, "is_raspberry_pi", lambda: True)
    monkeypatch.setattr(power, "_pi_snapshot", snapshot)
    monkeypatch.setattr(
        power,
        "_probe_fc",
        lambda _device: {"status": "disarmed", "heartbeat_age_s": 0.1},
    )
    monkeypatch.setattr(
        deploy_pi,
        "_runtime_service_state",
        lambda **_kwargs: "active" if state["running"] else "inactive",
    )
    monkeypatch.setattr(deploy_pi, "_runtime_service", service)
    monkeypatch.setattr(remote, "runtime_request", request)
    monkeypatch.setattr(
        deploy_pi,
        "_install_local",
        lambda project, **_: state["events"].append("install"),
    )
    return state


def apply_update(state):
    deploy_pi._apply_staged_update(
        state["stage"], state["project"], DEPLOYMENT_ID, offline=True
    )


def assert_local_state_preserved(project):
    assert (project / ".env").read_text() == "credentials\n"
    assert (project / "ai_drone/.env").read_text() == "nested credentials\n"
    assert (project / "drone.toml").read_text() == "per-machine configuration\n"
    assert (project / "artifacts/recording.bin").read_text() == "camera recording\n"
    assert (
        project / "drone-music-currently-from-00m17s.wav"
    ).read_text() == "tone recording\n"
    assert (
        project / "media/34_fuller_less_ambience.wav"
    ).read_text() == "tone reference\n"
    assert (
        project / "drone-tone-handover-backup-20260909.tar.gz"
    ).read_text() == "recovery archive\n"
    assert (
        project / "drone-tone-handover-backup-20260909.sha256"
    ).read_text() == "archive checksum\n"
    assert (
        project / "state/2026-09-09/startup-tone.md"
    ).read_text() == "ESC recovery evidence\n"


def test_transaction_updates_old_checkout_after_gate_and_restarts_runtime(
    maintenance_update,
):
    state = maintenance_update
    apply_update(state)
    project = state["project"]
    assert (project / "ai_drone/runtime.py").read_text() == "VALUE = 1\n"
    assert not (project / "legacy.py").exists()
    assert (project / "scripts/setup_runtime.py").is_file()
    assert_local_state_preserved(project)
    assert state["events"] == [
        "snapshot",
        {"maintenance": True},
        "stop",
        "snapshot",
        "snapshot",
        "install",
        "start",
        {"status": True},
    ]
    assert not list(project.parent.glob(".ai-drone-rollback-*"))


def test_failed_install_restores_source_environment_and_restarts_old_runtime(
    maintenance_update, monkeypatch
):
    state = maintenance_update

    def fail(project, **_):
        (project / ".venv/lib/dependency.py").unlink()
        _write_file(project, ".venv/lib/new-dependency.py", "partly installed\n")
        _write_file(project, ".venv/pyvenv.cfg", "new config\n")
        raise RuntimeError("dependency installation failed")

    monkeypatch.setattr(deploy_pi, "_install_local", fail)
    with pytest.raises(RuntimeError, match="dependency installation failed"):
        apply_update(state)
    project = state["project"]
    assert (project / "ai_drone/runtime.py").read_text() == "OLD_VERSION = 1\n"
    assert (project / "legacy.py").read_text() == "old source\n"
    assert (project / ".venv/lib/dependency.py").read_text() == "old dependency\n"
    assert (project / ".venv/pyvenv.cfg").read_text() == "old config\n"
    assert not (project / ".venv/lib/new-dependency.py").exists()
    assert_local_state_preserved(project)
    assert "start" in state["events"]
    assert {"maintenance": False} in state["events"]
    (backup,) = project.parent.glob(".ai-drone-rollback-*")
    assert backup.stat().st_mode & 0o777 == 0o700
    assert (backup / ".venv/lib/dependency.py").read_text() == "old dependency\n"
    assert not (backup / "drone.toml").exists()
    assert not (backup / "artifacts").exists()


@pytest.mark.parametrize(
    "change,reason",
    [
        ({"armed": True}, "armed"),
        ({"fresh": False}, "stale or unknown"),
        ({"clients": 2}, "runtime client"),
        ({"control_client": "owner"}, "control or network"),
        ({"network_busy": True}, "control or network"),
        ({"maintenance": True}, "existing maintenance"),
    ],
)
def test_transaction_blocks_unsafe_runtime_before_stop_or_source_mutation(
    maintenance_update, change, reason
):
    state = maintenance_update
    state["runtime"].update(change)
    with pytest.raises(RuntimeError, match=reason):
        apply_update(state)
    assert state["events"] == ["snapshot"]
    assert (state["project"] / "ai_drone/runtime.py").read_text() == "OLD_VERSION = 1\n"


def test_transaction_refuses_insufficient_rollback_space_before_maintenance(
    maintenance_update, monkeypatch
):
    state = maintenance_update
    monkeypatch.setattr(
        deploy_pi.shutil, "disk_usage", lambda _: shutil._ntuple_diskusage(1, 1, 0)
    )
    with pytest.raises(RuntimeError, match="insufficient free space"):
        apply_update(state)
    assert state["events"] == ["snapshot"]


def test_legacy_install_without_runtime_still_requires_fresh_disarmed_fc(
    maintenance_update, monkeypatch
):
    from ai_drone.cli import power

    state = maintenance_update
    state["running"] = False
    monkeypatch.setattr(power, "_probe_fc", lambda _: {"status": "unavailable"})
    with pytest.raises(RuntimeError, match="disarmed state"):
        apply_update(state)
    assert state["events"] == ["snapshot"]


def test_legacy_install_without_runtime_does_not_start_a_new_service(
    maintenance_update,
):
    state = maintenance_update
    state["running"] = False
    apply_update(state)
    assert state["events"] == ["snapshot", "snapshot", "snapshot", "install"]


def test_service_stop_failure_releases_latch_and_leaves_source_untouched(
    maintenance_update, monkeypatch
):
    state = maintenance_update

    def service(action):
        state["events"].append(action)
        if action == "stop":
            raise RuntimeError("stop failed")

    monkeypatch.setattr(deploy_pi, "_runtime_service", service)
    with pytest.raises(RuntimeError, match="stop failed"):
        apply_update(state)
    assert state["events"] == [
        "snapshot",
        {"maintenance": True},
        "stop",
        "start",
        {"status": True},
        {"maintenance": False},
    ]
    assert (state["project"] / "ai_drone/runtime.py").read_text() == "OLD_VERSION = 1\n"


def test_restart_failure_preserves_backup_for_recovery(maintenance_update, monkeypatch):
    state = maintenance_update

    def service(action):
        if action == "start":
            raise RuntimeError("restart failed")
        state["running"] = False

    monkeypatch.setattr(deploy_pi, "_runtime_service", service)
    with pytest.raises(RuntimeError, match="restart failed"):
        apply_update(state)
    (backup,) = state["project"].parent.glob(".ai-drone-rollback-*")
    assert (backup / "ai_drone/runtime.py").read_text() == "OLD_VERSION = 1\n"


def test_staged_deploy_bootstraps_uploaded_code_not_old_cwd_package(
    tmp_path, monkeypatch
):
    _write_runtime_source(tmp_path)
    calls = []
    monkeypatch.setattr(
        deploy.subprocess, "run", lambda command, **kwargs: calls.append(command)
    )
    deploy._deploy_transaction(_plan(), tmp_path)
    assert "realpath -m" in calls[0][-1]
    assert "umask 077; mkdir" in calls[1][-1]
    assert "tar -xzf" in calls[1][-1]
    bootstrap = calls[2][-1]
    assert "PYTHONPATH=/tmp/ai-drone-deploy-" in bootstrap
    assert "--python /home/seb/ai-drone/.venv/bin/python python -P -c" in bootstrap
    assert "_transaction_entry" in bootstrap
    assert "from ai_drone.cli.deploy import _transaction_entry" in bootstrap
    assert "--no-project --no-config --offline" in bootstrap


def test_transaction_preview_never_executes(tmp_path, monkeypatch, capsys):
    _write_runtime_source(tmp_path)
    monkeypatch.setattr(
        deploy.subprocess, "run", lambda *_args, **_: pytest.fail("executed")
    )
    deploy._deploy_transaction(_plan(dry_run=True), tmp_path)
    output = capsys.readouterr().out
    assert "verify disarmed idle state" in output
    assert "_transaction_entry" in output


@pytest.mark.parametrize("configured", [False, True])
def test_dependency_install_uses_uv_offline_and_preserves_pi_system_packages(
    tmp_path, monkeypatch, configured
):
    if configured:
        _write_file(
            tmp_path,
            ".venv/pyvenv.cfg",
            "include-system-site-packages = true\nversion_info = 3.13.5.final.0\n",
        )
    calls = []
    monkeypatch.setattr(deploy_pi.shutil, "which", lambda _: "/home/seb/.local/bin/uv")
    monkeypatch.setattr(
        deploy.subprocess,
        "run",
        lambda command, **kwargs: calls.append((command, kwargs)),
    )
    deploy_pi._install_local(tmp_path, offline=True)
    commands = [command for command, _kwargs in calls]
    assert commands[-2] == [
        "/home/seb/.local/bin/uv",
        "sync",
        "--locked",
        "--python",
        ".venv/bin/python",
        "--no-dev",
        "--group",
        "raspi",
        "--offline",
    ]
    assert all(kwargs == {"cwd": tmp_path, "check": True} for _, kwargs in calls)
    assert commands[-1][1:5] == ["run", "--no-project", "--no-config", "--offline"]
    assert commands[-1][6] == str(tmp_path / ".venv/bin/python")
    assert "Py_GIL_DISABLED" in commands[-1][-2]
    assert "picamera2, libcamera, pykms" in commands[-1][-2]
    assert commands[-1][-1] == "3.13"
    if configured:
        assert len(commands) == 2
    else:
        assert commands[0] == [
            "/home/seb/.local/bin/uv",
            "venv",
            "--clear",
            "--python",
            "/usr/bin/python3.13",
            "--system-site-packages",
            ".venv",
        ]


@pytest.mark.parametrize("version", ["3.13.5", "3.13.15.final.0"])
def test_deployment_preserves_separately_selected_interpreter(tmp_path, version):
    _write_file(
        tmp_path,
        ".venv/pyvenv.cfg",
        f"version = {version}\ninclude-system-site-packages = true\n",
    )
    _write_file(tmp_path, ".python-version", "3.14\n")
    assert deploy_pi._environment_interpreter(tmp_path) == (
        ".venv/bin/python",
        False,
        ".".join(version.split(".")[:2]),
    )


@pytest.mark.parametrize("system_packages", ["true", "false"])
@pytest.mark.parametrize("version", ["3.14.7", "3.14.7.final.0"])
def test_candidate_deployment_refuses_before_clearing_or_syncing_native_bindings(
    tmp_path, monkeypatch, system_packages, version
):
    config = (
        f"version_info = {version}\n"
        f"include-system-site-packages = {system_packages}\n"
        "executable = /candidate/interpreters/bin/python3.14\n"
    )
    _write_file(tmp_path, ".venv/pyvenv.cfg", config)
    native = ".venv/lib/python3.14/site-packages/libcamera/_libcamera.so"
    _write_file(tmp_path, native, "separately built native binding")
    calls = []
    monkeypatch.setattr(deploy_pi.shutil, "which", lambda _: "/usr/bin/uv")
    monkeypatch.setattr(
        deploy_pi.subprocess, "run", lambda *args, **kwargs: calls.append(args)
    )

    with pytest.raises(RuntimeError, match=r"Python 3\.14 deployment is blocked"):
        deploy_pi._install_local(tmp_path, offline=True)

    assert calls == []
    assert (tmp_path / ".venv/pyvenv.cfg").read_text() == config
    assert (tmp_path / native).read_text() == "separately built native binding"


def test_environment_repair_retains_explicit_base_interpreter(tmp_path):
    _write_file(
        tmp_path,
        ".venv/pyvenv.cfg",
        "version = 3.13.5\ninclude-system-site-packages = false\n"
        "executable = /usr/bin/python3.13\n",
    )
    assert deploy_pi._environment_interpreter(tmp_path) == (
        "/usr/bin/python3.13",
        True,
        "3.13",
    )


@pytest.mark.parametrize(
    "config",
    ["", "version = 3.15.0\n", "version = 3.13.5\n", "version = 3.14t.7\n"],
)
def test_unknown_environment_cannot_silently_change_python(tmp_path, config):
    _write_file(tmp_path, ".venv/pyvenv.cfg", config)
    with pytest.raises(RuntimeError, match=r"interpreter|Python"):
        deploy_pi._environment_interpreter(tmp_path)


def test_arming_during_backup_prevents_source_mutation(maintenance_update, monkeypatch):
    from ai_drone.cli import power

    state = maintenance_update
    observations = iter(
        [
            {"status": "disarmed", "heartbeat_age_s": 0.1},
            {"status": "armed", "heartbeat_age_s": 0.1},
        ]
    )
    monkeypatch.setattr(power, "_probe_fc", lambda _: next(observations))
    with pytest.raises(RuntimeError, match="disarmed state"):
        apply_update(state)
    assert "install" not in state["events"]
    assert "start" in state["events"]
    assert (state["project"] / "ai_drone/runtime.py").read_text() == "OLD_VERSION = 1\n"


def test_failed_rollback_keeps_runtime_stopped_and_preserves_backup(
    maintenance_update, monkeypatch
):
    state = maintenance_update
    copy = deploy_pi._copy_paths

    def copy_paths(source, destination, paths):
        if source.name.startswith(".ai-drone-rollback-"):
            raise OSError("restore failed")
        return copy(source, destination, paths)

    def install(*_args, **_kwargs):
        raise RuntimeError("installation failed")

    monkeypatch.setattr(deploy_pi, "_copy_paths", copy_paths)
    monkeypatch.setattr(deploy_pi, "_install_local", install)
    with pytest.raises(OSError, match="restore failed"):
        apply_update(state)
    assert "start" not in state["events"]
    (backup,) = state["project"].parent.glob(".ai-drone-rollback-*")
    assert (backup / "ai_drone/runtime.py").read_text() == "OLD_VERSION = 1\n"


@pytest.mark.parametrize(
    "failure",
    [
        {"fresh": False},
        {"source_known": False},
        {"component_id": 2},
        {"closed": True},
        {"error": "FC reader stopped"},
        {"network_error": "policy initialization failed"},
        {"heartbeat_age_s": 3},
        {"heartbeat_age_s": -0.1},
        {"heartbeat_age_s": float("nan")},
        {"heartbeat_age_s": True},
        {"updated_monotonic": float("inf")},
        {"updated_monotonic": -100},
    ],
)
def test_unhealthy_restart_preserves_backup_and_never_stops_new_clients(
    maintenance_update, monkeypatch, failure
):
    state = maintenance_update
    service = deploy_pi._runtime_service

    def restart(action):
        service(action)
        if action == "start":
            # Type=simple start succeeded, but its new process is not healthy.
            state["runtime"].update(failure)
            state["runtime"]["clients"] = 4
            state["runtime"]["control_client"] = "new-control-owner"

    monkeypatch.setattr(deploy_pi, "_runtime_service", restart)
    monkeypatch.setattr(
        deploy_pi,
        "_wait_runtime_ready",
        partial(deploy_pi._wait_runtime_ready, timeout=0.01),
    )
    with pytest.raises(RuntimeError, match="did not report healthy FC telemetry"):
        apply_update(state)
    assert state["events"].count("stop") == 1
    assert state["events"].count("start") == 1
    assert state["running"] is True
    assert (state["project"] / "ai_drone/runtime.py").read_text() == "VALUE = 1\n"
    (backup,) = state["project"].parent.glob(".ai-drone-rollback-*")
    assert backup.stat().st_mode & 0o777 == 0o700
    assert (backup / "ai_drone/runtime.py").read_text() == "OLD_VERSION = 1\n"
    assert (backup / ".venv/lib/dependency.py").read_text() == "old dependency\n"


def test_ready_restart_accepts_new_armed_controller_without_operator(
    maintenance_update, monkeypatch
):
    state = maintenance_update
    service = deploy_pi._runtime_service

    def restart(action):
        service(action)
        if action == "start":
            state["runtime"].update(
                armed=True,
                status="armed",
                clients=4,
                control_client="new-control-owner",
                operator_alive=False,
            )

    monkeypatch.setattr(deploy_pi, "_runtime_service", restart)
    apply_update(state)
    assert state["events"].count("stop") == 1
    assert state["events"].count("start") == 1
    assert not list(state["project"].parent.glob(".ai-drone-rollback-*"))


def test_restart_readiness_waits_for_rpc_and_active_service(
    maintenance_update, monkeypatch
):
    from ai_drone.mavlink import remote

    state = maintenance_update
    requests = iter(
        [ConnectionRefusedError("booting"), state["runtime"], state["runtime"]]
    )
    services = iter(["inactive", "active"])

    def status(_socket, request, *, timeout):
        assert request == {"status": True}
        assert 0 < timeout <= 1
        reply = next(requests)
        if isinstance(reply, Exception):
            raise reply
        return {**reply, "updated_monotonic": time.monotonic()}

    monkeypatch.setattr(remote, "runtime_request", status)
    monkeypatch.setattr(deploy_pi, "_runtime_service_state", lambda **_: next(services))
    deploy_pi._wait_runtime_ready(state["runtime"]["socket"], timeout=1)
    assert next(requests, None) is None
    assert next(services, None) is None


def test_restart_readiness_rejects_a_stale_status_even_with_fresh_flag(
    maintenance_update, monkeypatch
):
    from ai_drone.mavlink import remote

    status = {
        **maintenance_update["runtime"],
        "updated_monotonic": time.monotonic() - 1.8,
        "heartbeat_age_s": 0.3,
    }
    monkeypatch.setattr(remote, "runtime_request", lambda *_args, **_kwargs: status)
    with pytest.raises(RuntimeError, match="did not report healthy FC telemetry"):
        deploy_pi._wait_runtime_ready("unused", timeout=0.01)
