from __future__ import annotations

import json
import shlex
import shutil
import subprocess
import tarfile
from pathlib import Path

import pytest

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
    sync_method: str = "rsync",
    dry_run: bool = False,
) -> deploy.DeployPlan:
    return deploy.DeployPlan(
        target=_target(project_dir, user=user),
        mode=None,
        extra_args=(),
        dry_run=dry_run,
        sync_method=sync_method,
    )


def _write_file(root: Path, relative: str, content: str = "content\n") -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    return path


def _write_runtime_source(root: Path) -> None:
    _write_file(root, "README.md", "# ai-drone\n")
    _write_file(root, "pyproject.toml", "[project]\nname = 'ai-drone'\n")
    _write_file(root, "uv.lock", "version = 1\n")
    _write_file(root, "ai_drone/__init__.py", "")
    _write_file(root, "ai_drone/runtime.py", "VALUE = 1\n")
    _write_file(root, "scripts/setup-pi-dual-network.sh", "#!/bin/sh\n")
    _write_file(root, "scripts/setup-pi-hotspot.sh", "#!/bin/sh\n")


def _write_deployment_metadata(root: Path, deployment_id: str = DEPLOYMENT_ID) -> None:
    sync_paths = deploy._iter_sync_paths(root)
    (root / deploy.MANIFEST_NAME).write_bytes(
        deploy._manifest_bytes(sync_paths, root, deployment_id)
    )
    (root / deploy.SENTINEL_NAME).write_text(
        f"{deploy.SENTINEL_PREFIX}{deployment_id}\n"
    )


def _run_cleanup(root: Path, deployment_id: str = DEPLOYMENT_ID) -> int:
    return deploy._cleanup_entry(deployment_id, root)


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
        deploy.sync_project(_plan("/home/seb"))

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


@pytest.mark.parametrize("sync_method", ["rsync", "tar"])
def test_remote_preflight_runs_before_sync_without_network(
    tmp_path: Path, sync_method: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_runtime_source(tmp_path)
    calls: list[list[str]] = []

    def fake_run(
        command: list[str], **_kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        calls.append(list(command))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(deploy.subprocess, "run", fake_run)

    deploy.sync_project(_plan(sync_method=sync_method), tmp_path)

    assert calls[0][:2] == ["ssh", "seb@drone"]
    assert "realpath -m" in calls[0][-1]
    if sync_method == "rsync":
        assert calls[1][0] == "rsync"
    else:
        assert "tar -xzf" in calls[1][-1]
        assert "_cleanup_entry" in calls[2][-1]


@pytest.mark.parametrize(
    ("sync_method", "transport_marker"),
    [("rsync", "rsync -az"), ("tar", "tar -xzf")],
)
def test_dry_run_shows_preflight_before_transport(
    tmp_path: Path,
    sync_method: str,
    transport_marker: str,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    _write_runtime_source(tmp_path)

    def unexpected_run(*_args: object, **_kwargs: object) -> None:
        pytest.fail("dry-run must not execute subprocesses")

    monkeypatch.setattr(deploy.subprocess, "run", unexpected_run)

    deploy.sync_project(
        _plan(sync_method=sync_method, dry_run=True),
        tmp_path,
    )

    output = capsys.readouterr().out
    assert output.index("realpath -m") < output.index(transport_marker)


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


def test_root_install_uses_root_home_for_uv() -> None:
    target = DeployTarget(
        ssh_target="root@drone",
        user="root",
        address="drone",
        project_dir="/root/ai-drone",
        ssh_config=None,
    )
    plan = deploy.DeployPlan(target, None, (), False, "tar")

    assert "PATH=/root/.local/bin:$PATH" in deploy.install_command(plan)[-1]


def test_runtime_payload_is_allowlisted_and_omits_secrets(tmp_path: Path) -> None:
    _write_runtime_source(tmp_path)
    _write_file(tmp_path, ".env", "ROOT_SECRET=1\n")
    _write_file(tmp_path, ".env.production", "ROOT_SECRET=2\n")
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
    assert "scripts/setup-pi-dual-network.sh" in paths
    assert "scripts/setup-pi-hotspot.sh" in paths
    assert not any(".env" in part for path in paths for part in Path(path).parts)
    assert "ai_drone/__pycache__" not in paths
    assert "docs" not in paths
    assert "tests" not in paths
    assert "scripts/not-deployed.sh" not in paths
    assert "requirements-raspi.txt" not in paths
    assert "artifacts" not in paths


def test_rsync_uses_the_same_allowlist_and_protects_remote_state(
    tmp_path: Path,
) -> None:
    command = deploy.rsync_command(_plan(), tmp_path)
    rules = [
        command[index + 1]
        for index, argument in enumerate(command[:-1])
        if argument == "--filter"
    ]

    assert command[:4] == ["rsync", "-az", "--delete", "-e"]
    assert "protect artifacts" in rules
    assert "protect .venv" in rules
    assert "protect .git" in rules
    assert "protect .env" in rules
    assert "protect .env.*" in rules
    assert "hide .env" in rules
    assert "hide .env.*" in rules
    assert "show /ai_drone/***" in rules
    assert "show /pyproject.toml" in rules
    assert "show /uv.lock" in rules
    assert "show /scripts/setup-pi-dual-network.sh" in rules
    assert rules[-1] == "hide /***"
    assert command[-1] == "seb@drone:/home/seb/ai-drone/"


@pytest.mark.skipif(shutil.which("rsync") is None, reason="rsync is not installed")
def test_rsync_filters_delete_nonruntime_files_but_preserve_remote_state(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source"
    destination = tmp_path / "destination"
    _write_runtime_source(source)
    _write_file(source, ".env", "SOURCE_SECRET=1\n")
    _write_file(source, "ai_drone/.env.local", "SOURCE_SECRET=2\n")
    _write_file(source, "docs/not-deployed.md")
    _write_file(source, "scripts/not-deployed.sh")

    stale_paths = [
        _write_file(destination, "docs/stale.md"),
        _write_file(destination, "scripts/stale.sh"),
        _write_file(destination, "ai_drone/stale.py"),
    ]
    preserved_paths = [
        _write_file(destination, ".env", "REMOTE_SECRET=1\n"),
        _write_file(destination, "ai_drone/.env.local", "REMOTE_SECRET=2\n"),
        _write_file(destination, ".venv/pyvenv.cfg"),
        _write_file(destination, ".git/config"),
        _write_file(destination, "artifacts/recording.bin"),
        _write_file(destination, "ai_drone/__pycache__/runtime.pyc"),
    ]
    command = ["rsync", "-a", "--delete"]
    for rule in deploy._rsync_filter_rules():
        command.extend(["--filter", rule])
    command.extend([f"{source}/", f"{destination}/"])

    subprocess.run(command, check=True)

    assert all(not path.exists() for path in stale_paths)
    assert all(path.exists() for path in preserved_paths)
    assert (destination / ".env").read_text() == "REMOTE_SECRET=1\n"
    assert (destination / "ai_drone/.env.local").read_text() == "REMOTE_SECRET=2\n"
    assert (destination / "ai_drone/runtime.py").is_file()
    assert not (destination / "docs/not-deployed.md").exists()
    assert not (destination / "scripts/not-deployed.sh").exists()


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


@pytest.mark.skipif(shutil.which("python3") is None, reason="python3 is not installed")
def test_tar_cleanup_command_runs_from_the_uploaded_runtime(tmp_path: Path) -> None:
    source = tmp_path / "source"
    destination = tmp_path / "destination"
    _write_runtime_source(source)
    _write_file(source, "ai_drone/link/__init__.py", "")
    shutil.copy2(Path(deploy.__file__), source / "ai_drone" / "link" / "deploy.py")
    shutil.copy2(
        Path(deploy.__file__).with_name("targets.py"),
        source / "ai_drone" / "link" / "targets.py",
    )
    archive = deploy._create_sync_archive(source, DEPLOYMENT_ID)
    try:
        destination.mkdir()
        with tarfile.open(archive, "r:gz") as tar:
            tar.extractall(destination, filter="data")
    finally:
        archive.unlink(missing_ok=True)
    stale = _write_file(destination, "docs/stale.md")
    artifact = _write_file(destination, "artifacts/recording.bin")

    completed = subprocess.run(
        shlex.split(deploy._cleanup_script(DEPLOYMENT_ID)),
        cwd=destination,
        capture_output=True,
        check=False,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert not stale.exists()
    assert artifact.is_file()
    assert not (destination / deploy.MANIFEST_NAME).exists()
    assert not (destination / deploy.SENTINEL_NAME).exists()


def test_valid_tar_cleanup_deletes_stale_source_but_preserves_remote_state(
    tmp_path: Path,
) -> None:
    _write_runtime_source(tmp_path)
    _write_deployment_metadata(tmp_path)
    stale_paths = [
        _write_file(tmp_path, "docs/stale.md"),
        _write_file(tmp_path, "tests/stale.py"),
        _write_file(tmp_path, "scripts/stale.sh"),
        _write_file(tmp_path, "ai_drone/stale.py"),
    ]
    preserved_paths = [
        _write_file(tmp_path, ".env", "SECRET=1\n"),
        _write_file(tmp_path, "ai_drone/.env.local", "SECRET=2\n"),
        _write_file(tmp_path, ".venv/pyvenv.cfg"),
        _write_file(tmp_path, ".git/config"),
        _write_file(tmp_path, "artifacts/recording.bin"),
        _write_file(tmp_path, "ai_drone/__pycache__/runtime.pyc"),
    ]

    completed = _run_cleanup(tmp_path)

    assert completed == 0
    assert all(not path.exists() for path in stale_paths)
    assert all(path.exists() for path in preserved_paths)
    assert (tmp_path / "ai_drone/runtime.py").is_file()
    assert not (tmp_path / deploy.MANIFEST_NAME).exists()
    assert not (tmp_path / deploy.SENTINEL_NAME).exists()


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
def test_tar_cleanup_refuses_invalid_or_missing_metadata_without_deleting(
    tmp_path: Path, failure: str, capsys: pytest.CaptureFixture[str]
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

    completed = _run_cleanup(tmp_path)
    captured = capsys.readouterr()

    assert completed == 2
    assert "Refusing deployment cleanup" in captured.err
    assert stale.is_file()
    assert (tmp_path / "ai_drone/runtime.py").is_file()


def test_tar_cleanup_rejects_unsafe_manifest_path_before_deleting(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    _write_runtime_source(tmp_path)
    _write_deployment_metadata(tmp_path)
    stale = _write_file(tmp_path, "docs/must-survive.md")
    manifest_path = tmp_path / deploy.MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text())
    manifest["paths"].append("../outside")
    manifest_path.write_text(json.dumps(manifest))

    completed = _run_cleanup(tmp_path)
    captured = capsys.readouterr()

    assert completed == 2
    assert "unsafe path" in captured.err
    assert stale.is_file()


def test_repo_root_points_at_the_actual_repository() -> None:
    """REPO_ROOT is derived from module depth, so moving deploy.py must fail here."""
    assert (deploy.REPO_ROOT / "pyproject.toml").is_file()
    assert (deploy.REPO_ROOT / "ai_drone" / "__init__.py").is_file()
    assert Path(deploy.__file__).resolve().parents[1] == deploy.REPO_ROOT / "ai_drone"
