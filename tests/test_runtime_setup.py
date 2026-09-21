"""Installer safety and rollback using temporary files and simulated commands."""

from __future__ import annotations

import json
import os
import stat
import subprocess
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from ai_drone.settings import OperatorSettings, RuntimeSettings, Settings
from scripts import setup_runtime as setup

REQUIRE_DISARMED = setup.require_disarmed
CLIENT = "11111111-1111-4111-8111-111111111111"
AP = "22222222-2222-4222-8222-222222222222"
PROFILES = {
    CLIENT: {
        "name": "eduroam",
        "mode": "infrastructure",
        "autoconnect": True,
        "priority": 400,
    },
    AP: {"name": "Hotspot", "mode": "ap", "autoconnect": False, "priority": -10},
}
FRESH = {
    "status": "disarmed",
    "armed": False,
    "fresh": True,
    "source_known": True,
    "system_id": 1,
    "component_id": 1,
    "heartbeat_age_s": 0.1,
    "clients": 1,
}


@pytest.fixture
def host(tmp_path, monkeypatch):
    project = tmp_path / "project"
    project.mkdir()
    config = project / "drone.toml"
    manifest = tmp_path / "etc" / "profiles.json"
    config.write_text(
        "[runtime]\nnetwork_profiles = " + json.dumps(str(manifest)) + "\n"
    )
    unit = tmp_path / "systemd" / setup.UNIT
    locks_config = tmp_path / "etc" / "locks.conf"
    locks_directory = tmp_path / "run-locks"
    backups = tmp_path / "backups"
    keyfiles = tmp_path / "NetworkManager"
    keyfiles.mkdir()
    (keyfiles / "client.nmconnection").write_text("test-private-credential")
    plan = setup.Plan(
        project,
        config,
        "seb",
        Path("/home/seb/.local/bin/uv"),
        Settings(
            runtime=RuntimeSettings(
                network_profiles=str(manifest), socket=str(tmp_path / "vehicle.sock")
            )
        ),
    )
    events = []
    profiles = {key: dict(value) for key, value in PROFILES.items()}
    states = {
        setup.UNIT: {"load": "not-found", "active": "inactive", "enabled": "disabled"},
        setup.LEGACY: {"load": "loaded", "active": "active", "enabled": "enabled"},
    }

    def private(path, *, create=True):
        if create:
            path.mkdir(parents=True, mode=0o700, exist_ok=True)
        assert stat.S_IMODE(path.stat().st_mode) == 0o700

    def run(command, **_kwargs):
        events.append(command)
        if command[:2] == ["systemctl", "start"]:
            states[command[2]]["active"] = "active"
        elif command[:2] == ["systemctl", "stop"]:
            states[command[2]]["active"] = "inactive"
        elif command[:2] == ["systemd-tmpfiles", "--create"]:
            locks_directory.mkdir(mode=0o700, exist_ok=True)
        return subprocess.CompletedProcess(command, 0, "", "")

    def nmcli(arguments):
        events.append(["nmcli", *arguments])
        if arguments[:2] == ["-g", "GENERAL.AUTOCONNECT"]:
            return "yes"
        assert list(backups.glob("*/backup.json")), "mutation before durable backup"
        if arguments[:2] == ["connection", "modify"]:
            profiles[arguments[3]]["autoconnect"] = arguments[-1] == "yes"
        return ""

    def request(_socket, value, **_kwargs):
        events.append(["rpc", value])
        return FRESH if value == {"status": True} else value

    monkeypatch.setattr(setup, "UNIT_PATH", unit)
    monkeypatch.setattr(setup, "LOCKS_CONFIG", locks_config)
    monkeypatch.setattr(setup, "LOCKS_DIRECTORY", locks_directory)
    monkeypatch.setattr(setup, "_account_ids", lambda _user: (os.getuid(), os.getgid()))
    monkeypatch.setattr(setup, "BACKUPS", backups)
    monkeypatch.setattr(setup, "NM_PATHS", (keyfiles,))
    monkeypatch.setattr(setup, "_private_directory", private)
    monkeypatch.setattr(setup, "_service_state", lambda unit: dict(states[unit]))
    monkeypatch.setattr(
        setup,
        "_profiles",
        lambda: {key: dict(value) for key, value in profiles.items()},
    )
    monkeypatch.setattr(setup, "read_link", lambda **_kwargs: (CLIENT, True, False))
    monkeypatch.setattr(
        setup, "require_disarmed", lambda _plan: events.append(["disarmed"])
    )
    monkeypatch.setattr(setup, "run", run)
    monkeypatch.setattr(setup, "_nmcli", nmcli)
    monkeypatch.setattr(setup, "runtime_request", request)
    return SimpleNamespace(
        plan=plan,
        events=events,
        profiles=profiles,
        states=states,
        manifest=manifest,
        unit=unit,
        backups=backups,
        keyfiles=keyfiles,
    )


def arguments(host):
    return ["--project", str(host.plan.project), "--config", str(host.plan.config)]


def test_default_dry_run_has_no_commands_or_backups(host, capsys):
    assert setup.main(arguments(host)) == 0
    output = capsys.readouterr().out
    assert (
        "DRY RUN" in output
        and "User=seb" in output
        and "RuntimeDirectoryMode=0700" in output
    )
    assert '"runtime" "serve"' in output and "--no-sync" in output
    assert not host.events and not host.backups.exists()


def test_apply_requires_root_pi_before_any_commands(host, monkeypatch, capsys):
    monkeypatch.setattr(setup, "is_raspberry_pi", lambda: False)
    assert setup.main([*arguments(host), "--apply"]) == 1
    assert "requires root" in capsys.readouterr().err
    assert not host.events and not host.backups.exists()


@pytest.mark.parametrize("token_name", ["operator.key", "~/operator.key"])
def test_preflight_checks_token_as_service_user_and_existing_manifest_parent(
    host, monkeypatch, token_name
):
    import pwd

    home = host.plan.project / "user-home"
    home.mkdir()
    token = (
        home if token_name.startswith("~/") else host.plan.project
    ) / "operator.key"
    token.write_text("ab" * 32 + "\n")
    token.chmod(0o600)
    uv = host.plan.project / "uv"
    uv.touch(mode=0o755)
    (host.plan.project / "pyproject.toml").touch()
    plan = replace(
        host.plan,
        uv=uv,
        settings=replace(
            host.plan.settings,
            runtime=RuntimeSettings(network_profiles=str(host.manifest)),
            operator=OperatorSettings(
                endpoints=("http://127.0.0.1:9000",), token_file=token_name
            ),
        ),
    )
    monkeypatch.setattr(setup.os, "geteuid", lambda: 0)
    monkeypatch.setattr(setup, "is_raspberry_pi", lambda: True)
    monkeypatch.setattr(
        pwd, "getpwnam", lambda _name: SimpleNamespace(pw_uid=1000, pw_dir=str(home))
    )
    setup._validate_apply(plan)
    assert ["runuser", "-u", "seb", "--", "test", "-r", str(token)] in host.events
    assert [
        "runuser",
        "-u",
        "seb",
        "--",
        "test",
        "-x",
        str(host.manifest.parent.parent),
    ] in host.events
    assert "--directory" in host.events[-1]


def test_manifest_live_file_is_readable_while_private_backup_stays_private(host):
    directory = setup.install(host.plan)
    assert stat.S_IMODE(host.manifest.stat().st_mode) == 0o644
    assert stat.S_IMODE(host.manifest.parent.stat().st_mode) == 0o755
    assert stat.S_IMODE((directory / "backup.json").stat().st_mode) == 0o600


def test_unit_quotes_spaces_percent_and_quotes_without_shell_expansion(host):
    text = setup.unit_text(replace(host.plan, project=Path('/opt/drone 50% "test"')))
    assert 'WorkingDirectory="/opt/drone 50%% \\"test\\""' in text
    assert "Restart=on-failure" in text
    assert "bash" not in text and "sudo" not in text and "control" not in text


def test_install_disables_every_wifi_profile_after_private_backup(host, capsys):
    directory = setup.install(host.plan)
    assert all(not profile["autoconnect"] for profile in host.profiles.values())
    assert json.loads(host.manifest.read_text()) == {"schema": 1, "profiles": PROFILES}
    assert host.unit.read_text() == setup.unit_text(host.plan)
    metadata = json.loads((directory / "backup.json").read_text())
    assert metadata["profiles"] == PROFILES
    assert (
        directory / "nm-profiles-0" / "client.nmconnection"
    ).read_text() == "test-private-credential"
    assert stat.S_IMODE(directory.stat().st_mode) == 0o700
    assert all(
        stat.S_IMODE(path.stat().st_mode) == 0o600
        for path in directory.rglob("*")
        if path.is_file()
    )
    assert "test-private-credential" not in capsys.readouterr().out
    assert ["systemctl", "disable", setup.LEGACY] in host.events
    assert ["systemctl", "stop", setup.LEGACY] not in host.events
    assert not any(
        "up" in event or "down" in event or "reboot" in event for event in host.events
    )


def test_upgrade_keeps_previous_client_eligibility_when_nm_autoconnect_is_off(host):
    host.manifest.parent.mkdir()
    host.manifest.write_text(json.dumps({"schema": 1, "profiles": PROFILES}))
    host.profiles[CLIENT]["autoconnect"] = False
    setup.install(host.plan)
    assert (
        json.loads(host.manifest.read_text())["profiles"][CLIENT]["autoconnect"] is True
    )


def test_manifest_preserves_original_ap_flag_without_making_it_a_client(host):
    original = {key: dict(value) for key, value in PROFILES.items()}
    original[AP]["autoconnect"] = True
    selected = setup.eligibility(host.profiles, {"schema": 1, "profiles": original})
    assert selected["profiles"][AP]["autoconnect"] is True
    assert selected["profiles"][AP]["mode"] == "ap"


def test_unknown_or_changed_profile_mode_is_not_silently_enrolled(host):
    old = {key: dict(value) for key, value in PROFILES.items()}
    old[CLIENT]["mode"] = "ap"
    host.profiles[CLIENT]["autoconnect"] = False
    with pytest.raises(ValueError, match="at least one"):
        setup.eligibility(host.profiles, {"schema": 1, "profiles": old})


def test_existing_runtime_enters_maintenance_immediately_before_stop(host):
    host.states[setup.UNIT] = {
        "load": "loaded",
        "active": "active",
        "enabled": "enabled",
    }
    setup.install(host.plan)
    index = host.events.index(["rpc", {"maintenance": True}])
    assert host.events[index + 1] == ["systemctl", "stop", setup.UNIT]


def test_failed_existing_runtime_stop_releases_maintenance_and_preserves_flags(
    host, monkeypatch
):
    host.states[setup.UNIT] = {
        "load": "loaded",
        "active": "active",
        "enabled": "enabled",
    }
    original = setup.run

    def fail_stop(command, **kwargs):
        if command[:2] == ["systemctl", "stop"]:
            raise RuntimeError("could not stop")
        return original(command, **kwargs)

    monkeypatch.setattr(setup, "run", fail_stop)
    with pytest.raises(RuntimeError, match="could not stop"):
        setup.install(host.plan)
    assert ["rpc", {"maintenance": False}] in host.events
    assert host.profiles == PROFILES
    assert not host.unit.exists() and not host.manifest.exists()


def test_failure_restores_original_flags_and_files_without_touching_credentials(
    host, monkeypatch
):
    host.unit.parent.mkdir()
    host.unit.write_text("previous unit\n")
    host.manifest.parent.mkdir()
    old_manifest = json.dumps({"schema": 1, "profiles": PROFILES}) + "\n"
    host.manifest.write_text(old_manifest)
    host.states[setup.UNIT] = {
        "load": "loaded",
        "active": "active",
        "enabled": "enabled",
    }
    monkeypatch.setattr(
        setup,
        "_verify",
        lambda *_args: (_ for _ in ()).throw(RuntimeError("not ready")),
    )
    with pytest.raises(RuntimeError, match="not ready"):
        setup.install(host.plan)
    assert host.profiles == PROFILES
    assert host.unit.read_text() == "previous unit\n"
    assert host.manifest.read_text() == old_manifest
    assert (
        host.keyfiles / "client.nmconnection"
    ).read_text() == "test-private-credential"
    assert host.states[setup.UNIT]["active"] == "active"
    assert ["systemctl", "enable", setup.LEGACY] in host.events
    assert not any("up" in event or "down" in event for event in host.events)


def test_partial_nmcli_failure_also_rolls_back_all_flags(host, monkeypatch):
    original = setup._nmcli
    failed = False

    def failing(arguments):
        nonlocal failed
        if (
            arguments[:2] == ["connection", "modify"]
            and arguments[3] == AP
            and not failed
        ):
            failed = True
            raise RuntimeError("write failed")
        return original(arguments)

    monkeypatch.setattr(setup, "_nmcli", failing)
    with pytest.raises(RuntimeError, match="write failed"):
        setup.install(host.plan)
    assert host.profiles == PROFILES
    assert not host.unit.exists() and not host.manifest.exists()


def test_failed_verification_cannot_roll_back_a_new_recording_or_control_owner(
    host, monkeypatch
):
    def failed_verify(*_args):
        raise RuntimeError("another client began during verification")

    def refused(_path, payload, **_kwargs):
        assert payload == {"maintenance": True}
        raise RuntimeError("maintenance requires no other clients")

    monkeypatch.setattr(setup, "_verify", failed_verify)
    monkeypatch.setattr(setup, "runtime_request", refused)
    with pytest.raises(RuntimeError, match=r"rollback withheld.*no other clients"):
        setup.install(host.plan)
    assert ["systemctl", "stop", setup.UNIT] not in host.events
    assert host.states[setup.UNIT]["active"] == "active"
    assert all(not profile["autoconnect"] for profile in host.profiles.values())
    assert host.unit.exists() and host.manifest.exists()
    assert list(host.backups.glob("*/backup.json"))


def test_crashed_new_service_requires_fresh_passive_confirmation_before_rollback(
    host, monkeypatch
):
    def crashed(*_args):
        host.states[setup.UNIT] = {
            "load": "loaded",
            "active": "failed",
            "enabled": "enabled",
        }
        host.events.append(["service-crashed"])
        raise RuntimeError("startup failed")

    monkeypatch.setattr(setup, "_verify", crashed)
    with pytest.raises(RuntimeError, match="startup failed"):
        setup.install(host.plan)
    index = host.events.index(["service-crashed"])
    assert host.events[index + 1] == ["disarmed"]
    assert host.profiles == PROFILES
    assert not host.unit.exists() and not host.manifest.exists()


def test_failed_passive_confirmation_retains_backup_and_flags_without_stopping_any_owner(
    host, monkeypatch
):
    def crashed(*_args):
        host.states[setup.UNIT]["active"] = "failed"
        monkeypatch.setattr(
            setup,
            "require_disarmed",
            lambda _plan: (_ for _ in ()).throw(RuntimeError("serial busy")),
        )
        raise RuntimeError("startup failed")

    monkeypatch.setattr(setup, "_verify", crashed)
    with pytest.raises(RuntimeError, match=r"rollback withheld.*serial busy"):
        setup.install(host.plan)
    assert ["systemctl", "stop", setup.UNIT] not in host.events
    assert all(not profile["autoconnect"] for profile in host.profiles.values())
    assert list(host.backups.glob("*/backup.json"))


def test_backup_failure_prevents_network_or_service_mutation(host, monkeypatch):
    monkeypatch.setattr(
        setup, "backup", lambda *_args: (_ for _ in ()).throw(OSError("disk full"))
    )
    with pytest.raises(OSError, match="disk full"):
        setup.install(host.plan)
    assert host.events == [["disarmed"]]


@pytest.mark.parametrize(
    "changes",
    [
        {"armed": True},
        {"heartbeat_age_s": 3},
        {"heartbeat_age_s": float("nan")},
        {"heartbeat_age_s": True},
        {"system_id": 42},
        {"source_known": False},
        {"network_busy": True},
        {"control_client": "owner"},
        {"clients": 2},
    ],
)
def test_freshness_and_idle_guards_reject_unsafe_runtime_state(changes):
    with pytest.raises(RuntimeError):
        setup._fresh_disarmed({**FRESH, **changes})


def test_existing_runtime_is_queried_without_probing_serial(host, monkeypatch):
    from ai_drone.mavlink import connection

    host.states[setup.UNIT]["active"] = "active"
    monkeypatch.setattr(
        connection,
        "open_ardupilot_connection",
        lambda *_args, **_kwargs: pytest.fail("serial opened"),
    )
    REQUIRE_DISARMED(host.plan)


def test_revert_preview_never_accesses_a_backup_or_runs_commands(host, capsys):
    assert (
        setup.main([*arguments(host), "--revert", "/var/backups/ai-drone/not-present"])
        == 0
    )
    assert "DRY RUN: restore" in capsys.readouterr().out
    assert not host.events


def test_explicit_revert_restores_original_flags_and_removes_only_installed_files(
    host, monkeypatch
):
    directory = setup.install(host.plan)
    monkeypatch.setattr(setup, "_validate_apply", lambda _plan: None)
    assert setup.main([*arguments(host), "--revert", str(directory), "--apply"]) == 0
    assert host.profiles == PROFILES
    assert not host.unit.exists() and not host.manifest.exists()
    assert (directory / "backup.json").is_file()
    assert (
        host.keyfiles / "client.nmconnection"
    ).read_text() == "test-private-credential"
    assert ["rpc", {"maintenance": True}] in host.events


def test_revert_rejects_backup_for_other_project_before_mutation(host, monkeypatch):
    directory = setup.install(host.plan)
    metadata = json.loads((directory / "backup.json").read_text())
    metadata["project"] = "/elsewhere"
    (directory / "backup.json").write_text(json.dumps(metadata))
    monkeypatch.setattr(setup, "_validate_apply", lambda _plan: None)
    host.events.clear()
    assert setup.main([*arguments(host), "--revert", str(directory), "--apply"]) == 1
    assert not host.events


def test_no_eligible_client_refuses_install_before_mutation(host):
    host.profiles[CLIENT]["autoconnect"] = False
    with pytest.raises(ValueError, match="at least one"):
        setup.install(host.plan)
    assert host.events == [["disarmed"]]


def test_servo_lock_namespace_survives_service_restart_and_rollback(host):
    directory = setup.install(host.plan)
    locks = setup.LOCKS_DIRECTORY
    identity = locks.stat().st_ino
    (locks / "bcm12-servo.lock").touch(mode=0o600)
    assert (
        f"d {locks} 0700 {os.getuid()} {os.getgid()} -"
        in setup.LOCKS_CONFIG.read_text()
    )
    assert "ai-drone-locks" not in setup.unit_text(host.plan)
    setup.run(["systemctl", "stop", setup.UNIT])
    setup.run(["systemctl", "start", setup.UNIT])
    setup.safe_restore(
        host.plan, directory, json.loads((directory / "backup.json").read_text())
    )
    assert locks.stat().st_ino == identity
    assert (locks / "bcm12-servo.lock").exists()
    assert not setup.LOCKS_CONFIG.exists()


def test_provisioning_refuses_existing_unsafe_lock_directory(host):
    setup.LOCKS_DIRECTORY.mkdir(mode=0o755)
    with pytest.raises(PermissionError, match="ownership or mode"):
        setup.provision_locks(host.plan)
    assert not setup.LOCKS_CONFIG.exists()


def test_manually_started_runtime_must_be_stopped_before_install(host):
    Path(host.plan.settings.runtime.socket).touch()
    with pytest.raises(RuntimeError, match="manually launched"):
        setup.install(host.plan)
    assert not host.backups.exists()
