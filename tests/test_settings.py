from pathlib import Path

import pytest

from ai_drone.cli import main as cli
from ai_drone.link.targets import resolve_deploy_target
from ai_drone.settings import Settings, load_settings


def test_defaults_need_no_file(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    assert load_settings(environ={}) == Settings()
    captured = capsys.readouterr()
    assert "drone.toml not found" in captured.err
    with pytest.raises(ValueError, match="does not exist"):
        load_settings(tmp_path / "missing.toml")


def test_file_defaults_and_environment_overrides(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    Path("drone.toml").write_text(
        '[connection]\nhost="pilot@drone"\nproject_dir="/srv/drone"\n'
        'ssh_config="/path with spaces/config"\n'
        "[recording]\nstorage_reserve_mib=300\n"
        '[transfer]\ndestination="/data/new dataset"\n'
    )
    settings = load_settings(environ={})
    assert settings.recording.storage_reserve_mib == 300
    assert settings.transfer.destination == "/data/new dataset"
    target = resolve_deploy_target({})
    assert (target.user, target.address, target.project_dir) == (
        "pilot",
        "drone",
        "/srv/drone",
    )
    assert target.ssh_config == "/path with spaces/config"
    override = resolve_deploy_target(
        {"PI_HOST": "seb@local", "PI_DIR": "/opt/drone", "SSH_CONFIG": ""}
    )
    assert (override.ssh_target, override.project_dir, override.ssh_config) == (
        "seb@local",
        "/opt/drone",
        None,
    )


@pytest.mark.parametrize(
    "text",
    [
        "[unknown]\nx=1",
        "connection=1",
        '[connection]\nhsot="typo"',
        '[connection]\nhost=""',
        "[connection]\nhost=42",
        '[connection]\nhost="line\\nfeed"',
        "[recording]\nstorage_reserve_mib=true",
        "[recording]\nstorage_reserve_mib=nan",
        "[recording]\nstorage_check_interval=0",
        "[recording]\nstorage_reserve_mib=4",
        "[recording]\nstorage_warning_mib=32",
        "[transfer]\ndestination=42",
        "invalid: yaml",
    ],
)
def test_invalid_settings_fail_before_effects(tmp_path, text):
    config = tmp_path / "invalid.toml"
    config.write_text(text)
    with pytest.raises(ValueError):
        load_settings(config)


def test_cli_config_is_scoped_to_the_command(tmp_path, monkeypatch, capsys):
    config = tmp_path / "custom.toml"
    config.write_text('[connection]\nhost="pilot@custom"')
    monkeypatch.delenv("AI_DRONE_CONFIG", raising=False)
    assert cli.main(["--config", str(config), "connect", "--dry-run"]) == 0
    assert "pilot@custom" in capsys.readouterr().out
    import os

    assert "AI_DRONE_CONFIG" not in os.environ


def test_user_override_sets_default_home_without_overriding_explicit_host_user(
    tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    target = resolve_deploy_target({"PI_USER": "pilot"})
    assert target.ssh_target == "pilot@seb-is-pm"
    assert target.project_dir == "/home/pilot/ai-drone"
    target = resolve_deploy_target({"PI_USER": "pilot", "PI_HOST": "seb@local"})
    assert target.ssh_target == "seb@local"
    assert target.project_dir == "/home/seb/ai-drone"


def test_cli_rejects_missing_config_before_dispatch(tmp_path, monkeypatch):
    monkeypatch.setattr(
        cli, "import_module", lambda _: pytest.fail("command dispatched")
    )
    with pytest.raises(SystemExit) as error:
        cli.main(["--config", str(tmp_path / "missing"), "connect"])
    assert error.value.code == 2


def test_operator_and_runtime_settings_are_typed_and_defaults_remain_explicit(tmp_path):
    config = tmp_path / "operator.toml"
    config.write_text(
        '[operator]\nendpoints=["http://127.0.0.1:8787", "https://100.100.100.100"]\n'
        'token_file="/private/operator.key"\ninterval=1\ntimeout=5\nrequest_timeout=0.5\n'
        'ssh_host="seb@seb-is-pm"\nremote_port=18788\n'
        '[runtime]\nsocket="/run/ai-drone/vehicle.sock"\n'
        'status="/run/ai-drone/status.json"\ndevice="/dev/serial0"\n'
        'baud=115200\nnetwork_profiles="/etc/ai-drone/network-profiles.json"\n'
    )
    settings = load_settings(config)
    assert settings.operator.endpoints == (
        "http://127.0.0.1:8787",
        "https://100.100.100.100",
    )
    assert isinstance(settings.operator.endpoints, tuple)
    assert (settings.operator.interval, settings.operator.timeout) == (1, 5)
    assert (settings.operator.ssh_host, settings.operator.remote_port) == (
        "seb@seb-is-pm",
        18788,
    )
    assert settings.runtime.baud == 115200
    assert settings.runtime.socket == "/run/ai-drone/vehicle.sock"


@pytest.mark.parametrize(
    "endpoint",
    [
        "ftp://operator.local",
        "operator.local:8787",
        "http://",
        "http://user:password@127.0.0.1",
        "http://@127.0.0.1",
        "http://127.0.0.1/heartbeat",
        "http://127.0.0.1?secret=1",
        "http://127.0.0.1#fragment",
        "http://127.0.0.1:65536",
        "http://127.0.0.1:-1",
        "http://127.0.0.1:0",
        "http://127.0.0.1:bad",
        "http://bad host:8787",
    ],
)
def test_operator_rejects_non_origin_or_credentialed_endpoints(tmp_path, endpoint):
    config = tmp_path / "operator.toml"
    config.write_text(
        f'[operator]\nendpoints=["{endpoint}"]\ntoken_file="/private/key"\n'
    )
    with pytest.raises(ValueError):
        load_settings(config)


@pytest.mark.parametrize(
    "text",
    [
        '[operator]\nendpoints="http://127.0.0.1"',
        "[operator]\nendpoints=[42]",
        '[operator]\nendpoints=[""]',
        '[operator]\nendpoints=["http://127.0.0.1\\n"]',
        '[operator]\nendpoints=["http://127.0.0.1"]',
        '[operator]\ntoken_file="/private/key"',
        '[operator]\nendpoints=["http://a","http://b","http://c","http://d","http://e"]\ntoken_file="/key"',
        "[operator]\ninterval=0",
        "[operator]\ninterval=0.1",
        "[operator]\ninterval=nan",
        "[operator]\ninterval=true",
        "[operator]\ninterval=3\ntimeout=5",
        "[operator]\ntimeout=0",
        "[operator]\ntimeout=31",
        "[operator]\nrequest_timeout=0.01",
        "[operator]\nrequest_timeout=3",
        '[operator]\nssh_host="-option"',
        '[operator]\nssh_host="user@-option"',
        '[operator]\nssh_host="user@pi;bad"',
        '[operator]\nssh_host="user@pi\\tbad"',
        '[operator]\nssh_host="user@pi bad"',
        '[operator]\nssh_host="user@pi\\n"',
        '[operator]\nssh_host=""',
        "[operator]\nremote_port=0",
        "[operator]\nremote_port=65536",
        "[operator]\nremote_port=18787.5",
        "[operator]\nremote_port=true",
        "[runtime]\nbaud=115200.5",
        "[runtime]\nbaud=true",
        "[runtime]\nbaud=0",
        "[runtime]\nbaud=4000001",
        "[runtime]\nsocket=123",
        '[runtime]\nsocket=""',
        '[runtime]\nsocket="/run/vehicle\\n.sock"',
        '[runtime]\nstatus="/run/\\u0000status"',
        '[runtime]\nunknown="typo"',
    ],
)
def test_operator_and_runtime_reject_invalid_settings(tmp_path, text):
    config = tmp_path / "invalid-runtime.toml"
    config.write_text(text)
    with pytest.raises(ValueError):
        load_settings(config)
