import os
from types import SimpleNamespace

import pytest

from ai_drone.cli import main as cli
from ai_drone.settings import Settings, load_settings


@pytest.mark.parametrize("command", cli.COMMANDS)
def test_help_needs_no_hardware(command, capsys):
    with pytest.raises(SystemExit) as result:
        cli.main([command, "--help"])
    assert result.value.code == 0
    assert "usage:" in capsys.readouterr().out


def test_command_arguments_and_failure_status_are_preserved(monkeypatch):
    received = []

    def record(arguments):
        received.append(arguments)
        return 7

    monkeypatch.setattr(cli, "import_module", lambda _: SimpleNamespace(run=record))
    assert cli.main(["record", "--duration", "2", "--no-video"]) == 7
    assert received == [["--duration", "2", "--no-video"]]


def test_unknown_command_cannot_import_a_task(monkeypatch):
    monkeypatch.setattr(cli, "import_module", lambda _: pytest.fail("task imported"))
    with pytest.raises(SystemExit) as result:
        cli.main(["missing"])
    assert result.value.code == 2


def test_dispatch_injects_one_settings_without_environment_mutation(
    tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    config = tmp_path / "explicit.toml"
    config.write_text('[connection]\nhost = "custom@host"\n')
    monkeypatch.setenv("AI_DRONE_CONFIG", "unchanged-and-unread")

    def command(_arguments):
        first = load_settings()
        assert first is load_settings(environ=dict(os.environ))
        assert first.connection.host == "custom@host"
        assert os.environ["AI_DRONE_CONFIG"] == "unchanged-and-unread"
        return 0

    monkeypatch.setattr(cli, "import_module", lambda _: SimpleNamespace(run=command))
    assert cli.main(["--config", str(config), "record"]) == 0
    assert load_settings(environ={}) == Settings()
