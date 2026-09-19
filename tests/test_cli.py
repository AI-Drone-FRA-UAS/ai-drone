from types import SimpleNamespace

import pytest

from ai_drone.cli import main as cli


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
