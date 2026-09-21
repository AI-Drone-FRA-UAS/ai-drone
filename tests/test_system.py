import argparse
import signal
import subprocess
import threading
from pathlib import Path

import pytest

from ai_drone.system import handled_signals, namespace_to_flags, run, unit_state


def test_command_dry_run_prints_quoted_arguments_without_execution(monkeypatch, capsys):
    monkeypatch.setattr(
        subprocess, "run", lambda *_args, **_kwargs: pytest.fail("executed command")
    )
    assert run(["tool", "a b", "$(literal)"], dry_run=True) is None
    assert capsys.readouterr().out == "  tool 'a b' '$(literal)'\n"


def test_command_keeps_inherited_io_and_none_result(monkeypatch):
    calls = []

    def execute(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(subprocess, "run", execute)
    assert run(["tool"]) is None
    assert calls == [(["tool"], {"check": True})]


@pytest.mark.parametrize(
    "error", [OSError("unavailable"), subprocess.CalledProcessError(7, ["tool"])]
)
def test_command_errors_reach_the_callers_policy(monkeypatch, error):
    def fail(*_args, **_kwargs):
        raise error

    monkeypatch.setattr(subprocess, "run", fail)
    with pytest.raises(type(error)) as raised:
        run(["tool"])
    assert raised.value is error


def test_namespace_flags_roundtrip_values():
    args = argparse.Namespace(
        enabled=True,
        disabled=False,
        missing=None,
        paths=[Path("a b"), Path("c")],
        tag_ids=[1, 3],
        secret="skip",
    )
    assert namespace_to_flags(
        args, exclude=frozenset({"secret"}), aliases={"tag_ids": "--tag-id"}
    ) == [
        "--enabled",
        "--paths",
        "a b",
        "--paths",
        "c",
        "--tag-id",
        "1",
        "--tag-id",
        "3",
    ]


def test_service_absence_requires_the_expected_exit_code():
    for code in (0, 1, 2):
        state = unit_state(
            subprocess.CompletedProcess(
                [], code, "LoadState=not-found\nActiveState=inactive\n", ""
            )
        )
        assert state.absent is (code in (0, 1))
        assert state.active == "inactive"
    assert unit_state(subprocess.CompletedProcess([], 0, "", "")).load == "unknown"


def test_signal_scopes_restore_nested_and_exception_handlers():
    original = signal.getsignal(signal.SIGINT)
    try:
        with handled_signals({signal.SIGINT: signal.SIG_IGN}):
            assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN
            with handled_signals({signal.SIGINT: signal.SIG_DFL}):
                assert signal.getsignal(signal.SIGINT) == signal.SIG_DFL
            assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN
            raise RuntimeError("interrupted")
    except RuntimeError:
        pass
    assert signal.getsignal(signal.SIGINT) == original


def test_signal_scope_can_be_used_from_a_worker():
    completed = []

    def worker():
        with handled_signals({signal.SIGINT: signal.SIG_IGN}):
            completed.append(True)

    thread = threading.Thread(target=worker)
    thread.start()
    thread.join(timeout=1)
    assert completed == [True]


def test_all_signal_restorations_attempted_if_one_fails(monkeypatch):
    restored = []

    def install(number, handler):
        if handler == signal.SIG_DFL:
            restored.append(number)
            if number == signal.SIGTERM:
                raise OSError("restore failed")
        return signal.SIG_DFL

    monkeypatch.setattr(signal, "signal", install)
    with (
        pytest.raises(OSError, match="restore failed"),
        handled_signals(
            {signal.SIGINT: signal.SIG_IGN, signal.SIGTERM: signal.SIG_IGN}
        ),
    ):
        pass
    assert restored == [signal.SIGTERM, signal.SIGINT]
