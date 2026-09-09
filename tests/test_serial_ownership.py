from __future__ import annotations

import os
import shutil
import subprocess
import sys

import pytest

from ai_drone.mavlink import ownership

pytestmark = pytest.mark.skipif(
    not sys.platform.startswith("linux"), reason="Linux serial-owner inspection"
)


@pytest.fixture
def calls(monkeypatch):
    monkeypatch.setattr(ownership.sys, "platform", "linux")
    result = []
    return result


def install_runner(monkeypatch, calls, responses):
    pending = iter(responses)

    def run(command, **kwargs):
        calls.append(command)
        assert kwargs == {
            "capture_output": True,
            "text": True,
            "timeout": 3,
            "check": False,
        }
        response = next(pending)
        if isinstance(response, Exception):
            raise response
        return subprocess.CompletedProcess(command, *response)

    monkeypatch.setattr(ownership.subprocess, "run", run)


def test_no_owner_is_checked_read_only_by_canonical_device(
    monkeypatch, calls, tmp_path
):
    alias = tmp_path / "serial-alias"
    alias.symlink_to("/dev/null")
    install_runner(monkeypatch, calls, [(1, "", "")])
    ownership.require_available_serial(str(alias))
    assert calls == [["fuser", "/dev/null"]]


@pytest.mark.parametrize("response", [(0, "12345", "/dev/null:"), (0, "", "")])
def test_any_reported_owner_refuses_without_killing(monkeypatch, calls, response):
    install_runner(monkeypatch, calls, [response])
    with pytest.raises(RuntimeError, match="busy"):
        ownership.require_available_serial("/dev/null")
    assert calls == [["fuser", "/dev/null"]]


@pytest.mark.parametrize(
    "response",
    [
        (2, "", "failure"),
        (1, "", "inspection warning"),
        (1, "unexpected", ""),
        FileNotFoundError("missing fuser"),
        subprocess.TimeoutExpired("fuser", 3),
    ],
)
def test_incomplete_inspection_fails_closed(monkeypatch, calls, response):
    install_runner(monkeypatch, calls, [response])
    with pytest.raises(RuntimeError, match=r"[Cc]annot"):
        ownership.require_available_serial("/dev/null")
    assert len(calls) == 1


@pytest.mark.parametrize(
    "state", ["active", "activating", "deactivating", "reloading", "unknown"]
)
def test_pi_walk_startup_capture_and_shutdown_all_refuse(monkeypatch, calls, state):
    install_runner(
        monkeypatch, calls, [(0, f"LoadState=loaded\nActiveState={state}\n", "")]
    )
    with pytest.raises(RuntimeError, match="Recorder"):
        ownership.require_available_serial("/dev/null", on_pi=True)
    assert len(calls) == 1
    assert calls[0][:3] == ["systemctl", "show", "ai-drone-walk.service"]


@pytest.mark.parametrize(
    "returncode,state",
    [
        (0, "LoadState=loaded\nActiveState=inactive\n"),
        (0, "LoadState=loaded\nActiveState=failed\n"),
        (1, "LoadState=not-found\nActiveState=inactive\n"),
    ],
)
def test_idle_or_absent_pi_unit_then_checks_actual_owner(
    monkeypatch, calls, returncode, state
):
    install_runner(monkeypatch, calls, [(returncode, state, ""), (1, "", "")])
    ownership.require_available_serial("/dev/null", on_pi=True)
    assert len(calls) == 2 and calls[1] == ["fuser", "/dev/null"]


def test_unknown_service_state_refuses_before_fuser(monkeypatch, calls):
    install_runner(monkeypatch, calls, [(1, "", "cannot contact systemd")])
    with pytest.raises(RuntimeError, match="recorder service"):
        ownership.require_available_serial("/dev/null", on_pi=True)
    assert len(calls) == 1


def test_explicit_network_endpoint_does_not_open_or_inspect_serial(monkeypatch, calls):
    install_runner(monkeypatch, calls, [])
    ownership.require_available_serial("tcp:127.0.0.1:5760", on_pi=True)
    assert not calls


def test_regular_capture_file_is_not_a_live_device(monkeypatch, calls, tmp_path):
    capture = tmp_path / "old-log.tlog"
    capture.write_bytes(b"old telemetry")
    install_runner(monkeypatch, calls, [])
    with pytest.raises(RuntimeError, match="live serial"):
        ownership.require_available_serial(str(capture))
    assert not calls


def test_installed_fuser_accepts_absolute_path_and_finds_real_same_user_owner(tmp_path):
    executable = shutil.which("fuser")
    if executable is None:
        pytest.skip("fuser is not installed")
    path = (tmp_path / "owned-file").resolve()
    with path.open("w"):
        result = ownership._inspect([executable, str(path)])
    assert result.returncode == 0, result.stderr
    assert str(os.getpid()) in result.stdout.split()
