from __future__ import annotations

import argparse
import json
import signal
import time
from contextlib import contextmanager
from dataclasses import replace
from types import SimpleNamespace

import pytest

from ai_drone.cli import control
from ai_drone.flight.controller import FlightSafetyError, HumanControlTaken
from ai_drone.settings import Settings


def test_pi_refuses_control_before_accessing_fc_without_runtime(monkeypatch, tmp_path):
    monkeypatch.setattr(control, "is_raspberry_pi", lambda: True)
    settings = Settings()
    monkeypatch.setattr(
        control,
        "load_settings",
        lambda: replace(
            settings, runtime=replace(settings.runtime, status=str(tmp_path / "absent"))
        ),
    )
    monkeypatch.setattr(control, "_controller", lambda _: pytest.fail("FC opened"))
    assert (
        control.main(
            [
                "hover",
                "--foreground",
                "--confirm-flight",
                control.FLIGHT_CONFIRMATION,
            ]
        )
        == 1
    )


@pytest.mark.parametrize(
    "change",
    [
        {"operator_alive": False},
        {"operator_configured": False},
        {"wifi_connected": False},
        {"updated_monotonic": 0},
    ],
)
def test_runtime_control_gate_requires_current_operator_and_wifi(
    monkeypatch, tmp_path, change
):
    path = tmp_path / "status.json"
    value = {
        "updated_monotonic": time.monotonic(),
        "operator_alive": True,
        "operator_configured": True,
        "wifi_required": True,
        "wifi_connected": True,
    }
    path.write_text(json.dumps(value | change))
    with (
        pytest.raises((FlightSafetyError, RuntimeError)),
        control._operator_link(argparse.Namespace(runtime_status=str(path))),
    ):
        pytest.fail("unsafe operator gate passed")


def test_operator_and_handoff_callbacks_read_new_status_each_time(tmp_path):
    path = tmp_path / "status.json"
    value = {
        "updated_monotonic": time.monotonic(),
        "operator_alive": True,
        "operator_configured": True,
        "wifi_required": True,
        "wifi_connected": True,
    }
    path.write_text(json.dumps(value))
    with control._operator_link(
        argparse.Namespace(runtime_status=str(path))
    ) as callbacks:
        alive, handoff = callbacks
        assert alive() and not handoff()
        path.write_text(
            json.dumps(value | {"operator_alive": False, "human_requested": True})
        )
        assert not alive() and handoff()


@pytest.mark.skipif(not hasattr(signal, "SIGHUP"), reason="POSIX terminal signals")
def test_terminal_hangup_does_not_request_land_and_handlers_are_restored():
    before = {sig: signal.getsignal(sig) for sig in (signal.SIGHUP, signal.SIGTERM)}
    with control._termination_event() as stop:
        assert signal.getsignal(signal.SIGHUP) == signal.SIG_IGN
        assert not stop.is_set()
        handler = signal.getsignal(signal.SIGTERM)
        assert callable(handler)
        handler(signal.SIGTERM, None)
        assert stop.is_set()
    assert all(signal.getsignal(sig) == handler for sig, handler in before.items())


def test_detached_control_retains_config_and_cleanup_supervision(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(control.os, "getuid", lambda: 1000)
    monkeypatch.setattr(control.os, "getgid", lambda: 1000)
    monkeypatch.setattr(control.shutil, "which", lambda _: "/home/seb/.local/bin/uv")
    config = tmp_path / "local settings.toml"
    monkeypatch.setenv("AI_DRONE_CONFIG", str(config))

    def run(command, **kwargs):
        assert not kwargs.get("shell")
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout="LoadState=not-found")

    monkeypatch.setattr(control.subprocess, "run", run)
    args = control._parser().parse_args(
        [
            "hover",
            "--duration",
            "12",
            "--confirm-flight",
            control.FLIGHT_CONFIRMATION,
        ]
    )
    assert control._launch_hover(args) == 0
    command = calls[-1]
    assert command[:3] == ["sudo", "-n", "systemd-run"]
    assert "--property=TimeoutStopSec=infinity" in command
    assert "--property=KillSignal=SIGTERM" in command
    assert f"--setenv=AI_DRONE_CONFIG={config}" in command
    assert command[command.index("--duration") + 1] == "12.0"
    assert "--worker" in command and "--wait" not in command


def test_confirmed_pilot_handoff_finishes_with_supervision_without_landing(monkeypatch):
    calls = []
    drone = SimpleNamespace(flight_mode="ALT_HOLD", rc_channel_count=8)

    def takeoff(_altitude):
        raise HumanControlTaken("pilot took control")

    drone.takeoff = takeoff
    drone.supervise_human = lambda: calls.append("supervised through pilot disarm")
    record = SimpleNamespace(event=lambda name, **fields: calls.append((name, fields)))

    @contextmanager
    def session(_args):
        yield drone, record

    @contextmanager
    def operator(_args):
        yield lambda: False, lambda: True

    monkeypatch.setattr(control, "_flight_session", session)
    monkeypatch.setattr(control, "_operator_link", operator)
    args = control._parser().parse_args(
        [
            "hover",
            "--confirm-flight",
            control.FLIGHT_CONFIRMATION,
        ]
    )
    assert control.cmd_hover(args) == 0
    assert "supervised through pilot disarm" in calls
    assert calls[-1] == ("pilot_disarmed", {})
