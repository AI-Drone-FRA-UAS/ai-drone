"""Safety and target qualification for the standalone disarmed mount check."""

import importlib.util
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

_spec = importlib.util.spec_from_file_location(
    "disarmed_tag_mount", Path(__file__).parents[1] / "scripts/disarmed_tag_mount.py"
)
assert _spec is not None and _spec.loader is not None
mount_check = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = mount_check
_spec.loader.exec_module(mount_check)


def tag(tid=3, hamming=0, margin=50):
    return SimpleNamespace(tag_id=tid, hamming=hamming, decision_margin=margin)


def test_only_tag_three_opens_once():
    gate = mount_check.TagGate()
    assert not gate.observe(0, 10, [tag(0), tag(2)], 10.1)
    assert not gate.observe(1, 10, [tag()], 10.1)
    assert not gate.observe(2, 10, [tag()], 10.1)
    assert gate.observe(3, 10, [tag()], 10.1)
    assert not gate.observe(4, 10, [tag()], 10.1)


@pytest.mark.parametrize(
    "bad",
    [[], [tag(2)], [tag(hamming=1)], [tag(margin=29)], [tag(margin=float("nan"))]],
)
def test_bad_detection_breaks_confirmation(bad):
    gate = mount_check.TagGate()
    gate.observe(0, 10, [tag()], 10.1)
    gate.observe(1, 10, [tag()], 10.1)
    assert not gate.observe(2, 10, bad, 10.1)
    assert not gate.observe(3, 10, [tag()], 10.1)


def test_stale_and_skipped_frames_cannot_complete_confirmation():
    gate = mount_check.TagGate()
    gate.observe(0, 10, [tag()], 10.1)
    gate.observe(1, 10, [tag()], 10.1)
    assert not gate.observe(2, 10, [tag()], 11)
    gate.observe(3, 11, [tag()], 11.1)
    assert not gate.observe(5, 11, [tag()], 11.1)
    assert not gate.observe(6, 11, [tag()], 11.1)


@pytest.mark.parametrize("armed", [True, False])
def test_armed_or_stale_heartbeat_prevents_any_command(monkeypatch, armed):
    class Worker:
        def __init__(self, **kwargs):
            pass

        def start(self):
            pass

        def join(self, **kwargs):
            pass

        def is_alive(self):
            return False

    message = SimpleNamespace(
        get_type=lambda: "HEARTBEAT",
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
        base_mode=128,
    )
    connection = SimpleNamespace(recv_match=lambda **_: message if armed else None)
    times = iter([10.0, 13.0])
    monkeypatch.setattr(mount_check.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(mount_check.threading, "Thread", Worker)
    monkeypatch.setattr(
        mount_check.subprocess, "Popen", lambda *a, **k: pytest.fail("actuated")
    )
    with pytest.raises(RuntimeError, match=r"ARMED|heartbeat older"):
        mount_check.supervise(connection, 30, "uv", threading.Event())


@pytest.mark.parametrize("target, expected", [(3, ["close", "open"]), (2, ["close"])])
def test_supervisor_runs_mount_commands_in_order(monkeypatch, target, expected):
    clock = [10.0]
    frame_queue = []
    calls = []

    class Worker:
        def __init__(self, *, args, **kwargs):
            frame_queue.append(args[1])

        def start(self):
            pass

        def join(self, **kwargs):
            pass

        def is_alive(self):
            return False

    index = [-1]

    def drain(connection, last):
        clock[0] += 0.1
        index[0] += 1
        frame_queue[0].put_nowait((index[0], clock[0], [tag(target)]))
        return clock[0]

    def command(args, **kwargs):
        calls.append(args[-1])
        assert args[:-1] == [
            "uv",
            "run",
            "--no-sync",
            "--group",
            "raspi",
            "python",
            "-m",
            "ai_drone.cli.mount",
        ]
        return SimpleNamespace(poll=lambda: 0)

    monkeypatch.setattr(mount_check.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(mount_check.threading, "Thread", Worker)
    monkeypatch.setattr(mount_check, "drain_heartbeats", drain)
    monkeypatch.setattr(mount_check.subprocess, "Popen", command)
    result = mount_check.supervise(None, 1, "uv", threading.Event())
    assert result == (0 if target == 3 else 2)
    assert calls == expected
