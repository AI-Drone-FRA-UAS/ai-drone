"""Preserve the proven mount pulse lifecycle without GPIO or real delays."""

from __future__ import annotations

import argparse
import sys
import threading
import time
from types import SimpleNamespace

import pytest

import ai_drone.mount as mount
from ai_drone.capture.state import CaptureState
from ai_drone.cli import mount as mount_cli
from ai_drone.cli import servo as servo_cli
from ai_drone.cli import tag_servo_record as tag_cli


@pytest.fixture
def rig(monkeypatch):
    events = []
    detached = threading.Event()

    class Servo:
        # A strict signature also rejects newly forced pin factories/frame widths.
        def __init__(self, pin, *, min_pulse_width, max_pulse_width, initial_value):
            events.append(
                ("create", pin, min_pulse_width, max_pulse_width, initial_value)
            )
            self._value = initial_value

        @property
        def value(self):
            return self._value

        @value.setter
        def value(self, value):
            self._value = value
            events.append(("value", value))

        def detach(self):
            self._value = None
            events.append(("detach",))
            detached.set()

        def close(self):
            events.append(("close",))

    def lock():
        events.append(("lock",))
        return SimpleNamespace(close=lambda: events.append(("unlock",)))

    monkeypatch.setitem(sys.modules, "gpiozero", SimpleNamespace(Servo=Servo))
    monkeypatch.setattr(mount, "is_raspberry_pi", lambda: True)
    monkeypatch.setattr(mount, "ServoProcessLock", lock)
    monkeypatch.setattr(
        mount.time, "sleep", lambda delay: events.append(("hold", delay))
    )
    return SimpleNamespace(events=events, factory=Servo, lock=lock, detached=detached)


@pytest.mark.parametrize(("action", "value"), [("open", 0.0), ("close", -1.0)])
def test_mount_command_moves_holds_half_second_and_releases(rig, action, value):
    assert mount_cli.main([action]) == 0
    assert rig.events == [
        ("lock",),
        ("create", 12, 0.0009, 0.0021, None),
        ("value", value),
        ("hold", 0.5),
        ("close",),
        ("unlock",),
    ]


def test_interrupted_mount_hold_releases_gpio_before_ownership(rig, monkeypatch):
    def interrupted(delay):
        rig.events.append(("hold", delay))
        raise KeyboardInterrupt

    monkeypatch.setattr(mount.time, "sleep", interrupted)
    assert mount_cli.main(["open"]) == 130
    assert rig.events[-4:] == [("value", 0.0), ("hold", 0.5), ("close",), ("unlock",)]


def test_idle_controller_and_dry_run_never_create_gpio_or_lock(rig, monkeypatch):
    # Importing the real GPIO module must remain unnecessary, even on a non-Pi.
    monkeypatch.setitem(sys.modules, "gpiozero", None)
    monkeypatch.setattr(mount, "is_raspberry_pi", lambda: False)
    with mount.MountController():
        assert rig.events == []
    assert mount_cli.main(["open", "--dry-run"]) == 0
    assert mount_cli.main(["close", "--dry-run"]) == 0
    assert rig.events == [("hold", 0.5), ("hold", 0.5)]


def _tag_mount_session(rig, tmp_path, *, interrupted=False):
    class Stop(tag_cli.ActuationStop):
        def wait(self, timeout=None):
            rig.events.append(("hold", timeout))
            if interrupted:
                self.set()
            return self.is_set()

    stop = Stop()
    ready = threading.Event()
    ready.set()
    state = CaptureState(last_vehicle_heartbeat_monotonic=time.monotonic())
    config = tag_cli.TagServoConfig.from_args(
        argparse.Namespace(**tag_cli.mount_recording_defaults())
    )
    session = tag_cli.TagServoSession(
        config=config,
        event_path=tmp_path / "servo.jsonl",
        state=state,
        capture_stop=stop,
        ready=ready,
        servo_factory=rig.factory,
        process_lock_factory=rig.lock,
    )
    return session, state, stop


@pytest.mark.parametrize("interrupted", [False, True])
def test_tag_mount_opens_holds_detaches_and_never_recloses(rig, tmp_path, interrupted):
    session, state, stop = _tag_mount_session(rig, tmp_path, interrupted=interrupted)
    try:
        assert rig.events == [("lock",), ("create", 12, 0.0009, 0.0021, None)]
        for index in range(3):
            session.observe(
                SimpleNamespace(
                    frame_index=index,
                    elapsed_s=index / 30,
                    captured_monotonic=time.monotonic(),
                ),
                [SimpleNamespace(tag_id=3, hamming=0, decision_margin=60.0)],
                [{"id": 3}],
            )
        assert rig.detached.wait(timeout=2), "tag-mount worker did not release PWM"
        assert rig.events[2:] == [("value", 0.0), ("hold", 0.5), ("detach",)]
        assert stop.is_set() is interrupted
    finally:
        session.close()
    assert state.servo_pulses_completed == (0 if interrupted else 1)
    assert rig.events[2:] == [
        ("value", 0.0),
        ("hold", 0.5),
        ("detach",),
        ("detach",),
        ("close",),
        ("unlock",),
    ]


def test_tag_mount_shutdown_without_detection_never_moves(rig, tmp_path):
    session, _state, _stop = _tag_mount_session(rig, tmp_path)
    session.close()
    assert rig.events == [
        ("lock",),
        ("create", 12, 0.0009, 0.0021, None),
        ("detach",),
        ("close",),
        ("unlock",),
    ]


def test_all_servo_paths_share_exclusive_reusable_lock(tmp_path):
    pytest.importorskip("fcntl")
    assert mount.ServoProcessLock is servo_cli.ServoProcessLock
    assert mount.ServoProcessLock is tag_cli.ServoProcessLock
    path = tmp_path / "servo.lock"
    first = mount.ServoProcessLock(path)
    try:
        for factory in (servo_cli.ServoProcessLock, tag_cli.ServoProcessLock):
            with pytest.raises(RuntimeError, match="already owned"):
                factory(path)
    finally:
        first.close()
    second = tag_cli.ServoProcessLock(path)
    second.close()


@pytest.mark.parametrize("error_type", [OSError, KeyboardInterrupt])
def test_mount_factory_failure_releases_lock_and_allows_retry(
    monkeypatch, tmp_path, error_type
):
    pytest.importorskip("fcntl")
    lock_type = mount.ServoProcessLock
    path = tmp_path / "servo.lock"
    monkeypatch.setattr(mount, "ServoProcessLock", lambda: lock_type(path))

    def failed_factory(*_args, **_kwargs):
        raise error_type("construction failed")

    controller = mount.MountController(servo_factory=failed_factory)
    with pytest.raises(error_type):
        controller.open_mount()
    # This fails if construction left the exclusive lock held.
    retry = lock_type(path)
    retry.close()
    controller.close()
