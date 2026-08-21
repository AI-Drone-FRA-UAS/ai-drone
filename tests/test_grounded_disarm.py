"""An aircraft on the floor with its motors turning has to be stopped.

Twice on 2026-08-21 a climb that never lifted ended with the vehicle armed in
ALT_HOLD, motors at 1060, until somebody noticed. Two separate faults did that:
the teardown only verified a disarm when it believed the aircraft was flying,
and ArduPilot refuses an ordinary disarm while it believes it is flying -- which
on this airframe is exactly the belief that fails. The vehicle reported
climbing at +0.96 m/s while its rangefinder read 0.02 m for all 71 samples.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock

import pytest

from ai_drone import DroneController


def _grounded_and_refusing(monkeypatch) -> tuple[DroneController, MagicMock]:
    """A vehicle sitting at ground level that never reports itself disarmed."""

    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    connection.recv_match.return_value = None
    connection.mode_mapping.return_value = {"LAND": 9, "ALT_HOLD": 2, "STABILIZE": 0}
    controller.connection = connection
    controller.is_armed = True
    controller.flight_mode = "ALT_HOLD"
    controller.current_altitude = 0.02
    controller._ground_reference = 0.02
    # The estimate that made ArduPilot refuse: a climb the floor contradicts.
    controller.local_position_climb = 0.96
    monkeypatch.setattr(controller, "altitude_is_fresh", lambda *_a, **_k: True)
    monkeypatch.setattr(controller, "update_telemetry", lambda *_a, **_k: None)
    return controller, connection


def test_a_refused_disarm_on_the_ground_escalates_to_forcing_the_motors_off(
    monkeypatch,
) -> None:
    controller, _connection = _grounded_and_refusing(monkeypatch)
    monkeypatch.setattr(DroneController, "GROUNDED_FORCE_DISARM_AFTER_S", 0.0)
    forced: list[bool] = []
    monkeypatch.setattr(controller, "stop_now", lambda: forced.append(True))

    assert controller.ensure_landed(timeout=2.0, retry_every=0.1) is False

    assert forced, "the aircraft was left armed on the ground"


def test_an_ordinary_disarm_is_tried_before_anything_is_forced(monkeypatch) -> None:
    controller, _connection = _grounded_and_refusing(monkeypatch)
    order: list[str] = []
    monkeypatch.setattr(controller, "_request_disarm", lambda: order.append("ordinary"))
    monkeypatch.setattr(controller, "stop_now", lambda: order.append("forced"))

    controller.ensure_landed(timeout=4.0, retry_every=0.1)

    assert "ordinary" in order
    assert order.index("ordinary") < order.index("forced")


def test_a_vehicle_that_disarms_normally_is_never_forced(monkeypatch) -> None:
    controller, _connection = _grounded_and_refusing(monkeypatch)
    monkeypatch.setattr(DroneController, "GROUNDED_FORCE_DISARM_AFTER_S", 0.0)
    forced: list[bool] = []
    monkeypatch.setattr(controller, "stop_now", lambda: forced.append(True))

    def disarm_for_real() -> None:
        controller.is_armed = False

    monkeypatch.setattr(controller, "_request_disarm", disarm_for_real)
    monkeypatch.setattr(controller, "emergency_stop", disarm_for_real)

    assert controller.ensure_landed(timeout=2.0, retry_every=0.1) is True
    assert forced == []


def test_a_stale_rangefinder_is_not_grounds_for_forcing_a_disarm(monkeypatch) -> None:
    # Forcing the motors off is only safe because the floor is right there.
    # Without a fresh reading nobody can say that, and an airborne aircraft
    # must not be dropped on the strength of a number nobody can vouch for.
    controller, _connection = _grounded_and_refusing(monkeypatch)
    monkeypatch.setattr(DroneController, "GROUNDED_FORCE_DISARM_AFTER_S", 0.0)
    monkeypatch.setattr(controller, "altitude_is_fresh", lambda *_a, **_k: False)
    forced: list[bool] = []
    monkeypatch.setattr(controller, "stop_now", lambda: forced.append(True))

    controller.ensure_landed(timeout=1.5, retry_every=0.1)

    assert forced == []


def test_settling_needs_the_reading_to_hold_not_a_single_sample() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.current_altitude = 0.02
    controller._ground_reference = 0.02
    controller.last_telemetry_time = time.monotonic()

    assert controller.settled_on_the_ground() is False
    time.sleep(0.05)
    assert controller.settled_on_the_ground(for_seconds=0.01) is True


@pytest.mark.parametrize("flying", [True, False])
def test_the_teardown_verifies_a_disarm_whether_or_not_it_flew(
    monkeypatch, flying: bool
) -> None:
    # The is_flying guard is what let a grounded aircraft keep running: a
    # climb that never lifted never set it, so nothing ever checked whether
    # the single disarm request had been accepted.
    import argparse
    import contextlib

    from ai_drone.cli import control

    drone = MagicMock()
    drone.is_flying = flying
    drone.ensure_landed.return_value = True

    @contextlib.contextmanager
    def fake_controller(_args):
        yield drone

    class FakeWatcher:
        installed = ("SIGHUP",)
        reason = "armed"

        def __enter__(self):
            return self

        def __exit__(self, *_exc):
            return False

        def describe(self) -> str:
            return "watcher"

        def requested(self) -> bool:
            return False

    monkeypatch.setattr(control, "_controller", fake_controller)
    monkeypatch.setattr(control, "AbortOnHangUp", FakeWatcher)
    monkeypatch.setattr(control, "AbortKey", FakeWatcher)
    monkeypatch.setattr(control, "FlightRecorder", lambda *_a, **_k: MagicMock())

    args = argparse.Namespace(takeoff_alt=0.25, handler=None, confirm_flight="x")
    with (
        pytest.raises(RuntimeError, match="guard fired"),
        control._flight_session(args),
    ):
        raise RuntimeError("guard fired")

    drone.ensure_landed.assert_called_once()


def test_a_flight_that_ends_cleanly_is_not_landed_a_second_time(monkeypatch) -> None:
    import argparse
    import contextlib

    from ai_drone.cli import control

    drone = MagicMock()
    drone.is_flying = False

    @contextlib.contextmanager
    def fake_controller(_args):
        yield drone

    class FakeWatcher:
        installed = ()
        reason = "armed"

        def __enter__(self):
            return self

        def __exit__(self, *_exc):
            return False

        def describe(self) -> str:
            return "watcher"

        def requested(self) -> bool:
            return False

    monkeypatch.setattr(control, "_controller", fake_controller)
    monkeypatch.setattr(control, "AbortOnHangUp", FakeWatcher)
    monkeypatch.setattr(control, "AbortKey", FakeWatcher)
    monkeypatch.setattr(control, "FlightRecorder", lambda *_a, **_k: MagicMock())
    monkeypatch.setattr(control, "latest_dataflash_log", lambda _c: None)

    args = argparse.Namespace(takeoff_alt=0.25, handler=None, confirm_flight="x")
    with control._flight_session(args):
        pass

    drone.ensure_landed.assert_not_called()
