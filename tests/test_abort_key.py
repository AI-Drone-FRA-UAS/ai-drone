"""The one guard that does not wait for a measurement.

Every automatic guard acts on something it can measure, and on 2026-08-21 the
measurements all looked healthy while the aircraft was being flown into a
ceiling. These cover the keyboard abort that puts the operator back in the
loop, and the forced disarm it triggers.
"""

from __future__ import annotations

import io
import os
import pty
from unittest.mock import MagicMock

import pytest

from ai_drone import DroneController, FlightSafetyError
from ai_drone.abort_key import AbortKey


def test_a_watcher_without_a_terminal_says_so_rather_than_pretending() -> None:
    key = AbortKey(stream=io.StringIO())

    key.start()
    try:
        assert not key.armed
        assert not key.requested()
        # The operator has to be able to read this and know they are on their
        # own; a silent no-op would be worse than no abort key at all.
        assert "NOT ARMED" in key.describe()
        assert "cut power physically" in key.describe()
    finally:
        key.close()


def test_a_keypress_on_a_real_terminal_is_seen() -> None:
    controller_fd, terminal_fd = pty.openpty()
    try:
        with AbortKey(stream=os.fdopen(terminal_fd, "rb", buffering=0)) as key:
            assert key.armed, key.reason
            assert not key.requested()
            os.write(controller_fd, b" ")
            for _ in range(50):
                if key.requested():
                    break
                import time

                time.sleep(0.05)
            assert key.requested()
            assert "ARMED" in key.describe()
    finally:
        os.close(controller_fd)


def test_the_terminal_is_handed_back_the_way_it_was_found() -> None:
    termios = pytest.importorskip("termios")
    controller_fd, terminal_fd = pty.openpty()

    def echo_and_canonical(fd: int) -> tuple[bool, bool]:
        """The two settings cbreak turns off, read back from the driver.

        Compared instead of the whole attribute list because the terminal
        driver owns transient bits of its own -- PENDIN among them -- that it
        sets and clears without being asked.
        """

        lflag = termios.tcgetattr(fd)[3]
        return bool(lflag & termios.ECHO), bool(lflag & termios.ICANON)

    try:
        before = echo_and_canonical(terminal_fd)
        assert before == (True, True)
        with AbortKey(stream=os.fdopen(terminal_fd, "rb", buffering=0)) as key:
            assert key.armed
            assert echo_and_canonical(terminal_fd) == (False, False)
        # Raw mode left behind makes the shell that reported the crash unusable.
        assert echo_and_canonical(terminal_fd) == before
    finally:
        os.close(controller_fd)


def _controller() -> tuple[DroneController, MagicMock]:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    connection.recv_match.return_value = None
    connection.mode_mapping.return_value = {"LAND": 9, "STABILIZE": 0}
    controller.connection = connection
    controller.is_armed = True
    return controller, connection


def _forced_disarms(connection: MagicMock) -> list[object]:
    return [
        call
        for call in connection.mav.command_long_send.call_args_list
        if call.args[2] == 400 and call.args[5] == DroneController.FORCE_DISARM_MAGIC
    ]


def test_the_key_cuts_the_motors_rather_than_requesting_a_landing() -> None:
    controller, connection = _controller()
    controller.abort_requested = lambda: True

    with pytest.raises(FlightSafetyError, match="operator pressed the abort key"):
        controller.update_telemetry()

    assert len(_forced_disarms(connection)) == 1
    # LAND is altitude controlled, and answering a panic key with it is what
    # this whole mechanism exists to avoid.
    assert not connection.mav.set_mode_send.called


def test_no_key_means_no_interference() -> None:
    controller, connection = _controller()
    controller.abort_requested = lambda: False

    controller.update_telemetry()

    assert not _forced_disarms(connection)


def test_a_broken_abort_hook_neither_stops_nor_masks_a_flight() -> None:
    controller, connection = _controller()

    def raises() -> bool:
        raise RuntimeError("the watcher thread died")

    controller.abort_requested = raises

    # A broken hook must not stop a flight nobody asked to stop, and must not
    # take the flight down with it either.
    controller.update_telemetry()
    assert not _forced_disarms(connection)


def test_landing_cannot_swallow_the_operators_decision() -> None:
    controller, connection = _controller()
    controller.abort_requested = lambda: True

    # ensure_landed deliberately swallows FlightSafetyError so a landing is
    # never interrupted.  The abort key must survive that.
    controller.ensure_landed(timeout=1.0, retry_every=0.1)

    assert _forced_disarms(connection)
    assert not connection.mav.set_mode_send.called
