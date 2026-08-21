"""A keyboard abort the operator can reach before any guard notices.

Every automatic guard in this project acts on something it can measure. On
2026-08-21 the aircraft was flown into a ceiling by its own abort path, and
nothing measurable said so until it was over: the rangefinder read 0.02 m, the
battery was fine, telemetry was fresh. The person watching knew within a
second.

So this exists to put that person in the loop. It watches the terminal for a
single keypress and reports it; what to do about it is the controller's
business, not this module's.

Two properties matter more than the feature itself:

- It says whether it is actually armed. A watcher that silently did nothing --
  because there is no terminal, because stdin is a pipe -- would be worse than
  no watcher at all, because the operator would be relying on it.
- It restores the terminal it borrowed. Raw mode left behind makes the shell
  that reported the crash unusable.
"""

from __future__ import annotations

import contextlib
import os
import select
import sys
import threading
from collections.abc import Callable
from typing import IO, Any

# Long enough that the watcher costs nothing, short enough that shutdown after
# the flight is not perceptibly delayed.
_POLL_SECONDS = 0.2


class AbortKey:
    """Watch a terminal for one keypress, on a background thread.

    Used as a context manager. ``requested()`` is safe to call from any thread
    and never blocks, so a flight loop can consult it on every pass.
    """

    def __init__(self, stream: IO[Any] | None = None) -> None:
        self._stream = stream if stream is not None else sys.stdin
        self._pressed = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._restore: Callable[[], None] | None = None
        self.armed = False
        self.reason = "not started"

    # -- lifecycle ------------------------------------------------------

    def __enter__(self) -> AbortKey:
        self.start()
        return self

    def __exit__(self, *_exception: Any) -> None:
        self.close()

    def start(self) -> None:
        descriptor = self._descriptor()
        if descriptor is None:
            return
        if not self._enter_raw_mode(descriptor):
            return
        self._thread = threading.Thread(
            target=self._watch, args=(descriptor,), name="abort-key", daemon=True
        )
        self._thread.start()
        self.armed = True
        self.reason = "armed"

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=_POLL_SECONDS * 5)
            self._thread = None
        restore = self._restore
        self._restore = None
        if restore is not None:
            # Never let a terminal-restore failure mask the flight result.
            with contextlib.suppress(Exception):
                restore()
        self.armed = False

    # -- what the flight loop asks --------------------------------------

    def requested(self) -> bool:
        """Whether a key has been pressed. Never blocks."""

        return self._pressed.is_set()

    def describe(self) -> str:
        """One line an operator can act on, armed or not."""

        if self.armed:
            return "abort key ARMED: press any key to cut the motors"
        return (
            f"abort key NOT ARMED ({self.reason}). Nothing you type will stop "
            "the aircraft; be ready to cut power physically."
        )

    # -- internals -------------------------------------------------------

    def _descriptor(self) -> int | None:
        try:
            descriptor = self._stream.fileno()
        except (AttributeError, OSError, ValueError):
            self.reason = "stdin has no file descriptor"
            return None
        try:
            interactive = os.isatty(descriptor)
        except OSError:
            interactive = False
        if not interactive:
            self.reason = "stdin is not a terminal; run the command with 'ssh -t'"
            return None
        return descriptor

    def _enter_raw_mode(self, descriptor: int) -> bool:
        """Put the terminal in cbreak mode so one key registers without Enter."""

        try:
            import termios
            import tty
        except ImportError:  # pragma: no cover - POSIX only
            self.reason = "no termios on this platform"
            return False
        try:
            saved = termios.tcgetattr(descriptor)
        except termios.error as error:
            self.reason = f"could not read terminal settings ({error})"
            return False

        def restore() -> None:
            termios.tcsetattr(descriptor, termios.TCSADRAIN, saved)

        try:
            tty.setcbreak(descriptor)
        except termios.error as error:
            self.reason = f"could not set cbreak mode ({error})"
            return False
        self._restore = restore
        return True

    def _watch(self, descriptor: int) -> None:
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([descriptor], [], [], _POLL_SECONDS)
            except (OSError, ValueError):
                return
            if not ready:
                continue
            try:
                data = os.read(descriptor, 1)
            except OSError:
                return
            if data:
                self._pressed.set()
                return
