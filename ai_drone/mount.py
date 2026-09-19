"""Payload mount servo control on BCM GPIO 12."""

from __future__ import annotations

import math
import os
import time
from collections.abc import Callable
from contextlib import AbstractContextManager
from pathlib import Path
from typing import Any

from ai_drone.platform import is_raspberry_pi
from ai_drone.validation import finite_in_range

SERVO_GPIO_PIN = 12
DEFAULT_MIN_PULSE_US = 900
DEFAULT_MAX_PULSE_US = 2100
ABSOLUTE_MIN_PULSE_US = 750
ABSOLUTE_MAX_PULSE_US = 2250
DEFAULT_SETTLE_S = 0.5

# Standard mount positions
MOUNT_OPEN_VALUE = 0.0
MOUNT_CLOSE_VALUE = -1.0

_LOCK_PATH = Path("/tmp/ai-drone-bcm12-servo.lock")


def parse_servo_input(
    raw: str | float | int,
    *,
    min_us: int = DEFAULT_MIN_PULSE_US,
    max_us: int = DEFAULT_MAX_PULSE_US,
) -> float:
    """Parse a manual servo command (float -1..1, microseconds, or degrees).

    Corresponds to the manual input parsing in ``servo.py``:
    - Numeric or string float between -1.0 and 1.0 (e.g. ``0.0``, ``-1``)
    - Pulse width string ending with 'us' (e.g. ``1500us``)
    - Angle string ending with 'deg' between -60 and +60 (e.g. ``0deg``)
    """
    if isinstance(raw, (int, float)):
        return finite_in_range(float(raw), "mount value", minimum=-1.0, maximum=1.0)

    command = str(raw).strip().lower()
    if command.endswith("us"):
        pulse_us = int(command.removesuffix("us").strip())
        if not min_us <= pulse_us <= max_us:
            raise ValueError(f"pulse width must be between {min_us} and {max_us}us")
        return 2.0 * (pulse_us - min_us) / (max_us - min_us) - 1.0

    if command.endswith("deg"):
        degrees = float(command.removesuffix("deg").strip())
        if not math.isfinite(degrees) or not -60.0 <= degrees <= 60.0:
            raise ValueError("angle must be between -60 and +60 degrees")
        return degrees / 60.0

    target = float(command)
    return finite_in_range(target, "mount value", minimum=-1.0, maximum=1.0)


def pulse_us(
    value: float,
    *,
    min_us: int = DEFAULT_MIN_PULSE_US,
    max_us: int = DEFAULT_MAX_PULSE_US,
) -> int:
    """Convert a normalized value [-1.0, 1.0] to a pulse width in microseconds."""
    validated = finite_in_range(value, "mount value", minimum=-1.0, maximum=1.0)
    return round(min_us + (validated + 1.0) * (max_us - min_us) / 2.0)


class ServoProcessLock:
    """Prevent cooperating ai-drone processes from sharing BCM12 concurrently."""

    def __init__(self, path: Path = _LOCK_PATH) -> None:
        import fcntl

        self._fcntl = fcntl

        descriptor = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
        self._handle = os.fdopen(descriptor, "r+")
        try:
            self._fcntl.flock(
                self._handle.fileno(),
                self._fcntl.LOCK_EX | self._fcntl.LOCK_NB,
            )
        except BaseException as error:
            self._handle.close()
            if isinstance(error, BlockingIOError):
                raise RuntimeError(
                    f"payload servo GPIO {SERVO_GPIO_PIN} is already owned by another "
                    "ai-drone process"
                ) from error
            raise

    def close(self) -> None:
        if self._handle.closed:
            return
        try:
            self._fcntl.flock(self._handle.fileno(), self._fcntl.LOCK_UN)
        finally:
            self._handle.close()


def create_servo(
    pin: int = SERVO_GPIO_PIN,
    *,
    min_us: int = DEFAULT_MIN_PULSE_US,
    max_us: int = DEFAULT_MAX_PULSE_US,
    factory: Callable[..., Any] | None = None,
) -> Any:
    """Start with PWM off, retaining gpiozero's configured pin factory."""
    if factory is None:
        try:
            from gpiozero import Servo  # ty: ignore[unresolved-import]
        except ImportError as error:
            raise RuntimeError(
                "gpiozero is required on the Pi: sudo apt install python3-gpiozero"
            ) from error
        factory = Servo
    return factory(
        pin,
        min_pulse_width=min_us / 1_000_000.0,
        max_pulse_width=max_us / 1_000_000.0,
        initial_value=None,
    )


class MountController(AbstractContextManager["MountController"]):
    """Controller for the payload mount mechanism via a PWM micro servo."""

    def __init__(
        self,
        pin: int = SERVO_GPIO_PIN,
        *,
        min_pulse_us: int = DEFAULT_MIN_PULSE_US,
        max_pulse_us: int = DEFAULT_MAX_PULSE_US,
        servo: Any | None = None,
        servo_factory: Callable[..., Any] | None = None,
        dry_run: bool = False,
    ) -> None:
        if not min_pulse_us < 1500 < max_pulse_us:
            raise ValueError(
                "pulse geometry must satisfy min_pulse_us < 1500 < max_pulse_us"
            )
        if min_pulse_us < ABSOLUTE_MIN_PULSE_US:
            raise ValueError(f"min_pulse_us must be at least {ABSOLUTE_MIN_PULSE_US}")
        if max_pulse_us > ABSOLUTE_MAX_PULSE_US:
            raise ValueError(f"max_pulse_us must be at most {ABSOLUTE_MAX_PULSE_US}")

        self.pin = pin
        self.min_pulse_us = min_pulse_us
        self.max_pulse_us = max_pulse_us
        self.dry_run = dry_run

        self._lock: ServoProcessLock | None = None
        self._servo: Any | None = servo
        self._current_value: float | None = None
        self._is_closed = False

        self._servo_factory = servo_factory

    @property
    def is_closed(self) -> bool:
        return self._is_closed

    @property
    def current_value(self) -> float | None:
        return self._current_value

    def _ensure_servo(self) -> Any:
        if self._is_closed:
            raise RuntimeError("MountController is already closed.")
        if self._servo is not None:
            return self._servo

        if self.dry_run:
            return None

        if self._servo_factory is None and not is_raspberry_pi():
            raise RuntimeError(
                "Servo control requires a Raspberry Pi with BCM GPIO 12. "
                "For testing on non-Pi platforms, pass dry_run=True or a mock servo."
            )

        self._lock = ServoProcessLock()
        try:
            self._servo = create_servo(
                self.pin,
                min_us=self.min_pulse_us,
                max_us=self.max_pulse_us,
                factory=self._servo_factory,
            )
        except BaseException:
            if self._lock is not None:
                self._lock.close()
                self._lock = None
            raise

        return self._servo

    def set_position(
        self,
        value: float | int | str,
        *,
        settle_s: float = DEFAULT_SETTLE_S,
    ) -> float:
        """Set servo target value directly in the normalized range [-1.0, 1.0]."""
        target = parse_servo_input(
            value,
            min_us=self.min_pulse_us,
            max_us=self.max_pulse_us,
        )
        servo = self._ensure_servo()
        if servo is not None:
            servo.value = target
        self._current_value = target

        if settle_s > 0:
            time.sleep(settle_s)

        return target

    def open_mount(self, *, settle_s: float = DEFAULT_SETTLE_S) -> float:
        """Open the mount holder by steering the servo to 0.0."""
        return self.set_position(MOUNT_OPEN_VALUE, settle_s=settle_s)

    def close_mount(self, *, settle_s: float = DEFAULT_SETTLE_S) -> float:
        """Close the mount holder by steering the servo to -1.0."""
        return self.set_position(MOUNT_CLOSE_VALUE, settle_s=settle_s)

    def detach(self) -> None:
        """Stop active PWM pulse generation."""
        if self._servo is not None and hasattr(self._servo, "detach"):
            self._servo.detach()
        elif self._servo is not None and hasattr(self._servo, "value"):
            self._servo.value = None
        self._current_value = None

    def close(self) -> None:
        """Release the servo and process lock."""
        if self._is_closed:
            return
        self._is_closed = True
        try:
            if self._servo is not None:
                if hasattr(self._servo, "close"):
                    self._servo.close()
                self._servo = None
        finally:
            if self._lock is not None:
                self._lock.close()
                self._lock = None
        self._current_value = None

    def __enter__(self) -> MountController:
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()
