"""Unit tests for the payload mount servo interface."""

from __future__ import annotations

from typing import ClassVar

import pytest

from ai_drone.mount import (
    DEFAULT_MAX_PULSE_US,
    DEFAULT_MIN_PULSE_US,
    MOUNT_CLOSE_VALUE,
    MOUNT_OPEN_VALUE,
    MountController,
    parse_servo_input,
    pulse_us,
)


class FakeServo:
    """Mock servo to test servo actions offline without Raspberry Pi hardware."""

    instances: ClassVar[list[FakeServo]] = []

    def __init__(
        self,
        pin: int,
        *,
        min_pulse_width: float,
        max_pulse_width: float,
        initial_value: None = None,
    ) -> None:
        self.pin = pin
        self.min_pulse_width = min_pulse_width
        self.max_pulse_width = max_pulse_width
        self.initial_value = initial_value
        self._value: float | None = initial_value
        self.actions: list[tuple[str, float | None]] = []
        self.closed = False
        self.__class__.instances.append(self)

    @property
    def value(self) -> float | None:
        return self._value

    @value.setter
    def value(self, val: float | None) -> None:
        self._value = val
        self.actions.append(("value", val))

    def detach(self) -> None:
        self._value = None
        self.actions.append(("detach", None))

    def close(self) -> None:
        self.closed = True
        self.actions.append(("close", None))


@pytest.fixture(autouse=True)
def _cleanup_controller():
    FakeServo.instances.clear()
    yield
    FakeServo.instances.clear()


def test_open_mount_sets_zero() -> None:
    fake = FakeServo(12, min_pulse_width=0.0009, max_pulse_width=0.0021)
    controller = MountController(servo=fake)

    val = controller.open_mount(settle_s=0.0)

    assert val == MOUNT_OPEN_VALUE
    assert fake.value == 0.0
    assert ("value", 0.0) in fake.actions


def test_close_mount_sets_minus_one() -> None:
    fake = FakeServo(12, min_pulse_width=0.0009, max_pulse_width=0.0021)
    controller = MountController(servo=fake)

    val = controller.close_mount(settle_s=0.0)

    assert val == MOUNT_CLOSE_VALUE
    assert fake.value == -1.0
    assert ("value", -1.0) in fake.actions


def test_manual_set_position_in_range() -> None:
    fake = FakeServo(12, min_pulse_width=0.0009, max_pulse_width=0.0021)
    controller = MountController(servo=fake)

    controller.set_position(0.5, settle_s=0.0)
    assert fake.value == 0.5

    controller.set_position(-0.8, settle_s=0.0)
    assert fake.value == -0.8


def test_manual_set_position_out_of_range_raises() -> None:
    fake = FakeServo(12, min_pulse_width=0.0009, max_pulse_width=0.0021)
    controller = MountController(servo=fake)

    with pytest.raises(ValueError, match="mount value"):
        controller.set_position(1.5, settle_s=0.0)

    with pytest.raises(ValueError, match="mount value"):
        controller.set_position(-1.01, settle_s=0.0)


def test_parse_servo_input_supports_floats_microseconds_degrees() -> None:
    assert parse_servo_input(0.0) == 0.0
    assert parse_servo_input(-1) == -1.0
    assert parse_servo_input("0.5") == 0.5

    # 1500us is center (0.0) with min 900, max 2100
    assert parse_servo_input("1500us", min_us=900, max_us=2100) == 0.0
    # 900us is -1.0
    assert parse_servo_input("900us", min_us=900, max_us=2100) == -1.0
    # 2100us is 1.0
    assert parse_servo_input("2100us", min_us=900, max_us=2100) == 1.0

    # degrees (-60 to +60 mapped to -1 to +1)
    assert parse_servo_input("0deg") == 0.0
    assert parse_servo_input("-60deg") == -1.0
    assert parse_servo_input("60deg") == 1.0


def test_pulse_us_calculation() -> None:
    assert pulse_us(0.0, min_us=900, max_us=2100) == 1500
    assert pulse_us(-1.0, min_us=900, max_us=2100) == 900
    assert pulse_us(1.0, min_us=900, max_us=2100) == 2100


def test_context_manager() -> None:
    fake = FakeServo(12, min_pulse_width=0.0009, max_pulse_width=0.0021)
    with MountController(servo=fake) as controller:
        controller.open_mount(settle_s=0.0)
        assert fake.value == 0.0
        assert not controller.is_closed

    assert controller.is_closed
    assert fake.closed


def test_dry_run_mode() -> None:
    controller = MountController(dry_run=True)
    val_open = controller.open_mount(settle_s=0.0)
    assert val_open == 0.0
    val_close = controller.close_mount(settle_s=0.0)
    assert val_close == -1.0
    controller.close()


def test_default_constants() -> None:
    assert DEFAULT_MIN_PULSE_US == 900
    assert DEFAULT_MAX_PULSE_US == 2100
    assert MOUNT_OPEN_VALUE == 0.0
    assert MOUNT_CLOSE_VALUE == -1.0


def test_cli_mount_actions(capsys: pytest.CaptureFixture[str]) -> None:
    from ai_drone.cli.mount import main as cli_main

    assert cli_main(["open", "--dry-run", "-s", "0"]) == 0
    captured = capsys.readouterr()
    assert "Payload mount is OPEN." in captured.out

    assert cli_main(["close", "--dry-run", "-s", "0"]) == 0
    captured = capsys.readouterr()
    assert "Payload mount is CLOSED." in captured.out

    assert cli_main(["set", "0.25", "--dry-run", "-s", "0"]) == 0
    captured = capsys.readouterr()
    assert "0.25" in captured.out
