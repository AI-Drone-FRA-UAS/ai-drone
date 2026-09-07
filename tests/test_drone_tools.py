"""Unit tests for the hardware-test entry points."""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

from ai_drone.cli import servo
from ai_drone.platform import is_raspberry_pi


def test_platform_detection_rejects_non_pi(tmp_path) -> None:
    model = tmp_path / "model"
    model.write_text("Desktop PC")

    assert is_raspberry_pi(model) is False


def test_servo_input_parsing() -> None:
    assert servo._target_value_from_input("1500us", min_us=900, max_us=2100) == 0.0
    assert servo._target_value_from_input("30deg", min_us=900, max_us=2100) == 0.5
    assert servo._target_value_from_input("-0.25", min_us=900, max_us=2100) == -0.25

    with pytest.raises(ValueError, match=r"between -1\.0 and 1\.0"):
        servo._target_value_from_input("nan", min_us=900, max_us=2100)


@pytest.mark.parametrize(
    "arguments",
    [
        ["--confirm-actuation", "SERVO_CLEAR"],
        ["--mode", "manual"],
        ["--mode", "manual", "--confirm-actuation", "servo_clear"],
        [
            "--mode",
            "manual",
            "--confirm-actuation",
            "SERVO_CLEAR",
            "--min-us",
            "1500",
        ],
        [
            "--mode",
            "sweep",
            "--confirm-actuation",
            "SERVO_CLEAR",
            "--sweep-step",
            "nan",
        ],
        [
            "--mode",
            "manual",
            "--confirm-actuation",
            "SERVO_CLEAR",
            "--pin",
            "13",
        ],
        [
            "--mode",
            "manual",
            "--confirm-actuation",
            "SERVO_CLEAR",
            "--min-us",
            "500",
        ],
        [
            "--mode",
            "manual",
            "--confirm-actuation",
            "SERVO_CLEAR",
            "--max-us",
            "2500",
        ],
    ],
)
def test_servo_rejects_unsafe_arguments_before_platform_or_gpio(
    monkeypatch,
    arguments,
) -> None:
    def unexpected_platform_check() -> bool:
        raise AssertionError("platform and GPIO checks must follow argument validation")

    monkeypatch.setattr(servo, "is_raspberry_pi", unexpected_platform_check)

    with pytest.raises(SystemExit) as error:
        servo.main(arguments)

    assert error.value.code == 2


def test_servo_initializes_only_after_exact_confirmation(monkeypatch) -> None:
    instances = []

    class FakeServo:
        def __init__(
            self,
            pin,
            *,
            min_pulse_width,
            max_pulse_width,
            initial_value,
        ):
            self.pin = pin
            self.min_pulse_width = min_pulse_width
            self.max_pulse_width = max_pulse_width
            self.initial_value = initial_value
            self.closed = False
            instances.append(self)

        def close(self) -> None:
            self.closed = True

    monkeypatch.setattr(servo, "is_raspberry_pi", lambda: True)
    monkeypatch.setitem(sys.modules, "gpiozero", SimpleNamespace(Servo=FakeServo))
    monkeypatch.setattr("builtins.input", lambda _prompt: "q")

    result = servo.main(["--mode", "manual", "--confirm-actuation", "SERVO_CLEAR"])

    assert result == 0
    assert len(instances) == 1
    assert instances[0].pin == 12
    assert instances[0].initial_value is None
    assert instances[0].closed is True


@pytest.mark.parametrize(
    ("minimum", "maximum", "requested", "normalized"),
    [
        (900, 2200, 1500, -1 / 13),
        (900, 2200, 1550, 0),
        (800, 2100, 1500, 1 / 13),
        (900, 2200, 900, -1),
        (900, 2200, 2200, 1),
    ],
)
def test_asymmetric_pulse_commands_and_reported_width_agree(
    minimum, maximum, requested, normalized
):
    value = servo._target_value_from_input(
        f"{requested}us", min_us=minimum, max_us=maximum
    )
    assert value == pytest.approx(normalized)
    assert servo._pulse_us(value, min_us=minimum, max_us=maximum) == requested


def test_asymmetric_pulse_commands_match_gpiozero_mock_pwm_output():
    gpiozero = pytest.importorskip("gpiozero")
    mock = pytest.importorskip("gpiozero.pins.mock")

    factory = mock.MockFactory(pin_class=mock.MockPWMPin)
    with gpiozero.Servo(
        12,
        min_pulse_width=0.0009,
        max_pulse_width=0.0022,
        initial_value=None,
        pin_factory=factory,
    ) as output:
        for requested in (900, 1200, 1500, 1800, 2200):
            output.value = servo._target_value_from_input(
                f"{requested}us", min_us=900, max_us=2200
            )
            assert factory.pin(12).state / factory.pin(12).frequency == pytest.approx(
                requested / 1_000_000
            )


def test_standalone_servo_initialization_interrupt_releases_process_lock(monkeypatch):
    lock = SimpleNamespace(closed=False)
    lock.close = lambda: setattr(lock, "closed", True)

    def interrupted(*_a, **_kw):
        raise KeyboardInterrupt

    monkeypatch.setattr(servo, "is_raspberry_pi", lambda: True)
    monkeypatch.setattr(servo, "ServoProcessLock", lambda: lock)
    monkeypatch.setitem(sys.modules, "gpiozero", SimpleNamespace(Servo=interrupted))
    assert servo.main(["--mode", "manual", "--confirm-actuation", "SERVO_CLEAR"]) == 130
    assert lock.closed
