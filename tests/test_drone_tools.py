"""Unit tests for the hardware-test entry points."""

from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import pytest

from ai_drone.cli import lidar, servo
from ai_drone.mavlink import console, health
from ai_drone.mavlink import devices as mavlink_devices
from ai_drone.platform import is_raspberry_pi


def test_requested_serial_device_is_used(tmp_path) -> None:
    device = tmp_path / "tty-test"
    device.touch()

    assert lidar._find_device(str(device)) == device


def test_rangefinder_message_becomes_csv_row(monkeypatch) -> None:
    monkeypatch.setattr(lidar.time, "monotonic", lambda: 12.5)
    message = SimpleNamespace(
        get_type=lambda: "RANGEFINDER",
        distance=0.75,
        voltage=4.9,
    )

    row = lidar._row(message, started=10.0)

    assert row["elapsed_s"] == 2.5
    assert row["message"] == "RANGEFINDER"
    assert row["distance_m"] == 0.75
    assert row["voltage_v"] == 4.9


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


def test_drone_console_uses_requested_device(tmp_path, monkeypatch) -> None:
    device = tmp_path / "tty-test"
    device.touch()
    monkeypatch.setattr(console.shutil, "which", lambda _name: "/venv/bin/mavproxy.py")

    command = console._command(
        ["--device", str(device), "--baud", "57600", "--show-errors"]
    )

    assert command == [
        "/venv/bin/mavproxy.py",
        f"--master={device}",
        "--baudrate=57600",
        "--show-errors",
    ]


def test_drone_console_prefers_stable_device(tmp_path, monkeypatch) -> None:
    stable_device = tmp_path / "flywoo"
    stable_device.touch()
    monkeypatch.setattr(console, "STABLE_DEVICE", stable_device)
    monkeypatch.setattr(mavlink_devices.Path, "glob", lambda _path, _pattern: [])
    monkeypatch.setattr(console.shutil, "which", lambda _name: "/venv/bin/mavproxy.py")

    command = console._command([])

    assert command[1] == f"--master={stable_device}"


def test_drone_health_remote_command_contains_requested_connection() -> None:
    command = health._remote_command("/dev/serial0", 115200, 12.0)

    assert command.startswith(".venv/bin/python -m ai_drone.mavlink.health --usb-only")
    assert "--usb-device /dev/serial0" in command
    assert "--baud 115200 --timeout 12.0" in command
    assert "--local-label 'Pi UART'" in command


def test_drone_health_ignores_parameter_from_other_mavlink_source(
    monkeypatch, tmp_path
) -> None:
    requested: list[tuple[int, int, bytes, int]] = []
    messages = iter(
        [
            SimpleNamespace(
                param_id="SYSID_THISMAV",
                param_value=99,
                get_type=lambda: "PARAM_VALUE",
                get_srcSystem=lambda: 2,
                get_srcComponent=lambda: 1,
            ),
            SimpleNamespace(
                param_id="SYSID_THISMAV",
                param_value=1,
                get_type=lambda: "PARAM_VALUE",
                get_srcSystem=lambda: 1,
                get_srcComponent=lambda: 1,
            ),
        ]
    )

    class FakeConnection:
        target_system = 1
        target_component = 1
        mav = SimpleNamespace(
            param_request_read_send=lambda *args: requested.append(args)
        )
        closed = False

        def wait_heartbeat(self, *, timeout):
            return SimpleNamespace(base_mode=0)

        def recv_match(self, **_kwargs):
            return next(messages, None)

        def close(self) -> None:
            self.closed = True

    connection = FakeConnection()
    monkeypatch.setattr(
        health.mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )
    monkeypatch.setattr(
        health,
        "require_fresh_disarmed_heartbeat",
        lambda *_args, **_kwargs: SimpleNamespace(base_mode=0),
    )

    result = health.check_local_link(tmp_path / "serial", timeout=0.1)

    assert result.system_id == 1
    assert requested == [(1, 1, b"SYSID_THISMAV", -1)]
    assert connection.closed is True


def test_drone_health_rejects_an_initially_armed_vehicle(monkeypatch, tmp_path) -> None:
    connection = SimpleNamespace(
        wait_heartbeat=lambda *, timeout: SimpleNamespace(base_mode=128),
        close=lambda: setattr(connection, "closed", True),
        closed=False,
    )
    monkeypatch.setattr(
        health.mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )

    with pytest.raises(RuntimeError, match="vehicle is ARMED"):
        health.check_local_link(tmp_path / "serial", timeout=0.1)

    assert connection.closed is True


def test_drone_health_rejects_vehicle_arming_while_waiting_for_parameter(
    monkeypatch, tmp_path
) -> None:
    armed_heartbeat = SimpleNamespace(
        base_mode=128,
        get_type=lambda: "HEARTBEAT",
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
    )

    class FakeConnection:
        target_system = 1
        target_component = 1
        mav = SimpleNamespace(param_request_read_send=lambda *_args: None)
        closed = False

        def wait_heartbeat(self, *, timeout):
            return SimpleNamespace(base_mode=0)

        def recv_match(self, **_kwargs):
            return armed_heartbeat

        def close(self) -> None:
            self.closed = True

    connection = FakeConnection()
    monkeypatch.setattr(
        health.mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )

    with pytest.raises(RuntimeError, match="vehicle became ARMED"):
        health.check_local_link(tmp_path / "serial", timeout=0.1)

    assert connection.closed is True


def test_drone_health_requires_final_fresh_disarmed_state_after_parameter(
    monkeypatch, tmp_path
) -> None:
    parameter = SimpleNamespace(
        param_id="SYSID_THISMAV",
        param_value=1,
        get_type=lambda: "PARAM_VALUE",
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
    )

    class FakeConnection:
        target_system = 1
        target_component = 1
        mav = SimpleNamespace(param_request_read_send=lambda *_args: None)
        closed = False

        def wait_heartbeat(self, *, timeout):
            return SimpleNamespace(base_mode=0)

        def recv_match(self, **_kwargs):
            return parameter

        def close(self) -> None:
            self.closed = True

    connection = FakeConnection()
    monkeypatch.setattr(
        health.mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )
    monkeypatch.setattr(
        health,
        "require_fresh_disarmed_heartbeat",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("Vehicle reported ARMED")
        ),
    )

    with pytest.raises(RuntimeError, match="ARMED"):
        health.check_local_link(tmp_path / "serial", timeout=0.1)

    assert connection.closed


def test_drone_health_pi_check_uses_ssh(monkeypatch) -> None:
    calls = []

    def fake_run(command, check):
        calls.append((command, check))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(health.subprocess, "run", fake_run)

    assert health.check_pi_link("seb@seb-is-pm") is True
    assert calls[0][0][:6] == [
        "ssh",
        "-F",
        os.devnull,
        "-o",
        "ConnectTimeout=8",
        "seb@seb-is-pm",
    ]


def test_drone_health_threads_explicit_ssh_config_from_environment(
    monkeypatch,
) -> None:
    calls = []

    def fake_check(
        host,
        device,
        baud,
        timeout,
        *,
        ssh_config,
    ):
        calls.append((host, device, baud, timeout, ssh_config))
        return True

    monkeypatch.setattr(health, "check_pi_link", fake_check)

    result = health.run(
        ["--pi-only"],
        environ={
            "PI_HOST": "seb@192.168.4.1",
            "SSH_CONFIG": "/tmp/custom-ssh-config",
        },
    )

    assert result == 0
    assert calls == [
        (
            "seb@192.168.4.1",
            "/dev/serial0",
            115200,
            10.0,
            "/tmp/custom-ssh-config",
        )
    ]
