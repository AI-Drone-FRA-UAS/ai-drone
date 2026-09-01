"""Unit tests for the hardware-test entry points."""

from __future__ import annotations

from types import SimpleNamespace

from ai_drone import console, health
from ai_drone import mavlink_devices
from ai_drone.cli import lidar, picam, servo


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


def test_picam_is_disabled_off_pi(monkeypatch) -> None:
    monkeypatch.setattr(picam.Path, "read_text", lambda _path: "Desktop PC")

    assert picam._is_raspberry_pi() is False


def test_servo_input_parsing() -> None:
    assert servo._target_value_from_input("1500us", min_us=900, max_us=2100) == 0.0
    assert servo._target_value_from_input("30deg", min_us=900, max_us=2100) == 0.5
    assert servo._target_value_from_input("-0.25", min_us=900, max_us=2100) == -0.25


def test_servo_conversions() -> None:
    assert servo._pulse_to_value(1500, min_us=900, max_us=2100) == 0.0
    assert servo._pulse_to_value(2100, min_us=900, max_us=2100) == 1.0
    assert servo._pulse_to_value(900, min_us=900, max_us=2100) == -1.0
    assert servo._value_to_pulse(0.0, min_us=900, max_us=2100) == 1500
    assert servo._value_to_pulse(1.0, min_us=900, max_us=2100) == 2100
    assert servo._value_to_pulse(-1.0, min_us=900, max_us=2100) == 900
    assert servo._pulse_to_angle(1500, min_us=900, max_us=2100) == 0.0
    assert servo._pulse_to_angle(2100, min_us=900, max_us=2100) == 60.0
    assert servo._pulse_to_angle(900, min_us=900, max_us=2100) == -60.0


def test_servo_mock_simulation() -> None:
    mock = servo.MockServo(18)
    assert mock.pin == 18
    assert mock.value == 0.0
    mock.value = 0.5
    assert mock.value == 0.5
    mock.close()
    assert mock.is_closed is True



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

    assert command.startswith(".venv/bin/python -c ")
    assert command.endswith("/dev/serial0 115200 12.0")


def test_drone_health_pi_check_uses_ssh(monkeypatch) -> None:
    calls = []

    def fake_run(command, check):
        calls.append((command, check))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(health.subprocess, "run", fake_run)

    assert health.check_pi_link("seb@seb-is-pm") is True
    assert calls[0][0][:4] == [
        "ssh",
        "-o",
        "ConnectTimeout=8",
        "seb@seb-is-pm",
    ]
