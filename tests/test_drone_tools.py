"""Unit tests for the hardware-test entry points."""

from __future__ import annotations

from types import SimpleNamespace

from ai_drone import console, health
import test_lidar
import test_picam


def test_requested_serial_device_is_used(tmp_path) -> None:
    device = tmp_path / "tty-test"
    device.touch()

    assert test_lidar._find_device(str(device)) == device


def test_rangefinder_message_becomes_csv_row(monkeypatch) -> None:
    monkeypatch.setattr(test_lidar.time, "monotonic", lambda: 12.5)
    message = SimpleNamespace(
        get_type=lambda: "RANGEFINDER",
        distance=0.75,
        voltage=4.9,
    )

    row = test_lidar._row(message, started=10.0)

    assert row["elapsed_s"] == 2.5
    assert row["message"] == "RANGEFINDER"
    assert row["distance_m"] == 0.75
    assert row["voltage_v"] == 4.9


def test_picam_is_disabled_off_pi(monkeypatch) -> None:
    monkeypatch.setattr(test_picam.Path, "read_text", lambda _path: "Desktop PC")

    assert test_picam._is_raspberry_pi() is False


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
