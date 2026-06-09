"""Unit tests for the hardware-test entry points."""

from __future__ import annotations

from types import SimpleNamespace

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
