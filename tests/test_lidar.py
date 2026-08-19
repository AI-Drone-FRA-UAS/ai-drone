from __future__ import annotations

from types import SimpleNamespace

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.cli import lidar


def test_sensor_stream_request_uses_only_message_interval_commands() -> None:
    sent: list[tuple[object, ...]] = []
    connection = SimpleNamespace(
        target_system=1,
        target_component=1,
        mav=SimpleNamespace(command_long_send=lambda *args: sent.append(args)),
    )

    lidar._request_sensor_messages(connection)

    assert len(sent) == len(lidar.SENSOR_RATES_HZ)
    assert all(values[2] == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL for values in sent)
    assert all(values[2] != mavlink.MAV_CMD_COMPONENT_ARM_DISARM for values in sent)


def _distance_sensor(**overrides):
    values = {
        "get_type": lambda: "DISTANCE_SENSOR",
        "current_distance": 75,
        "min_distance": 20,
        "max_distance": 300,
        "type": 0,
        "id": 2,
        "orientation": 25,
        "covariance": 4,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_distance_sensor_row_preserves_identity_orientation_and_bounds(
    monkeypatch,
) -> None:
    monkeypatch.setattr(lidar.time, "monotonic", lambda: 12.5)

    row = lidar._row(_distance_sensor(), started=10.0)

    assert row["sensor_id"] == 2
    assert row["sensor_type_name"] == "MAV_DISTANCE_SENSOR_LASER"
    assert row["orientation"] == 25
    assert row["orientation_name"] == "MAV_SENSOR_ROTATION_PITCH_270"
    assert row["distance_m"] == 0.75
    assert row["distance_valid"] == 1
    assert row["min_distance_m"] == 0.2
    assert row["max_distance_m"] == 3.0
    assert row["covariance_cm2"] == 4


def test_out_of_range_distance_is_retained_but_not_counted_as_valid(
    monkeypatch,
) -> None:
    monkeypatch.setattr(lidar.time, "monotonic", lambda: 12.5)
    row = lidar._row(_distance_sensor(current_distance=400), started=10.0)

    groups, message_count = lidar._distance_groups([row])

    assert row["distance_m"] == 4.0
    assert row["distance_valid"] == 0
    assert groups == {}
    assert message_count == 1


def test_distance_summary_keeps_sensor_ids_and_orientations_separate(
    monkeypatch,
) -> None:
    monkeypatch.setattr(lidar.time, "monotonic", lambda: 12.5)
    downward = lidar._row(_distance_sensor(id=0, orientation=25), started=10.0)
    forward = lidar._row(_distance_sensor(id=1, orientation=0), started=10.0)

    groups, message_count = lidar._distance_groups([downward, forward])

    assert message_count == 2
    assert len(groups) == 2
    assert any("id=0" in label and "PITCH_270" in label for label in groups)
    assert any("id=1" in label and "ROTATION_NONE" in label for label in groups)


def test_non_finite_legacy_rangefinder_values_are_blank(monkeypatch) -> None:
    monkeypatch.setattr(lidar.time, "monotonic", lambda: 12.5)
    message = SimpleNamespace(
        get_type=lambda: "RANGEFINDER",
        distance=float("nan"),
        voltage=float("inf"),
    )

    row = lidar._row(message, started=10.0)

    assert row["distance_m"] == ""
    assert row["voltage_v"] == ""
    assert row["distance_valid"] == 0


def test_csv_write_is_atomic_and_removes_temporary_file_on_error(
    tmp_path, monkeypatch
) -> None:
    output = tmp_path / "sensor.csv"
    output.write_text("previous complete capture\n")

    class FailingWriter:
        def writeheader(self) -> None:
            pass

        def writerows(self, _rows) -> None:
            raise OSError("synthetic write failure")

    monkeypatch.setattr(
        lidar.csv, "DictWriter", lambda *_args, **_kwargs: FailingWriter()
    )

    with pytest.raises(OSError, match="synthetic write failure"):
        lidar._write_csv(output, [])

    assert output.read_text() == "previous complete capture\n"
    assert list(tmp_path.glob(".sensor.csv.*.tmp")) == []


def test_lidar_rejects_non_finite_duration_before_device_access(monkeypatch) -> None:
    monkeypatch.setattr(
        lidar,
        "_find_device",
        lambda _requested: pytest.fail("device access must follow argument validation"),
    )

    with pytest.raises(SystemExit) as error:
        lidar.main(["--duration", "nan"])

    assert error.value.code == 2


def test_lidar_closes_connection_when_initial_heartbeat_times_out(
    tmp_path, monkeypatch
) -> None:
    device = tmp_path / "serial"
    device.touch()
    connection = SimpleNamespace(
        wait_heartbeat=lambda *, timeout: None,
        close=lambda: setattr(connection, "closed", True),
        closed=False,
    )
    monkeypatch.setattr(
        lidar.mavutil, "mavlink_connection", lambda *_args, **_kwargs: connection
    )

    with pytest.raises(SystemExit, match="No ArduPilot heartbeat"):
        lidar.main(["--device", str(device), "--duration", "0.01"])

    assert connection.closed
