"""Offline review uses recording schemas and documented MAVLink field units."""

import csv
import json
import math
from pathlib import Path
from typing import Any

import pytest

from ai_drone.review.data import Series, export_recording


def message(
    name: str,
    elapsed: float,
    fields: dict[str, Any],
    *,
    source: tuple[int, int] = (1, 1),
) -> dict[str, Any]:
    return {
        "elapsed_s": elapsed,
        "timestamp_utc": "2026-09-09T10:29:06.969122+00:00",
        "message": name,
        "source_system": source[0],
        "source_component": source[1],
        "fields": fields,
    }


def recording(
    tmp_path: Path,
    telemetry: list[dict[str, Any]],
    *,
    camera: list[dict[str, Any]] | None = None,
    duration: float = 10,
) -> tuple[Path, Path]:
    capture = tmp_path / "capture"
    capture.mkdir()
    (capture / "manifest.json").write_text(
        json.dumps(
            {
                "actual_duration_s": duration,
                "started_utc": "2026-09-09T10:29:06.950700+00:00",
                "completed": True,
            }
        )
    )
    for name, rows in (("telemetry", telemetry), ("camera", camera or [])):
        (capture / f"{name}.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in rows)
        )
    return capture, tmp_path / "review"


def rows(output: Path, name: str) -> list[dict[str, str]]:
    with (output / name).open(newline="") as handle:
        return list(csv.DictReader(handle))


def series(payload: dict[str, Any], name: str) -> dict[str, Any]:
    return next(item for item in payload["series"] if item["id"] == name)


def distance(elapsed: float, orientation: int, value: int) -> dict[str, Any]:
    return message(
        "DISTANCE_SENSOR",
        elapsed,
        {
            "id": 0,
            "orientation": orientation,
            "current_distance": value,
            "min_distance": 10,
            "max_distance": 1500 if orientation == 0 else 100,
            "signal_quality": 0,
        },
    )


def test_ranges_use_orientation_even_when_ids_collide_and_isolate_fc(
    tmp_path: Path,
) -> None:
    forward = distance(1, 0, 152)
    down = distance(1, 25, 45)
    native_sensor = {**distance(1, 0, 500), "source_component": 88}
    other_vehicle = {**distance(1, 25, 99), "source_system": 2}
    invalid_source = {**distance(1, 0, 1400), "source_system": True}
    capture, output = recording(
        tmp_path, [forward, down, native_sensor, other_vehicle, invalid_source]
    )
    before = (capture / "telemetry.jsonl").read_bytes()
    payload = export_recording(capture, output)
    assert series(payload, "range_forward")["points"] == [[1, 1.52, True]]
    assert series(payload, "range_downward")["points"] == [[1, 0.45, True]]
    exported = rows(output, "ranges.csv")
    assert [(row["direction"], row["sensor_id"]) for row in exported] == [
        ("forward", "0"),
        ("downward", "0"),
    ]
    assert payload["messages"] == {"DISTANCE_SENSOR": 2}
    assert any("Excluded 3" in warning for warning in payload["warnings"])
    assert (capture / "telemetry.jsonl").read_bytes() == before


def test_invalid_range_readings_are_retained_without_affecting_valid_extrema(
    tmp_path: Path,
) -> None:
    readings = [
        distance(index, 0, value) for index, value in enumerate([152, 0, 1600, 5])
    ]
    readings += [
        message("DISTANCE_SENSOR", 4, {**readings[0]["fields"], "signal_quality": 1}),
        message("DISTANCE_SENSOR", 5, {**readings[0]["fields"], "min_distance": -1}),
        message("DISTANCE_SENSOR", 6, {**readings[0]["fields"], "signal_quality": 255}),
    ]
    capture, output = recording(tmp_path, readings)
    payload = export_recording(capture, output)
    exported = rows(output, "ranges.csv")
    assert [float(row["distance_m"]) for row in exported] == [
        1.52,
        0,
        16,
        0.05,
        1.52,
        1.52,
        1.52,
    ]
    assert [row["valid"] for row in exported] == ["True", *["False"] * 6]
    assert exported[2]["invalid_reason"] == "outside_configured_range"
    assert exported[4]["invalid_reason"] == "invalid_signal_quality"
    summary = next(
        item for item in payload["summary"] if item["label"] == "Forward MT-15"
    )
    assert summary["samples"] == 7 and summary["valid_samples"] == 1
    assert summary["min"] == summary["max"] == 1.52


def test_series_accepts_same_time_missing_and_numeric_samples() -> None:
    envelope = Series("example", "Example", "Example", "m", 1)
    for value in (None, 2, 1, None, 3):
        envelope.add(0.5, value)
    assert envelope.samples == 5 and envelope.valid_samples == 3
    assert envelope.minimum == 1 and envelope.maximum == 3
    assert [0.5, None, False] in envelope.chart()["points"]
    assert [0.5, 1, True] in envelope.chart()["points"]
    assert [0.5, 3, True] in envelope.chart()["points"]


def test_csv_keeps_all_samples_while_chart_envelope_is_bounded(tmp_path: Path) -> None:
    telemetry = [
        distance(index / 1000, 0, 100 + index % 1000) for index in range(10_001)
    ]
    telemetry[3456]["fields"]["current_distance"] = 1500
    telemetry[3457]["fields"]["current_distance"] = 9
    capture, output = recording(tmp_path, telemetry)
    payload = export_recording(capture, output)
    assert len(rows(output, "ranges.csv")) == 10_001
    points = series(payload, "range_forward")["points"]
    assert len(points) <= 4005
    assert points[0] == [0, 1, True]
    assert points[-1] == [10, 1, True]
    assert [3.456, 15, True] in points
    assert [3.457, 0.09, False] in points
    assert payload["csv_rows"]["ranges.csv"] == 10_001


def test_imu_units_keep_raw_values_separate_from_calibrated_formats(
    tmp_path: Path,
) -> None:
    fields = {
        "xacc": 1000,
        "yacc": 0,
        "zacc": -1000,
        "xgyro": 250,
        "xmag": 320,
        "temperature": 0,
    }
    telemetry = [
        message("RAW_IMU", 1, {**fields, "id": 0, "temperature": 6413}),
        message("SCALED_IMU", 1, fields),
        message("SCALED_IMU2", 2, fields),
        message("SCALED_IMU3", 3, fields),
        message(
            "HIGHRES_IMU",
            4,
            {"id": 2, "xacc": 9.8, "xgyro": -0.25, "xmag": 0.32, "temperature": 0},
        ),
    ]
    capture, output = recording(tmp_path, telemetry)
    payload = export_recording(capture, output)
    raw, first, second, third, highres = rows(output, "imu.csv")
    assert raw["xacc_raw"] == "1000.0" and raw["acceleration_x_m_s2"] == ""
    assert raw["xgyro_raw"] == "250.0" and raw["gyro_x_rad_s"] == ""
    assert raw["xmag_raw"] == "320.0" and raw["mag_x_mgauss"] == ""
    assert float(raw["temperature_c"]) == 64.13
    assert series(payload, "RAW_IMU_0_acc_x")["unit"] == "raw"
    for sensor_id, row in enumerate((first, second, third)):
        assert row["sensor_id"] == str(sensor_id)
        assert float(row["acceleration_x_m_s2"]) == pytest.approx(9.80665)
        assert float(row["gyro_x_rad_s"]) == 0.25
        assert float(row["mag_x_mgauss"]) == 320
        assert row["temperature_c"] == ""
    assert float(highres["acceleration_x_m_s2"]) == 9.8
    assert float(highres["gyro_x_rad_s"]) == -0.25
    assert float(highres["mag_x_mgauss"]) == 320
    assert float(highres["temperature_c"]) == 0


def test_flow_units_validity_and_unknown_distance_are_independent(
    tmp_path: Path,
) -> None:
    flow = {
        "sensor_id": 0,
        "quality": 58,
        "flow_x": -2,
        "flow_y": 4,
        "flow_comp_m_x": -2.83,
        "flow_comp_m_y": 4.45,
        "flow_rate_x": -2.84,
        "flow_rate_y": 4.46,
        "ground_distance": -1,
    }
    integrated = {
        "sensor_id": 0,
        "quality": 255,
        "integrated_x": 0.03,
        "integrated_y": -0.01,
        "integration_time_us": 20_000,
        "distance": 0,
    }
    capture, output = recording(
        tmp_path,
        [
            message("OPTICAL_FLOW", 1, flow),
            message("OPTICAL_FLOW_RAD", 1, integrated),
            message("OPTICAL_FLOW", 2, {**flow, "quality": 0}),
            message("OPTICAL_FLOW_RAD", 2, {**integrated, "integration_time_us": 0}),
            message("OPTICAL_FLOW", 3, {**flow, "quality": 256}),
        ],
    )
    payload = export_recording(capture, output)
    first, second, zero_quality, zero_time, invalid_quality = rows(
        output, "optical_flow.csv"
    )
    assert first["flow_x_deprecated_rad_s"] == "-2.0"
    assert first["rate_x_rad_s"] == "-2.84"
    assert first["velocity_x_m_s"] == "-2.83"
    assert first["valid"] == "True" and first["distance_valid"] == "False"
    assert first["distance_m"] == "-1.0"
    assert second["distance_valid"] == "True"
    assert second["integrated_x_rad"] == "0.03" and second["rate_x_rad_s"] == ""
    assert (
        zero_quality["valid"]
        == invalid_quality["valid"]
        == zero_time["valid"]
        == "False"
    )
    assert zero_time["invalid_reason"] == "missing_or_nonpositive_integration_time"
    assert series(payload, "flow_integrated_x")["points"] == [
        [1, 0.03, True],
        [2, 0.03, False],
    ]
    assert any(
        "Both optical-flow formats" in warning for warning in payload["warnings"]
    )


def test_battery_arrays_sentinels_and_energy_units_do_not_duplicate_sys_charts(
    tmp_path: Path,
) -> None:
    battery = {
        "id": 0,
        "voltages": [14208, *[65535] * 9],
        "voltages_ext": [0] * 4,
        "current_battery": 128,
        "battery_remaining": 97,
        "temperature": 32767,
        "current_consumed": 67,
        "energy_consumed": 34,
        "time_remaining": 0,
        "charge_state": 1,
        "fault_bitmask": 0,
    }
    unknown = {
        **battery,
        "voltages": [65535] * 10,
        "current_battery": -1,
        "battery_remaining": -1,
        "current_consumed": -1,
        "energy_consumed": -1,
    }
    cells = {
        **battery,
        "id": 1,
        "voltages": [4100] * 10,
        "voltages_ext": [4200, 0, 0, 0],
        "temperature": -1234,
        "time_remaining": 100,
        "current_battery": -200,
    }
    capture, output = recording(
        tmp_path,
        [
            message("BATTERY_STATUS", 1, battery),
            message("BATTERY_STATUS", 2, unknown),
            message("BATTERY_STATUS", 3, cells),
            message(
                "SYS_STATUS",
                1,
                {
                    "voltage_battery": 14209,
                    "current_battery": 122,
                    "battery_remaining": 97,
                },
            ),
            message(
                "SYS_STATUS",
                2,
                {
                    "voltage_battery": 65535,
                    "current_battery": -1,
                    "battery_remaining": -1,
                },
            ),
        ],
    )
    payload = export_recording(capture, output)
    first, missing, multicell = rows(output, "battery.csv")
    assert float(first["voltage_v"]) == 14.208
    assert json.loads(first["voltages_mv"]) == battery["voltages"]
    assert float(first["current_a"]) == 1.28
    assert first["temperature_c"] == first["time_remaining_s"] == ""
    assert first["consumed_mah"] == "67.0" and first["consumed_j"] == "3400.0"
    for field in (
        "voltage_v",
        "current_a",
        "remaining_percent",
        "temperature_c",
        "consumed_mah",
        "consumed_j",
    ):
        assert missing[field] == ""
    assert float(multicell["voltage_v"]) == 45.2
    assert float(multicell["temperature_c"]) == -12.34
    assert float(multicell["current_a"]) == -2
    assert multicell["battery_id"] == "1"
    assert series(payload, "battery_voltage")["points"] == [
        [1, 14.209, True],
        [2, None, False],
    ]
    assert payload["csv_rows"]["battery.csv"] == 3


def test_camera_schema_and_attitude_pressure_units_match_recording(
    tmp_path: Path,
) -> None:
    camera_row = {
        "elapsed_s": 0.0668,
        "frame": 0,
        "timestamp_utc": "2026-09-09T10:29:07.057041+00:00",
        "metadata": {
            "SensorTimestamp": 1613450240000,
            "ExposureTime": 33044,
            "FrameDuration": 33322,
            "AnalogueGain": 4.37,
            "Lux": 295.6,
            "FocusFoM": 829,
        },
        "tags": [{"id": 7, "center": [320, 240]}],
    }
    capture, output = recording(
        tmp_path,
        [
            message("ATTITUDE", 1, {"roll": math.pi / 2, "pitch": -math.pi, "yaw": 0}),
            message("SCALED_PRESSURE", 1, {"press_abs": 998.24, "temperature": 6230}),
            message(
                "LOCAL_POSITION_NED",
                1,
                {"x": 1, "y": 2, "z": -3, "vx": 0.1, "vy": -0.2, "vz": 0.3},
            ),
        ],
        camera=[camera_row],
    )
    payload = export_recording(capture, output)
    camera = rows(output, "camera.csv")[0]
    assert camera["timestamp_utc"] == camera_row["timestamp_utc"]
    assert float(camera["elapsed_s"]) == 0.0668 and camera["frame"] == "0"
    assert camera["sensor_timestamp_ns"] == "1613450240000"
    assert camera["exposure_us"] == "33044" and camera["frame_duration_us"] == "33322"
    assert json.loads(camera["tag_ids"]) == [7]
    assert payload["started_utc"] == "2026-09-09T10:29:06.950700+00:00"
    attitude, position = rows(output, "motion.csv")
    assert float(attitude["roll_deg"]) == 90 and float(attitude["pitch_deg"]) == -180
    assert (
        float(position["down_m"]) == -3 and float(position["velocity_east_m_s"]) == -0.2
    )
    environment = rows(output, "environment.csv")[0]
    assert float(environment["pressure_hpa"]) == 998.24
    assert float(environment["temperature_c"]) == pytest.approx(62.3)


def test_partial_malformed_capture_keeps_later_intact_records(tmp_path: Path) -> None:
    capture, output = recording(tmp_path, [])
    (capture / "manifest.json").write_text('{"completed":')
    malformed_rows = [
        [],
        {"elapsed_s": -1},
        {"elapsed_s": float("nan")},
        {"elapsed_s": 10**400},
        message(
            "DISTANCE_SENSOR",
            1,
            {
                "orientation": {},
                "current_distance": 10,
                "min_distance": 0,
                "max_distance": 100,
            },
        ),
        message("HEARTBEAT", 1, {"base_mode": 10**400}),
        message("STATUSTEXT", 2, {"text": "=1+1", "severity": 2}),
        {**message("ATTITUDE", 3, {}), "fields": None},
        distance(4, 0, 123),
    ]
    content = (
        b"\xff\xfe\n{broken\n"
        + "".join(json.dumps(row) + "\n" for row in malformed_rows).encode()
    )
    (capture / "telemetry.jsonl").write_bytes(content + b'{"elapsed_s":')
    (capture / "camera.jsonl").write_text(
        json.dumps({"elapsed_s": 2, "frame": 1, "metadata": None, "tags": None}) + "\n"
    )
    payload = export_recording(capture, output)
    assert not payload["completed"] and payload["duration_s"] == 4
    assert series(payload, "range_forward")["points"] == [[4, 1.23, True]]
    assert rows(output, "ranges.csv")[0]["invalid_reason"] == "invalid_orientation"
    assert rows(output, "events.csv")[1]["text"] == "'=1+1"
    assert any("Unreadable rows" in warning for warning in payload["warnings"])
    assert any("valid elapsed time" in warning for warning in payload["warnings"])
    assert any(
        "No readable completion manifest" in warning for warning in payload["warnings"]
    )
    assert (capture / "telemetry.jsonl").read_bytes() == content + b'{"elapsed_s":'
    json.dumps(payload, allow_nan=False)


def test_missing_parts_and_capture_interval_are_explicit(tmp_path: Path) -> None:
    capture, output = recording(tmp_path, [distance(1, 0, 123), distance(11, 0, 456)])
    (capture / "camera.jsonl").unlink()
    payload = export_recording(capture, output)
    assert payload["csv_rows"]["ranges.csv"] == 1
    assert any(
        "Missing or unreadable camera.jsonl" in warning
        for warning in payload["warnings"]
    )
    assert any(
        "outside the recorded capture interval" in warning
        for warning in payload["warnings"]
    )
