from __future__ import annotations

from types import SimpleNamespace

from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.cli.record import _parser, _resolution, _validate_args
from ai_drone.recording import (
    TELEMETRY_RATES_HZ,
    create_recording_paths,
    is_armed_vehicle_heartbeat,
    json_safe,
    request_telemetry_messages,
    video_timestamp_summary,
)


def test_create_recording_paths_creates_expected_dataset(tmp_path) -> None:
    paths = create_recording_paths(tmp_path / "run")

    assert paths.root.is_dir()
    assert paths.video == paths.root / "camera.h264"
    assert paths.telemetry_tlog == paths.root / "telemetry.tlog"
    assert paths.manifest == paths.root / "manifest.json"


def test_json_safe_converts_nested_binary_values() -> None:
    assert json_safe({"data": b"\x01\xff", "nested": (1, bytearray(b"a"))}) == {
        "data": "01ff",
        "nested": [1, "61"],
    }


def test_request_telemetry_never_sends_an_arm_command() -> None:
    sent: list[tuple] = []
    connection = SimpleNamespace(
        target_system=1,
        target_component=0,
        mav=SimpleNamespace(command_long_send=lambda *values: sent.append(values)),
    )

    requested = request_telemetry_messages(connection)

    assert set(requested).issubset(TELEMETRY_RATES_HZ)
    assert sent
    assert all(values[2] == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL for values in sent)
    assert all(values[2] != mavlink.MAV_CMD_COMPONENT_ARM_DISARM for values in sent)


class _Heartbeat:
    def __init__(self, *, system: int, armed: bool) -> None:
        self.base_mode = mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        self._system = system

    def get_type(self) -> str:
        return "HEARTBEAT"

    def get_srcSystem(self) -> int:  # noqa: N802 - pymavlink API spelling
        return self._system


def test_armed_check_uses_only_the_vehicle_heartbeat() -> None:
    assert is_armed_vehicle_heartbeat(
        _Heartbeat(system=1, armed=True), vehicle_system=1
    )
    assert not is_armed_vehicle_heartbeat(
        _Heartbeat(system=200, armed=True), vehicle_system=1
    )
    assert not is_armed_vehicle_heartbeat(
        _Heartbeat(system=1, armed=False), vehicle_system=1
    )


def test_record_parser_accepts_configurable_seconds() -> None:
    parser = _parser()
    args = parser.parse_args(["--duration", "12.5", "--resolution", "1280x960"])
    _validate_args(parser, args)

    assert args.duration == 12.5
    assert args.resolution == (1280, 960)
    assert _resolution("640x480") == (640, 480)


def test_video_timestamp_summary_reports_frame_count_and_span(tmp_path) -> None:
    path = tmp_path / "camera.pts"
    path.write_text("0.000\n33.333\n66.667\n")

    assert video_timestamp_summary(path) == {
        "encoded_frames": 3,
        "encoded_span_s": 0.066667,
        "encoded_duration_s": 0.100001,
    }
