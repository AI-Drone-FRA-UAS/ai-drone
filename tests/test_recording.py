from __future__ import annotations

import io
import json
import queue
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

import ai_drone.capture.reporting as capture_reporting
import ai_drone.capture.workers as capture_workers
import ai_drone.cli.record as inspect_cli
from ai_drone.capture.reporting import _component_report, _observe_sensor_message
from ai_drone.capture.state import (
    AnalysisFrame,
    CaptureState,
    CaptureWindow,
)
from ai_drone.capture.workers import DetectionWorker, TelemetryWorker
from ai_drone.cli.record import (
    _cleanup_capture,
    _parser,
    _safe_video_timestamp_summary,
    _start_capture_epoch,
    _stop_detection_worker,
    _validate_args,
    _wait_for_fresh_heartbeat,
    _write_preview_frame,
    run,
)
from ai_drone.cli_parsing import parse_even_resolution
from ai_drone.durability import IntervalSync
from ai_drone.mavlink.safety import is_armed_vehicle_heartbeat
from ai_drone.recording import (
    TELEMETRY_RATES_HZ,
    create_recording_paths,
    json_safe,
    request_message_intervals,
    request_telemetry_messages,
    telemetry_record,
    video_timestamp_summary,
    write_json_line,
)
from ai_drone.vision.apriltags import (
    CameraCalibration,
    PoseEstimationError,
    TagDetection,
    TagPose,
)


def test_create_recording_paths_creates_expected_dataset(tmp_path) -> None:
    paths = create_recording_paths(tmp_path / "run")

    assert paths.root.is_dir()
    assert paths.video == paths.root / "camera.h264"
    assert paths.telemetry_tlog == paths.root / "telemetry.tlog"
    assert paths.manifest == paths.root / "manifest.json"


def test_create_recording_paths_uses_utc_when_output_omitted(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.chdir(tmp_path)
    now_utc = datetime(2026, 9, 21, 14, 30, 0, tzinfo=UTC)
    paths = create_recording_paths(now=now_utc)
    assert paths.root.name == "20260921-143000"


def test_create_recording_paths_reserves_unique_directories_concurrently(
    tmp_path,
) -> None:
    candidate = tmp_path / "run"

    with ThreadPoolExecutor(max_workers=8) as executor:
        paths = list(
            executor.map(lambda _: create_recording_paths(candidate), range(8))
        )

    roots = {path.root for path in paths}
    assert len(roots) == 8
    assert all(root.is_dir() for root in roots)


def test_json_safe_converts_nested_binary_values() -> None:
    assert json_safe({"data": b"\x01\xff", "nested": (1, bytearray(b"a"))}) == {
        "data": "01ff",
        "nested": [1, "61"],
    }


def test_json_safe_normalizes_non_finite_scalars_and_arrays() -> None:
    converted = json_safe(
        {
            "nan": float("nan"),
            "positive_infinity": float("inf"),
            "array": np.array([1.0, float("-inf")]),
        }
    )

    assert converted == {
        "nan": None,
        "positive_infinity": None,
        "array": [1.0, None],
    }


def test_write_json_line_emits_strict_json_for_non_finite_values() -> None:
    output = io.StringIO()

    write_json_line(output, {"measurement": float("nan")})

    assert json.loads(output.getvalue()) == {"measurement": None}
    assert "NaN" not in output.getvalue()


def test_request_telemetry_never_sends_an_arm_command() -> None:
    sent: list[tuple] = []
    connection = SimpleNamespace(
        target_system=1,
        target_component=0,
        mav=SimpleNamespace(command_long_send=lambda *values: sent.append(values)),
    )

    requested = request_telemetry_messages(connection)

    assert set(requested).issubset(TELEMETRY_RATES_HZ)
    assert len(sent) == len(requested) == 25
    assert {"RPM", "ESC_TELEMETRY_1_TO_4"}.issubset(requested)
    assert all(values[2] == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL for values in sent)
    assert all(values[2] != mavlink.MAV_CMD_COMPONENT_ARM_DISARM for values in sent)
    assert all(values[3] == 0 for values in sent)
    assert all(values[6:] == (0, 0, 0, 0, 0) for values in sent)


def test_reported_rpm_and_esc_values_are_recorded_without_throttle_inference():
    from pymavlink.dialects.v20 import ardupilotmega as mavlink2

    rpm = mavlink2.MAVLink_rpm_message(1234.5, 0)
    esc = mavlink2.MAVLink_esc_telemetry_1_to_4_message(
        [30] * 4, [1600] * 4, [100] * 4, [20] * 4, [1200, 1300, 0, 1500], [1] * 4
    )
    assert telemetry_record(rpm, elapsed_s=1)["fields"]["rpm1"] == 1234.5
    assert telemetry_record(esc, elapsed_s=1)["fields"]["rpm"] == [1200, 1300, 0, 1500]


@pytest.mark.parametrize("failure", ["full", "unreadable"])
def test_storage_preflight_stops_before_opening_hardware(
    tmp_path, monkeypatch, failure
):
    import ai_drone.storage as storage

    def available(_path):
        if failure == "unreadable":
            raise OSError("cannot inspect filesystem")
        return SimpleNamespace(free=8 * storage.MIB)

    monkeypatch.setattr(storage.shutil, "disk_usage", available)
    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        inspect_cli,
        "resolve_mavlink_endpoint",
        lambda *_a, **_kw: pytest.fail("hardware opened"),
    )
    output = tmp_path / "preflight"
    assert run(["--duration", "0.01", "--output-dir", str(output)]) == 1
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["completed"] is False
    assert manifest["stop_reason"] == (
        "storage_full" if failure == "full" else "storage_error"
    )
    assert manifest["error"]


@pytest.mark.parametrize(
    "arguments",
    [
        ["--storage-stop-mib", "0"],
        ["--storage-stop-mib", "256"],
        ["--storage-reserve-mib", "2048"],
        ["--storage-check-interval", "nan"],
    ],
)
def test_recording_rejects_invalid_storage_thresholds(arguments):
    parser = _parser()
    args = parser.parse_args(arguments)
    with pytest.raises(SystemExit):
        _validate_args(parser, args)


def test_recording_storage_settings_allow_explicit_overrides(tmp_path, monkeypatch):
    config = tmp_path / "drone.toml"
    config.write_text(
        "[recording]\nstorage_reserve_mib = 128\nstorage_warning_mib = 512\nstorage_stop_mib = 8\nstorage_check_interval = 1\n"
    )
    monkeypatch.setenv("AI_DRONE_CONFIG", str(config))
    args = _parser().parse_args(["--storage-reserve-mib", "200"])
    assert args.storage_reserve_mib == 200
    assert args.storage_warning_mib == 512
    assert args.storage_stop_mib == 8
    assert args.storage_check_interval == 1


@pytest.mark.parametrize("rate", [0.0, -1.0, float("nan"), float("inf")])
def test_message_interval_helper_rejects_invalid_rates(rate: float) -> None:
    connection = SimpleNamespace(
        target_system=1,
        target_component=0,
        mav=SimpleNamespace(
            command_long_send=lambda *_values: pytest.fail(
                "invalid rates must be rejected before sending"
            )
        ),
    )

    with pytest.raises(ValueError, match="finite and positive"):
        request_message_intervals(connection, {42: rate})


class _Heartbeat:
    def __init__(self, *, system: int, armed: bool) -> None:
        self.base_mode = mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        self._system = system

    def get_type(self) -> str:
        return "HEARTBEAT"

    def get_srcSystem(self) -> int:
        return self._system

    def get_srcComponent(self) -> int:
        return 1


def test_armed_check_uses_only_the_vehicle_heartbeat() -> None:
    assert is_armed_vehicle_heartbeat(_Heartbeat(system=1, armed=True), system_id=1)
    assert not is_armed_vehicle_heartbeat(
        _Heartbeat(system=200, armed=True), system_id=1
    )
    assert not is_armed_vehicle_heartbeat(
        _Heartbeat(system=1, armed=False), system_id=1
    )


def test_record_parser_accepts_configurable_seconds() -> None:
    parser = _parser()
    args = parser.parse_args(["--duration", "12.5", "--resolution", "1280x960"])
    _validate_args(parser, args)

    assert args.duration == 12.5
    assert args.resolution == (1280, 960)
    assert parse_even_resolution("640x480") == (640, 480)


def test_passive_flight_requires_explicit_allowance() -> None:
    parser = _parser()
    accepted = parser.parse_args(["--allow-flight"])
    _validate_args(parser, accepted)
    assert accepted.allow_flight
    assert not parser.parse_args([]).allow_flight


@pytest.mark.parametrize(
    "arguments",
    [
        ["--duration", "nan"],
        ["--timeout", "inf"],
        ["--frame-timeout", "nan"],
        ["--frame-timeout", "0"],
        ["--warmup", "nan"],
        ["--decimate", "inf"],
    ],
)
def test_record_parser_rejects_non_finite_numbers(arguments) -> None:
    parser = _parser()
    args = parser.parse_args(arguments)

    with pytest.raises(SystemExit):
        _validate_args(parser, args)


@pytest.mark.parametrize("target_id", [-1, 587])
def test_record_parser_rejects_tag_id_outside_tag36h11(target_id: int) -> None:
    parser = _parser()
    args = parser.parse_args(["--target-id", str(target_id)])

    with pytest.raises(SystemExit):
        _validate_args(parser, args)


def test_video_timestamp_summary_reports_frame_count_and_span(tmp_path) -> None:
    path = tmp_path / "camera.pts"
    path.write_text("0.000\n33.333\n66.667\n")

    assert video_timestamp_summary(path) == {
        "encoded_frames": 3,
        "encoded_span_s": 0.066667,
        "encoded_duration_s": 0.100001,
    }


class _FailingDetector:
    backend_name = "failing-test-detector"

    def detect(self, grayscale: np.ndarray) -> list[TagDetection]:
        del grayscale
        raise RuntimeError("synthetic detector failure")


class _BlockingDetector:
    backend_name = "blocking-test-detector"

    def __init__(self, entered: threading.Event, release: threading.Event) -> None:
        self.entered = entered
        self.release = release

    def detect(self, grayscale: np.ndarray) -> list[TagDetection]:
        del grayscale
        self.entered.set()
        self.release.wait(timeout=1.0)
        return []


def _analysis_frame(index: int = 0) -> AnalysisFrame:
    return AnalysisFrame(
        frame_index=index,
        elapsed_s=0.1 * index,
        grayscale=np.zeros((4, 4), dtype=np.uint8),
        metadata={},
    )


def test_detection_worker_failure_stops_capture(tmp_path) -> None:
    frames: queue.Queue[AnalysisFrame | None] = queue.Queue()
    frames.put(_analysis_frame())
    stop = threading.Event()
    state = CaptureState()
    worker = DetectionWorker(
        frames=frames,
        output=tmp_path / "camera.jsonl",
        detector=_FailingDetector(),
        calibration=None,
        tag_size=0.16,
        resolution=(4, 4),
        max_reprojection_error=2.0,
        target_id=None,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
    )

    worker.start()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert stop.is_set()
    assert state.worker_error == "AprilTag worker: synthetic detector failure"


def _recording_calibration() -> CameraCalibration:
    return CameraCalibration(
        image_width=4,
        image_height=4,
        camera_matrix=((2.0, 0.0, 2.0), (0.0, 2.0, 2.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0),
    )


def _tag_detection(tag_id: int) -> TagDetection:
    return TagDetection(
        tag_id=tag_id,
        corners=np.array([[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]]),
        center=(2.0, 2.0),
        hamming=0,
        decision_margin=80.0,
    )


def test_detection_worker_records_pose_rejection_then_recovers(
    tmp_path, monkeypatch
) -> None:
    rejected = _tag_detection(7)
    other = _tag_detection(8)
    detected_frames = iter([[rejected, other], [rejected]])
    observed = []
    calls = 0

    def pose(detection, *_args, **_kwargs):
        nonlocal calls
        calls += 1
        if calls == 1:
            raise PoseEstimationError("no positive-depth pose for tag 7")
        return TagPose(
            tag_id=detection.tag_id,
            rotation_vector=(0.0, 0.0, 0.0),
            translation_m=(0.0, 0.0, 1.0),
            distance_m=1.0,
            reprojection_error_px=0.1,
        )

    class Detector:
        backend_name = "pose-test-detector"

        def detect(self, grayscale):
            del grayscale
            return next(detected_frames)

    class Observer:
        def observe(self, frame, detections, tag_records):
            observed.append((frame.frame_index, [item.tag_id for item in detections]))
            assert len(tag_records) == (2 if frame.frame_index == 0 else 1)

    monkeypatch.setattr(capture_workers, "estimate_pose", pose)
    frames: queue.Queue[AnalysisFrame | None] = queue.Queue()
    for item in (_analysis_frame(0), _analysis_frame(1), None):
        frames.put(item)
    stop = threading.Event()
    state = CaptureState()
    output = tmp_path / "camera.jsonl"
    worker = DetectionWorker(
        frames=frames,
        output=output,
        detector=Detector(),
        calibration=_recording_calibration(),
        tag_size=0.16,
        resolution=(4, 4),
        max_reprojection_error=2.0,
        target_id=None,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
        observer=Observer(),
    )

    worker.run()

    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert not stop.is_set()
    assert state.worker_error is None
    assert state.processed_frames == 2
    assert state.tag_detections == 3
    assert state.tag_ids == {7: 2, 8: 1}
    assert frames.unfinished_tasks == 0
    assert observed == [(0, [8]), (1, [7])]
    failed = records[0]["tags"][0]
    assert failed["id"] == 7
    assert failed["pose_valid"] is False
    assert failed["pose_error"] == "no positive-depth pose for tag 7"
    assert "camera_xyz_m" not in failed
    assert records[0]["tags"][1]["pose_valid"] is True
    assert records[1]["tags"][0]["pose_valid"] is True
    assert "pose_error" not in records[1]["tags"][0]


@pytest.mark.parametrize(
    ("failure", "expected_error"),
    [
        ("invalid_calibration", "camera focal lengths"),
        ("aspect_ratio", "different aspect ratios"),
        ("missing_opencv", "OpenCV is required"),
        ("backend_failure", "synthetic OpenCV failure"),
    ],
)
def test_detection_worker_keeps_pose_setup_and_backend_failures_fatal(
    tmp_path, monkeypatch, failure, expected_error
) -> None:
    calibration = _recording_calibration()
    resolution = (4, 4)
    if failure == "invalid_calibration":
        with pytest.raises(ValueError, match=expected_error):
            replace(
                calibration,
                camera_matrix=((-2.0, 0.0, 2.0), (0.0, 2.0, 2.0), (0.0, 0.0, 1.0)),
            )
        return
    elif failure == "aspect_ratio":
        resolution = (4, 2)

    def backend_failure(*_args, **_kwargs):
        raise RuntimeError("synthetic OpenCV failure")

    monkeypatch.setitem(
        sys.modules,
        "cv2",
        SimpleNamespace(solvePnPGeneric=backend_failure, SOLVEPNP_IPPE_SQUARE=7)
        if failure == "backend_failure"
        else None,
    )
    frames: queue.Queue[AnalysisFrame | None] = queue.Queue()
    frames.put(_analysis_frame())
    frames.put(None)
    stop = threading.Event()
    state = CaptureState()
    observed = []

    class Detector:
        backend_name = "pose-test-detector"

        def detect(self, grayscale):
            del grayscale
            return [_tag_detection(7)]

    class Observer:
        def observe(self, frame, detections, tag_records):
            del frame, detections, tag_records
            observed.append(True)

    output = tmp_path / "camera.jsonl"
    worker = DetectionWorker(
        frames=frames,
        output=output,
        detector=Detector(),
        calibration=calibration,
        tag_size=0.16,
        resolution=resolution,
        max_reprojection_error=2.0,
        target_id=None,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
        observer=Observer(),
    )

    worker.run()

    assert stop.is_set()
    assert state.worker_error is not None
    assert expected_error in state.worker_error
    assert state.processed_frames == 0
    assert observed == []
    assert output.read_text() == ""


def test_detection_worker_shutdown_is_bounded_when_queue_is_full(tmp_path) -> None:
    entered = threading.Event()
    release = threading.Event()
    frames: queue.Queue[AnalysisFrame | None] = queue.Queue(maxsize=1)
    frames.put(_analysis_frame())
    state = CaptureState()
    worker = DetectionWorker(
        frames=frames,
        output=tmp_path / "camera.jsonl",
        detector=_BlockingDetector(entered, release),
        calibration=None,
        tag_size=0.16,
        resolution=(4, 4),
        max_reprojection_error=2.0,
        target_id=None,
        stop=threading.Event(),
        state=state,
        sync=IntervalSync(0.0),
    )
    worker.start()
    assert entered.wait(timeout=1.0)
    frames.put_nowait(_analysis_frame(index=1))

    started = time.monotonic()
    try:
        _stop_detection_worker(worker, frames, state, timeout=0.02)
        elapsed = time.monotonic() - started

        assert elapsed < 0.5
        assert worker.is_alive()
        assert state.dropped_analysis_frames == 1
        assert state.worker_error == "AprilTag worker did not stop within 0.02 seconds"
    finally:
        release.set()
        worker.join(timeout=1.0)

    assert not worker.is_alive()


class _TelemetryMessage:
    def __init__(
        self,
        message_type: str,
        *,
        armed: bool = False,
        observed: threading.Event | None = None,
        system: int = 1,
        component: int = 1,
    ) -> None:
        self.message_type = message_type
        self.base_mode = mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        self.observed = observed
        self.system = system
        self.component = component

    def get_type(self) -> str:
        if self.observed is not None:
            self.observed.set()
        return self.message_type

    def get_srcSystem(self) -> int:
        return self.system

    def get_srcComponent(self) -> int:
        return self.component

    def to_dict(self) -> dict[str, object]:
        return {"mavpackettype": self.message_type}


class _QueuedConnection:
    def __init__(self) -> None:
        self.messages: queue.Queue[_TelemetryMessage] = queue.Queue()

    def recv_match(self, *, blocking: bool, timeout: float):
        assert blocking
        try:
            return self.messages.get(timeout=timeout)
        except queue.Empty:
            return None


@pytest.mark.parametrize("foreign_source", [(2, 1), (1, 191)])
@pytest.mark.parametrize("include_selected", [False, True])
@pytest.mark.parametrize("orientation", [0, 25])
def test_telemetry_health_uses_selected_source_and_keeps_all_raw_messages(
    tmp_path, foreign_source, include_selected, orientation
) -> None:
    def sensor_messages(system, component, *, distance_cm, quality):
        messages = []
        for message_type, fields in (
            (
                "DISTANCE_SENSOR",
                {
                    "orientation": orientation,
                    "current_distance": distance_cm,
                    "min_distance": 1,
                    "max_distance": 1000,
                },
            ),
            ("RANGEFINDER", {"distance": distance_cm / 100.0}),
            ("OPTICAL_FLOW_RAD", {"quality": quality}),
            ("SCALED_IMU", {}),
        ):
            message = _TelemetryMessage(
                message_type, system=system, component=component
            )
            vars(message).update(fields)
            messages.append(message)
        return messages

    foreign = sensor_messages(*foreign_source, distance_cm=900, quality=255)
    selected = (
        sensor_messages(1, 1, distance_cm=35, quality=40) if include_selected else []
    )
    messages = iter([*foreign, *selected, *foreign])
    stop = threading.Event()

    def receive(**_kwargs):
        message = next(messages, None)
        if message is None:
            stop.set()
        return message

    state = CaptureState()
    window = CaptureWindow(duration=None)
    window.begin()
    output = tmp_path / "telemetry.jsonl"
    worker = TelemetryWorker(
        connection=SimpleNamespace(recv_match=receive),
        output=output,
        vehicle_system=1,
        vehicle_component=1,
        window=window,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
    )

    worker.run()

    assert state.worker_error is None
    types = ("DISTANCE_SENSOR", "RANGEFINDER", "OPTICAL_FLOW_RAD", "SCALED_IMU")
    assert state.telemetry_counts == {
        name: 3 if include_selected else 2 for name in types
    }
    assert state.vehicle_telemetry_counts == (
        dict.fromkeys(types, 1) if include_selected else {}
    )
    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert [
        (record["source_system"], record["source_component"]) for record in records
    ] == (
        [foreign_source] * 4
        + ([(1, 1)] * 4 if include_selected else [])
        + [foreign_source] * 4
    )
    report = _component_report(
        state,
        on_pi=True,
        flight_controller="ok",
        camera="unavailable",
        detector="unavailable",
        details={},
        duration=1.0,
    )
    assert report["flight_controller"]["messages"] == (4 if include_selected else 0)
    assert report["downward_rangefinder"]["status"] == (
        "ok" if include_selected else "no_data"
    )
    assert report["downward_rangefinder"]["samples"] == (1 if include_selected else 0)
    assert report["downward_rangefinder"]["latest_m"] == (
        0.35 if include_selected else None
    )
    forward_selected = include_selected and orientation == 0
    assert report["forward_rangefinder"]["status"] == (
        "ok" if forward_selected else "no_data"
    )
    assert report["forward_rangefinder"]["samples"] == (1 if forward_selected else 0)
    assert report["forward_rangefinder"]["latest_m"] == (
        0.35 if forward_selected else None
    )
    assert state.legacy_range_samples == (1 if include_selected else 0)
    assert state.latest_legacy_range_m == (0.35 if include_selected else None)
    assert (state.flow_observed_monotonic is not None) is include_selected
    assert report["optical_flow"]["status"] == ("ok" if include_selected else "no_data")
    assert report["optical_flow"]["samples"] == (1 if include_selected else 0)
    assert report["optical_flow"]["quality"] == (40 if include_selected else None)


def test_telemetry_worker_detects_arming_before_capture_epoch(tmp_path) -> None:
    connection = _QueuedConnection()
    stop = threading.Event()
    state = CaptureState()
    window = CaptureWindow(duration=1.0)
    worker = TelemetryWorker(
        connection=connection,
        output=tmp_path / "telemetry.jsonl",
        vehicle_system=1,
        vehicle_component=1,
        window=window,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
    )
    worker.start()

    connection.messages.put(_TelemetryMessage("HEARTBEAT", armed=True))
    assert stop.wait(timeout=1.0)
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert state.armed_abort
    assert state.worker_error == "vehicle became ARMED during camera startup or capture"
    assert not window.started.is_set()
    assert (tmp_path / "telemetry.jsonl").read_text() == ""


def test_passive_flight_worker_accepts_arming_before_ready(tmp_path) -> None:
    connection = _QueuedConnection()
    stop = threading.Event()
    state = CaptureState()
    window = CaptureWindow(duration=1.0)
    window.begin()
    worker = TelemetryWorker(
        connection=connection,
        output=tmp_path / "telemetry.jsonl",
        vehicle_system=1,
        vehicle_component=1,
        window=window,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
        allow_armed_at_any_time=True,
    )
    worker.start()

    connection.messages.put(_TelemetryMessage("HEARTBEAT", armed=True))
    deadline = time.monotonic() + 1
    while state.telemetry_counts["HEARTBEAT"] < 1 and time.monotonic() < deadline:
        time.sleep(0.001)
    assert not stop.is_set()
    stop.set()
    worker.join(timeout=1.0)

    assert not state.armed_abort
    assert state.saw_armed
    assert state.last_vehicle_state == "armed"
    assert state.worker_error is None
    assert not state.disarmed_heartbeat.is_set()
    assert state.telemetry_counts["HEARTBEAT"] == 1
    assert not window.ready.is_set()


def test_passive_flight_worker_records_arm_and_disarm_after_ready(tmp_path) -> None:
    connection = _QueuedConnection()
    stop = threading.Event()
    state = CaptureState(last_vehicle_state="disarmed")
    window = CaptureWindow(duration=5.0)
    window.begin()
    window.ready.set()
    worker = TelemetryWorker(
        connection=connection,
        output=tmp_path / "telemetry.jsonl",
        vehicle_system=1,
        vehicle_component=1,
        window=window,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
        allow_armed_at_any_time=True,
    )
    worker.start()

    connection.messages.put(_TelemetryMessage("HEARTBEAT", armed=True))
    deadline = time.monotonic() + 1.0
    while state.telemetry_counts["HEARTBEAT"] < 1 and time.monotonic() < deadline:
        time.sleep(0.001)

    assert state.telemetry_counts["HEARTBEAT"] == 1
    assert not stop.is_set()
    assert state.saw_armed
    assert not state.saw_disarmed_after_arm
    assert state.last_vehicle_state == "armed"
    assert not state.disarmed_heartbeat.is_set()

    connection.messages.put(_TelemetryMessage("HEARTBEAT", armed=False))
    deadline = time.monotonic() + 1.0
    while state.telemetry_counts["HEARTBEAT"] < 2 and time.monotonic() < deadline:
        time.sleep(0.001)
    stop.set()
    worker.join(timeout=1.0)

    records = [
        json.loads(line)
        for line in (tmp_path / "telemetry.jsonl").read_text().splitlines()
    ]
    assert [record["message"] for record in records] == ["HEARTBEAT", "HEARTBEAT"]
    assert not state.armed_abort
    assert state.worker_error is None
    assert state.saw_disarmed_after_arm
    assert state.last_vehicle_state == "disarmed"


def test_passive_flight_worker_ignores_other_vehicle_arming(tmp_path) -> None:
    connection = _QueuedConnection()
    stop = threading.Event()
    state = CaptureState(last_vehicle_state="disarmed")
    window = CaptureWindow(duration=1.0)
    window.begin()
    worker = TelemetryWorker(
        connection=connection,
        output=tmp_path / "telemetry.jsonl",
        vehicle_system=1,
        vehicle_component=1,
        window=window,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
        allow_armed_at_any_time=True,
    )
    worker.start()

    observed = threading.Event()
    connection.messages.put(
        _TelemetryMessage("HEARTBEAT", armed=True, observed=observed, system=2)
    )
    assert observed.wait(timeout=1.0)
    stop.set()
    worker.join(timeout=1.0)

    assert not state.armed_abort
    assert not state.saw_armed
    assert state.last_vehicle_state == "disarmed"


def test_telemetry_worker_discards_pre_epoch_messages(tmp_path) -> None:
    connection = _QueuedConnection()
    stop = threading.Event()
    state = CaptureState()
    window = CaptureWindow(duration=1.0)
    worker = TelemetryWorker(
        connection=connection,
        output=tmp_path / "telemetry.jsonl",
        vehicle_system=1,
        vehicle_component=1,
        window=window,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
    )
    worker.start()

    pre_epoch_observed = threading.Event()
    connection.messages.put(_TelemetryMessage("ATTITUDE", observed=pre_epoch_observed))
    assert pre_epoch_observed.wait(timeout=1.0)
    window.begin()
    connection.messages.put(_TelemetryMessage("SYS_STATUS"))
    deadline = time.monotonic() + 1.0
    while state.telemetry_counts["SYS_STATUS"] == 0 and time.monotonic() < deadline:
        time.sleep(0.001)
    stop.set()
    worker.join(timeout=1.0)

    records = [
        json.loads(line)
        for line in (tmp_path / "telemetry.jsonl").read_text().splitlines()
    ]
    assert [record["message"] for record in records] == ["SYS_STATUS"]
    assert state.telemetry_counts == {"SYS_STATUS": 1}


def test_capture_epoch_opens_raw_log_before_publishing_start(tmp_path) -> None:
    window = CaptureWindow(duration=2.0)
    calls = []

    def setup_logfile(path: str) -> None:
        calls.append((path, window.started.is_set()))

    connection = SimpleNamespace(setup_logfile=setup_logfile)
    state = CaptureState()
    stop = threading.Event()
    tlog = tmp_path / "telemetry.tlog"

    assert _start_capture_epoch(connection, tlog, window, stop, state)

    started, started_utc, deadline = window.require_started()
    assert calls == [(str(tlog), False)]
    assert window.started.is_set()
    assert started_utc.tzinfo is UTC
    assert deadline == pytest.approx(started + 2.0)
    assert state.worker_error is None


def test_capture_epoch_failure_aborts_without_publishing_start(tmp_path) -> None:
    def fail_setup(_path: str) -> None:
        raise OSError("cannot open tlog")

    window = CaptureWindow(duration=2.0)
    state = CaptureState()
    stop = threading.Event()

    assert not _start_capture_epoch(
        SimpleNamespace(setup_logfile=fail_setup),
        tmp_path / "telemetry.tlog",
        window,
        stop,
        state,
    )
    assert stop.is_set()
    assert not window.started.is_set()
    assert state.worker_error == "start synchronized capture epoch: cannot open tlog"


@pytest.mark.parametrize("require_disarmed", [False, True])
def test_fresh_heartbeat_gate_is_satisfied_only_by_monitor_worker(
    tmp_path, require_disarmed
) -> None:
    cleared = threading.Event()

    class SignalingEvent(threading.Event):
        def clear(self) -> None:
            super().clear()
            cleared.set()

    connection = _QueuedConnection()
    stop = threading.Event()
    state = CaptureState(
        disarmed_heartbeat=SignalingEvent(), vehicle_heartbeat=SignalingEvent()
    )
    worker = TelemetryWorker(
        connection=connection,
        output=tmp_path / "telemetry.jsonl",
        vehicle_system=1,
        vehicle_component=1,
        window=CaptureWindow(duration=1.0),
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
    )
    outcome = []
    worker.start()
    waiter = threading.Thread(
        target=lambda: outcome.append(
            _wait_for_fresh_heartbeat(
                stop, state, timeout=0.5, require_disarmed=require_disarmed
            )
        )
    )
    waiter.start()
    assert cleared.wait(timeout=1.0)

    connection.messages.put(_TelemetryMessage("HEARTBEAT", armed=False))
    waiter.join(timeout=1.0)
    stop.set()
    worker.join(timeout=1.0)

    assert not waiter.is_alive()
    assert outcome == [None]
    assert state.worker_error is None


def test_video_timestamp_summary_failure_is_recorded_without_raising(tmp_path) -> None:
    timestamps = tmp_path / "camera.pts"
    timestamps.write_text("partial-line\n")
    state = CaptureState()

    summary = _safe_video_timestamp_summary(timestamps, state)

    assert summary == {
        "encoded_frames": 0,
        "encoded_span_s": 0.0,
        "encoded_duration_s": 0.0,
    }
    assert state.worker_error is not None
    assert state.worker_error.startswith("summarize H.264 timestamps:")


def test_preview_write_false_reports_exact_missing_artifact(tmp_path) -> None:
    path = tmp_path / "first-frame.jpg"
    cv2 = SimpleNamespace(imwrite=lambda _path, _frame: False)

    with pytest.raises(RuntimeError, match=r"first-frame.*first-frame\.jpg"):
        _write_preview_frame(
            cv2,
            path,
            np.zeros((2, 2), dtype=np.uint8),
            "first-frame",
        )


def test_cleanup_attempts_every_resource_and_retains_first_error(tmp_path) -> None:
    events = []

    class Camera:
        def stop_encoder(self, _encoder) -> None:
            events.append("stop_encoder")
            raise RuntimeError("encoder stop failed")

        def stop(self) -> None:
            events.append("stop_camera")
            raise RuntimeError("camera stop failed")

        def close(self) -> None:
            events.append("close_camera")

    class Logfile:
        def flush(self) -> None:
            events.append("flush_log")
            raise OSError("flush failed")

        def close(self) -> None:
            events.append("close_log")

    class Connection:
        def __init__(self) -> None:
            self.logfile = Logfile()

        def recv_match(self, **_kwargs):
            return None

        def close(self) -> None:
            events.append("close_connection")

    class Cv2:
        def imwrite(self, path: str, _frame) -> bool:
            events.append(f"write:{path}")
            return not path.endswith("first-frame.jpg")

    paths = create_recording_paths(tmp_path / "cleanup")
    state = CaptureState()
    stop = threading.Event()
    connection = Connection()
    telemetry = TelemetryWorker(
        connection=connection,
        output=paths.telemetry_events,
        vehicle_system=1,
        vehicle_component=1,
        window=CaptureWindow(1.0),
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
    )
    detection = DetectionWorker(
        frames=queue.Queue(),
        output=paths.camera_events,
        detector=_FailingDetector(),
        calibration=None,
        tag_size=0.16,
        resolution=(2, 2),
        max_reprojection_error=2.0,
        target_id=None,
        stop=stop,
        state=state,
        sync=IntervalSync(0.0),
    )
    frame = np.zeros((2, 2), dtype=np.uint8)

    _cleanup_capture(
        camera=Camera(),
        encoder=object(),
        camera_started=True,
        encoder_started=True,
        telemetry_worker=telemetry,
        detection_worker=detection,
        frames=queue.Queue(),
        cv2=Cv2(),
        paths=paths,
        first_frame=frame,
        last_frame=frame,
        connection=connection,
        stop=stop,
        state=state,
    )
    paths.manifest.write_text("manifest still reachable\n")

    assert events == [
        "stop_encoder",
        "stop_camera",
        "close_camera",
        f"write:{paths.first_frame}",
        f"write:{paths.last_frame}",
        "flush_log",
        "close_log",
        "close_connection",
    ]
    assert connection.logfile is None
    assert state.worker_error == "stop H.264 encoder: encoder stop failed"
    assert paths.manifest.read_text() == "manifest still reachable\n"


def test_cleanup_does_not_replace_original_capture_error(tmp_path) -> None:
    class Camera:
        def close(self) -> None:
            raise RuntimeError("cleanup failed")

    connection = SimpleNamespace(logfile=None, close=lambda: None)
    paths = create_recording_paths(tmp_path / "original-error")
    state = CaptureState(worker_error="original capture failure")

    _cleanup_capture(
        camera=Camera(),
        encoder=None,
        camera_started=False,
        encoder_started=False,
        telemetry_worker=None,
        detection_worker=None,
        frames=queue.Queue(),
        cv2=SimpleNamespace(imwrite=lambda *_args: True),
        paths=paths,
        first_frame=None,
        last_frame=None,
        connection=connection,
        stop=threading.Event(),
        state=state,
    )

    assert state.worker_error == "original capture failure"


def test_cleanup_fsyncs_raw_telemetry_and_camera_artifacts(
    tmp_path, monkeypatch
) -> None:
    paths = create_recording_paths(tmp_path / "durable")
    paths.video.write_bytes(b"video")
    paths.video_timestamps.write_text("0.0\n")
    logfile = paths.telemetry_tlog.open("wb")
    logfile.write(b"telemetry")
    connection = SimpleNamespace(
        logfile=logfile,
        close=lambda: None,
    )
    synced: list[int] = []
    monkeypatch.setattr(
        inspect_cli.os, "fsync", lambda descriptor: synced.append(descriptor)
    )

    _cleanup_capture(
        camera=None,
        encoder=None,
        camera_started=False,
        encoder_started=False,
        telemetry_worker=None,
        detection_worker=None,
        frames=queue.Queue(),
        cv2=None,
        paths=paths,
        first_frame=None,
        last_frame=None,
        connection=connection,
        stop=threading.Event(),
        state=CaptureState(),
    )

    assert len(synced) == 3
    assert logfile.closed
    assert connection.logfile is None


def test_inspector_classifies_observed_range_and_flow_streams() -> None:
    state = CaptureState()
    _observe_sensor_message(
        state,
        SimpleNamespace(
            get_type=lambda: "DISTANCE_SENSOR",
            orientation=25,
            current_distance=42,
            min_distance=2,
            max_distance=1200,
        ),
    )
    _observe_sensor_message(
        state,
        SimpleNamespace(get_type=lambda: "OPTICAL_FLOW_RAD", quality=180),
    )

    components = _component_report(
        state,
        on_pi=True,
        flight_controller="ok",
        camera="unavailable",
        detector="unavailable",
        details={},
        duration=2.0,
    )

    assert components["downward_rangefinder"] == {
        "status": "ok",
        "samples": 1,
        "latest_m": 0.42,
        "source": "DISTANCE_SENSOR",
        "rate_hz": 0.5,
    }
    assert components["forward_rangefinder"]["status"] == "no_data"
    assert components["optical_flow"]["status"] == "ok"
    assert components["servo"]["status"] == "not_detectable"


@pytest.mark.parametrize(
    ("flight_controller", "camera", "detector", "range_status", "tag_status"),
    [
        ("ok", "ok", "ok", "no_data", "no_data"),
        ("ok", "unavailable", "unavailable", "no_data", "unavailable"),
        ("unavailable", "ok", "ok", "unavailable", "no_data"),
        ("unavailable", "unavailable", "unavailable", "unavailable", "unavailable"),
    ],
)
def test_inspector_reports_every_source_availability_combination(
    flight_controller: str,
    camera: str,
    detector: str,
    range_status: str,
    tag_status: str,
) -> None:
    components = _component_report(
        CaptureState(),
        on_pi=False,
        flight_controller=flight_controller,
        camera=camera,
        detector=detector,
        details={},
        duration=1.0,
    )

    assert components["downward_rangefinder"]["status"] == range_status
    assert components["apriltags"]["status"] == tag_status


def test_inspector_succeeds_and_writes_manifest_when_all_hardware_is_absent(
    tmp_path, monkeypatch
) -> None:
    output = tmp_path / "inspection"
    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        inspect_cli,
        "resolve_mavlink_endpoint",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(FileNotFoundError("none")),
    )

    assert run(["--duration", "0.01", "--output-dir", str(output)]) == 0

    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["completed"] is True
    assert manifest["components"]["flight_controller"]["status"] == "unavailable"
    assert manifest["components"]["camera"]["status"] == "unavailable"


def test_passive_flight_recording_reports_missing_hardware_and_finishes(
    tmp_path, monkeypatch, capsys
) -> None:
    output = tmp_path / "manual-unavailable"
    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        inspect_cli,
        "resolve_mavlink_endpoint",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(FileNotFoundError("none")),
    )

    assert (
        run(
            [
                "--duration",
                "0.01",
                "--output-dir",
                str(output),
                "--allow-flight",
            ]
        )
        == 0
    )

    assert "READY: recording available sources" in capsys.readouterr().out
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["completed"] is True
    assert manifest["safety"]["allow_flight"] is True
    assert manifest["error"] is None
    assert manifest["components"]["flight_controller"]["status"] == "unavailable"
    assert manifest["components"]["camera"]["status"] == "unavailable"


@pytest.mark.parametrize("allow_flight", [False, True])
def test_only_explicit_passive_flight_accepts_initially_armed_vehicle(
    tmp_path, monkeypatch, allow_flight: bool
) -> None:
    output = tmp_path / "armed"

    class Connection:
        target_system = 1
        target_component = 1
        closed = False
        logfile = None

        def wait_heartbeat(self, *, timeout):
            return _Heartbeat(system=1, armed=True)

        def close(self) -> None:
            self.closed = True

        def recv_match(self, *, blocking, timeout):
            time.sleep(min(timeout, 0.001))
            return _TelemetryMessage("HEARTBEAT", armed=True)

        def setup_logfile(self, path):
            self.logfile = Path(path).open("wb")  # noqa: SIM115 - recorder closes it

    connection = Connection()
    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        inspect_cli, "resolve_mavlink_endpoint", lambda *_args, **_kwargs: "tcp:sim"
    )
    monkeypatch.setattr(
        inspect_cli,
        "open_ardupilot_connection",
        lambda *_args, **_kwargs: connection,
    )
    monkeypatch.setattr(
        inspect_cli, "request_telemetry_messages", lambda _connection: []
    )

    arguments = ["--duration", "0.01", "--output-dir", str(output)]
    if allow_flight:
        arguments.append("--allow-flight")

    assert run(arguments) == (0 if allow_flight else 3)
    assert connection.closed
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["armed_abort"] is not allow_flight
    assert manifest["safety"]["initial_vehicle_state"] == "armed"
    assert manifest["safety"]["allow_flight"] is allow_flight
    assert manifest["safety"]["saw_armed"] is True
    assert manifest["safety"]["saw_disarmed_after_arm"] is False
    assert manifest["safety"]["last_vehicle_state"] == "armed"


def test_startup_interrupt_closes_flight_controller_and_writes_failure_manifest(
    tmp_path, monkeypatch
):
    class InterruptedConnection:
        logfile = None
        closed = False

        def wait_heartbeat(self, **_kwargs):
            raise KeyboardInterrupt

        def close(self):
            self.closed = True

    connection = InterruptedConnection()
    output = tmp_path / "startup-interrupt"
    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        inspect_cli, "resolve_mavlink_endpoint", lambda *_a, **_kw: "mock:fc"
    )
    monkeypatch.setattr(
        inspect_cli, "open_ardupilot_connection", lambda *_a, **_kw: connection
    )

    assert run(["--duration", "0.01", "--output-dir", str(output)]) == 1
    assert connection.closed
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["completed"] is False
    assert manifest["stop_reason"] == "operator_interrupt"
    assert "interrupted" in manifest["error"]


@pytest.mark.parametrize(
    "failure", [RuntimeError("summary failed"), KeyboardInterrupt()]
)
def test_summary_failure_writes_minimal_manifest(tmp_path, monkeypatch, failure):
    output = tmp_path / "summary-failure"
    monkeypatch.setattr(inspect_cli, "_start_recording", lambda recording: None)
    monkeypatch.setattr(inspect_cli, "_record_capture", lambda recording: None)

    def fail(recording):
        raise failure

    monkeypatch.setattr(inspect_cli, "_finish_recording", fail)
    assert run(["--output-dir", str(output)]) == 1
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["completed"] is False
    assert manifest["finalization_error"] == (str(failure) or type(failure).__name__)


def test_capture_snapshot_is_coherent_and_owns_counter_copies():
    state = CaptureState()
    done = threading.Event()

    def produce():
        for index in range(1000):
            with state.lock:
                state.telemetry_counts[str(index)] += 1
                state.vehicle_telemetry_counts[str(index)] += 1
        done.set()

    worker = threading.Thread(target=produce)
    worker.start()
    while not done.is_set():
        snapshot = state.snapshot()
        assert snapshot.telemetry_counts == snapshot.vehicle_telemetry_counts
    worker.join(timeout=1)
    snapshot = state.snapshot()
    state.telemetry_counts.clear()
    assert sum(snapshot.telemetry_counts.values()) == 1000
    state.record_error("first")
    state.record_error("cleanup")
    assert state.snapshot().errors == ["first", "cleanup"]
    assert state.worker_error == "first"


class _UartRetryConnection:
    target_system = target_component = 1
    logfile = None

    def __init__(self, retry_result, reopen_ok):
        self.retry_result = retry_result
        self.reopen_ok = reopen_ok
        self.events = []
        self.sent = []
        self.mav = SimpleNamespace(
            total_bytes_received=1564,
            total_packets_received=118,
            total_receive_errors=117,
            command_long_send=lambda *values: self.sent.append(values),
        )

    def wait_heartbeat(self, *, timeout):
        self.events.append(("wait", timeout))
        if len([event for event in self.events if event[0] == "wait"]) == 1:
            return None
        assert self.events[-2] == ("reset",)
        if self.retry_result:
            return _Heartbeat(system=1, armed=self.retry_result == "armed")
        return None

    def reset(self):
        self.events.append(("reset",))
        assert not self.sent
        return self.reopen_ok

    def setup_logfile(self, path):
        pass

    def recv_match(self, *, blocking, timeout):
        time.sleep(min(timeout, 0.002))
        return _TelemetryMessage("HEARTBEAT", armed=False)

    def close(self):
        self.events.append(("close",))


@pytest.mark.parametrize(
    (
        "on_pi",
        "endpoint",
        "retry_result",
        "reopen_ok",
        "expected_waits",
        "expected_code",
    ),
    [
        (True, "/dev/serial0", "disarmed", True, 2, 0),
        (True, "/dev/ttyAMA0", "disarmed", True, 2, 0),
        (True, "/dev/serial0", "armed", True, 2, 3),
        (True, "/dev/serial0", None, True, 2, 0),
        (True, "/dev/serial0", None, False, 1, 0),
        (False, "/dev/serial0", None, True, 1, 0),
        (True, "/dev/ttyACM0", None, True, 1, 0),
        (True, "tcp:127.0.0.1:5760", None, True, 1, 0),
    ],
)
def test_initial_uart_timeout_retries_once_only_on_pi_and_preserves_safety(
    tmp_path,
    monkeypatch,
    capsys,
    on_pi,
    endpoint,
    retry_result,
    reopen_ok,
    expected_waits,
    expected_code,
) -> None:
    output = tmp_path / "uart-retry"
    connection = _UartRetryConnection(retry_result, reopen_ok)
    events, sent = connection.events, connection.sent
    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: on_pi)
    monkeypatch.setattr(
        inspect_cli, "resolve_mavlink_endpoint", lambda *_a, **_kw: endpoint
    )
    monkeypatch.setattr(
        inspect_cli, "open_ardupilot_connection", lambda *_a, **_kw: connection
    )
    monkeypatch.setitem(sys.modules, "cv2", None)

    assert (
        run(["--duration", "0.03", "--timeout", "0.5", "--output-dir", str(output)])
        == expected_code
    )

    assert events.count(("wait", 0.5)) == expected_waits
    should_retry = on_pi and endpoint in ("/dev/serial0", "/dev/ttyAMA0")
    assert events.count(("reset",)) == int(should_retry)
    assert events.count(("close",)) == 1
    manifest = json.loads((output / "manifest.json").read_text())
    log = capsys.readouterr().out
    if should_retry:
        assert "bytes=1564 packets=118 parse_errors=117" in log
        outcome = (
            "reopen failed"
            if not reopen_ok
            else ("recovered" if retry_result else "no heartbeat")
        )
        assert f"Pi UART retry: {outcome} after" in log
    else:
        assert "Pi UART retry:" not in log
    if should_retry and retry_result == "disarmed":
        assert manifest["components"]["flight_controller"]["status"] == "ok"
        assert sent and all(
            values[2] == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL for values in sent
        )
    else:
        assert sent == []
    assert manifest["armed_abort"] is (should_retry and retry_result == "armed")
    if should_retry and retry_result is None:
        assert manifest["components"]["flight_controller"]["status"] == "unavailable"
        assert "Pi UART" in manifest["components"]["flight_controller"]["detail"]


def test_timestamp_parse_failure_cannot_publish_success_manifest(tmp_path, monkeypatch):
    output = tmp_path / "bad-timestamps"
    paths = create_recording_paths(output)
    paths.video_timestamps.write_text("not-a-timestamp\n")
    monkeypatch.setattr(inspect_cli, "create_recording_paths", lambda _path: paths)
    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        inspect_cli,
        "resolve_mavlink_endpoint",
        lambda *_a, **_kw: (_ for _ in ()).throw(FileNotFoundError("none")),
    )

    assert run(["--duration", "0.01", "--output-dir", str(output)]) == 1
    manifest = json.loads(paths.manifest.read_text())
    assert manifest["completed"] is False
    assert "summarize H.264 timestamps" in manifest["error"]


def test_telemetry_duration_stop_records_reason_before_setting_stop(tmp_path):
    state = CaptureState()
    window = CaptureWindow(1)
    window.begin()
    window.deadline = time.monotonic() - 1

    class CheckedStop(threading.Event):
        def set(self):
            assert state.stop_reason == "duration_elapsed"
            super().set()

    stop = CheckedStop()
    worker = TelemetryWorker(
        connection=_QueuedConnection(),
        output=tmp_path / "telemetry.jsonl",
        vehicle_system=1,
        vehicle_component=1,
        window=window,
        stop=stop,
        state=state,
        sync=IntervalSync(0),
    )
    worker.run()
    assert stop.is_set()
    assert state.worker_error is None


def test_camera_startup_interrupt_releases_acquired_camera(tmp_path, monkeypatch):
    import sys

    class InterruptedCamera:
        closed = False

        def create_video_configuration(self, **_kwargs):
            return {}

        def configure(self, _config):
            raise KeyboardInterrupt

        def close(self):
            self.closed = True

    camera = InterruptedCamera()
    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: True)
    monkeypatch.setattr(
        inspect_cli,
        "resolve_mavlink_endpoint",
        lambda *_a, **_kw: (_ for _ in ()).throw(FileNotFoundError("none")),
    )
    monkeypatch.setattr(
        inspect_cli,
        "create_detector",
        lambda *_a, **_kw: SimpleNamespace(backend_name="mock"),
    )
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(setNumThreads=lambda _threads: None)
    )
    monkeypatch.setitem(
        sys.modules, "picamera2", SimpleNamespace(Picamera2=lambda: camera)
    )
    monkeypatch.setitem(
        sys.modules, "picamera2.encoders", SimpleNamespace(H264Encoder=object)
    )
    monkeypatch.setitem(
        sys.modules, "picamera2.outputs", SimpleNamespace(FileOutput=object)
    )
    output = tmp_path / "camera-interrupt"

    assert run(["--duration", "0.01", "--output-dir", str(output)]) == 1
    assert camera.closed
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["completed"] is False
    assert manifest["stop_reason"] == "operator_interrupt"


class _FrameRequest:
    def __init__(self):
        self.releases = 0

    def release(self):
        self.releases += 1


class _AsyncFrameCamera:
    def __init__(self):
        self.now = 0.0
        self.result = None
        self.cancelled = False
        self.requests = 0
        self.cancellations = 0
        self.waits = []
        self.on_wait = None
        self.on_cancel = None
        self.signal_function = None

    def capture_request(self, *, wait, signal_function):
        assert wait is False
        self.requests += 1
        self.signal_function = signal_function
        return self

    def wait(self, job, *, timeout):
        from concurrent.futures import CancelledError

        assert job is self
        self.waits.append(timeout)
        if self.result is not None:
            return self.result
        if self.cancelled:
            raise CancelledError
        self.now += timeout
        if self.on_wait is not None:
            self.on_wait()
        if self.result is not None:
            return self.result
        raise TimeoutError

    def cancel_all_and_flush(self):
        self.cancellations += 1
        if self.on_cancel is not None:
            self.on_cancel()
        else:
            self.cancelled = True

    def complete(self, request):
        self.result = request
        assert self.signal_function is not None
        self.signal_function(self)


def _bounded_frame(camera, state, stop, *, deadline=None, frame_timeout=0.25):
    return inspect_cli._capture_request_bounded(
        camera,
        stop=stop,
        state=state,
        deadline=deadline,
        frame_timeout=frame_timeout,
    )


def test_stalled_frame_request_stops_capture_and_cancels_pending_job(monkeypatch):
    camera = _AsyncFrameCamera()
    state, stop = CaptureState(), threading.Event()
    monkeypatch.setattr(inspect_cli.time, "monotonic", lambda: camera.now)

    with pytest.raises(
        TimeoutError, match=r"camera did not deliver a frame within 0\.25 seconds"
    ):
        _bounded_frame(camera, state, stop)
    assert stop.is_set()
    assert state.stop_reason == "camera_stalled"
    assert state.worker_error is not None
    assert "0.25 seconds" in state.worker_error
    assert camera.cancellations == 1
    assert camera.now == pytest.approx(0.25)
    assert max(camera.waits) <= 0.1


def test_frame_wait_respects_recording_deadline_without_false_stall(monkeypatch):
    camera = _AsyncFrameCamera()
    state, stop = CaptureState(), threading.Event()
    monkeypatch.setattr(inspect_cli.time, "monotonic", lambda: camera.now)

    assert _bounded_frame(camera, state, stop, deadline=0.15) is None
    assert state.stop_reason == "duration_elapsed"
    assert state.worker_error is None
    assert camera.now == pytest.approx(0.15)
    assert camera.cancellations == 1
    assert stop.is_set()


def test_disarm_stop_cancels_frame_wait_without_waiting_for_timeout(monkeypatch):
    camera = _AsyncFrameCamera()
    state, stop = CaptureState(), threading.Event()
    monkeypatch.setattr(inspect_cli.time, "monotonic", lambda: camera.now)

    def disarm():
        state.set_stop_reason("vehicle_disarmed")
        stop.set()

    camera.on_wait = disarm
    assert _bounded_frame(camera, state, stop) is None
    assert camera.now == pytest.approx(0.1)
    assert state.stop_reason == "vehicle_disarmed"
    assert state.worker_error is None
    assert camera.cancellations == 1


def test_normal_async_request_transfers_release_to_capture_loop(monkeypatch):
    camera = _AsyncFrameCamera()
    state, stop = CaptureState(), threading.Event()
    request = _FrameRequest()
    monkeypatch.setattr(inspect_cli.time, "monotonic", lambda: camera.now)
    camera.on_wait = lambda: camera.complete(request)

    received = _bounded_frame(camera, state, stop)
    assert received is request
    assert request.releases == 0
    assert camera.cancellations == 0
    received.release()
    assert request.releases == 1
    assert state.worker_error is None


@pytest.mark.parametrize("completion_time", ["during_cancel", "after_cancel"])
def test_request_completing_during_cancellation_is_released_once(
    monkeypatch, completion_time
):
    camera = _AsyncFrameCamera()
    state, stop = CaptureState(), threading.Event()
    request = _FrameRequest()
    monkeypatch.setattr(inspect_cli.time, "monotonic", lambda: camera.now)
    camera.on_wait = stop.set
    camera.on_cancel = (
        (lambda: camera.complete(request))
        if completion_time == "during_cancel"
        else (lambda: None)
    )

    assert _bounded_frame(camera, state, stop) is None
    if completion_time == "after_cancel":
        camera.complete(request)
    assert camera.signal_function is not None
    camera.signal_function(camera)
    assert request.releases == 1
    assert state.worker_error is None


def test_completed_request_at_deadline_is_released_instead_of_processed(monkeypatch):
    camera = _AsyncFrameCamera()
    state, stop = CaptureState(), threading.Event()
    request = _FrameRequest()
    monkeypatch.setattr(inspect_cli.time, "monotonic", lambda: camera.now)
    camera.on_wait = lambda: camera.complete(request)

    assert _bounded_frame(camera, state, stop, deadline=0.1) is None
    assert request.releases == 1
    assert state.stop_reason == "duration_elapsed"


class _StalledRecordingCamera(_AsyncFrameCamera):
    closed = False
    encoder_stopped = False
    stopped = False

    def create_video_configuration(self, **_kwargs):
        return {}

    def configure(self, _config):
        pass

    def start(self):
        pass

    def start_encoder(self, _encoder, output, **_kwargs):
        output.outputframe(b"fake", keyframe=True, timestamp=0)

    def stop_encoder(self, _encoder):
        self.encoder_stopped = True

    def stop(self):
        self.stopped = True

    def close(self):
        self.closed = True

    def wait(self, job, *, timeout):
        time.sleep(timeout)
        return super().wait(job, timeout=timeout)


class _RecordingVideoOutput:
    recording = True
    dead = False

    def __init__(self, file, *, pts):
        Path(file).write_bytes(b"fake")
        Path(pts).write_text("0.0\n")

    def outputframe(self, *_args):
        pass


@pytest.mark.parametrize("no_video", [False, True])
def test_camera_stall_writes_failed_manifest_and_attempts_camera_cleanup(
    tmp_path, monkeypatch, no_video
):
    camera = _StalledRecordingCamera()
    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: True)
    monkeypatch.setattr(
        inspect_cli,
        "resolve_mavlink_endpoint",
        lambda *_a, **_kw: (_ for _ in ()).throw(FileNotFoundError("none")),
    )
    monkeypatch.setattr(
        inspect_cli,
        "create_detector",
        lambda *_a, **_kw: (_ for _ in ()).throw(RuntimeError("no detector")),
    )
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(setNumThreads=lambda _threads: None)
    )
    monkeypatch.setitem(
        sys.modules, "picamera2", SimpleNamespace(Picamera2=lambda: camera)
    )
    monkeypatch.setitem(
        sys.modules,
        "picamera2.encoders",
        SimpleNamespace(H264Encoder=lambda **_kw: object()),
    )
    monkeypatch.setitem(
        sys.modules,
        "picamera2.outputs",
        SimpleNamespace(FileOutput=_RecordingVideoOutput),
    )
    output = tmp_path / "stalled-camera"

    assert (
        run(
            [
                "--duration",
                "0.2",
                "--warmup",
                "0",
                "--frame-timeout",
                "0.01",
                "--output-dir",
                str(output),
                *(["--no-video"] if no_video else []),
            ]
        )
        == 1
    )
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["completed"] is False
    assert manifest["stop_reason"] == "camera_stalled"
    assert "camera did not deliver a frame" in manifest["error"]
    assert manifest["components"]["camera"]["status"] == "error"
    assert camera.cancellations == 1
    assert camera.closed and camera.stopped
    assert camera.encoder_stopped is not no_video


class _AnalysisFrameRequest(_FrameRequest):
    def make_array(self, name):
        assert name == "lores"
        return np.zeros((6, 4), dtype=np.uint8)

    def get_metadata(self):
        return {"SensorTimestamp": 123}


class _RecordingCamera:
    def __init__(self):
        self.requests = []
        self.closed = []
        self.frames_after_video_stop = []

    def create_video_configuration(self, **options):
        return options

    def configure(self, config):
        assert config["lores"]["size"] == (4, 4)

    def start(self):
        pass

    def capture_request(self, **_kwargs):
        request = _AnalysisFrameRequest()
        self.requests.append(request)
        if "encoder stopped" in self.closed:
            self.frames_after_video_stop.append(request)
        return request

    def start_encoder(self, _encoder, output, **_kwargs):
        output.outputframe(b"video", keyframe=True, timestamp=0)

    def stop_encoder(self, _encoder):
        self.closed.append("encoder stopped")

    def wait(self, request, *, timeout):
        time.sleep(min(0.005, timeout))
        return request

    def cancel_all_and_flush(self):
        pass

    def stop(self):
        self.closed.append("camera stopped")

    def close(self):
        self.closed.append("camera closed")


def _write_test_preview(path, _frame):
    Path(path).write_bytes(b"preview")
    return True


@pytest.mark.parametrize("preview", [False, True])
@pytest.mark.parametrize(
    "video_mode",
    ["disabled", "reserve_at_start", "reserve_while_recording", "full_while_recording"],
)
def test_video_storage_policy_preserves_analysis_logs_and_optional_preview(
    tmp_path, monkeypatch, preview, video_mode
):
    import ai_drone.storage as storage
    import ai_drone.vision.stream as stream

    camera = _RecordingCamera()
    requests, closed = camera.requests, camera.closed
    frames_after_video_stop = camera.frames_after_video_stop
    streamed = []
    video_started = video_mode in {"reserve_while_recording", "full_while_recording"}
    full = video_mode == "full_while_recording"

    def available(_path):
        free_mib = 2048
        if video_mode == "reserve_at_start" or (video_started and len(requests) >= 3):
            free_mib = 8 if full else 260
        return SimpleNamespace(free=free_mib * storage.MIB)

    monkeypatch.setattr(storage.shutil, "disk_usage", available)

    monkeypatch.setattr(inspect_cli, "is_raspberry_pi", lambda: True)
    monkeypatch.setattr(
        inspect_cli,
        "resolve_mavlink_endpoint",
        lambda *_a, **_kw: (_ for _ in ()).throw(FileNotFoundError("none")),
    )
    monkeypatch.setattr(
        inspect_cli,
        "create_detector",
        lambda *_a, **_kw: SimpleNamespace(
            backend_name="native", detect=lambda _frame: [_tag_detection(7)]
        ),
    )
    monkeypatch.setattr(
        CameraCalibration, "load", lambda _path: _recording_calibration()
    )
    monkeypatch.setattr(
        capture_workers,
        "estimate_pose",
        lambda detection, *_a, **_kw: TagPose(
            detection.tag_id, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 1.0, 0.1
        ),
    )
    monkeypatch.setitem(
        sys.modules, "picamera2", SimpleNamespace(Picamera2=lambda: camera)
    )
    monkeypatch.setitem(
        sys.modules,
        "picamera2.encoders",
        SimpleNamespace(H264Encoder=lambda **_kw: object()) if video_started else None,
    )
    monkeypatch.setitem(
        sys.modules,
        "picamera2.outputs",
        SimpleNamespace(FileOutput=_RecordingVideoOutput) if video_started else None,
    )
    monkeypatch.setitem(
        sys.modules,
        "cv2",
        SimpleNamespace(
            setNumThreads=lambda _threads: None,
            imwrite=_write_test_preview,
            imencode=lambda *_args: (True, np.array([1, 2, 3], dtype=np.uint8)),
            IMWRITE_JPEG_QUALITY=1,
        ),
    )
    monkeypatch.setattr(stream, "push_frame", streamed.append)
    monkeypatch.setattr(
        stream,
        "start_server",
        lambda **_kwargs: SimpleNamespace(
            shutdown=lambda: closed.append("stream stopped"),
            server_close=lambda: closed.append("stream closed"),
        ),
    )
    output = tmp_path / "without-video"

    assert run(
        [
            *(["--no-video"] if video_mode == "disabled" else []),
            "--storage-check-interval",
            "0.001",
            "--duration",
            "0.08",
            "--warmup",
            "0",
            "--resolution",
            "4x4",
            "--analysis-resolution",
            "4x4",
            "--calibration",
            "calibration.json",
            "--output-dir",
            str(output),
            *(["--stream"] if preview else []),
        ]
    ) == (1 if full else 0)

    manifest = json.loads((output / "manifest.json").read_text())
    records = [
        json.loads(line) for line in (output / "camera.jsonl").read_text().splitlines()
    ]
    assert records and records[0]["tags"][0]["camera_xyz_m"] == [0.0, 0.0, 1.0]
    assert records[0]["tags"][0]["pose_valid"] is True
    assert manifest["completed"] is not full
    assert manifest["camera"]["video_enabled"] is (video_mode != "disabled")
    assert manifest["camera"]["encoded_frames"] == int(video_started)
    assert manifest["components"]["camera"]["status"] == "ok"
    assert manifest["components"]["apriltags"]["detections"] > 0
    assert ("video" in manifest["files"]) is video_started
    assert (output / "camera.h264").exists() is video_started
    assert (output / "camera.pts").exists() is video_started
    assert requests and all(request.releases == 1 for request in requests)
    assert bool(streamed) is preview
    assert [item for item in closed if item != "encoder stopped"] == (
        ["stream stopped", "stream closed"] if preview else []
    ) + [
        "camera stopped",
        "camera closed",
    ]
    assert closed.count("encoder stopped") == int(video_started)
    events = [
        json.loads(line)["event"]
        for line in (output / "storage.jsonl").read_text().splitlines()
    ]
    expected_event = {
        "disabled": "storage_sample",
        "reserve_at_start": "video_skipped_storage",
        "reserve_while_recording": "video_stopped_storage",
        "full_while_recording": "capture_stopped_storage",
    }[video_mode]
    assert expected_event in events
    assert manifest["stop_reason"] == ("storage_full" if full else "duration_elapsed")
    if video_mode == "reserve_while_recording":
        assert frames_after_video_stop
        assert events.count("video_stopped_storage") == 1
        assert manifest["storage"]["video_stopped_reason"] == "storage_reserve"


def _distance_message(cm, *, orientation=25, signal_quality=0):
    return SimpleNamespace(
        get_type=lambda: "DISTANCE_SENSOR",
        orientation=orientation,
        current_distance=cm,
        min_distance=1,
        max_distance=1000,
        signal_quality=signal_quality,
    )


def _range_message(metres):
    return SimpleNamespace(get_type=lambda: "RANGEFINDER", distance=metres)


def _sensor_report(state, *, observed_at):
    return _component_report(
        state,
        on_pi=True,
        flight_controller="ok",
        camera="ok",
        detector="ok",
        details={},
        duration=1,
        observed_at=observed_at,
    )


@pytest.mark.parametrize("flow_type", ["OPTICAL_FLOW", "OPTICAL_FLOW_RAD"])
def test_live_and_manifest_samples_expire_during_silence_and_recover(
    monkeypatch, capsys, flow_type
):
    state = CaptureState()
    now = [10.0]
    monkeypatch.setattr(capture_reporting.time, "monotonic", lambda: now[0])

    def observe(downward, forward, quality):
        state.observe_vehicle_state(armed=False)
        _observe_sensor_message(state, _distance_message(downward))
        _observe_sensor_message(state, _distance_message(forward, orientation=0))
        _observe_sensor_message(
            state, SimpleNamespace(get_type=lambda: flow_type, quality=quality)
        )

    observe(42, 155, 255)
    for observed_at in (10.0, 12.0):
        now[0] = observed_at
        report = _sensor_report(state, observed_at=now[0])
        assert all(
            report[name]["status"] == "ok"
            for name in (
                "flight_controller",
                "downward_rangefinder",
                "forward_rangefinder",
                "optical_flow",
            )
        )
        capture_reporting._print_live_status(state)
        line = capsys.readouterr().out
        assert "vehicle_state=disarmed heartbeat_status=ok" in line
        assert "downward_m=0.42 forward_m=1.55 flow_quality=255" in line

    now[0] = 12.01
    report = _sensor_report(state, observed_at=now[0])
    assert all(
        report[name]["status"] == "stale"
        for name in (
            "flight_controller",
            "downward_rangefinder",
            "forward_rangefinder",
            "optical_flow",
        )
    )
    assert report["downward_rangefinder"]["latest_m"] == 0.42
    assert report["forward_rangefinder"]["latest_m"] == 1.55
    assert report["optical_flow"]["quality"] == 255
    assert state.last_vehicle_state == "disarmed"
    capture_reporting._print_live_status(state)
    line = capsys.readouterr().out
    assert "vehicle_state=unknown heartbeat_status=stale" in line
    assert "downward_m=stale forward_m=stale flow_quality=stale" in line
    assert "0.42" not in line and "1.55" not in line and "255" not in line

    now[0] = 13.0
    observe(45, 160, 180)
    recovered = _sensor_report(state, observed_at=now[0])
    assert recovered["flight_controller"]["status"] == "ok"
    assert recovered["optical_flow"]["status"] == "ok"
    assert recovered["optical_flow"]["samples"] == 2
    capture_reporting._print_live_status(state)
    line = capsys.readouterr().out
    assert "vehicle_state=disarmed heartbeat_status=ok" in line
    assert "downward_m=0.45 forward_m=1.6 flow_quality=180" in line
    assert state.worker_error is None and state.stop_reason is None


@pytest.mark.parametrize(
    ("quality", "status"), [(None, "unknown_quality"), (0, "low_quality")]
)
def test_flow_freshness_is_independent_of_quality_and_heartbeat(
    monkeypatch, quality, status
):
    state = CaptureState()
    now = [10.0]
    monkeypatch.setattr(capture_reporting.time, "monotonic", lambda: now[0])
    state.observe_vehicle_state(armed=False)
    message = SimpleNamespace(get_type=lambda: "OPTICAL_FLOW", quality=quality)
    _observe_sensor_message(state, message)
    assert _sensor_report(state, observed_at=10)["optical_flow"]["status"] == status
    assert _sensor_report(state, observed_at=9)["optical_flow"]["status"] == "stale"

    now[0] = 13.0
    assert _sensor_report(state, observed_at=13)["optical_flow"]["status"] == "stale"
    _observe_sensor_message(state, message)
    report = _sensor_report(state, observed_at=13)
    assert report["optical_flow"]["status"] == status
    assert report["flight_controller"]["status"] == "stale"
    assert (
        _sensor_report(state, observed_at=9)["flight_controller"]["status"] == "stale"
    )


def test_silent_telemetry_stays_running_until_duration_and_keeps_received_records(
    tmp_path, monkeypatch
):
    now = [10.0]
    monkeypatch.setattr(capture_workers.time, "monotonic", lambda: now[0])
    state = CaptureState()
    stop = threading.Event()
    window = CaptureWindow(5.0)
    window.begin()
    flow = _TelemetryMessage("OPTICAL_FLOW")
    vars(flow)["quality"] = 180
    messages = iter([_TelemetryMessage("HEARTBEAT"), flow])
    stale_checks = []

    def receive(*, blocking, timeout):
        assert blocking and not stop.is_set()
        now[0] = min(15.0, round(now[0] + timeout, 6))
        message = next(messages, None)
        if message is None and now[0] > 12.2:
            report = _sensor_report(state, observed_at=now[0])
            stale_checks.append(report["flight_controller"]["status"])
            assert report["optical_flow"]["status"] == "stale"
        return message

    output = tmp_path / "telemetry.jsonl"
    worker = TelemetryWorker(
        connection=SimpleNamespace(recv_match=receive),
        output=output,
        vehicle_system=1,
        vehicle_component=1,
        window=window,
        stop=stop,
        state=state,
        sync=IntervalSync(0),
    )
    worker.run()

    assert stale_checks and set(stale_checks) == {"stale"}
    assert now[0] == 15.0
    assert stop.is_set() and state.stop_reason == "duration_elapsed"
    assert state.worker_error is None and not state.armed_abort
    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert [record["message"] for record in records] == ["HEARTBEAT", "OPTICAL_FLOW"]
    assert state.latest_flow_quality == 180
    assert state.flow_observed_monotonic == pytest.approx(10.2)


@pytest.mark.parametrize(
    ("orientation", "component"),
    [(0, "forward_rangefinder"), (25, "downward_rangefinder")],
)
@pytest.mark.parametrize("signal_quality", [0, 1, 73])
def test_range_health_excludes_explicitly_invalid_signal_quality(
    monkeypatch, orientation, component, signal_quality
):
    state = CaptureState()
    monkeypatch.setattr(capture_reporting.time, "monotonic", lambda: 10.0)
    _observe_sensor_message(
        state,
        _distance_message(150, orientation=orientation, signal_quality=signal_quality),
    )

    report = _sensor_report(state, observed_at=10)[component]
    valid = signal_quality != 1
    assert report["status"] == ("ok" if valid else "no_data")
    assert report["samples"] == (1 if valid else 0)
    assert report["latest_m"] == (1.5 if valid else None)
    assert state.distance_observed_monotonic.get(orientation) == (
        10.0 if valid else None
    )


@pytest.mark.parametrize(
    ("orientation", "component"),
    [(0, "forward_rangefinder"), (25, "downward_rangefinder")],
)
def test_invalid_range_does_not_replace_or_refresh_last_valid_sample(
    monkeypatch, orientation, component
):
    state = CaptureState()
    now = [10.0]
    monkeypatch.setattr(capture_reporting.time, "monotonic", lambda: now[0])
    _observe_sensor_message(state, _distance_message(150, orientation=orientation))
    now[0] = 11.5
    _observe_sensor_message(
        state, _distance_message(900, orientation=orientation, signal_quality=1)
    )

    assert _sensor_report(state, observed_at=12)[component]["status"] == "ok"
    report = _sensor_report(state, observed_at=12.01)[component]
    assert report["status"] == "stale"
    assert report["samples"] == 1
    assert report["latest_m"] == 1.5
    assert state.distance_observed_monotonic[orientation] == 10.0


def test_forward_and_downward_ranges_keep_independent_values_and_freshness(
    monkeypatch,
):
    state = CaptureState()
    now = [10.0]
    monkeypatch.setattr(capture_reporting.time, "monotonic", lambda: now[0])
    _observe_sensor_message(state, _distance_message(155, orientation=0))
    _observe_sensor_message(state, _distance_message(42))
    now[0] = 13.0
    _observe_sensor_message(state, _distance_message(47))
    _observe_sensor_message(state, _range_message(0.47))

    report = _sensor_report(state, observed_at=13)
    assert report["forward_rangefinder"] == {
        "status": "stale",
        "samples": 1,
        "latest_m": 1.55,
        "rate_hz": 1.0,
    }
    assert report["downward_rangefinder"] == {
        "status": "ok",
        "samples": 2,
        "latest_m": 0.47,
        "source": "DISTANCE_SENSOR",
        "rate_hz": 2.0,
    }
    # Cleanup after a recording must not age a sample past its capture end.
    assert (
        _sensor_report(state, observed_at=11)["forward_rangefinder"]["status"] == "ok"
    )
    # Future-dated observations cannot establish current health.
    assert (
        _sensor_report(state, observed_at=9)["forward_rangefinder"]["status"] == "stale"
    )
    _observe_sensor_message(state, _distance_message(160, orientation=0))
    recovered = _sensor_report(state, observed_at=13)["forward_rangefinder"]
    assert recovered["status"] == "ok"
    assert recovered["samples"] == 2
    assert recovered["latest_m"] == 1.6


def test_downward_range_prefers_oriented_stream_without_double_counting(monkeypatch):
    state = CaptureState()
    monkeypatch.setattr(inspect_cli.time, "monotonic", lambda: 10.0)
    _observe_sensor_message(state, _range_message(0.9))
    for _ in range(20):
        _observe_sensor_message(state, _distance_message(42))
    for _ in range(9):
        _observe_sensor_message(state, _range_message(0.9))

    report = _sensor_report(state, observed_at=10)["downward_rangefinder"]
    assert report == {
        "status": "ok",
        "samples": 20,
        "latest_m": 0.42,
        "source": "DISTANCE_SENSOR",
        "rate_hz": 20.0,
    }


def test_downward_range_uses_fresh_legacy_only_when_oriented_source_is_stale(
    monkeypatch,
):
    state = CaptureState()
    now = [10.0]
    monkeypatch.setattr(inspect_cli.time, "monotonic", lambda: now[0])
    _observe_sensor_message(state, _distance_message(42))
    now[0] = 13.0
    _observe_sensor_message(state, _range_message(0.8))

    report = _sensor_report(state, observed_at=13)["downward_rangefinder"]
    assert report["source"] == "RANGEFINDER"
    assert report["samples"] == 1
    assert report["latest_m"] == 0.8
    assert report["status"] == "ok"
    assert (
        _sensor_report(state, observed_at=16)["downward_rangefinder"]["status"]
        == "stale"
    )
    # Final capture summaries use the capture end, not later fsync completion.
    assert (
        _sensor_report(state, observed_at=10)["downward_rangefinder"]["source"]
        == "DISTANCE_SENSOR"
    )


@pytest.mark.parametrize(
    ("quality", "expected"), [(0, "low_quality"), (None, "unknown_quality"), (50, "ok")]
)
def test_optical_flow_status_requires_positive_quality(quality, expected):
    state = CaptureState()
    _observe_sensor_message(
        state, SimpleNamespace(get_type=lambda: "OPTICAL_FLOW", quality=quality)
    )
    report = _sensor_report(state, observed_at=time.monotonic())["optical_flow"]
    assert report["status"] == expected
    assert report["samples"] == 1


@pytest.mark.parametrize("observed_at", [90.0, 99.0, 99.5, 101.0, float("nan")])
def test_old_or_invalid_heartbeat_cannot_publish_fresh_disarmed_state(
    monkeypatch, observed_at
):
    monkeypatch.setattr(capture_workers.time, "monotonic", lambda: 100.0)
    state = CaptureState()
    assert state.observe_vehicle_state(armed=True, observed_at=99.5)
    state.vehicle_heartbeat.clear()
    assert not state.observe_vehicle_state(armed=False, observed_at=observed_at)
    assert state.last_vehicle_heartbeat_monotonic == 99.5
    assert state.last_vehicle_state == "armed"
    assert not state.saw_disarmed_after_arm
    assert not state.vehicle_heartbeat.is_set()
    assert not state.disarmed_heartbeat.is_set()


def test_queued_telemetry_keeps_original_epoch_timestamps_and_sensor_age(
    tmp_path, monkeypatch
):
    now = [90.0]
    monkeypatch.setattr(capture_workers.time, "monotonic", lambda: now[0])
    window = CaptureWindow(20.0)
    window.begin()
    now[0] = 100.0
    state = CaptureState()
    stop = threading.Event()
    assert state.observe_vehicle_state(armed=True, observed_at=100.0)
    state.vehicle_heartbeat.clear()

    def stamped(kind, received, **fields):
        message = _TelemetryMessage(kind)
        vars(message).update(
            _received_monotonic=received, _timestamp=1_760_000_000.25, **fields
        )
        return message

    messages = iter(
        [
            stamped("HEARTBEAT", 99.5, base_mode=0),
            stamped("OPTICAL_FLOW", 91.0, quality=180),
            stamped("ATTITUDE", 89.0),
        ]
    )

    def receive(**_kwargs):
        message = next(messages, None)
        if message is None:
            stop.set()
        return message

    output = tmp_path / "delayed.jsonl"
    TelemetryWorker(
        connection=SimpleNamespace(recv_match=receive),
        output=output,
        vehicle_system=1,
        vehicle_component=1,
        window=window,
        stop=stop,
        state=state,
        sync=IntervalSync(0),
        allow_armed_at_any_time=True,
        stop_after_disarm=True,
    ).run()

    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert [record["message"] for record in records] == ["HEARTBEAT", "OPTICAL_FLOW"]
    assert [record["elapsed_s"] for record in records] == [9.5, 1.0]
    assert all(
        record["timestamp_utc"] == "2025-10-09T08:53:20.250000+00:00"
        for record in records
    )
    assert state.last_vehicle_state == "armed" and not state.saw_disarmed_after_arm
    assert not state.vehicle_heartbeat.is_set()
    assert state.stop_reason is None and state.worker_error is None
    assert state.flow_observed_monotonic == 91.0
    assert _sensor_report(state, observed_at=100.0)["optical_flow"]["status"] == "stale"
    assert state.telemetry_counts["ATTITUDE"] == 0


def test_fresh_heartbeat_wait_rejects_receipt_before_the_request(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(capture_workers.time, "monotonic", lambda: now[0])
    state = CaptureState()
    observed = []

    class DelayedHeartbeats(threading.Event):
        def wait(self, timeout=None):
            stamp = 99.9 if not observed else 100.1
            now[0] += 0.1
            state.observe_vehicle_state(armed=False, observed_at=stamp)
            observed.append(stamp)
            return True

    state.disarmed_heartbeat = DelayedHeartbeats()
    _wait_for_fresh_heartbeat(threading.Event(), state, timeout=1.0)
    assert observed == [99.9, 100.1]


def test_passive_tlog_flushes_each_packet_and_syncs_on_json_schedule(
    tmp_path, monkeypatch
):
    stop = threading.Event()
    state = CaptureState()
    window = CaptureWindow(duration=None)
    window.begin()
    sync = IntervalSync(5)
    path = tmp_path / "telemetry.tlog"
    synchronized = []
    monkeypatch.setattr(capture_workers.os, "fsync", synchronized.append)
    received = 0
    with path.open("wb") as logfile:

        def receive(**_kwargs):
            nonlocal received
            if received == 2:
                stop.set()
                return None
            if received == 1:
                assert path.read_bytes() == b"packet"
                assert logfile.fileno() not in synchronized
                sync._due_at = 0
            received += 1
            logfile.write(b"packet")
            return _TelemetryMessage("ATTITUDE")

        TelemetryWorker(
            connection=SimpleNamespace(recv_match=receive, logfile=logfile),
            output=tmp_path / "telemetry.jsonl",
            vehicle_system=1,
            vehicle_component=1,
            window=window,
            stop=stop,
            state=state,
            sync=sync,
        ).run()
        assert path.read_bytes() == b"packetpacket"
        assert synchronized.count(logfile.fileno()) == 1
    assert state.worker_error is None
    assert state.telemetry_counts["ATTITUDE"] == 2


def test_passive_tlog_sync_failure_is_reported_and_stops_capture(tmp_path, monkeypatch):
    stop = threading.Event()
    state = CaptureState()
    window = CaptureWindow(duration=None)
    window.begin()
    sync = IntervalSync(5)
    sync._due_at = 0
    original_sync = capture_workers.os.fsync
    with (tmp_path / "telemetry.tlog").open("wb") as logfile:

        def fail_sync(descriptor):
            if descriptor == logfile.fileno():
                raise OSError("tlog fsync failed")
            original_sync(descriptor)

        monkeypatch.setattr(capture_workers.os, "fsync", fail_sync)
        TelemetryWorker(
            connection=SimpleNamespace(
                recv_match=lambda **_: _TelemetryMessage("ATTITUDE"),
                logfile=logfile,
            ),
            output=tmp_path / "telemetry.jsonl",
            vehicle_system=1,
            vehicle_component=1,
            window=window,
            stop=stop,
            state=state,
            sync=sync,
        ).run()
    assert stop.is_set()
    assert "tlog fsync failed" in (state.worker_error or "")
