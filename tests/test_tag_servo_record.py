"""Safety and behavior tests for armed-flight AprilTag servo recording."""

from __future__ import annotations

import json
import queue
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import ClassVar

import numpy as np
import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

import ai_drone.cli.record as record_cli
from ai_drone.cli.record import (
    AnalysisFrame,
    CaptureState,
    CaptureWindow,
    TelemetryWorker,
    _parser,
    _queue_analysis_frame,
    _validate_args,
)
from ai_drone.cli.servo import ServoProcessLock
from ai_drone.cli.tag_servo_record import (
    ARMED_FLIGHT_CONFIRMATION,
    TagServoConfig,
    TagServoSession,
)
from ai_drone.durability import IntervalSync
from ai_drone.mavlink.parameters import request_parameter
from ai_drone.recording import request_telemetry_messages
from ai_drone.vision.apriltags import TagDetection


def _arguments(*extra: str) -> list[str]:
    return [
        "--all-tags",
        "--active-us",
        "1800",
        "--rest-us",
        "1200",
        "--pulse-duration",
        "0.01",
        "--settle-duration",
        "0",
        "--confirm-actuation",
        "SERVO_CLEAR",
        "--confirm-armed-flight",
        ARMED_FLIGHT_CONFIRMATION,
        *extra,
    ]


def _parse(*extra: str):
    parser = _parser(operation="tag-servo")
    args = parser.parse_args(_arguments(*extra))
    _validate_args(parser, args, operation="tag-servo")
    return args


def test_active_parser_accepts_optional_count_or_unbounded_runtime() -> None:
    unbounded = _parse()
    bounded = _parse("--stop-after", "3", "--duration", "120")

    assert unbounded.stop_after is None
    assert unbounded.duration is None
    assert bounded.stop_after == 3
    assert bounded.duration == 120.0
    assert bounded.backend == "native"


@pytest.mark.parametrize(
    "arguments",
    [
        [],
        ["--all-tags"],
        [
            "--all-tags",
            "--active-us",
            "1800",
            "--rest-us",
            "1200",
            "--pulse-duration",
            "0.1",
            "--confirm-actuation",
            "servo_clear",
            "--confirm-armed-flight",
            ARMED_FLIGHT_CONFIRMATION,
        ],
    ],
)
def test_active_parser_requires_selection_positions_and_exact_confirmations(
    arguments: list[str],
) -> None:
    parser = _parser(operation="tag-servo")

    with pytest.raises(SystemExit):
        args = parser.parse_args(arguments)
        _validate_args(parser, args, operation="tag-servo")


@pytest.mark.parametrize(
    "extra",
    [
        ("--stop-after", "0"),
        ("--stop-after", "588"),
        ("--confirmation-frames", "1"),
        ("--min-decision-margin", "nan"),
        ("--max-detection-age", "1.1"),
        ("--max-heartbeat-age", "1"),
        ("--active-us", "1200", "--rest-us", "1200"),
        ("--active-us", "2200"),
        ("--pulse-duration", "2.1"),
        ("--settle-duration", "-0.1"),
    ],
)
def test_active_parser_rejects_unsafe_numeric_settings(extra: tuple[str, ...]) -> None:
    parser = _parser(operation="tag-servo")
    arguments = _arguments(*extra)

    with pytest.raises(SystemExit):
        args = parser.parse_args(arguments)
        _validate_args(parser, args, operation="tag-servo")


def test_active_parser_requires_unique_valid_tag_allowlist() -> None:
    parser = _parser(operation="tag-servo")
    base = [
        "--active-us",
        "1800",
        "--rest-us",
        "1200",
        "--pulse-duration",
        "0.1",
        "--confirm-actuation",
        "SERVO_CLEAR",
        "--confirm-armed-flight",
        ARMED_FLIGHT_CONFIRMATION,
    ]

    for identifiers in (("1", "1"), ("587",)):
        arguments = [value for tag_id in identifiers for value in ("--tag-id", tag_id)]
        with pytest.raises(SystemExit):
            args = parser.parse_args([*arguments, *base])
            _validate_args(parser, args, operation="tag-servo")

    with pytest.raises(SystemExit):
        args = parser.parse_args(
            ["--tag-id", "1", "--tag-id", "2", "--stop-after", "3", *base]
        )
        _validate_args(parser, args, operation="tag-servo")


def test_payload_servo_process_lock_is_exclusive_and_reusable(tmp_path: Path) -> None:
    path = tmp_path / "servo.lock"
    first = ServoProcessLock(path)
    try:
        with pytest.raises(RuntimeError, match="already owned"):
            ServoProcessLock(path)
    finally:
        first.close()

    second = ServoProcessLock(path)
    second.close()


class _FakeLock:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _FakeServo:
    instances: ClassVar[list[_FakeServo]] = []

    def __init__(
        self,
        pin: int,
        *,
        min_pulse_width: float,
        max_pulse_width: float,
        initial_value: None,
    ) -> None:
        self.pin = pin
        self.min_pulse_width = min_pulse_width
        self.max_pulse_width = max_pulse_width
        self.initial_value = initial_value
        self.actions: list[tuple[str, float | None]] = []
        self._value: float | None = initial_value
        self.closed = False
        self.__class__.instances.append(self)

    @property
    def value(self) -> float | None:
        return self._value

    @value.setter
    def value(self, value: float | None) -> None:
        self._value = value
        self.actions.append(("value", value))

    def detach(self) -> None:
        self._value = None
        self.actions.append(("detach", None))

    def close(self) -> None:
        self.closed = True
        self.actions.append(("close", None))


def _config(**changes: object) -> TagServoConfig:
    config = TagServoConfig(
        allowed_tag_ids=None,
        stop_after=None,
        confirmation_frames=3,
        minimum_decision_margin=30.0,
        maximum_detection_age_s=1.0,
        maximum_heartbeat_age_s=5.0,
        minimum_pulse_us=900,
        maximum_pulse_us=2100,
        active_pulse_us=1800,
        rest_pulse_us=1200,
        pulse_duration_s=0.01,
        settle_duration_s=0.0,
    )
    return replace(config, **changes)


def _detection(
    tag_id: int,
    *,
    hamming: int | None = 0,
    margin: float | None = 60.0,
) -> TagDetection:
    return TagDetection(
        tag_id=tag_id,
        corners=np.zeros((4, 2), dtype=np.float64),
        center=(0.0, 0.0),
        hamming=hamming,
        decision_margin=margin,
    )


def _frame(index: int, *, captured: float | None = None) -> AnalysisFrame:
    return AnalysisFrame(
        frame_index=index,
        elapsed_s=index / 30.0,
        grayscale=np.zeros((4, 4), dtype=np.uint8),
        metadata={},
        captured_monotonic=time.monotonic() if captured is None else captured,
    )


def _records(*detections: TagDetection) -> list[dict[str, object]]:
    return [{"id": detection.tag_id} for detection in detections]


def _wait_for(predicate, *, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert predicate()


def _session(
    tmp_path: Path,
    *,
    config: TagServoConfig | None = None,
) -> tuple[TagServoSession, CaptureState, threading.Event, _FakeLock, _FakeServo]:
    _FakeServo.instances.clear()
    state = CaptureState()
    state.last_vehicle_heartbeat_monotonic = time.monotonic()
    stop = threading.Event()
    ready = threading.Event()
    ready.set()
    fake_lock = _FakeLock()
    session = TagServoSession(
        config=config or _config(),
        event_path=tmp_path / "servo.jsonl",
        state=state,
        capture_stop=stop,
        ready=ready,
        servo_factory=_FakeServo,
        process_lock_factory=lambda: fake_lock,
    )
    return session, state, stop, fake_lock, _FakeServo.instances[0]


def _observe(session: TagServoSession, tag_id: int, count: int = 1) -> None:
    for index in range(count):
        detection = _detection(tag_id)
        session.observe(_frame(index), [detection], _records(detection))


def test_three_consecutive_frames_command_one_bounded_pulse(tmp_path: Path) -> None:
    session, state, stop, fake_lock, servo = _session(
        tmp_path, config=_config(stop_after=1)
    )
    try:
        _observe(session, 7, 2)
        assert servo.actions == []

        _observe(session, 7)
        _wait_for(lambda: state.servo_pulses_completed == 1)

        assert stop.is_set()
        assert state.stop_reason == "tag_limit_reached"
        assert state.completed_servo_tag_ids == (7,)
        assert servo.actions[:3] == [
            ("value", 0.5),
            ("value", -0.5),
            ("detach", None),
        ]
    finally:
        session.close()

    assert fake_lock.closed
    events = [
        json.loads(line) for line in (tmp_path / "servo.jsonl").read_text().splitlines()
    ]
    assert "servo_pulse_completed_commanded" in {event["event"] for event in events}
    assert all(event["feedback_available"] is False for event in events)


def test_one_frame_low_quality_and_disallowed_tags_never_move_servo(
    tmp_path: Path,
) -> None:
    session, state, _stop, _lock, servo = _session(
        tmp_path,
        config=_config(allowed_tag_ids=frozenset({4})),
    )
    try:
        good = _detection(4)
        session.observe(_frame(0), [good], _records(good))
        bad_hamming = _detection(4, hamming=1)
        session.observe(_frame(1), [bad_hamming], _records(bad_hamming))
        low_margin = _detection(4, margin=5.0)
        session.observe(_frame(2), [low_margin], _records(low_margin))
        wrong_id = _detection(5)
        session.observe(_frame(3), [wrong_id], _records(wrong_id))
        time.sleep(0.02)

        assert state.servo_pulses_completed == 0
        assert servo.actions == []
    finally:
        session.close()


def test_pre_ready_detections_do_not_count_toward_confirmation(
    tmp_path: Path,
) -> None:
    _FakeServo.instances.clear()
    state = CaptureState()
    state.last_vehicle_heartbeat_monotonic = time.monotonic()
    stop = threading.Event()
    ready = threading.Event()
    session = TagServoSession(
        config=_config(),
        event_path=tmp_path / "servo.jsonl",
        state=state,
        capture_stop=stop,
        ready=ready,
        servo_factory=_FakeServo,
        process_lock_factory=_FakeLock,
    )
    servo = _FakeServo.instances[0]
    try:
        _observe(session, 4, 10)
        ready.set()
        _observe(session, 4, 2)
        time.sleep(0.02)
        assert servo.actions == []

        _observe(session, 4)
        _wait_for(lambda: state.servo_pulses_completed == 1)
    finally:
        session.close()


def test_completed_tag_id_is_latched_for_the_entire_run(tmp_path: Path) -> None:
    session, state, _stop, _lock, servo = _session(tmp_path)
    try:
        _observe(session, 11, 3)
        _wait_for(lambda: state.servo_pulses_completed == 1)
        first_action_count = len(servo.actions)

        session.observe(_frame(20), [], [])
        _observe(session, 11, 20)
        time.sleep(0.03)

        assert state.completed_servo_tag_ids == (11,)
        assert state.servo_pulses_completed == 1
        assert len(servo.actions) == first_action_count
    finally:
        session.close()


def test_stop_after_three_counts_completed_distinct_ids(tmp_path: Path) -> None:
    session, state, stop, _lock, _servo = _session(
        tmp_path,
        config=_config(stop_after=3),
    )
    try:
        for tag_id in (3, 8, 13):
            _observe(session, tag_id, 3)
            _wait_for(lambda tag_id=tag_id: tag_id in state.completed_servo_tag_ids)

        assert stop.is_set()
        assert state.stop_reason == "tag_limit_reached"
        assert state.completed_servo_tag_ids == (3, 8, 13)
        assert state.servo_pulses_completed == 3
    finally:
        session.close()


def test_stale_frame_cannot_trigger_and_latest_queue_replaces_old_frame(
    tmp_path: Path,
) -> None:
    session, state, _stop, _lock, servo = _session(tmp_path)
    try:
        stale = time.monotonic() - 2.0
        detection = _detection(2)
        for index in range(5):
            session.observe(
                _frame(index, captured=stale),
                [detection],
                _records(detection),
            )
        time.sleep(0.02)

        assert state.servo_pulses_completed == 0
        assert servo.actions == []
    finally:
        session.close()

    frames: queue.Queue[AnalysisFrame | None] = queue.Queue(maxsize=1)
    old = _frame(1)
    new = _frame(2)
    frames.put_nowait(old)
    capture_state = CaptureState()
    _queue_analysis_frame(frames, new, capture_state, latest_wins=True)

    assert frames.get_nowait() is new
    assert capture_state.dropped_analysis_frames == 1


def test_stale_selected_heartbeat_rejects_actuation_and_stops(tmp_path: Path) -> None:
    session, state, stop, _lock, servo = _session(tmp_path)
    try:
        state.last_vehicle_heartbeat_monotonic = time.monotonic() - 10.0
        _observe(session, 6, 3)
        _wait_for(stop.is_set)

        assert state.servo_pulses_completed == 0
        assert state.stop_reason == "heartbeat_stale"
        assert state.worker_error is not None
        assert servo.actions == []
    finally:
        session.close()


def test_stop_during_active_pulse_returns_to_rest_without_counting(
    tmp_path: Path,
) -> None:
    session, state, stop, _lock, servo = _session(
        tmp_path,
        config=_config(pulse_duration_s=0.5),
    )
    try:
        _observe(session, 9, 3)
        _wait_for(lambda: ("value", 0.5) in servo.actions)
        stop.set()
        session.stop_accepting()
        _wait_for(lambda: ("detach", None) in servo.actions)

        assert state.servo_pulses_completed == 0
        assert servo.actions[:3] == [
            ("value", 0.5),
            ("value", -0.5),
            ("detach", None),
        ]
    finally:
        session.close()


def test_session_initialization_failure_releases_gpio_and_process_lock(
    tmp_path: Path,
) -> None:
    event_path = tmp_path / "servo.jsonl"
    event_path.write_text("already exists")
    state = CaptureState()
    fake_lock = _FakeLock()
    _FakeServo.instances.clear()

    with pytest.raises(FileExistsError):
        TagServoSession(
            config=_config(),
            event_path=event_path,
            state=state,
            capture_stop=threading.Event(),
            ready=threading.Event(),
            servo_factory=_FakeServo,
            process_lock_factory=lambda: fake_lock,
        )

    assert fake_lock.closed
    assert _FakeServo.instances[0].actions == [("detach", None), ("close", None)]


class _InitialHeartbeat:
    autopilot = mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
    type = mavlink.MAV_TYPE_QUADROTOR
    base_mode = mavlink.MAV_MODE_FLAG_SAFETY_ARMED

    def get_type(self) -> str:
        return "HEARTBEAT"

    def get_srcSystem(self) -> int:
        return 1

    def get_srcComponent(self) -> int:
        return 1


class _ArmedConnection:
    target_system = 0
    target_component = 0
    logfile = None

    def __init__(self) -> None:
        self.closed = False

    def wait_heartbeat(self, *, timeout: float):
        return _InitialHeartbeat()

    def recv_match(self, **_kwargs):
        return None

    def close(self) -> None:
        self.closed = True


def test_active_command_accepts_matching_initial_armed_heartbeat(
    tmp_path: Path, monkeypatch, capsys
) -> None:
    output = tmp_path / "armed-no-pi"
    connection = _ArmedConnection()
    monkeypatch.setattr(record_cli, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        record_cli, "resolve_mavlink_endpoint", lambda *_args, **_kwargs: "tcp:sim"
    )
    monkeypatch.setattr(
        record_cli.mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )
    monkeypatch.setattr(record_cli, "request_parameter", lambda *_args, **_kwargs: 0.0)
    monkeypatch.setattr(record_cli, "request_telemetry_messages", lambda *_args: [])

    result = record_cli.run(
        ["--output-dir", str(output), *_arguments()],
        operation="tag-servo",
    )

    assert result == 1
    assert "READY:" not in capsys.readouterr().out
    assert connection.closed
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["armed_abort"] is False
    assert manifest["safety"]["initial_vehicle_state"] == "armed"
    assert manifest["safety"]["saw_armed"] is True
    assert manifest["safety"]["arming_skipchk"] == 0.0
    assert manifest["stop_reason"] == "startup_failed"


def test_active_command_refuses_skipped_arming_checks_before_stream_requests(
    tmp_path: Path, monkeypatch
) -> None:
    output = tmp_path / "unsafe-arming-checks"
    connection = _ArmedConnection()
    requested = []
    monkeypatch.setattr(record_cli, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        record_cli, "resolve_mavlink_endpoint", lambda *_args, **_kwargs: "tcp:sim"
    )
    monkeypatch.setattr(
        record_cli.mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )
    monkeypatch.setattr(record_cli, "request_parameter", lambda *_args, **_kwargs: 4.0)
    monkeypatch.setattr(
        record_cli,
        "request_telemetry_messages",
        lambda *_args: requested.append("called"),
    )

    assert (
        record_cli.run(
            ["--output-dir", str(output), *_arguments()],
            operation="tag-servo",
        )
        == 1
    )

    assert requested == []
    assert connection.closed
    manifest = json.loads((output / "manifest.json").read_text())
    assert "ARMING_SKIPCHK=4" in manifest["components"]["flight_controller"]["detail"]


class _HeartbeatMessage:
    def __init__(self, *, armed: bool) -> None:
        self.base_mode = mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0

    def get_type(self) -> str:
        return "HEARTBEAT"

    def get_srcSystem(self) -> int:
        return 1

    def get_srcComponent(self) -> int:
        return 1

    def to_dict(self) -> dict[str, object]:
        return {"mavpackettype": "HEARTBEAT", "base_mode": self.base_mode}


class _HeartbeatQueueConnection:
    def __init__(self) -> None:
        self.messages: queue.Queue[_HeartbeatMessage] = queue.Queue()

    def recv_match(self, *, blocking: bool, timeout: float):
        assert blocking
        try:
            return self.messages.get(timeout=timeout)
        except queue.Empty:
            return None


def test_active_telemetry_policy_accepts_armed_and_stops_after_disarm(
    tmp_path: Path,
) -> None:
    connection = _HeartbeatQueueConnection()
    stop = threading.Event()
    state = CaptureState()
    state.observe_vehicle_state(armed=True)
    window = CaptureWindow(duration=None)
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
        stop_after_disarm=True,
    )
    worker.start()
    connection.messages.put(_HeartbeatMessage(armed=True))
    time.sleep(0.01)
    assert not stop.is_set()
    assert not state.armed_abort

    connection.messages.put(_HeartbeatMessage(armed=False))
    _wait_for(stop.is_set)
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert state.stop_reason == "vehicle_disarmed"
    assert state.saw_disarmed_after_arm
    records = [
        json.loads(line)
        for line in (tmp_path / "telemetry.jsonl").read_text().splitlines()
    ]
    assert [record["message"] for record in records] == ["HEARTBEAT", "HEARTBEAT"]


def test_active_outbound_mavlink_is_read_and_interval_requests_only() -> None:
    calls: list[tuple[str, tuple[object, ...]]] = []

    class ParamMessage:
        param_id = "ARMING_SKIPCHK"
        param_value = 0.0

        def get_type(self) -> str:
            return "PARAM_VALUE"

        def get_srcSystem(self) -> int:
            return 1

        def get_srcComponent(self) -> int:
            return 1

    class Mav:
        def param_request_read_send(self, *values: object) -> None:
            calls.append(("PARAM_REQUEST_READ", values))

        def command_long_send(self, *values: object) -> None:
            calls.append(("COMMAND_LONG", values))

    class Connection:
        target_system = 1
        target_component = 1
        mav = Mav()

        def recv_match(self, **_kwargs):
            return ParamMessage()

    connection = Connection()

    assert (
        request_parameter(
            connection,
            "ARMING_SKIPCHK",
            require_disarmed=False,
        )
        == 0.0
    )
    requested = request_telemetry_messages(connection)

    assert len(requested) == 23
    assert [name for name, _values in calls].count("PARAM_REQUEST_READ") == 1
    command_calls = [values for name, values in calls if name == "COMMAND_LONG"]
    assert len(command_calls) == 23
    assert all(
        values[2] == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL for values in command_calls
    )
    assert all(
        values[2] != mavlink.MAV_CMD_COMPONENT_ARM_DISARM for values in command_calls
    )
    assert all(values[3] == 0 for values in command_calls)
    assert all(values[6:] == (0, 0, 0, 0, 0) for values in command_calls)
