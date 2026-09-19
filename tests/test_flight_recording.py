from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from ai_drone.flight import recording
from ai_drone.flight.recording import FlightRecorder
from ai_drone.mavlink.shared import SharedMavlink
from ai_drone.settings import RecordingSettings
from ai_drone.storage import MIB


class Connection:
    def __init__(self) -> None:
        self.mav = MagicMock()
        self.target_system = self.target_component = 1
        self.logfile = None
        self.messages = queue.Queue()
        self.closed = False
        self.received = 0

    def setup_logfile(self, path: str) -> None:
        self.logfile = Path(path).open("wb")  # noqa: SIM115 - connection owns it

    def recv_match(self, *, blocking=False, timeout=0):
        try:
            message = self.messages.get(timeout=timeout if blocking else 0)
        except queue.Empty:
            return None
        if isinstance(message, BaseException):
            raise message
        self.received += 1
        return message

    def close(self, *, timeout=2):
        assert timeout <= 2
        self.closed = True
        if self.logfile is not None:
            self.logfile.close()
            self.logfile = None


def test_flight_recorder_writes_complete_bundle(tmp_path) -> None:
    connection = Connection()
    record = FlightRecorder(connection, {"command": "hover"}, tmp_path / "flight")
    assert connection.logfile is not None
    connection.logfile.write(b"telemetry")
    record.event("hover_started", target_alt_m=0.4)
    record.finish()
    record.close()

    manifest = json.loads((record.root / "manifest.json").read_text())
    events = [
        json.loads(line)
        for line in (record.root / "events.jsonl").read_text().splitlines()
    ]
    assert manifest["completed"] is True
    assert manifest["metadata"] == {"command": "hover"}
    assert "OPTICAL_FLOW" in manifest["requested_messages"]
    assert [event["event"] for event in events] == [
        "recording_started",
        "hover_started",
        "completed",
    ]
    assert (record.root / "telemetry.tlog").read_bytes() == b"telemetry"
    assert connection.logfile is None
    assert connection.closed


def manifest(record):
    return json.loads((record.root / "manifest.json").read_text())


def test_startup_must_create_telemetry_log_before_allowing_flight(
    tmp_path, monkeypatch
):
    endpoint = Connection()

    def full(_path):
        raise OSError("disk full before arming")

    monkeypatch.setattr(endpoint, "setup_logfile", full)
    with pytest.raises(OSError, match="before arming"):
        FlightRecorder(endpoint, {}, tmp_path / "failed")
    assert endpoint.closed
    failed = json.loads((tmp_path / "failed" / "manifest.json").read_text())
    assert not failed["completed"]
    assert "disk full" in failed["recording_error"]


def test_startup_refuses_to_consume_the_final_storage_floor(tmp_path, monkeypatch):
    endpoint = Connection()
    monkeypatch.setattr(
        recording.shutil, "disk_usage", lambda _: SimpleNamespace(free=16 * MIB)
    )
    with pytest.raises(RuntimeError, match="storage floor"):
        FlightRecorder(endpoint, {}, tmp_path / "floor")
    assert endpoint.closed


def test_incomplete_manifest_exists_before_recording_finishes(tmp_path):
    record = FlightRecorder(Connection(), {}, tmp_path / "record")
    try:
        initial = manifest(record)
        assert initial["completed"] is False
        assert initial["ended_utc"] is None
    finally:
        record.finish()
        record.close()
    assert manifest(record)["completed"] is True


def test_background_reader_drains_its_own_subscription(tmp_path):
    endpoint = Connection()
    record = FlightRecorder(endpoint, {}, tmp_path / "record")
    try:
        for _ in range(100):
            endpoint.messages.put(object())
        deadline = time.monotonic() + 1
        while endpoint.received < 100 and time.monotonic() < deadline:
            threading.Event().wait(0.005)
        assert endpoint.received == 100
    finally:
        record.finish()
        record.close()
    assert manifest(record)["completed"] is True


@pytest.mark.parametrize(
    "failure", [OSError("disk full"), RuntimeError("MAVLink tlog queue overflow")]
)
def test_background_tlog_failure_disables_only_recording_and_preserves_control(
    tmp_path, failure
):
    controller = Connection()
    endpoint = Connection()
    record = FlightRecorder(endpoint, {}, tmp_path / "record")
    endpoint.messages.put(failure)
    assert record._closed.wait(1)
    previous = record.recording_error
    record.event("after_failure")
    record.finish()
    record.close()
    assert record.recording_error == previous
    assert not record.completed
    assert endpoint.closed
    assert not controller.closed
    controller.mav.heartbeat_send()
    controller.mav.heartbeat_send.assert_called_once()
    assert not manifest(record)["completed"]
    assert str(failure) in manifest(record)["recording_error"]


def test_event_write_failure_never_escapes_into_flight_control(
    tmp_path, monkeypatch, caplog
):
    endpoint = Connection()
    record = FlightRecorder(endpoint, {}, tmp_path / "record")
    original = recording.write_json_line

    def fail(handle, value):
        if value["event"] == "fail":
            raise OSError("event disk full")
        return original(handle, value)

    monkeypatch.setattr(recording, "write_json_line", fail)
    record.event("fail")
    assert record._closed.wait(1)
    record.finish()
    record.close()
    assert "event disk full" in (record.recording_error or "")
    assert "flight control continues" in caplog.text
    assert manifest(record)["recording_error"] == record.recording_error
    assert not manifest(record)["completed"]


def test_storage_floor_finalizes_recording_early_without_stopping_controller(
    tmp_path, monkeypatch
):
    free = 100 * MIB
    monkeypatch.setattr(
        recording.shutil, "disk_usage", lambda _: SimpleNamespace(free=free)
    )
    controller = Connection()
    endpoint = Connection()
    record = FlightRecorder(
        endpoint,
        {},
        tmp_path / "record",
        settings=RecordingSettings(storage_stop_mib=20, storage_check_interval=0.01),
    )
    free = 20 * MIB
    assert record._closed.wait(1)
    record.finish()
    record.close()
    assert endpoint.closed and not controller.closed
    assert "storage floor" in (record.recording_error or "")
    assert not manifest(record)["completed"]
    assert manifest(record)["ended_utc"] is not None


def test_controller_event_path_does_not_wait_for_disk_or_a_full_queue(
    tmp_path, monkeypatch
):
    endpoint = Connection()
    entered, release = threading.Event(), threading.Event()

    def blocked_reader(**_kwargs):
        entered.set()
        release.wait(2)

    monkeypatch.setattr(endpoint, "recv_match", blocked_reader)
    record = FlightRecorder(endpoint, {}, tmp_path / "record")
    try:
        assert entered.wait(1)
        before = time.monotonic()
        for index in range(257):
            record.event("event", index=index)
        assert time.monotonic() - before < 0.2
        assert "queue overflow" in (record.recording_error or "")
    finally:
        release.set()
        record.close()
    assert not manifest(record)["completed"]


def test_final_log_flush_failure_cannot_abort_control_cleanup(tmp_path, monkeypatch):
    endpoint = Connection()
    original = endpoint.close

    def failing_close(*, timeout):
        original(timeout=timeout)
        raise RuntimeError("tlog flush failed")

    monkeypatch.setattr(endpoint, "close", failing_close)
    record = FlightRecorder(endpoint, {}, tmp_path / "record")
    record.finish()
    record.close()
    record.close()
    assert "tlog flush failed" in (record.recording_error or "")
    assert not manifest(record)["completed"]


def test_close_returns_with_an_incomplete_record_if_its_worker_does_not_stop(
    tmp_path, monkeypatch
):
    endpoint = Connection()
    entered, release = threading.Event(), threading.Event()

    def stuck_reader(**_kwargs):
        entered.set()
        release.wait(5)

    monkeypatch.setattr(endpoint, "recv_match", stuck_reader)
    record = FlightRecorder(endpoint, {}, tmp_path / "record")
    try:
        assert entered.wait(1)
        before = time.monotonic()
        record.close()
        assert time.monotonic() - before < 3.5
        assert "cleanup did not finish" in (record.recording_error or "")
        assert not record.completed
    finally:
        release.set()
        assert record._closed.wait(1)
        record.close()
    assert endpoint.closed
    assert not manifest(record)["completed"]


def test_manifest_write_failure_preserves_initial_incomplete_bundle(
    tmp_path, monkeypatch
):
    record = FlightRecorder(Connection(), {}, tmp_path / "record")

    def fail(*_args, **_kwargs):
        raise OSError("manifest disk full")

    monkeypatch.setattr(recording, "atomic_write_text", fail)
    record.finish()
    record.close()
    assert "manifest disk full" in (record.recording_error or "")
    assert not record.completed
    assert manifest(record)["completed"] is False


def test_flight_failure_is_not_erased_by_later_finish_or_recording_cleanup(tmp_path):
    record = FlightRecorder(Connection(), {}, tmp_path / "record")
    record.finish(RuntimeError("flight aborted"))
    record.finish()
    record.close()
    assert not record.completed
    assert "flight aborted" in manifest(record)["error"]


def test_real_shared_tlog_disk_failure_leaves_controller_heartbeat_alive(tmp_path):
    raw = Connection()
    with SharedMavlink(raw) as hub:
        control = hub.subscribe("controller")
        record_endpoint = hub.subscribe("recording")
        record = FlightRecorder(record_endpoint, {}, tmp_path / "record")
        assert record_endpoint.logfile is not None
        original = record_endpoint.logfile._handle

        class FullDisk:
            def __getattr__(self, name):
                return getattr(original, name)

            def write(self, _packet):
                raise OSError("injected tlog disk full")

        sink: Any = record_endpoint.logfile
        sink._handle = FullDisk()

        def heartbeat():
            message = mavlink.MAVLink_heartbeat_message(
                type=mavlink.MAV_TYPE_QUADROTOR,
                autopilot=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode=0,
                custom_mode=0,
                system_status=mavlink.MAV_STATE_STANDBY,
                mavlink_version=3,
            )
            message.pack(mavlink.MAVLink(None, srcSystem=1, srcComponent=1))
            return message

        try:
            raw.messages.put(heartbeat())
            assert (
                control.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                is not None
            )
            assert record._closed.wait(1)
            assert "injected tlog disk full" in (record.recording_error or "")
            assert not raw.closed
            raw.messages.put(heartbeat())
            assert (
                control.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                is not None
            )
            control.mav.heartbeat_send(6, 8, 0, 0, 0)
            raw.mav.heartbeat_send.assert_called_once_with(6, 8, 0, 0, 0)
        finally:
            record.close()
            control.close()
