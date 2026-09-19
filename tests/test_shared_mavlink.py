"""Offline concurrency and loss accounting for the sole MAVLink reader."""

from __future__ import annotations

import queue
import struct
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

import ai_drone.mavlink.shared as shared
from ai_drone.mavlink.shared import (
    SharedMavlink,
    SharedMavlinkError,
    received_monotonic,
)


def message(kind="HEARTBEAT", *, system=1, component=1, armed=False, mode=5):
    return SimpleNamespace(
        get_type=lambda: kind,
        get_srcSystem=lambda: system,
        get_srcComponent=lambda: component,
        get_msgbuf=lambda: kind.encode(),
        autopilot=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        type=mavlink.MAV_TYPE_QUADROTOR,
        base_mode=mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        | (mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0),
        custom_mode=mode,
    )


class Wire:
    def __init__(self):
        self.incoming = queue.Queue()
        self.readers = set()
        self.writes = []
        self.closes = 0
        self.mav = SimpleNamespace(
            command_long_send=lambda *args: self.writes.append(("command", args)),
            param_request_read_send=lambda *args: self.writes.append(
                ("parameter", args)
            ),
            heartbeat_send=lambda *args: self.writes.append(("heartbeat", args)),
            set_callback=lambda *_args: pytest.fail("replaced decoder callback"),
            total_bytes_received=123,
        )

    def recv_match(self, *, blocking, timeout):
        assert blocking
        self.readers.add(threading.get_ident())
        try:
            item = self.incoming.get(timeout=timeout)
        except queue.Empty:
            return None
        if isinstance(item, BaseException):
            raise item
        return item

    def close(self):
        self.closes += 1
        self.incoming.put(None)


@pytest.fixture
def link():
    wire = Wire()
    hub = SharedMavlink(wire)
    yield wire, hub
    hub.close()


def test_independent_filters_preserve_order_and_original_reception_time(link):
    wire, hub = link
    control = hub.subscribe("control")
    recorder = hub.subscribe("recording")
    samples = [message(), message("ATTITUDE"), message("PARAM_VALUE")]
    before = time.monotonic()
    for sample in samples:
        wire.incoming.put(sample)
    assert (
        control.recv_match(type=["PARAM_VALUE"], blocking=True, timeout=1) is samples[2]
    )
    after = time.monotonic()
    assert control.recv_match() is None
    assert [recorder.recv_match(blocking=True, timeout=1) for _ in samples] == samples
    assert before <= received_monotonic(samples[0]) <= after
    assert len(wire.readers) == 1
    assert control.flightmode == "LOITER"


def test_endpoint_mode_tracks_its_heartbeat_not_a_faster_consumer(link):
    wire, hub = link
    slow = hub.subscribe("slow")
    fast = hub.subscribe("fast")
    first, last = message(mode=5), message(mode=9)
    wire.incoming.put(first)
    wire.incoming.put(last)
    assert fast.wait_heartbeat(timeout=1) is first
    assert fast.wait_heartbeat(timeout=1) is last
    assert fast.flightmode == "LAND"
    assert slow.wait_heartbeat(timeout=1) is first
    assert slow.flightmode == "LOITER"
    assert slow.mode_mapping()["LAND"] == 9


def test_status_requires_selected_fresh_ardupilot_source(link):
    wire, hub = link
    observer = hub.subscribe("observer")
    for sample in (message(system=42), message(component=42)):
        wire.incoming.put(sample)
        observer.recv_match(blocking=True, timeout=1)
        assert hub.status()["status"] == "unknown"
    sample = message(armed=True, mode=9)
    sample._received_monotonic = time.monotonic() - 10
    wire.incoming.put(sample)
    assert observer.recv_match(blocking=True, timeout=1) is sample
    status = hub.status()
    assert status["source_known"] and status["armed"]
    assert status["mode"] == "LAND" and not status["fresh"]
    assert status["heartbeat_monotonic"] == sample._received_monotonic
    assert status["status"] == "unknown"
    wire.incoming.put(message())
    observer.wait_heartbeat(timeout=1)
    assert hub.status()["status"] == "disarmed"


def test_late_heartbeat_cannot_revert_newer_armed_status(link):
    wire, hub = link
    endpoint = hub.subscribe("observer")
    fresh = message(armed=True)
    stale = message(armed=False)
    stale._received_monotonic = time.monotonic() - 1
    for sample in (fresh, stale):
        wire.incoming.put(sample)
        endpoint.wait_heartbeat(timeout=1)
    assert hub.status()["status"] == "armed"
    assert hub.status()["heartbeat_monotonic"] == received_monotonic(fresh)


def test_filter_timeout_conditions_and_fixed_targets(link):
    wire, hub = link
    endpoint = hub.subscribe("control")
    wire.incoming.put(message("ATTITUDE"))
    assert endpoint.recv_match(type="HEARTBEAT", blocking=True, timeout=0.02) is None
    assert endpoint.recv_match(blocking=True, timeout=0) is None
    with pytest.raises(ValueError, match="conditions"):
        endpoint.recv_match(condition="HEARTBEAT.base_mode == 0")
    with pytest.raises(ValueError, match="fixed"):
        endpoint.target_system = 42
    endpoint.target_system = 1
    endpoint.target_component = 1
    with pytest.raises(RuntimeError, match="cannot reset"):
        endpoint.reset()
    with pytest.raises(AttributeError, match="parser method"):
        endpoint.mav.set_callback(None)
    assert endpoint.mav.total_bytes_received == 123


def test_overflow_fails_slow_consumer_without_blocking_other_readers_or_land(link):
    wire, hub = link
    slow = hub.subscribe("slow", capacity=1)
    fast = hub.subscribe("fast", capacity=8)
    for _ in range(3):
        wire.incoming.put(message())
    for _ in range(3):
        assert fast.wait_heartbeat(timeout=1) is not None
    with pytest.raises(SharedMavlinkError, match="queue overflow"):
        slow.recv_match()
    slow.mav.command_long_send(1, 1, mavlink.MAV_CMD_NAV_LAND)
    assert wire.writes[-1][1][-1] == mavlink.MAV_CMD_NAV_LAND
    assert hub.status()["fresh"]


def test_reader_error_propagates_and_preserves_best_effort_write(link):
    wire, hub = link
    first, second = hub.subscribe("first"), hub.subscribe("second")
    wire.incoming.put(OSError("serial disconnected"))
    for endpoint in (first, second):
        with pytest.raises(SharedMavlinkError, match="serial disconnected"):
            endpoint.recv_match(blocking=True, timeout=1)
    first.arducopter_disarm()
    assert wire.writes[-1][1][4] == 0
    assert not hub.status()["fresh"]
    with pytest.raises(SharedMavlinkError):
        hub.subscribe("late")


def test_all_send_paths_are_serialized(link):
    wire, hub = link
    endpoints = [hub.subscribe(str(index)) for index in range(8)]
    guard = threading.Lock()
    sending = 0
    peak = 0

    def send(*_args):
        nonlocal sending, peak
        with guard:
            sending += 1
            peak = max(peak, sending)
        threading.Event().wait(0.002)
        with guard:
            sending -= 1

    wire.mav.command_long_send = send
    workers = [
        threading.Thread(target=endpoint.arducopter_arm) for endpoint in endpoints
    ]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join(timeout=1)
        assert not worker.is_alive()
    assert peak == 1
    endpoints[0].param_fetch_one("ARMING_SKIPCHK")
    endpoints[1].param_fetch_one(7)
    assert wire.writes[-2:] == [
        ("parameter", (1, 1, b"ARMING_SKIPCHK", -1)),
        ("parameter", (1, 1, b"", 7)),
    ]


def test_endpoint_close_unsubscribes_without_closing_reused_name(link):
    wire, hub = link
    old = hub.subscribe("control")
    old.close()
    current = hub.subscribe("control")
    old.close()
    wire.incoming.put(message())
    assert current.wait_heartbeat(timeout=1) is not None
    assert wire.closes == 0
    with pytest.raises(SharedMavlinkError, match="closed"):
        old.mav.heartbeat_send()


def test_tlogs_are_independent_complete_and_exclude_bad_data(link, tmp_path):
    wire, hub = link
    first, second = hub.subscribe("first"), hub.subscribe("second")
    first_path, second_path = tmp_path / "first.tlog", tmp_path / "second.tlog"
    first.setup_logfile(first_path)
    second.setup_logfile(second_path)
    for kind in ("HEARTBEAT", "BAD_DATA"):
        wire.incoming.put(message(kind))
        first.recv_match(blocking=True, timeout=1)
        second.recv_match(blocking=True, timeout=1)
    first.logfile.flush()
    assert first.logfile.fileno() >= 0
    first.close()
    assert first.logfile.closed
    first.logfile = None
    assert first.logfile is None and wire.closes == 0
    wire.incoming.put(message("ATTITUDE"))
    second.recv_match(blocking=True, timeout=1)
    second.close()
    encoded = first_path.read_bytes()
    (timestamp,) = struct.unpack(">Q", encoded[:8])
    assert timestamp % 4 == 0 and abs(timestamp / 1e6 - time.time()) < 5
    assert encoded[8:] == b"HEARTBEAT"
    assert second_path.read_bytes()[: len(encoded)] == encoded
    assert second_path.read_bytes()[len(encoded) + 8 :] == b"ATTITUDE"


def test_log_backpressure_is_reported_while_reader_keeps_working(monkeypatch, tmp_path):
    path = tmp_path / "blocked.tlog"
    real_open = Path.open
    handle = real_open(path, "wb")
    entered, release = threading.Event(), threading.Event()

    class BlockedFile:
        def write(self, data):
            entered.set()
            assert release.wait(timeout=2)
            return handle.write(data)

        def __getattr__(self, name):
            return getattr(handle, name)

    monkeypatch.setattr(
        Path,
        "open",
        lambda self, *args, **kwargs: (
            BlockedFile() if self == path else real_open(self, *args, **kwargs)
        ),
    )
    wire = Wire()
    hub = SharedMavlink(wire)
    logger = hub.subscribe("logger", capacity=1)
    observer = hub.subscribe("observer")
    logger.setup_logfile(path)
    try:
        for index in range(3):
            wire.incoming.put(message())
            assert observer.recv_match(blocking=True, timeout=1) is not None
            if index < 2:
                logger.recv_match(blocking=True, timeout=1)
            if index == 0:
                assert entered.wait(timeout=1)
        with pytest.raises(SharedMavlinkError, match="tlog queue overflow"):
            logger.recv_match()
        assert hub.status()["fresh"]
    finally:
        release.set()
        with pytest.raises(SharedMavlinkError, match="tlog queue overflow"):
            hub.close()
    assert handle.closed


@pytest.mark.parametrize("operation", ["write", "fsync"])
def test_disk_failure_is_visible_without_stopping_other_consumers(
    monkeypatch, tmp_path, operation
):
    path = tmp_path / "failed.tlog"
    real_open = Path.open
    handle = real_open(path, "wb")
    closed = threading.Event()

    class FailedFile:
        def write(self, data):
            if operation == "write":
                raise OSError("disk full")
            return handle.write(data)

        def close(self):
            handle.close()
            closed.set()

        def __getattr__(self, name):
            return getattr(handle, name)

    monkeypatch.setattr(
        Path,
        "open",
        lambda self, *args, **kwargs: (
            FailedFile() if self == path else real_open(self, *args, **kwargs)
        ),
    )
    wire = Wire()
    hub = SharedMavlink(wire)
    logger, observer = hub.subscribe("logger"), hub.subscribe("observer")
    logger.setup_logfile(path)
    if operation == "fsync":
        original_sync = shared.os.fsync

        def fail_sync(descriptor):
            if not handle.closed and descriptor == handle.fileno():
                raise OSError("disk full")
            original_sync(descriptor)

        monkeypatch.setattr(shared.os, "fsync", fail_sync)
        assert logger.logfile is not None
        logger.logfile._sync._due_at = 0
    try:
        wire.incoming.put(message())
        assert observer.wait_heartbeat(timeout=1) is not None
        observer.mav.heartbeat_send(6, 8, 0, 0, 0)
        assert wire.writes[-1] == ("heartbeat", (6, 8, 0, 0, 0))
        assert closed.wait(timeout=1)
        with pytest.raises(SharedMavlinkError, match="disk full"):
            logger.recv_match()
        wire.incoming.put(message())
        assert observer.wait_heartbeat(timeout=1) is not None
    finally:
        with pytest.raises(SharedMavlinkError, match="disk full"):
            hub.close()


def test_failed_log_open_does_not_close_or_poison_endpoint(link, tmp_path):
    wire, hub = link
    endpoint = hub.subscribe("recording")
    with pytest.raises(FileNotFoundError):
        endpoint.setup_logfile(tmp_path / "missing" / "file.tlog")
    wire.incoming.put(message())
    assert endpoint.wait_heartbeat(timeout=1) is not None
    assert endpoint.logfile is None and wire.closes == 0


def test_close_wakes_blocked_receivers_and_closes_transport_once():
    wire = Wire()
    hub = SharedMavlink(wire)
    endpoints = [hub.subscribe(str(index)) for index in range(2)]
    errors = []

    def receive(endpoint):
        try:
            endpoint.recv_match(blocking=True)
        except SharedMavlinkError as error:
            errors.append(str(error))

    workers = [
        threading.Thread(target=receive, args=(endpoint,), daemon=True)
        for endpoint in endpoints
    ]
    for worker in workers:
        worker.start()
    hub.close()
    hub.close()
    for worker in workers:
        worker.join(timeout=1)
        assert not worker.is_alive()
    assert len(errors) == 2 and wire.closes == 1


def test_close_is_bounded_when_transport_ignores_timeout(monkeypatch):
    wire = Wire()
    entered, release = threading.Event(), threading.Event()

    def blocked(**_kwargs):
        entered.set()
        release.wait(timeout=2)

    monkeypatch.setattr(wire, "recv_match", blocked)
    hub = SharedMavlink(wire)
    try:
        assert entered.wait(timeout=1)
        started = time.monotonic()
        with pytest.raises(SharedMavlinkError, match="reader did not stop"):
            hub.close(timeout=0.03)
        assert time.monotonic() - started < 0.5
    finally:
        release.set()
        hub.close()


def test_failed_reader_start_releases_transport(monkeypatch):
    def fail(_thread):
        raise RuntimeError("no threads")

    wire = Wire()
    monkeypatch.setattr(threading.Thread, "start", fail)
    with pytest.raises(RuntimeError, match="no threads"):
        SharedMavlink(wire)
    assert wire.closes == 1


def test_reception_time_fallback_and_invalid_stamps():
    sample = message()
    assert received_monotonic(sample, default=123.0) == 123.0
    sample._received_monotonic = 456.0
    assert received_monotonic(sample, default=123.0) == 456.0
    sample._received_monotonic = float("nan")
    with pytest.raises(ValueError, match="timestamp"):
        received_monotonic(sample)


def test_tlog_preserves_original_receipt_when_transport_delivery_is_delayed(
    link, tmp_path
):
    wire, hub = link
    endpoint = hub.subscribe("flight-recording")
    path = tmp_path / "flight.tlog"
    endpoint.setup_logfile(path)
    original = message("HEARTBEAT")
    original._timestamp = time.time() - 10
    original._received_monotonic = time.monotonic() - 10
    wire.incoming.put(original)
    observed = endpoint.recv_match(blocking=True, timeout=1)
    endpoint.close()
    logged_timestamp = struct.unpack(">Q", path.read_bytes()[:8])[0] / 1_000_000
    assert observed._timestamp == original._timestamp
    assert logged_timestamp == pytest.approx(original._timestamp, abs=0.000004, rel=0)
    assert not hub.status()["fresh"]


@pytest.mark.parametrize(
    "timestamp", [True, float("nan"), float("inf"), -1, "1700000000"]
)
def test_invalid_wall_receipt_fails_explicitly_instead_of_fabricating_a_timestamp(
    link, timestamp
):
    wire, hub = link
    endpoint = hub.subscribe("flight-recording")
    incoming = message("HEARTBEAT")
    incoming._timestamp = timestamp
    wire.incoming.put(incoming)
    with pytest.raises(SharedMavlinkError, match="wall receipt timestamp"):
        endpoint.recv_match(blocking=True, timeout=1)
    assert hub.status()["status"] == "unknown"


def test_tlog_flushes_before_close_and_syncs_periodically_in_its_writer(
    link, tmp_path, monkeypatch
):
    wire, hub = link
    endpoint = hub.subscribe("recording")
    path = tmp_path / "durable.tlog"
    endpoint.setup_logfile(path)
    sink = endpoint.logfile
    assert sink is not None
    synced = threading.Event()
    sync_threads = []

    def sync(descriptor):
        if descriptor == sink.fileno():
            sync_threads.append(threading.get_ident())
            synced.set()

    monkeypatch.setattr(shared.os, "fsync", sync)
    wire.incoming.put(message())
    endpoint.recv_match(blocking=True, timeout=1)
    deadline = time.monotonic() + 1
    while not path.read_bytes() and time.monotonic() < deadline:
        threading.Event().wait(0.005)
    assert path.read_bytes()[8:] == b"HEARTBEAT"
    assert not synced.is_set()
    sink._sync._due_at = 0
    wire.incoming.put(message("ATTITUDE"))
    endpoint.recv_match(blocking=True, timeout=1)
    assert synced.wait(1)
    assert sync_threads == [sink._thread.ident]
    assert sync_threads[0] != threading.get_ident()
    assert not sink.closed
