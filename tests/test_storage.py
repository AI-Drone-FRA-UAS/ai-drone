from dataclasses import replace
from types import SimpleNamespace

import pytest

from ai_drone import storage
from ai_drone.durability import IntervalSync
from ai_drone.storage import MIB, StorageMonitor, StoragePolicy, storage_decision


@pytest.mark.parametrize("video", [False, True])
def test_storage_preserves_log_reserve_and_estimates_only_video(video):
    policy = StoragePolicy()
    free = int(policy.video_stop_bytes + 30 * policy.video_bytes_per_second)
    result = storage_decision(policy, free, video=video)
    assert not result.stop_video and not result.stop_capture
    assert result.estimated_video_seconds == (30 if video else None)
    result = storage_decision(policy, int(policy.video_stop_bytes), video=video)
    assert result.stop_video is video
    assert not result.stop_capture
    assert result.free_bytes > policy.reserve_mib * MIB


def test_storage_warning_and_log_floor_are_independent_of_video():
    policy = StoragePolicy()
    result = storage_decision(policy, int(policy.warning_mib * MIB), video=False)
    assert result.warning and not result.stop_capture
    result = storage_decision(policy, int(policy.stop_mib * MIB), video=False)
    assert result.stop_capture and result.warning and not result.stop_video


def test_video_buffer_covers_delayed_sampling_and_shutdown():
    policy = StoragePolicy(
        check_interval=10, max_pause_s=20, video_bytes_per_second=5 * MIB
    )
    assert policy.video_stop_bytes == (256 + 200) * MIB


@pytest.mark.parametrize(
    "values",
    [
        {"stop_mib": 0},
        {"stop_mib": 256},
        {"reserve_mib": 2048},
        {"check_interval": 0},
        {"warning_mib": float("nan")},
        {"video_bytes_per_second": float("inf")},
    ],
)
def test_invalid_storage_policy_is_rejected(values):
    with pytest.raises(ValueError):
        replace(StoragePolicy(), **values)


def test_monitor_bounds_samples_and_logs_warning_transitions(tmp_path, monkeypatch):
    import json

    free = iter([2048 * MIB, 512 * MIB, 510 * MIB])
    monkeypatch.setattr(
        storage.shutil, "disk_usage", lambda _: SimpleNamespace(free=next(free))
    )
    monitor = StorageMonitor(
        tmp_path / "storage.jsonl", StoragePolicy(), IntervalSync(0)
    )
    try:
        assert monitor.check(10, video=True) is not None
        assert monitor.check(11, video=True) is None
        assert monitor.check(12, video=True) is not None
        assert monitor.check(14, video=True) is not None
        assert monitor.manifest()["minimum_free_bytes"] == 510 * MIB
    finally:
        monitor.close()
    events = [json.loads(line) for line in monitor.path.read_text().splitlines()]
    assert [row["event"] for row in events].count("storage_sample") == 3
    assert [row["event"] for row in events].count("storage_warning") == 1


def test_monitor_propagates_log_write_failure(tmp_path, monkeypatch):
    monitor = StorageMonitor(
        tmp_path / "storage.jsonl", StoragePolicy(), IntervalSync(0)
    )
    monkeypatch.setattr(
        storage,
        "write_json_line",
        lambda *_a: (_ for _ in ()).throw(OSError("disk full")),
    )
    try:
        with pytest.raises(OSError, match="disk full"):
            monitor.check(10, video=False)
    finally:
        monitor.close()
