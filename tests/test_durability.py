from __future__ import annotations

import os
from pathlib import Path

import pytest

from ai_drone.durability import (
    DEFAULT_SYNC_INTERVAL_S,
    IntervalSync,
    atomic_write_text,
    fsync_directory,
    synced_stream,
)


def test_atomic_write_text_creates_parents_and_writes_content(tmp_path: Path) -> None:
    target = tmp_path / "nested" / "snapshot.json"

    atomic_write_text(target, '{"a":1}\n')

    assert target.read_text(encoding="utf-8") == '{"a":1}\n'


def test_atomic_write_text_replaces_previous_content(tmp_path: Path) -> None:
    target = tmp_path / "snapshot.json"
    atomic_write_text(target, "first\n")

    atomic_write_text(target, "second\n")

    assert target.read_text(encoding="utf-8") == "second\n"
    assert sorted(item.name for item in tmp_path.iterdir()) == ["snapshot.json"]


def test_atomic_write_text_never_exposes_partial_contents(tmp_path: Path) -> None:
    target = tmp_path / "snapshot.json"
    atomic_write_text(target, "original\n")
    observed: list[str] = []

    def capture_then_fail(source: str | Path, destination: str | Path) -> None:
        # A reader racing the replacement must still see the previous version.
        observed.append(target.read_text(encoding="utf-8"))
        raise OSError("simulated power loss during rename")

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(os, "replace", capture_then_fail)
        with pytest.raises(OSError, match="simulated power loss"):
            atomic_write_text(target, "replacement\n")

    assert observed == ["original\n"]
    assert target.read_text(encoding="utf-8") == "original\n"


def test_atomic_write_text_removes_its_temporary_file_on_failure(
    tmp_path: Path,
) -> None:
    target = tmp_path / "snapshot.json"

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(
            os, "replace", lambda *_: (_ for _ in ()).throw(OSError("rename failed"))
        )
        with pytest.raises(OSError, match="rename failed"):
            atomic_write_text(target, "content\n")

    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("interval", [float("nan"), float("inf"), -1.0])
def test_interval_sync_rejects_invalid_intervals(interval: float) -> None:
    with pytest.raises(ValueError, match="finite and non-negative"):
        IntervalSync(interval)


def test_interval_sync_defaults_to_a_bounded_window() -> None:
    assert IntervalSync().interval_s == DEFAULT_SYNC_INTERVAL_S
    assert IntervalSync().periodic is True
    assert IntervalSync(0.0).periodic is False


def test_interval_sync_flushes_every_record_but_syncs_on_the_interval(
    tmp_path: Path,
) -> None:
    synced: list[int] = []
    sync = IntervalSync(5.0)

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(os, "fsync", synced.append)
        with (tmp_path / "stream.jsonl").open("w", encoding="utf-8") as handle:
            # The first record only arms the window; it does not sync.
            assert sync.after_record(handle, now=100.0) is False
            assert sync.after_record(handle, now=104.9) is False
            assert sync.after_record(handle, now=105.0) is True
            assert sync.after_record(handle, now=109.9) is False
            assert sync.after_record(handle, now=110.0) is True

    assert len(synced) == 2


def test_interval_sync_never_syncs_when_disabled(tmp_path: Path) -> None:
    synced: list[int] = []
    sync = IntervalSync(0.0)

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(os, "fsync", synced.append)
        with (tmp_path / "stream.jsonl").open("w", encoding="utf-8") as handle:
            for elapsed in range(0, 1000, 100):
                assert sync.after_record(handle, now=float(elapsed)) is False

    assert synced == []


def test_interval_sync_records_reach_the_file_without_closing_it(
    tmp_path: Path,
) -> None:
    stream = tmp_path / "stream.jsonl"
    sync = IntervalSync(0.0)

    with stream.open("w", encoding="utf-8") as handle:
        handle.write('{"record":1}\n')
        sync.after_record(handle)
        # A crashed process loses nothing already flushed to the system.
        assert stream.read_text(encoding="utf-8") == '{"record":1}\n'


def test_synced_stream_persists_the_tail_on_an_early_return(tmp_path: Path) -> None:
    stream = tmp_path / "stream.jsonl"
    synced: list[int] = []

    def writer() -> None:
        with pytest.MonkeyPatch.context() as patch:
            patch.setattr(os, "fsync", synced.append)
            with synced_stream(stream, IntervalSync(0.0)) as handle:
                handle.write('{"record":1}\n')
                return

    writer()

    assert stream.read_text(encoding="utf-8") == '{"record":1}\n'
    assert len(synced) == 1


def test_synced_stream_persists_the_tail_when_the_writer_fails(tmp_path: Path) -> None:
    stream = tmp_path / "stream.jsonl"
    synced: list[int] = []

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(os, "fsync", synced.append)
        with (
            pytest.raises(RuntimeError, match="worker failed"),
            synced_stream(stream, IntervalSync(0.0)) as handle,
        ):
            handle.write('{"record":1}\n')
            raise RuntimeError("worker failed")

    assert stream.read_text(encoding="utf-8") == '{"record":1}\n'
    assert len(synced) == 1


def test_fsync_directory_is_a_no_op_off_posix(tmp_path: Path) -> None:
    opened: list[object] = []

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(os, "name", "nt")
        patch.setattr(os, "open", lambda *args, **kwargs: opened.append(args))
        fsync_directory(tmp_path)

    assert opened == []


@pytest.mark.skipif(os.name != "posix", reason="directory fsync is POSIX-only")
def test_fsync_directory_closes_its_descriptor(tmp_path: Path) -> None:
    closed: list[int] = []
    real_close = os.close

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(os, "fsync", lambda descriptor: None)
        patch.setattr(
            os,
            "close",
            lambda descriptor: (closed.append(descriptor), real_close(descriptor))[0],
        )
        fsync_directory(tmp_path)

    assert len(closed) == 1
