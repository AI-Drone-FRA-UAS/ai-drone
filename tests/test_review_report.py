"""Offline report publication and optional camera-container behavior."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from ai_drone.cli import report
from ai_drone.review import video


@pytest.fixture
def recording(tmp_path: Path) -> Path:
    path = tmp_path / "recording"
    path.mkdir()
    (path / "manifest.json").write_text(
        json.dumps({"actual_duration_s": 1, "completed": True})
    )
    (path / "telemetry.jsonl").write_text("")
    (path / "camera.jsonl").write_text("")
    return path


def test_report_is_portable_and_preserves_raw(
    recording: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        report, "make_browser_video", lambda *_: pytest.fail("video disabled")
    )
    raw = {path.name: path.read_bytes() for path in recording.iterdir()}
    index = report.build_report(recording, video=False)
    assert index == recording / "review/index.html"
    summary = json.loads(index.with_name("summary.json").read_text())
    assert {item["href"] for item in summary["files"]} >= {
        "ranges.csv",
        "../telemetry.jsonl",
        "../manifest.json",
    }
    assert summary["camera"]["video"] is None
    assert (index.parent / "ranges.csv").read_text().startswith("elapsed_s,")
    for name, content in raw.items():
        assert (recording / name).read_bytes() == content


def test_failed_build_preserves_previous_report(
    recording: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    index = report.build_report(recording, video=False)
    previous = index.read_bytes()

    def fail(_payload: dict) -> str:
        raise ValueError("invalid chart payload")

    monkeypatch.setattr(report, "render_report", fail)
    with pytest.raises(ValueError, match="invalid chart"):
        report.build_report(recording, video=False)
    assert index.read_bytes() == previous
    assert not list(index.parent.glob(".build-*"))


def test_reject_raw_directory_as_output(recording: Path) -> None:
    with pytest.raises(ValueError, match="separate directory"):
        report.build_report(recording, recording)
    with pytest.raises(ValueError, match="separate directory"):
        report.build_report(recording, recording.parent)


def test_relative_asset_links_escape_filename_delimiters(tmp_path: Path) -> None:
    assert (
        report._relative(tmp_path / "raw #1" / "camera.h264", tmp_path / "review")
        == "../raw%20%231/camera.h264"
    )


@pytest.mark.parametrize(
    "content,expected",
    [
        ("# timecode format v2\n0\n40\n80\n", 25),
        ("0\n33.3\n66.6\n", 1000 / 33.3),
        ("0\n0\n", None),
        ("50\n20\n", None),
        ("0\nnan\n", None),
        ("garbage", None),
        ("0\n", None),
    ],
)
def test_timestamp_frame_rate(
    tmp_path: Path, content: str, expected: float | None
) -> None:
    pts = tmp_path / "camera.pts"
    pts.write_text(content)
    actual = video.frame_rate(pts)
    assert actual == pytest.approx(expected) if expected else actual is None


def test_browser_video_copies_without_claiming_precise_sync(
    recording: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (recording / "camera.h264").write_bytes(b"raw-camera")
    (recording / "camera.pts").write_text("0\n40\n80\n")
    output = recording / "browser.mp4"
    calls = []
    monkeypatch.setattr(video.shutil, "which", lambda _: "/usr/bin/ffmpeg")

    def convert(command: list[str], **kwargs: object) -> SimpleNamespace:
        calls.append((command, kwargs))
        output.write_bytes(b"mp4")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(video.subprocess, "run", convert)
    available, note = video.make_browser_video(recording, output)
    assert available and "approximate" in note and "not precisely aligned" in note
    argv, kwargs = calls[0]
    assert argv[argv.index("-r") + 1] == "25.000000000"
    assert argv[argv.index("-c:v") + 1] == "copy"
    assert argv[argv.index("-f") + 1] == "h264"
    assert argv[argv.index("-protocol_whitelist") + 1] == "file"
    assert "shell" not in kwargs
    assert (recording / "camera.h264").read_bytes() == b"raw-camera"


def test_video_failure_is_optional(
    recording: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (recording / "camera.h264").write_bytes(b"raw")
    monkeypatch.setattr(video.shutil, "which", lambda _: None)
    available, note = video.make_browser_video(recording, recording / "camera.mp4")
    assert not available and "ffmpeg" in note
    assert not (recording / "camera.mp4").exists()
