"""Host validation of the tests-only camera with real detection and H.264."""

import json
import subprocess

import pytest

from ai_drone.cli import record
from tests.sitl_workload_camera import install_camera, require_codecs


def video_frame_count(path):
    _, ffprobe = require_codecs()
    result = subprocess.run(
        [
            ffprobe,
            "-v",
            "error",
            "-count_frames",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=codec_name,nb_read_frames",
            "-of",
            "json",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    stream = json.loads(result.stdout)["streams"][0]
    assert stream["codec_name"] == "h264"
    return int(stream["nb_read_frames"])


@pytest.mark.parametrize("pressure", [False, True])
def test_generated_camera_runs_real_recorder_detector_encoder_and_teardown(
    tmp_path, monkeypatch, pressure
):
    low_space = tmp_path / "low-space"
    install_camera(monkeypatch, low_space)
    monkeypatch.setattr(record, "_start_flight_capture", lambda _recording: None)
    original = record._check_capture_health

    def pressure_after_frames(recording, now):
        if pressure and recording.state.snapshot().camera_frames >= 8:
            low_space.touch()
        return original(recording, now)

    monkeypatch.setattr(record, "_check_capture_health", pressure_after_frames)
    output = tmp_path / "capture"
    assert (
        record.run(
            [
                "--duration",
                "2",
                "--warmup",
                "0",
                "--backend",
                "opencv",
                "--threads",
                "2",
                "--fps",
                "12",
                "--storage-check-interval",
                "0.2",
                "--resolution",
                "640x480",
                "--analysis-resolution",
                "640x480",
                "--output-dir",
                str(output),
            ]
        )
        == 0
    )
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["completed"] is True
    assert manifest["camera"]["encoded_frames"] >= 5
    assert manifest["components"]["apriltags"]["detections"] >= 10
    assert manifest["safety"]["gpio_servo_actuation_enabled"] is False
    assert (
        video_frame_count(output / "camera.h264")
        == manifest["camera"]["encoded_frames"]
    )
    rows = [
        json.loads(line) for line in (output / "camera.jsonl").read_text().splitlines()
    ]
    assert rows and {tag["id"] for row in rows for tag in row["tags"]} == {3}
    if pressure:
        assert manifest["storage"]["video_stopped_reason"] == "storage_reserve"
        assert len(rows) > manifest["camera"]["encoded_frames"] + 5
