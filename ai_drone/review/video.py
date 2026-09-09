"""Optional browser copy of raw H.264; original frame timestamps stay intact."""

from __future__ import annotations

import math
import shutil
import statistics
import subprocess
from pathlib import Path


def frame_rate(path: Path) -> float | None:
    """Estimate playback rate from monotonic Picamera2 millisecond timecodes."""
    previous: float | None = None
    intervals: list[float] = []
    try:
        with path.open(encoding="utf-8") as handle:
            for line in handle:
                if line.startswith("#") or not line.strip():
                    continue
                current = float(line)
                if not math.isfinite(current) or current < 0:
                    return None
                if previous is not None:
                    if current <= previous:
                        return None
                    if len(intervals) < 10000:
                        intervals.append(current - previous)
                previous = current
    except (OSError, ValueError):
        return None
    if not intervals:
        return None
    rate = 1000 / statistics.median(intervals)
    return rate if 1 <= rate <= 240 else None


def make_browser_video(recording: Path, output: Path) -> tuple[bool, str]:
    """Copy encoded packets into MP4 when possible; this is not a precise sync."""
    note = (
        "Video plays independently with approximate frame timing estimated from "
        "camera.pts. Telemetry receipt and camera exposure are not precisely aligned; "
        "the original H.264, PTS and camera metadata remain available."
    )
    raw = recording / "camera.h264"
    executable = shutil.which("ffmpeg")
    if not raw.is_file() or raw.stat().st_size == 0:
        return False, "No recorded camera video is available."
    if executable is None:
        return (
            False,
            "Browser video requires ffmpeg. Raw H.264 and camera timestamps are retained.",
        )
    rate = frame_rate(recording / "camera.pts")
    if rate is None:
        return (
            False,
            "Camera timestamps are missing or invalid; raw video is retained without guessing its playback rate.",
        )
    try:
        result = subprocess.run(
            [
                executable,
                "-nostdin",
                "-loglevel",
                "error",
                "-y",
                "-r",
                f"{rate:.9f}",
                "-f",
                "h264",
                "-protocol_whitelist",
                "file",
                "-i",
                str(raw),
                "-map",
                "0:v:0",
                "-c:v",
                "copy",
                "-movflags",
                "+faststart",
                str(output),
            ],
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        output.unlink(missing_ok=True)
        return (
            False,
            "Browser video conversion could not finish; original video and timestamps are retained.",
        )
    if result.returncode or not output.is_file() or output.stat().st_size == 0:
        output.unlink(missing_ok=True)
        return (
            False,
            "Browser video conversion failed; original video and timestamps are retained.",
        )
    return True, note
