"""Build a portable offline walkthrough report without contacting hardware."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path
from typing import Any
from urllib.parse import quote

from ai_drone.review.data import CSV_FIELDS, export_recording
from ai_drone.review.html import render_report
from ai_drone.review.video import make_browser_video

RAW_FILES = {
    "manifest.json": "Recording manifest",
    "telemetry.jsonl": "Raw telemetry JSONL",
    "telemetry.tlog": "Raw MAVLink log",
    "camera.jsonl": "Raw camera metadata",
    "camera.h264": "Original H.264 video",
    "camera.pts": "Original video timestamps",
}


def _relative(path: Path, output: Path) -> str:
    return quote(Path(os.path.relpath(path, output)).as_posix(), safe="/")


def build_report(
    recording: Path,
    output: Path | None = None,
    *,
    video: bool = True,
    system: int = 1,
    component: int = 1,
) -> Path:
    """Preserve input files and publish the HTML last, after all exports succeed."""
    recording = recording.expanduser().resolve()
    output = (output or recording / "review").expanduser().resolve()
    if not recording.is_dir():
        raise ValueError(f"recording directory does not exist: {recording}")
    if output == recording or output in recording.parents:
        raise ValueError(
            "report output must be a separate directory from the raw recording"
        )
    if not any(
        (recording / name).is_file() for name in ("telemetry.jsonl", "camera.jsonl")
    ):
        raise ValueError(
            "no telemetry.jsonl or camera.jsonl found in recording directory"
        )
    if not 1 <= system <= 255 or not 1 <= component <= 255:
        raise ValueError("MAVLink source IDs must be between 1 and 255")
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".build-", dir=output) as temporary:
        staging = Path(temporary)
        payload = export_recording(
            recording, staging, system=system, component=component
        )
        payload["files"] = [{"label": name, "href": name} for name in CSV_FIELDS] + [
            {"label": label, "href": _relative(recording / name, output)}
            for name, label in RAW_FILES.items()
            if (recording / name).is_file()
        ]
        camera: dict[str, Any] = {
            "frames": payload["csv_rows"].get("camera.csv", 0),
            "video": None,
            "video_note": "Browser video creation was disabled. Original video and timestamps are retained.",
            "first_frame": None,
            "last_frame": None,
        }
        if video:
            available, note = make_browser_video(recording, staging / "camera.mp4")
            camera.update(video="camera.mp4" if available else None, video_note=note)
        for key, filename in (
            ("first_frame", "first-frame.jpg"),
            ("last_frame", "last-frame.jpg"),
        ):
            if (recording / filename).is_file():
                camera[key] = _relative(recording / filename, output)
        payload["camera"] = camera
        (staging / "summary.json").write_text(
            json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        (staging / "index.html").write_text(render_report(payload), encoding="utf-8")
        # The browser entry point is only replaced after all its new assets exist.
        for path in staging.iterdir():
            if path.name != "index.html":
                os.replace(path, output / path.name)
        os.replace(staging / "index.html", output / "index.html")
    return output / "index.html"


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "recording", type=Path, help="directory produced by drone-inspect or drone-walk"
    )
    parser.add_argument("--output-dir", type=Path, help="defaults to RECORDING/review")
    parser.add_argument(
        "--no-video", action="store_true", help="skip the optional ffmpeg MP4 copy"
    )
    parser.add_argument(
        "--system", type=int, default=1, help="FC MAVLink system ID (default: 1)"
    )
    parser.add_argument(
        "--component", type=int, default=1, help="FC MAVLink component ID (default: 1)"
    )
    args = parser.parse_args(arguments)
    try:
        path = build_report(
            args.recording,
            args.output_dir,
            video=not args.no_video,
            system=args.system,
            component=args.component,
        )
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Report failed: {exc}\n")
    print(f"Report: {path}")
    print(
        "Open index.html in a browser; keep the recording directory with its CSVs and camera files together."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
