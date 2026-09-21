"""Tests-only generated camera and CPU H.264 adapter; no hardware is opened.

The real recorder, OpenCV detector and output/finalization code remain in use.
This models host workload, not Picamera2, IMX500 or Pi encoder performance.
"""

from __future__ import annotations

import json
import os
import queue
import re
import resource
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import CancelledError
from pathlib import Path
from types import ModuleType, SimpleNamespace

import cv2
import numpy as np


def require_codecs() -> tuple[str, str]:
    ffmpeg, ffprobe = shutil.which("ffmpeg"), shutil.which("ffprobe")
    if ffmpeg is None or ffprobe is None:
        raise RuntimeError("required workload gate needs ffmpeg and ffprobe")
    return ffmpeg, ffprobe


def generated_frame(width: int, height: int, index: int) -> np.ndarray:
    image = np.full((height, width), 255, dtype=np.uint8)
    side = min(width, height) // 3
    tag = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 3, side
    )
    left = (width - side) // 2 + index % 21 - 10
    top = (height - side) // 2
    image[top : top + side, left : left + side] = tag
    return image


class FileOutput:
    def __init__(self, path: str, *, pts: str):
        self.path, self.pts = Path(path), Path(pts)
        self.recording = False
        self.dead = False
        self.frames = 0

    def start(self):
        self.handle = self.path.open("wb")
        self.timestamps = self.pts.open("w")
        self.timestamps.write("# timecode format v2\n")
        self.recording = True

    def outputframe(
        self, frame, keyframe=True, timestamp=None, packet=None, audio=False
    ):
        del keyframe, packet, audio
        if not self.recording:
            raise RuntimeError("encoded frame arrived after output stopped")
        # Deliberate, measured recorder-only disk latency; never delay control I/O.
        if self.frames and self.frames % 11 == 0:
            time.sleep(0.04)
        self.handle.write(frame)
        self.handle.flush()
        assert timestamp is not None
        self.timestamps.write(f"{timestamp / 1000:.3f}\n")
        self.timestamps.flush()
        self.frames += 1

    def stop(self):
        if self.recording:
            self.recording = False
            self.handle.close()
            self.timestamps.close()


class H264Encoder:
    def __init__(self, *, bitrate: int, repeat: bool):
        del repeat
        self.bitrate = bitrate
        self.frames: queue.Queue[bytes | None] = queue.Queue(maxsize=4)
        self.errors: list[str] = []
        self.stopped = False

    def start(self, output, size, fps):
        ffmpeg, _ = require_codecs()
        self.output, self.fps = output, fps
        self.process = subprocess.Popen(
            [
                ffmpeg,
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pixel_format",
                "gray",
                "-video_size",
                f"{size[0]}x{size[1]}",
                "-framerate",
                str(fps),
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-threads",
                "2",
                "-preset",
                "ultrafast",
                "-tune",
                "zerolatency",
                "-b:v",
                str(self.bitrate),
                "-pix_fmt",
                "yuv420p",
                "-x264-params",
                "aud=1:keyint=1:repeat-headers=1",
                "-f",
                "h264",
                "pipe:1",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        output.start()
        self.writer = threading.Thread(target=self._write, daemon=True)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.writer.start()
        self.reader.start()

    def _write(self):
        assert self.process.stdin is not None
        try:
            while (frame := self.frames.get()) is not None:
                self.process.stdin.write(frame)
                self.process.stdin.flush()
        except Exception as error:
            self.errors.append(str(error))
        finally:
            self.process.stdin.close()

    def _read(self):
        assert self.process.stdout is not None
        buffer = b""
        index = 0
        try:
            while chunk := os.read(self.process.stdout.fileno(), 65536):
                buffer += chunk
                while (
                    len(
                        starts := list(
                            re.finditer(b"\x00\x00(?:\x00)?\x01\x09", buffer)
                        )
                    )
                    >= 2
                ):
                    boundary = starts[1].start()
                    self.output.outputframe(
                        buffer[:boundary], timestamp=round(index * 1e6 / self.fps)
                    )
                    index += 1
                    buffer = buffer[boundary:]
            if buffer:
                self.output.outputframe(buffer, timestamp=round(index * 1e6 / self.fps))
        except Exception as error:
            self.errors.append(str(error))

    def submit(self, frame):
        if self.errors:
            raise RuntimeError(f"synthetic encoder failed: {self.errors}")
        self.frames.put(frame.tobytes(), timeout=2)

    def stop(self):
        if self.stopped:
            return
        assert self.process.stdout is not None and self.process.stderr is not None
        self.stopped = True
        try:
            self.frames.put(None, timeout=2)
            self.writer.join(timeout=5)
            self.process.wait(timeout=5)
            self.reader.join(timeout=5)
            if self.writer.is_alive() or self.reader.is_alive():
                raise RuntimeError("synthetic encoder worker did not stop")
            if self.process.returncode or self.errors:
                raise RuntimeError(
                    f"H.264 encoding failed: {self.errors} {self.process.stderr.read().decode()}"
                )
        finally:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait(timeout=5)
            self.process.stdout.close()
            self.process.stderr.close()
            self.output.stop()


class Request:
    def __init__(self, image, captured_ns):
        height, width = image.shape
        self.image = np.full((height * 3 // 2, width), 128, dtype=np.uint8)
        self.image[:height] = image
        self.captured_ns = captured_ns
        self.released = False

    def make_array(self, name):
        assert name == "lores"
        return self.image

    def get_metadata(self):
        return {"SensorTimestamp": self.captured_ns, "ExposureTime": 10000}

    def release(self):
        if self.released:
            raise RuntimeError("generated request released twice")
        self.released = True


class GeneratedCamera:
    def __init__(self):
        self.index = 0
        self.encoder = None
        self.cancelled = False

    def create_video_configuration(self, **configuration):
        return configuration

    def configure(self, configuration):
        self.size = configuration["main"]["size"]
        self.analysis = configuration["lores"]["size"]
        self.fps = configuration["controls"]["FrameRate"]

    def start(self):
        self.next_frame = time.monotonic()

    def _frame(self):
        captured = time.monotonic_ns()
        width, height = self.analysis
        image = generated_frame(width, height, self.index)
        if self.encoder is not None:
            self.encoder.submit(cv2.resize(image, self.size))
        self.index += 1
        self.next_frame = time.monotonic() + 1 / self.fps
        return Request(image, captured)

    def start_encoder(self, encoder, output, *, name):
        assert name == "main"
        self.encoder = encoder
        encoder.start(output, self.size, self.fps)
        # Two access units let the streaming parser delimit the first real frame.
        self._frame().release()
        time.sleep(1 / self.fps)
        self._frame().release()

    def capture_request(self, *, wait, signal_function):
        assert wait is False
        return SimpleNamespace(request=None, signal=signal_function)

    def wait(self, job, *, timeout):
        if job.request is not None:
            return job.request
        if self.cancelled:
            raise CancelledError
        remaining = self.next_frame - time.monotonic()
        if remaining > timeout:
            time.sleep(timeout)
            raise TimeoutError
        if remaining > 0:
            time.sleep(remaining)
        job.request = self._frame()
        job.signal(job)
        return job.request

    def cancel_all_and_flush(self):
        self.cancelled = True

    def stop_encoder(self, encoder):
        self.encoder = None
        encoder.stop()

    def stop(self):
        self.cancel_all_and_flush()

    def close(self):
        self.stop()


def install_camera(monkeypatch, low_space: Path):
    from ai_drone import mount, storage
    from ai_drone.cli import record, tag_servo_record

    require_codecs()
    for name, values in (
        ("picamera2", {"Picamera2": GeneratedCamera}),
        ("picamera2.encoders", {"H264Encoder": H264Encoder}),
        ("picamera2.outputs", {"FileOutput": FileOutput}),
    ):
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(record, "is_raspberry_pi", lambda: True)

    def no_actuation(*_args, **_kwargs):
        raise AssertionError("synthetic workload must not construct an actuator")

    monkeypatch.setattr(mount, "create_servo", no_actuation)
    monkeypatch.setattr(mount, "ServoProcessLock", no_actuation)
    monkeypatch.setattr(tag_servo_record, "TagServoSession", no_actuation)
    original = storage.shutil.disk_usage

    def measured_disk(path):
        observed = original(path)
        return (
            observed._replace(free=128 * storage.MIB)
            if low_space.exists()
            else observed
        )

    monkeypatch.setattr(storage.shutil, "disk_usage", measured_disk)


def main():
    import pytest

    from ai_drone.cli.main import main as dispatch
    from tests.test_sitl import _require_loopback_namespace

    _require_loopback_namespace()
    low_space, evidence, *arguments = sys.argv[1:]
    if (
        arguments[0] != "record"
        or "--device" not in arguments
        or any(item in arguments for item in ("--tag-servo", "--confirm-actuation"))
    ):
        raise RuntimeError("only explicit passive recording is supported")
    endpoint = arguments[arguments.index("--device") + 1]
    if not endpoint.startswith("unix:/tmp/"):
        raise RuntimeError("workload capture requires the isolated test runtime")
    started = time.monotonic()
    try:
        with pytest.MonkeyPatch.context() as monkeypatch:
            install_camera(monkeypatch, Path(low_space))
            return dispatch(arguments)
    finally:
        own, children = (
            resource.getrusage(kind)
            for kind in (resource.RUSAGE_SELF, resource.RUSAGE_CHILDREN)
        )
        Path(evidence).write_text(
            json.dumps(
                {
                    "camera_source": "test-generated-tag36h11-id3",
                    "actuation_enabled": False,
                    "detector": "real-opencv-aruco",
                    "encoder": "host-ffmpeg-libx264",
                    "elapsed_s": time.monotonic() - started,
                    "user_cpu_s": own.ru_utime,
                    "system_cpu_s": own.ru_stime,
                    "maximum_rss_kib": own.ru_maxrss,
                    "encoder_user_cpu_s": children.ru_utime,
                    "encoder_system_cpu_s": children.ru_stime,
                    "encoder_maximum_rss_kib": children.ru_maxrss,
                    "write_delay_s": 0.04,
                    "write_delay_every_frames": 11,
                },
                indent=2,
            )
        )


if __name__ == "__main__":
    raise SystemExit(main())
