"""Close the mount, then open once on tag 3 while monitoring disarmed telemetry.

Run on the Pi from ~/ai-drone:
    uv run --locked --group raspi python scripts/disarmed_tag_mount.py --duration 30
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import shutil
import signal
import subprocess
import threading
import time
from contextlib import suppress
from dataclasses import dataclass
from typing import Any

from ai_drone.mavlink.connection import open_ardupilot_connection
from ai_drone.mavlink.ownership import require_available_serial
from ai_drone.mavlink.safety import (
    heartbeat_is_armed,
    is_vehicle_message,
    require_ardupilot_heartbeat,
)
from ai_drone.platform import is_raspberry_pi
from ai_drone.vision.apriltags import create_detector


def event(name: str, **fields: Any) -> None:
    print(json.dumps({"event": name, **fields}), flush=True)


@dataclass
class TagGate:
    """Require three consecutive, fresh, uncorrected native tag-3 detections."""

    count: int = 0
    previous_frame: int = -1
    triggered: bool = False

    def observe(self, index: int, captured: float, tags: list[Any], now: float) -> bool:
        if self.triggered:
            return False
        valid = 0 <= now - captured <= 0.5 and any(
            tag.tag_id == 3
            and tag.hamming == 0
            and tag.decision_margin is not None
            and math.isfinite(tag.decision_margin)
            and tag.decision_margin >= 30
            for tag in tags
        )
        if index != self.previous_frame + 1:
            self.count = 0
        self.previous_frame = index
        self.count = self.count + 1 if valid else 0
        self.triggered = self.count >= 3
        return self.triggered


def camera_worker(
    stop: threading.Event, frames: queue.Queue, errors: queue.Queue
) -> None:
    camera = None
    try:
        from picamera2 import Picamera2  # ty: ignore[unresolved-import]

        detector = create_detector("native", threads=4, decimate=1.0)
        camera = Picamera2()
        camera.configure(
            camera.create_video_configuration(
                main={"format": "YUV420", "size": (640, 480)},
                raw={"size": (2028, 1520)},
                controls={"FrameRate": 30},
                queue=False,
                buffer_count=4,
            )
        )
        camera.start()
        if stop.wait(2):
            return
        index = 0
        while not stop.is_set():
            captured = time.monotonic()
            array = camera.capture_array("main")
            tags = detector.detect(array[:480, :640])
            if frames.full():
                with suppress(queue.Empty):
                    frames.get_nowait()
            frames.put_nowait((index, captured, tags))
            index += 1
    except Exception as error:
        errors.put(str(error))
    finally:
        if camera is not None:
            camera.close()


def stop_command(child: subprocess.Popen | None) -> None:
    if child is not None and child.poll() is None:
        os.killpg(child.pid, signal.SIGINT)
        try:
            child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
            event("forced_command_stop", warning="PWM cleanup could not be confirmed")


def drain_heartbeats(connection: Any, last_heartbeat: float) -> float:
    """Consume queued telemetry and abort on any observed armed heartbeat."""
    while True:
        message = connection.recv_match(blocking=False)
        if message is None:
            return last_heartbeat
        if message.get_type() == "HEARTBEAT" and is_vehicle_message(
            message, system_id=1, component_id=1
        ):
            if heartbeat_is_armed(message):
                raise RuntimeError("vehicle reported ARMED; stopping mount test")
            last_heartbeat = time.monotonic()


def supervise(connection: Any, duration: float, uv: str, stop: threading.Event) -> int:
    frames: queue.Queue = queue.Queue(maxsize=1)
    errors: queue.Queue = queue.Queue()
    worker = threading.Thread(
        target=camera_worker, args=(stop, frames, errors), daemon=True
    )
    child = None
    command_started = 0.0
    state = "camera_startup"
    started = time.monotonic()
    last_heartbeat = started
    last_camera = started
    deadline = started + 20
    gate = TagGate()
    worker.start()
    try:
        while not stop.is_set():
            # Drain all queued link traffic before considering any GPIO command.
            last_heartbeat = drain_heartbeats(connection, last_heartbeat)
            now = time.monotonic()
            if now - last_heartbeat > 2:
                raise RuntimeError("flight-controller heartbeat older than 2 seconds")
            if not errors.empty():
                raise RuntimeError(f"camera: {errors.get_nowait()}")
            if now >= deadline:
                event("finished", reason="duration_elapsed", opened=False)
                return 2
            if state != "camera_startup" and now - last_camera > 2:
                raise RuntimeError("camera stopped delivering frames")
            if child is not None:
                result = child.poll()
                if result is not None:
                    if result:
                        raise RuntimeError(f"mount command exited with status {result}")
                    child = None
                    if state == "opening":
                        event(
                            "finished",
                            reason="tag_3_open_command_completed",
                            opened=True,
                            physical_position_verified=False,
                            vehicle_state="disarmed",
                        )
                        return 0
                    state = "watching"
                    gate = TagGate()
                    started = now
                    deadline = now + duration
                    event(
                        "ready",
                        duration_s=duration,
                        target_id=3,
                        vehicle_state="disarmed",
                    )
                elif now - command_started > 5:
                    raise RuntimeError("mount command exceeded 5-second limit")
            try:
                index, captured, tags = frames.get_nowait()
            except queue.Empty:
                stop.wait(0.01)
                continue
            last_camera = now
            action = None
            if state == "camera_startup" and 0 <= now - captured <= 0.5:
                action = "close"
                state = "closing"
            elif state == "watching":
                event(
                    "detections",
                    elapsed_s=round(now - started, 3),
                    ids=[t.tag_id for t in tags],
                    frame=index,
                )
                if gate.observe(index, captured, tags, now):
                    action = "open"
                    state = "opening"
            if action:
                if stop.is_set() or time.monotonic() - last_heartbeat > 2:
                    raise RuntimeError(
                        "actuation cancelled: stopped or stale heartbeat"
                    )
                event(
                    "mount_command",
                    action=action,
                    vehicle_state="disarmed",
                    heartbeat_age_s=round(time.monotonic() - last_heartbeat, 3),
                )
                # Reuse the user's exact CLI, without syncing an active environment.
                child = subprocess.Popen(
                    [uv, "run", "--no-sync", "--group", "raspi", "drone_mount", action],
                    start_new_session=True,
                )
                command_started = time.monotonic()
        raise RuntimeError("operator stopped mount test")
    finally:
        stop_command(child)
        stop.set()
        worker.join(timeout=3)
        if worker.is_alive():
            event("camera_cleanup_timeout")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 300:
        parser.error("duration must be between 1 and 300 seconds")
    if not is_raspberry_pi():
        parser.error("run this command on the drone Pi, inside ~/ai-drone")
    uv = shutil.which("uv")
    if uv is None:
        parser.error("uv executable not found")
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda *_: stop.set())
    connection = None
    try:
        require_available_serial("/dev/serial0", on_pi=True)
        connection = open_ardupilot_connection("/dev/serial0", baud=115200)
        heartbeat = require_ardupilot_heartbeat(
            connection, system_id=1, component_id=1, timeout=10
        )
        if heartbeat_is_armed(heartbeat):
            raise RuntimeError("vehicle is ARMED; no mount commands sent")
        return supervise(connection, args.duration, uv, stop)
    except Exception as error:
        event("aborted", reason=str(error))
        return 1
    finally:
        if connection is not None:
            connection.close()


if __name__ == "__main__":
    raise SystemExit(main())
