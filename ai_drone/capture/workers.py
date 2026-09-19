"""Telemetry and AprilTag workers for the synchronized recording session."""

from __future__ import annotations

import json
import os
import queue
import threading
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, TextIO

from ai_drone.capture.reporting import _observe_sensor_message
from ai_drone.capture.state import (
    AnalysisFrame,
    CaptureState,
    CaptureWindow,
    DetectionObserver,
)
from ai_drone.durability import IntervalSync, synced_stream
from ai_drone.mavlink.safety import heartbeat_is_armed, is_vehicle_message
from ai_drone.mavlink.shared import received_monotonic
from ai_drone.recording import json_safe, telemetry_record, write_json_line
from ai_drone.vision.apriltags import (
    CameraCalibration,
    Detector,
    PoseEstimationError,
    TagDetection,
    estimate_pose,
)


class TelemetryWorker(threading.Thread):
    """Monitor arming immediately, then record from the shared capture epoch."""

    def __init__(
        self,
        *,
        connection: Any,
        output: Path,
        vehicle_system: int,
        vehicle_component: int,
        window: CaptureWindow,
        stop: threading.Event,
        state: CaptureState,
        sync: IntervalSync,
        allow_armed_at_any_time: bool = False,
        stop_after_disarm: bool = False,
    ) -> None:
        super().__init__(name="telemetry-recorder", daemon=True)
        self.connection = connection
        self.output = output
        self.vehicle_system = vehicle_system
        self.vehicle_component = vehicle_component
        self.window = window
        self.stop = stop
        self.state = state
        self.sync = sync
        self.allow_armed_at_any_time = allow_armed_at_any_time
        self.stop_after_disarm = stop_after_disarm

    def _observe_heartbeat(
        self, message: Any, *, selected_vehicle: bool, received_at: float
    ) -> tuple[bool, bool]:
        """Return disarm-stop and armed-abort decisions before any recording I/O."""
        if not (
            message.get_type() == "HEARTBEAT"
            and selected_vehicle
            and self.state.heartbeat_is_current(received_at)
        ):
            return False, False
        armed = heartbeat_is_armed(message)
        disarm_stop = not armed and self.stop_after_disarm and self.state.saw_armed
        if disarm_stop:
            # Close actuation before publishing a fresh disarmed heartbeat.
            self.state.set_stop_reason("vehicle_disarmed")
            self.stop.set()
        self.state.observe_vehicle_state(armed=armed, observed_at=received_at)
        armed_abort = armed and not self.allow_armed_at_any_time
        if armed_abort:
            self.state.armed_abort = True
            self.state.record_error(
                "vehicle became ARMED during camera startup or capture"
            )
            self.stop.set()
        return disarm_stop, armed_abort

    def _record_message(
        self,
        handle: TextIO,
        message: Any,
        *,
        selected_vehicle: bool,
        received_at: float,
        started: float,
    ) -> None:
        message_type = message.get_type()
        self.state.telemetry_counts[message_type] += 1
        if selected_vehicle:
            self.state.vehicle_telemetry_counts[message_type] += 1
            _observe_sensor_message(self.state, message, observed_at=received_at)
        record = telemetry_record(message, elapsed_s=received_at - started)
        timestamp = getattr(message, "_timestamp", None)
        if isinstance(timestamp, int | float) and not isinstance(timestamp, bool):
            record["timestamp_utc"] = datetime.fromtimestamp(timestamp, UTC).isoformat()
        write_json_line(handle, record)
        synced = self.sync.after_record(handle)
        logfile = getattr(self.connection, "logfile", None)
        if logfile is not None:
            logfile.flush()
            if synced:
                os.fsync(logfile.fileno())

    def run(self) -> None:
        try:
            with synced_stream(self.output, self.sync) as handle:
                while not self.stop.is_set():
                    deadline = self.window.deadline
                    if deadline is not None and time.monotonic() >= deadline:
                        self.state.set_stop_reason("duration_elapsed")
                        self.stop.set()
                        return
                    timeout = (
                        min(0.1, max(0.0, deadline - time.monotonic()))
                        if deadline is not None
                        else 0.1
                    )
                    message = self.connection.recv_match(
                        blocking=True,
                        timeout=timeout,
                    )
                    if message is None:
                        continue
                    received_at = received_monotonic(message)
                    selected_vehicle = is_vehicle_message(
                        message,
                        system_id=self.vehicle_system,
                        component_id=self.vehicle_component,
                    )
                    disarm_stop, armed_abort = self._observe_heartbeat(
                        message,
                        selected_vehicle=selected_vehicle,
                        received_at=received_at,
                    )
                    if armed_abort:
                        return
                    started = self.window.started_monotonic
                    deadline = self.window.deadline
                    if disarm_stop and started is None:
                        self.state.set_stop_reason("vehicle_disarmed")
                        self.stop.set()
                        return
                    if started is None or received_at < started:
                        continue
                    if deadline is not None and received_at >= deadline:
                        self.state.set_stop_reason("duration_elapsed")
                        self.stop.set()
                        return
                    self._record_message(
                        handle,
                        message,
                        selected_vehicle=selected_vehicle,
                        received_at=received_at,
                        started=started,
                    )
                    if disarm_stop:
                        self.state.set_stop_reason("vehicle_disarmed")
                        self.stop.set()
                        return
        except Exception as error:  # keep the camera cleanup path deterministic
            self.state.record_error(f"telemetry worker: {error}")
            self.stop.set()


class DetectionWorker(threading.Thread):
    """Decode tags without delaying the video encoder or telemetry reader."""

    def __init__(
        self,
        *,
        frames: queue.Queue[AnalysisFrame | None],
        output: Path,
        detector: Detector,
        calibration: CameraCalibration | None,
        tag_size: float,
        resolution: tuple[int, int],
        max_reprojection_error: float,
        target_id: int | None,
        stop: threading.Event,
        state: CaptureState,
        sync: IntervalSync,
        observer: DetectionObserver | None = None,
    ) -> None:
        super().__init__(name="apriltag-recorder", daemon=True)
        self.frames = frames
        self.output = output
        self.detector = detector
        self.calibration = calibration
        self.tag_size = tag_size
        self.resolution = resolution
        self.max_reprojection_error = max_reprojection_error
        self.target_id = target_id
        self.stop = stop
        self.state = state
        self.sync = sync
        self.observer = observer

    def _describe_detection(
        self, detection: TagDetection
    ) -> tuple[dict[str, Any], bool]:
        tag: dict[str, Any] = {
            "id": detection.tag_id,
            "center_px": list(detection.center),
            "corners_px": detection.corners.tolist(),
            "hamming": detection.hamming,
            "decision_margin": detection.decision_margin,
        }
        pose_rejected = False
        if self.calibration is not None:
            try:
                pose = estimate_pose(
                    detection,
                    self.calibration,
                    tag_size_m=self.tag_size,
                    image_width=self.resolution[0],
                    image_height=self.resolution[1],
                )
            except PoseEstimationError as error:
                tag.update(pose_valid=False, pose_error=str(error))
                pose_rejected = True
            else:
                tag.update(
                    camera_xyz_m=list(pose.translation_m),
                    distance_m=pose.distance_m,
                    reprojection_error_px=pose.reprojection_error_px,
                    pose_valid=(
                        pose.reprojection_error_px <= self.max_reprojection_error
                    ),
                )
        return tag, pose_rejected

    def run(self) -> None:
        visible_ids: set[int] = set()
        try:
            with synced_stream(self.output, self.sync) as handle:
                while True:
                    item = self.frames.get()
                    try:
                        if item is None:
                            return
                        detections = self.detector.detect(item.grayscale)
                        if self.target_id is not None:
                            detections = [
                                detection
                                for detection in detections
                                if detection.tag_id == self.target_id
                            ]
                        tags = []
                        observer_detections = []
                        for detection in detections:
                            tag, pose_rejected = self._describe_detection(detection)
                            if not pose_rejected:
                                observer_detections.append(detection)
                            tags.append(tag)
                            self.state.tag_ids[detection.tag_id] += 1
                        self.state.tag_detections += len(tags)
                        self.state.processed_frames += 1
                        current_ids = {int(tag["id"]) for tag in tags}
                        self.state.visible_tag_ids = tuple(sorted(current_ids))
                        if self.observer is not None:
                            self.observer.observe(item, observer_detections, tags)
                        for event, identifiers in (
                            ("detected", current_ids - visible_ids),
                            ("lost", visible_ids - current_ids),
                        ):
                            for identifier in sorted(identifiers):
                                print(
                                    json.dumps(
                                        {
                                            "event": f"apriltag_{event}",
                                            "id": identifier,
                                            "elapsed_s": round(item.elapsed_s, 6),
                                        },
                                        sort_keys=True,
                                    ),
                                    flush=True,
                                )
                        visible_ids = current_ids
                        write_json_line(
                            handle,
                            {
                                "timestamp_utc": datetime.now(UTC).isoformat(),
                                "elapsed_s": round(item.elapsed_s, 6),
                                "frame": item.frame_index,
                                "metadata": json_safe(item.metadata),
                                "tags": tags,
                            },
                        )
                        self.sync.after_record(handle)
                    finally:
                        self.frames.task_done()
        except Exception as error:
            self.state.record_error(f"AprilTag worker: {error}")
            self.stop.set()
