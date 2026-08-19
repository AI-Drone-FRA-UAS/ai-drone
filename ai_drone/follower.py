"""Person-target extraction and the canonical body-frame follow controller."""

from __future__ import annotations

import logging
import math
import time
from typing import Any, NamedTuple, Protocol

logger = logging.getLogger(__name__)
PERSON_CLASS_ID = 0


class FlightController(Protocol):
    battery_voltage: float | None
    current_altitude: float | None
    max_altitude: float
    is_flying: bool
    is_armed: bool

    def update_telemetry(self) -> None: ...
    def emergency_stop(self) -> None: ...
    def send_velocity_body(
        self, vx: float, vy: float, vz: float, yaw_rate_deg: float = 0.0
    ) -> None: ...


class PersonTarget(NamedTuple):
    distance_m: float
    offset_x_px: float
    offset_y_px: float
    confidence: float
    box_area: float


def _positive_finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return result


def _box(detection: Any) -> tuple[float, float, float, float] | None:
    raw = getattr(detection, "box", getattr(detection, "xyxy", detection))
    try:
        values = tuple(float(raw[index]) for index in range(4))
    except (IndexError, KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    x1, y1, x2, y2 = values
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def get_person_target(
    detections: Any,
    frame_width: int,
    frame_height: int,
    focal_length_px: float | None = None,
    default_person_height_m: float = 1.70,
    min_confidence: float = 0.40,
) -> PersonTarget | None:
    """Return the largest confident person box using explicit pinhole geometry."""

    if (
        isinstance(frame_width, bool)
        or isinstance(frame_height, bool)
        or frame_width <= 0
        or frame_height <= 0
    ):
        raise ValueError("frame dimensions must be positive integers")
    if not math.isfinite(min_confidence) or not 0.0 <= min_confidence <= 1.0:
        raise ValueError("min_confidence must be finite and between 0 and 1")
    person_height = _positive_finite(default_person_height_m, "default_person_height_m")
    focal = (
        _positive_finite(focal_length_px, "focal_length_px")
        if focal_length_px is not None
        else (frame_width / 2.0) / math.tan(math.radians(66.0) / 2.0)
    )
    if detections is None:
        return None

    candidates: list[PersonTarget] = []
    for index in range(len(detections)):
        detection = detections[index]
        try:
            confidence = float(getattr(detection, "confidence", 1.0))
            class_id = int(getattr(detection, "class_id", PERSON_CLASS_ID))
        except (TypeError, ValueError):
            continue
        bounds = _box(detection)
        if (
            bounds is None
            or not math.isfinite(confidence)
            or confidence < min_confidence
            or class_id != PERSON_CLASS_ID
        ):
            continue
        x1, y1, x2, y2 = bounds
        width, height = x2 - x1, y2 - y1
        candidates.append(
            PersonTarget(
                distance_m=max(0.3, min(15.0, person_height * focal / height)),
                offset_x_px=(x1 + x2) / 2.0 - frame_width / 2.0,
                offset_y_px=(y1 + y2) / 2.0 - frame_height / 2.0,
                confidence=confidence,
                box_area=width * height,
            )
        )
    return max(candidates, key=lambda target: target.box_area, default=None)


class AutonomousFollower:
    """Bounded proportional controller for experimental forward-camera following."""

    def __init__(
        self,
        drone: FlightController,
        target_dist_m: float = 2.0,
        max_vx: float = 0.3,
        max_yaw_rate_deg: float = 20.0,
        kp_dist: float = 0.4,
        kp_yaw: float = 0.25,
        lost_timeout_s: float = 3.0,
        min_battery_v: float = 14.4,
    ) -> None:
        self.drone = drone
        self.target_dist_m = _positive_finite(target_dist_m, "target_dist_m")
        self.max_vx = min(_positive_finite(max_vx, "max_vx"), 1.0)
        self.max_yaw_rate_deg = min(
            _positive_finite(max_yaw_rate_deg, "max_yaw_rate_deg"), 45.0
        )
        self.kp_dist = _positive_finite(kp_dist, "kp_dist")
        self.kp_yaw = _positive_finite(kp_yaw, "kp_yaw")
        self.lost_timeout_s = _positive_finite(lost_timeout_s, "lost_timeout_s")
        self.min_battery_v = _positive_finite(min_battery_v, "min_battery_v")
        self._last_target_time = time.monotonic()
        self._is_tracking = False

    def compute_velocity_command(
        self,
        target: PersonTarget | None,
        now: float | None = None,
    ) -> tuple[float, float, float, float]:
        timestamp = time.monotonic() if now is None else float(now)
        if target is None:
            if (
                self._is_tracking
                and timestamp - self._last_target_time >= self.lost_timeout_s
            ):
                logger.warning("Person target lost; commanding hover")
                self._is_tracking = False
            return 0.0, 0.0, 0.0, 0.0
        self._last_target_time = timestamp
        self._is_tracking = True
        distance_error = target.distance_m - self.target_dist_m
        vx = (
            0.0
            if abs(distance_error) < 0.15
            else max(-self.max_vx, min(self.max_vx, distance_error * self.kp_dist))
        )
        yaw = (
            0.0
            if abs(target.offset_x_px) < 15.0
            else max(
                -self.max_yaw_rate_deg,
                min(self.max_yaw_rate_deg, target.offset_x_px * self.kp_yaw),
            )
        )
        return round(vx, 3), 0.0, 0.0, round(yaw, 2)

    def check_safety_guardrails(self) -> None:
        voltage = self.drone.battery_voltage
        if voltage is not None and 0.0 < voltage < self.min_battery_v:
            self.drone.emergency_stop()
            raise RuntimeError("flight stopped by battery guard")
        altitude = self.drone.current_altitude
        if altitude is not None and altitude > self.drone.max_altitude:
            self.drone.emergency_stop()
            raise RuntimeError("flight stopped by altitude guard")
        if self.drone.is_flying:
            altitude_fresh = getattr(self.drone, "altitude_is_fresh", None)
            heartbeat_fresh = getattr(self.drone, "heartbeat_is_fresh", None)
            if callable(altitude_fresh) and not altitude_fresh():
                self.drone.emergency_stop()
                raise RuntimeError("flight stopped because altitude is stale")
            if callable(heartbeat_fresh) and not heartbeat_fresh():
                self.drone.emergency_stop()
                raise RuntimeError("flight stopped because heartbeat is stale")

    def run_simulated_tracking(self, duration_s: float = 15.0) -> None:
        duration = _positive_finite(duration_s, "duration_s")
        started = time.monotonic()
        try:
            while (elapsed := time.monotonic() - started) < duration:
                self.drone.update_telemetry()
                self.check_safety_guardrails()
                target = PersonTarget(
                    distance_m=max(1.5, 3.5 - 2.0 * elapsed / duration),
                    offset_x_px=50.0 * math.sin(elapsed * 1.5),
                    offset_y_px=0.0,
                    confidence=0.85,
                    box_area=15_000.0,
                )
                command = self.compute_velocity_command(target)
                logger.info(
                    "[SIM] distance=%.2f m offset=%.1f px -> vx=%.2f yaw=%.1f",
                    target.distance_m,
                    target.offset_x_px,
                    command[0],
                    command[3],
                )
                if self.drone.is_flying and self.drone.is_armed:
                    self.drone.send_velocity_body(*command)
                time.sleep(0.1)
        finally:
            if self.drone.is_flying and self.drone.is_armed:
                self.drone.send_velocity_body(0.0, 0.0, 0.0, 0.0)

    def run_live_tracking(
        self,
        confidence: float = 0.40,
        max_duration_s: float | None = None,
        *,
        focal_length_px: float | None = None,
        person_height_m: float = 1.70,
    ) -> None:
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be finite and between 0 and 1")
        if focal_length_px is None:
            logger.warning(
                "No measured focal length supplied; using the legacy 66-degree HFOV approximation"
            )
            focal: float | None = None
        else:
            focal = _positive_finite(focal_length_px, "focal_length_px")
        person_height = _positive_finite(person_height_m, "person_height_m")
        duration = (
            None
            if max_duration_s is None
            else _positive_finite(max_duration_s, "max_duration_s")
        )
        try:
            from modlib.devices import AiCamera  # ty: ignore[unresolved-import]
            from modlib.models.zoo import (  # ty: ignore[unresolved-import]
                NanoDetPlus416x416,
            )
        except ImportError as error:
            raise RuntimeError(
                "live follow requires the Raspberry Pi modlib stack"
            ) from error

        device = AiCamera()
        device.deploy(NanoDetPlus416x416())
        started = time.monotonic()
        try:
            with device as stream:
                for frame in stream:
                    if duration is not None and time.monotonic() - started >= duration:
                        break
                    self.drone.update_telemetry()
                    self.check_safety_guardrails()
                    target = get_person_target(
                        frame.detections,
                        frame.width,
                        frame.height,
                        focal_length_px=focal,
                        default_person_height_m=person_height,
                        min_confidence=confidence,
                    )
                    self.drone.send_velocity_body(
                        *self.compute_velocity_command(target)
                    )
        finally:
            if self.drone.is_flying and self.drone.is_armed:
                self.drone.send_velocity_body(0.0, 0.0, 0.0, 0.0)
