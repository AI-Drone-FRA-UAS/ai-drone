"""Selected-vehicle sensor observations and capture health summaries."""

from __future__ import annotations

import math
import time
from typing import Any

from ai_drone.capture.state import CaptureSnapshot, CaptureState
from ai_drone.mavlink.safety import distance_sensor_valid, is_fresh

_OBSERVATION_FRESHNESS_S = 2.0


def _observation_is_fresh(observed: float | None, now: float) -> bool:
    return is_fresh(observed, now, _OBSERVATION_FRESHNESS_S)


def _heartbeat_status(state: CaptureSnapshot, observed_at: float) -> str:
    if state.last_vehicle_heartbeat_monotonic is None:
        return "no_data"
    return (
        "ok"
        if _observation_is_fresh(state.last_vehicle_heartbeat_monotonic, observed_at)
        else "stale"
    )


def _observe_sensor_message(
    state: CaptureState, message: Any, *, observed_at: float | None = None
) -> None:
    """Keep the small live summary separate from the lossless JSONL record."""

    with state.lock:
        observed_at = time.monotonic() if observed_at is None else observed_at
        message_type = message.get_type()
        if message_type == "DISTANCE_SENSOR":
            orientation = int(message.orientation)
            current_cm = int(message.current_distance)
            if distance_sensor_valid(message, require_bounds=True):
                state.distance_samples[orientation] += 1
                state.latest_distance_m[orientation] = current_cm / 100.0
                state.distance_observed_monotonic[orientation] = observed_at
        elif message_type == "RANGEFINDER":
            distance = float(message.distance)
            if math.isfinite(distance) and distance > 0:
                state.legacy_range_samples += 1
                state.latest_legacy_range_m = distance
                state.legacy_range_observed_monotonic = observed_at
        elif message_type in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}:
            state.optical_flow_samples += 1
            quality = getattr(message, "quality", None)
            state.latest_flow_quality = int(quality) if quality is not None else None
            state.flow_observed_monotonic = observed_at


def _downward_range_summary(
    state: CaptureSnapshot,
    observed_at: float,
) -> tuple[int, float | None, str | None, bool]:
    """Choose one range telemetry format instead of adding duplicate reports."""

    oriented = state.distance_observed_monotonic.get(25)
    legacy = state.legacy_range_observed_monotonic
    oriented_fresh = _observation_is_fresh(oriented, observed_at)
    legacy_fresh = _observation_is_fresh(legacy, observed_at)
    if oriented_fresh or (oriented is not None and not legacy_fresh):
        return (
            state.distance_samples[25],
            state.latest_distance_m.get(25),
            "DISTANCE_SENSOR",
            oriented_fresh,
        )
    if legacy is not None:
        return (
            state.legacy_range_samples,
            state.latest_legacy_range_m,
            "RANGEFINDER",
            legacy_fresh,
        )
    return 0, None, None, False


def _sample_status(samples: int, parent: str, *, fresh: bool = True) -> str:
    if parent != "ok":
        return "unavailable"
    if not samples:
        return "no_data"
    return "ok" if fresh else "stale"


def _flow_status(state: CaptureSnapshot, parent: str, observed_at: float) -> str:
    status = _sample_status(
        state.optical_flow_samples,
        parent,
        fresh=_observation_is_fresh(state.flow_observed_monotonic, observed_at),
    )
    if status != "ok":
        return status
    if state.latest_flow_quality is None:
        return "unknown_quality"
    return "low_quality" if state.latest_flow_quality <= 0 else "ok"


def _component_report(
    state: CaptureState | CaptureSnapshot,
    *,
    on_pi: bool,
    flight_controller: str,
    camera: str,
    detector: str,
    details: dict[str, str],
    duration: float,
    observed_at: float | None = None,
) -> dict[str, dict[str, object]]:
    state = state.snapshot()
    observed_at = time.monotonic() if observed_at is None else observed_at
    downward, downward_m, range_source, range_fresh = _downward_range_summary(
        state, observed_at
    )
    downward_status = _sample_status(downward, flight_controller, fresh=range_fresh)
    flow_status = _flow_status(state, flight_controller, observed_at)
    forward = state.distance_samples[0]
    forward_status = _sample_status(
        forward,
        flight_controller,
        fresh=_observation_is_fresh(
            state.distance_observed_monotonic.get(0), observed_at
        ),
    )
    tag_status = _sample_status(state.tag_detections, camera)
    if detector != "ok":
        tag_status = "unavailable"
    report: dict[str, dict[str, object]] = {
        "pi": {"status": "ok" if on_pi else "unavailable"},
        "flight_controller": {
            "status": (
                _heartbeat_status(state, observed_at)
                if flight_controller == "ok"
                else flight_controller
            ),
            "messages": sum(state.vehicle_telemetry_counts.values()),
        },
        "camera": {"status": camera, "frames": state.camera_frames},
        "downward_rangefinder": {
            "status": downward_status,
            "samples": downward,
            "latest_m": downward_m,
            "source": range_source,
        },
        "forward_rangefinder": {
            "status": forward_status,
            "samples": forward,
            "latest_m": state.latest_distance_m.get(0),
        },
        "optical_flow": {
            "status": flow_status,
            "samples": state.optical_flow_samples,
            "quality": state.latest_flow_quality,
        },
        "apriltags": {
            "status": tag_status,
            "detections": state.tag_detections,
            "ids": dict(sorted(state.tag_ids.items())),
        },
        "servo": {
            "status": "not_detectable",
            "detail": "BCM12 has no passive servo-presence feedback",
        },
    }
    for name, detail in details.items():
        report.setdefault(name, {"status": "unavailable"})["detail"] = detail
    if duration > 0:
        for name in ("downward_rangefinder", "forward_rangefinder", "optical_flow"):
            samples = report[name]["samples"]
            if isinstance(samples, int):
                report[name]["rate_hz"] = round(samples / duration, 3)
    return report


def _print_live_status(
    state: CaptureState | CaptureSnapshot,
    *,
    tag_servo: bool = False,
    stop_after: int | None = None,
) -> None:
    def live_sample(value: float | int | None, count: int, *, fresh: bool) -> str:
        if not count:
            return "unavailable"
        if not fresh:
            return "stale"
        return str(value) if value is not None else "unavailable"

    state = state.snapshot()
    observed_at = time.monotonic()
    down_count, downward, _, down_fresh = _downward_range_summary(state, observed_at)
    down_display = live_sample(downward, down_count, fresh=down_fresh)
    forward_display = live_sample(
        state.latest_distance_m.get(0),
        state.distance_samples[0],
        fresh=_observation_is_fresh(
            state.distance_observed_monotonic.get(0), observed_at
        ),
    )
    flow_display = live_sample(
        state.latest_flow_quality,
        state.optical_flow_samples,
        fresh=_observation_is_fresh(state.flow_observed_monotonic, observed_at),
    )
    heartbeat_status = _heartbeat_status(state, observed_at)
    vehicle_state = (
        state.last_vehicle_state
        if heartbeat_status == "ok" and state.last_vehicle_state is not None
        else "unknown"
    )
    tag_status = "DETECTED" if state.visible_tag_ids else "NO_TAG"
    progress = (
        f"{state.servo_pulses_completed}/{stop_after}"
        if stop_after is not None
        else f"{state.servo_pulses_completed}/unbounded"
    )
    active_fields = (
        f" apriltag={tag_status} visible_ids={list(state.visible_tag_ids)} "
        f"servo_progress={progress} "
        f"pending_ids={list(state.pending_servo_tag_ids)}"
        if tag_servo
        else ""
    )
    print(
        "status "
        f"vehicle_state={vehicle_state} heartbeat_status={heartbeat_status} "
        f"camera_frames={state.camera_frames} "
        f"telemetry={sum(state.telemetry_counts.values())} "
        f"downward_m={down_display} "
        f"forward_m={forward_display} "
        f"flow_quality={flow_display} "
        f"tags={state.tag_detections}"
        f"{active_fields}",
        flush=True,
    )
