"""Selected-vehicle sensor observations and capture health summaries."""

from __future__ import annotations

import math
import time
from typing import Any

from ai_drone.capture.state import CaptureState

_RANGE_FRESHNESS_S = 2.0


def _observe_sensor_message(state: CaptureState, message: Any) -> None:
    """Keep the small live summary separate from the lossless JSONL record."""

    message_type = message.get_type()
    if message_type == "DISTANCE_SENSOR":
        orientation = int(message.orientation)
        current_cm = int(message.current_distance)
        minimum_cm = int(message.min_distance)
        maximum_cm = int(message.max_distance)
        # MAVLink defines 0 as unknown/not supplied and 1 as invalid.
        signal_quality = int(getattr(message, "signal_quality", 0))
        if (
            current_cm > 0
            and minimum_cm <= current_cm <= maximum_cm
            and signal_quality != 1
        ):
            state.distance_samples[orientation] += 1
            state.latest_distance_m[orientation] = current_cm / 100.0
            state.distance_observed_monotonic[orientation] = time.monotonic()
    elif message_type == "RANGEFINDER":
        distance = float(message.distance)
        if math.isfinite(distance) and distance > 0:
            state.legacy_range_samples += 1
            state.latest_legacy_range_m = distance
            state.legacy_range_observed_monotonic = time.monotonic()
    elif message_type in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}:
        state.optical_flow_samples += 1
        quality = getattr(message, "quality", None)
        state.latest_flow_quality = int(quality) if quality is not None else None


def _downward_range_summary(
    state: CaptureState,
    observed_at: float,
) -> tuple[int, float | None, str | None, bool]:
    """Choose one range telemetry format instead of adding duplicate reports."""

    oriented = state.distance_observed_monotonic.get(25)
    legacy = state.legacy_range_observed_monotonic
    oriented_fresh = (
        oriented is not None and 0 <= observed_at - oriented <= _RANGE_FRESHNESS_S
    )
    legacy_fresh = (
        legacy is not None and 0 <= observed_at - legacy <= _RANGE_FRESHNESS_S
    )
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


def _component_report(
    state: CaptureState,
    *,
    on_pi: bool,
    flight_controller: str,
    camera: str,
    detector: str,
    details: dict[str, str],
    duration: float,
    observed_at: float | None = None,
) -> dict[str, dict[str, object]]:
    def dependent(samples: int, parent: str) -> str:
        if parent != "ok":
            return "unavailable"
        return "ok" if samples else "no_data"

    observed_at = time.monotonic() if observed_at is None else observed_at
    downward, downward_m, range_source, range_fresh = _downward_range_summary(
        state, observed_at
    )
    downward_status = dependent(downward, flight_controller)
    if downward_status == "ok" and not range_fresh:
        downward_status = "stale"
    flow_status = dependent(state.optical_flow_samples, flight_controller)
    if flow_status == "ok":
        if state.latest_flow_quality is None:
            flow_status = "unknown_quality"
        elif state.latest_flow_quality <= 0:
            flow_status = "low_quality"
    forward = state.distance_samples[0]
    forward_status = dependent(forward, flight_controller)
    forward_observed = state.distance_observed_monotonic.get(0)
    if forward_status == "ok" and (
        forward_observed is None
        or not 0 <= observed_at - forward_observed <= _RANGE_FRESHNESS_S
    ):
        forward_status = "stale"
    tag_status = dependent(state.tag_detections, camera)
    if detector != "ok":
        tag_status = "unavailable"
    report: dict[str, dict[str, object]] = {
        "pi": {"status": "ok" if on_pi else "unavailable"},
        "flight_controller": {
            "status": flight_controller,
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
    state: CaptureState,
    *,
    tag_servo: bool = False,
    stop_after: int | None = None,
) -> None:
    downward = _downward_range_summary(state, time.monotonic())[1]
    forward = state.latest_distance_m.get(0)
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
        f"vehicle_state={state.last_vehicle_state or 'unavailable'} "
        f"camera_frames={state.camera_frames} "
        f"telemetry={sum(state.telemetry_counts.values())} "
        f"downward_m={downward if downward is not None else 'unavailable'} "
        f"forward_m={forward if forward is not None else 'unavailable'} "
        f"flow_quality={state.latest_flow_quality if state.latest_flow_quality is not None else 'unavailable'} "
        f"tags={state.tag_detections}"
        f"{active_fields}",
        flush=True,
    )
