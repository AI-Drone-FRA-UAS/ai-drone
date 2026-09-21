"""Frozen engineering targets, measured against truth in isolated pinned SITL.

These are engineering experiments, never physical flight qualification. Each
case retains truth, reported attitude, injection timing and command events even
when an acceptance assertion fails. No truth value is fed to the controller.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import ClassVar

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from tests.test_sitl import (
    _ardupilot_root,
    _assert_running_sitl_configuration,
    _ExternalMavlinkSensors,
    _production_hover_arguments,
    _running_cli,
    _running_sitl,
)
from tests.test_sitl import operator_presence as operator_presence

pytestmark = pytest.mark.sitl

# Freeze before running candidates. Changes require an operating-envelope decision.
HEIGHT_ERROR_M = 0.10
CEILING_M = 0.8
XY_DISPLACEMENT_M = 0.5
YAW_DEGREES = 10.0
SETTLING_S = 2.0
EXPERIMENT_PROFILE = {
    "EK3_SRC1_YAW": 0.0,
    "COMPASS_USE": 0.0,
    "COMPASS_USE2": 0.0,
    "COMPASS_USE3": 0.0,
}


def _angle_delta(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


def _xy_displacement(samples: list[dict]) -> float:
    origin = samples[0]
    metres_per_e7 = math.pi * 6_371_000 / 180 / 1e7
    longitude_scale = math.cos(math.radians(origin["lat"] / 1e7))
    return max(
        math.hypot(
            (sample["lat"] - origin["lat"]) * metres_per_e7,
            (sample["lon"] - origin["lon"]) * metres_per_e7 * longitude_scale,
        )
        for sample in samples
    )


def _yaw_metrics(truth: list[dict], attitudes: list[dict]) -> dict[str, float]:
    errors = []
    for sample in truth:
        estimate = min(attitudes, key=lambda row: abs(row["at"] - sample["at"]))
        assert abs(estimate["at"] - sample["at"]) < 0.15
        errors.append(_angle_delta(estimate["yaw"], sample["yaw"]))
    return {
        "truth_heading_motion_deg": math.degrees(
            max(abs(_angle_delta(sample["yaw"], truth[0]["yaw"])) for sample in truth)
        ),
        "initial_estimate_offset_deg": math.degrees(errors[0]),
        "estimate_drift_deg": math.degrees(
            max(abs(_angle_delta(error, errors[0])) for error in errors)
        ),
    }


def _summarize(
    sensors, events: list[dict], wall_offset: float, duration: float
) -> dict:
    from datetime import datetime

    start_event = next(
        row
        for row in events
        if row["event"] in {"altitude_hold_started", "loiter_started"}
    )
    start = (
        datetime.fromisoformat(start_event["timestamp_utc"]).timestamp() - wall_offset
    )
    hold = [
        row
        for row in sensors.truth
        if start + SETTLING_S <= row["at"] <= start + duration
    ]
    armed = [row for row in sensors.truth if row["armed"]]
    assert hold and armed and sensors.attitudes, "missing independent evidence"
    assert hold[-1]["at"] - hold[0]["at"] >= duration - SETTLING_S - 0.3
    target = start_event.get("floor_target_m", 0.52)
    return {
        "hold_duration_s": duration,
        "floor_target_m": target,
        "maximum_height_m": max(row["height"] for row in armed),
        "hold_height_error_m": max(abs(row["height"] - target) for row in hold),
        "hold_xy_displacement_m": _xy_displacement(hold),
        "armed_xy_displacement_m": _xy_displacement(armed),
        "armed_duration_s": armed[-1]["at"] - armed[0]["at"],
        "hold_yaw": _yaw_metrics(hold, sensors.attitudes),
        "armed_yaw": _yaw_metrics(armed, sensors.attitudes),
        "modes": sensors.mode_transitions(),
        "final_disarmed": sensors._current_armed is False,
    }


def _save_evidence(tmp_path, sensors, metadata):
    (tmp_path / "truth.json").write_text(json.dumps(sensors.truth))
    (tmp_path / "attitudes.json").write_text(json.dumps(sensors.attitudes))
    (tmp_path / "observations.json").write_text(
        json.dumps(
            {
                "status": sensors.status_by_mode,
                "flight": sensors.flight_observations,
                "range": sensors.range_observations,
                "metadata": metadata,
                "injections": getattr(sensors, "injections", []),
                "parameter_readback": getattr(sensors, "parameter_readback", {}),
            },
            indent=2,
        )
    )


def _run_profile(
    tmp_path: Path,
    operation: str,
    duration: float,
    heading: float,
    overlay: dict[str, float] | None = None,
    sensor_factory: type[_ExternalMavlinkSensors] = _ExternalMavlinkSensors,
) -> None:
    root = _ardupilot_root()
    wall_offset = time.time() - time.monotonic()
    metadata = {
        "operation": operation,
        "duration": duration,
        "heading": heading,
        "overlay": overlay,
        "wall_minus_monotonic_s": wall_offset,
    }
    reference = time.monotonic()
    with (
        pytest.MonkeyPatch.context() as environment,
        _running_sitl(
            root,
            tmp_path,
            heading=heading,
            overlay=overlay,
            sensor_factory=sensor_factory,
        ) as sensors,
    ):
        environment.setenv("AI_DRONE_PROFILE_REFERENCE_MONOTONIC", str(reference))
        try:
            _assert_running_sitl_configuration(sensors, overlay=overlay)
            sensors.reset_observations()
            arguments = _production_hover_arguments(duration)
            arguments[0] = operation
            if overlay:
                arguments += ["--navigation-profile", "flow-inertial-experimental"]
            with _running_cli(tmp_path, operation, "control", *arguments) as (
                process,
                log,
            ):
                result = process.wait(timeout=150)
                assert result == 0, log.read_text()[-8000:]
            sensors.wait_for_disarm(timeout=10)
            if isinstance(sensors, _DisturbedSensors):
                assert sensors.injections, "disturbance never activated"
                for name, value in sensors.disturbance.items():
                    assert sensors.parameter_readback.get(name) == pytest.approx(
                        value
                    ), name
            events_path = next(tmp_path.glob("artifacts/flights/*/events.jsonl"))
            events = [json.loads(line) for line in events_path.read_text().splitlines()]
            summary = _summarize(sensors, events, wall_offset, duration)
            (tmp_path / "summary.json").write_text(json.dumps(summary, indent=2))
            print(json.dumps(summary))
            assert summary["final_disarmed"]
            assert summary["maximum_height_m"] < CEILING_M
            assert summary["hold_height_error_m"] <= HEIGHT_ERROR_M
            # Vertical hold records an explicit clearance criterion; it does not
            # claim horizontal position control. Both operations need clearance.
            assert summary["hold_xy_displacement_m"] <= XY_DISPLACEMENT_M
            for window in ("hold_yaw", "armed_yaw"):
                assert summary[window]["truth_heading_motion_deg"] <= YAW_DEGREES
                assert summary[window]["estimate_drift_deg"] <= YAW_DEGREES
            if operation == "hover":
                original = sensors.assert_flight_result()
                assert original["max_horizontal_drift_m"] <= XY_DISPLACEMENT_M
            else:
                assert "LOITER" not in summary["modes"]
                assert "GUIDED_NOGPS" in summary["modes"]
        finally:
            _save_evidence(tmp_path, sensors, metadata)


@pytest.mark.parametrize("operation", ["altitude-hold", "hover"])
@pytest.mark.parametrize("duration,heading", [(10.0, 0.0), (30.0, 120.0)])
def test_compass_profile_truth(
    tmp_path: Path, operation: str, duration: float, heading: float
) -> None:
    _run_profile(tmp_path, operation, duration, heading)


class _DisturbedSensors(_ExternalMavlinkSensors):
    """Apply a changing field to every compass and gyro bias after liftoff.

    The field is NED milligauss and gyro bias is radians/s, as declared in
    pinned SITL.cpp. PARAM_VALUE readback proves each requested injection.
    """

    disturbance: ClassVar[dict[str, float]] = {
        "SIM_MAG_ALY_X": -800.0,
        "SIM_MAG_ALY_Y": 600.0,
        "SIM_MAG_ALY_Z": 500.0,
        "SIM_GYR1_BIAS_Z": 0.003,
        "SIM_GYR2_BIAS_Z": 0.003,
        "SIM_GYR3_BIAS_Z": 0.003,
    }

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.injections: list[dict] = []
        self.parameter_readback: dict[str, float] = {}
        self.airborne_since = None
        self.next_injection = 0.0

    def _observe_vehicle_message(self, message):
        if message.get_type() == "PARAM_VALUE":
            name = message.param_id
            if isinstance(name, bytes):
                name = name.decode("ascii")
            self.parameter_readback[name.rstrip("\x00")] = message.param_value
        super()._observe_vehicle_message(message)

    def _send_sensors(self, sender, state, ground_altitude_m):
        now = time.monotonic()
        if self._current_armed and state.alt - ground_altitude_m > 0.3:
            if self.airborne_since is None:
                self.airborne_since = now
            if now - self.airborne_since >= 2.0 and now >= self.next_injection:
                for name, value in self.disturbance.items():
                    if self.parameter_readback.get(name) != pytest.approx(value):
                        sender.param_set_send(
                            1,
                            1,
                            name.encode("ascii"),
                            value,
                            mavlink.MAV_PARAM_TYPE_REAL32,
                        )
                        self.injections.append(
                            {"at": now, "name": name, "value": value}
                        )
                self.next_injection = now + 0.5
        super()._send_sensors(sender, state, ground_altitude_m)


@pytest.mark.parametrize("operation", ["altitude-hold", "hover"])
@pytest.mark.parametrize("magnetic", [False, True], ids=["normal", "changing-field"])
@pytest.mark.parametrize(
    "duration,heading", [(10.0, 0.0), (30.0, 120.0), (30.0, 240.0)]
)
def test_inertial_profile_truth(
    tmp_path: Path, operation: str, magnetic: bool, duration: float, heading: float
) -> None:
    overlay = dict(EXPERIMENT_PROFILE)
    if magnetic:
        # Global anomaly acts on every enabled compass; no healthy spare hides it.
        overlay.update(
            {
                "SIM_MAG_ALY_X": 600.0,
                "SIM_MAG_ALY_Y": -800.0,
                "SIM_MAG_ALY_Z": 500.0,
                "SIM_MAG_ALY_HGT": 100.0,
            }
        )
    _run_profile(
        tmp_path,
        operation,
        duration,
        heading,
        overlay,
        _DisturbedSensors if magnetic else _ExternalMavlinkSensors,
    )
