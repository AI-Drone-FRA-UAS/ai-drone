"""Production cleanup under sensor loss, all endpoints confined to SITL."""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from tests.test_sitl import (
    _ardupilot_root,
    _assert_running_sitl_configuration,
    _connect,
    _ExternalMavlinkSensors,
    _production_hover_arguments,
    _running_cli,
    _running_sitl,
    _set_sitl_parameter,
)
from tests.test_sitl import operator_presence as operator_presence
from tests.test_sitl_profiles import _save_evidence, _xy_displacement

pytestmark = pytest.mark.sitl


class _SensorFault:
    def __init__(self, sender, fault):
        self.sender, self.fault = sender, fault

    def distance_sensor_send(self, *values):
        if self.fault in {"range-loss", "aiding-loss"}:
            return
        altered = list(values)
        if self.fault == "range-orientation":
            altered[6] = 0
        elif self.fault == "native-range-id":
            altered[5] = 7
        elif self.fault == "range-discontinuity":
            altered[3] = 120
        self.sender.distance_sensor_send(*altered)

    def optical_flow_send(self, *values):
        if self.fault in {"flow-loss", "aiding-loss"}:
            return
        altered = list(values)
        if self.fault == "flow-quality":
            altered[6] = 0
        self.sender.optical_flow_send(*altered)


def _fault_sensors(fault: str) -> type[_ExternalMavlinkSensors]:
    class FaultSensors(_ExternalMavlinkSensors):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.injections = []
            self.loiter_started = None

        def _send_sensors(self, sender, state, ground_altitude_m):
            now = time.monotonic()
            if self._current_mode == "LOITER" and self.loiter_started is None:
                self.loiter_started = now
            if self.loiter_started is not None and now - self.loiter_started >= 2:
                if not self.injections:
                    self.injections.append({"at": now, "fault": fault})
                sender = _SensorFault(sender, fault)
            super()._send_sensors(sender, state, ground_altitude_m)

    return FaultSensors


@pytest.mark.parametrize(
    "fault",
    [
        "flow-loss",
        "flow-quality",
        "range-loss",
        "range-orientation",
        "native-range-id",
        "range-discontinuity",
        "aiding-loss",
    ],
)
def test_loiter_sensor_fault_lands(tmp_path: Path, fault: str) -> None:
    with _running_sitl(
        _ardupilot_root(), tmp_path, sensor_factory=_fault_sensors(fault)
    ) as sensors:
        try:
            _assert_running_sitl_configuration(sensors)
            sensors.reset_observations()
            with _running_cli(
                tmp_path, fault, "control", *_production_hover_arguments(30)
            ) as (process, log):
                result = process.wait(timeout=100)
                if fault == "native-range-id":
                    assert result == 0, log.read_text()
                else:
                    assert result != 0, log.read_text()
            sensors.wait_for_disarm(timeout=15)
            assert sensors.injections
            injection = sensors.injections[0]["at"]
            land = next(
                at
                for at, mode, _armed in sensors.flight_observations
                if at >= injection and mode == "LAND"
            )
            # Bounded response, including telemetry expiry and FC mode reporting.
            if fault == "native-range-id":
                # Pinned AP_RangeFinder_MAVLink.cpp accepts native packets by
                # orientation, ignoring their ID; GCS republishes backend ID 0.
                # A changed upstream ID is therefore NOT an invalid companion
                # observation. Wrong FC output IDs are rejected in mock tests.
                readings = [
                    row
                    for row in sensors.range_observations
                    if row["observed_monotonic"] >= injection
                ]
                assert readings and {row["id"] for row in readings} == {0}
                assert land - injection >= 25
            else:
                assert land - injection < 10, log.read_text()
            armed = [row for row in sensors.truth if row["armed"]]
            summary = {
                "fault": fault,
                "injected_at": injection,
                "land_at": land,
                "response_s": land - injection,
                "armed_xy_displacement_m": _xy_displacement(armed),
                "maximum_height_m": max(row["height"] for row in armed),
                "final_disarmed": sensors._current_armed is False,
            }
            (tmp_path / "summary.json").write_text(json.dumps(summary, indent=2))
            print(json.dumps(summary))
            assert summary["maximum_height_m"] < 0.8
            assert summary["final_disarmed"]
            # Sensor recovery cannot restart the failed operation; test completes
            # in LAND and the process exits with its original fault visible.
            assert sensors.mode_transitions()[-1] == "LAND"
        finally:
            _save_evidence(tmp_path, sensors, {"fault": fault})


def test_compass_anomaly_before_arm_refuses(tmp_path: Path) -> None:
    with _running_sitl(_ardupilot_root(), tmp_path) as sensors:
        try:
            _assert_running_sitl_configuration(sensors)
            connection = _connect()
            try:
                for name, value in {
                    "SIM_MAG_ALY_X": 800.0,
                    "SIM_MAG_ALY_Y": -800.0,
                    "SIM_MAG_ALY_Z": 600.0,
                    "SIM_MAG_ALY_HGT": 100.0,
                }.items():
                    _set_sitl_parameter(connection, name, value)
            finally:
                connection.close()
            time.sleep(3)
            sensors.reset_observations()
            with _running_cli(
                tmp_path, "prearm-field", "control", *_production_hover_arguments(10)
            ) as (process, log):
                result = process.wait(timeout=90)
            assert result != 0, log.read_text()
            assert True not in sensors.armed_states, log.read_text()
            assert any(
                word in log.read_text().lower()
                for word in ("pre-arm", "compass", "magnetic", "mag field")
            )
        finally:
            _save_evidence(
                tmp_path,
                sensors,
                {
                    "fault": "all-compass-prearm-anomaly",
                    "field_mgauss": [800, -800, 600],
                    "decay_height_m": 100,
                },
            )


def _offset_sensors(offset: float) -> type[_ExternalMavlinkSensors]:
    class OffsetSensors(_ExternalMavlinkSensors):
        @staticmethod
        def _sensor_values(state, ground_altitude_m):
            # A simulated launch platform above the optical-flow/range floor.
            return _ExternalMavlinkSensors._sensor_values(
                state, ground_altitude_m - offset
            )

    return OffsetSensors


@pytest.mark.parametrize("offset,allowed", [(0.1, True), (0.3, False)])
def test_floor_offset_is_checked_before_climb(
    tmp_path: Path, offset: float, allowed: bool
) -> None:
    with _running_sitl(
        _ardupilot_root(), tmp_path, sensor_factory=_offset_sensors(offset)
    ) as sensors:
        try:
            _assert_running_sitl_configuration(sensors)
            sensors.reset_observations()
            with _running_cli(
                tmp_path, "offset", "control", *_production_hover_arguments(10)
            ) as (process, log):
                result = process.wait(timeout=100)
            if allowed:
                assert result == 0, log.read_text()
                sensors.wait_for_disarm(timeout=10)
                assert max(sensors.altitudes_m) + offset < 0.8
                assert max(sensors.altitudes_m) >= 0.45
            else:
                assert result != 0, log.read_text()
                assert True not in sensors.armed_states, log.read_text()
                assert max(sensors.altitudes_m) < 0.05
                assert "maximum altitude" in log.read_text().lower(), log.read_text()
        finally:
            _save_evidence(
                tmp_path,
                sensors,
                {"launch_platform_m_above_floor": offset, "expected_to_arm": allowed},
            )


def test_compass_changing_field_response(tmp_path: Path) -> None:
    """Retain the baseline profile's response to a field step after liftoff."""
    from tests.test_sitl_profiles import _DisturbedSensors, _yaw_metrics

    overlay = {"SIM_MAG_ALY_HGT": 100.0}
    with _running_sitl(
        _ardupilot_root(), tmp_path, sensor_factory=_DisturbedSensors, overlay=overlay
    ) as sensors:
        try:
            _assert_running_sitl_configuration(sensors, overlay=overlay)
            sensors.reset_observations()
            with _running_cli(
                tmp_path, "compass-field", "control", *_production_hover_arguments(30)
            ) as (process, log):
                result = process.wait(timeout=110)
            sensors.wait_for_disarm(timeout=15)
            assert sensors.injections
            for name, value in sensors.disturbance.items():
                assert sensors.parameter_readback.get(name) == pytest.approx(value)
            armed = [row for row in sensors.truth if row["armed"]]
            summary = {
                "exit_code": result,
                "maximum_height_m": max(row["height"] for row in armed),
                "armed_xy_displacement_m": _xy_displacement(armed),
                "armed_yaw": _yaw_metrics(armed, sensors.attitudes),
                "final_disarmed": sensors._current_armed is False,
                "diagnostic": log.read_text()[-2000:],
            }
            (tmp_path / "summary.json").write_text(json.dumps(summary, indent=2))
            print(json.dumps(summary))
            assert summary["maximum_height_m"] < 0.8
            assert summary["final_disarmed"]
            assert sensors.mode_transitions()[-1] == "LAND"
            # A completed operation must still satisfy independent motion bounds.
            # A refused/aborted operation must retain its concrete error instead
            # of claiming a qualified hold through the disturbance.
            if result == 0:
                assert summary["armed_xy_displacement_m"] <= 0.5
                assert summary["armed_yaw"]["truth_heading_motion_deg"] <= 10
                assert summary["armed_yaw"]["estimate_drift_deg"] <= 10
            else:
                assert summary["diagnostic"].strip()
        finally:
            _save_evidence(
                tmp_path,
                sensors,
                {
                    "profile": "flow-compass",
                    "fault": "changing-field-after-liftoff",
                    "overlay": overlay,
                },
            )
