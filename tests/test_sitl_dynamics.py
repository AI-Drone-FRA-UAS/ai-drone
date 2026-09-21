"""Actual pinned-FC dynamics and mode faults, confined to the SITL namespace.

No controller is mocked. Physics faults use pinned SITL parameters; the receiver
case feeds AP_RCProtocol_UDP. Impact-cut traces are failure-response evidence,
never a claim that the aircraft retained a safe flight/clearance envelope.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from contextlib import closing, contextmanager, nullcontext
from datetime import datetime
from pathlib import Path
from typing import Any

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from tests.test_sitl import (
    _ardupilot_root,
    _assert_running_sitl_configuration,
    _ExternalMavlinkSensors,
    _production_hover_arguments,
    _require_loopback_namespace,
    _running_cli,
    _running_sitl,
)
from tests.test_sitl import operator_presence as operator_presence
from tests.test_sitl_faults import _SensorFault
from tests.test_sitl_profiles import _save_evidence, _xy_displacement

pytestmark = pytest.mark.sitl

# SITL.cpp ENGINE_FAIL is a servo-output bitmask. SITL_State.cpp:304-315 scales
# all four X-frame motor PWM inputs around 1000 before advancing the physics.
ALL_MOTORS = 15.0
NO_THRUST = {"SIM_ENGINE_FAIL": ALL_MOTORS, "SIM_ENGINE_MUL": 0.0}
# SIM_Tether.cpp applies a spring/damper force after the free line is exhausted;
# SIM_Aircraft.cpp adds it to physical acceleration. Upstream TestTetherStuck
# exercises the same model. These describe a simulated restraint, not hardware.
SHORT_TETHER = {
    "SIM_TETH_ENABLE": 1.0,
    "SIM_TETH_LINELEN": 0.10,
    "SIM_TETH_DENSITY": 0.0,
    "SIM_TETH_SPGCNST": 255.0,
    "SIM_TETH_DMPCNST": 10.0,
}
# Copter::gcs_mode_enabled() lists LOITER at bit 5, independently of mode ID 5.
BLOCK_LOITER = {"FLTMODE_GCSBLOCK": 32.0}


class _DynamicsSensors(_ExternalMavlinkSensors):
    fault: str

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.injections: list[dict] = []
        self.parameter_readback: dict[str, float] = {}
        self.ekf_observations: list[dict] = []
        self.rc_observations: list[dict] = []
        self.parameter_observations: list[dict] = []
        self.loiter_started: float | None = None
        self.next_write = 0.0

    def _observe_vehicle_message(self, message):
        if message.get_type() == "PARAM_VALUE" and (
            message.get_srcSystem(),
            message.get_srcComponent(),
        ) == (1, 1):
            name = message.param_id
            if isinstance(name, bytes):
                name = name.decode("ascii")
            self.parameter_readback[name.rstrip("\x00")] = float(message.param_value)
            self.parameter_observations.append(
                {
                    "at": time.monotonic(),
                    "name": name.rstrip("\x00"),
                    "value": float(message.param_value),
                }
            )
        if message.get_type() == "EKF_STATUS_REPORT" and (
            message.get_srcSystem(),
            message.get_srcComponent(),
        ) == (1, 1):
            self.ekf_observations.append(
                {"at": time.monotonic(), "flags": int(message.flags)}
            )
        if message.get_type() == "RC_CHANNELS" and (
            message.get_srcSystem(),
            message.get_srcComponent(),
        ) == (1, 1):
            self.rc_observations.append(
                {
                    "at": time.monotonic(),
                    "count": int(message.chancount),
                    "throttle_pwm": int(message.chan3_raw),
                }
            )
        super()._observe_vehicle_message(message)

    def _send_sensors(self, sender, state, ground_altitude_m):
        now = time.monotonic()
        if self._current_mode == "LOITER" and self.loiter_started is None:
            self.loiter_started = now
        active = self._current_armed and (
            self.fault == "missing-relative-aiding"
            or (self.loiter_started is not None and now - self.loiter_started >= 2.0)
        )
        if active and not self.injections:
            self.injections.append(
                {
                    "at": now,
                    "fault": self.fault,
                    "height": state.alt - ground_altitude_m,
                }
            )
            if self.fault == "unexpected-mode":
                # A genuine FC mode transition, with no handoff intent or RC.
                sender.set_mode_send(1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 2)
        if self.injections and self.fault == "missing-relative-aiding":
            sender = _SensorFault(sender, "flow-loss")
        elif active and now >= self.next_write:
            parameters = {
                "unexpected-descent": {"SIM_ENGINE_FAIL": ALL_MOTORS},
                "unexpected-low-throttle-rc": {"SIM_RC_FAIL": 0.0},
            }.get(self.fault, {})
            for name, value in parameters.items():
                if self.parameter_readback.get(name) != pytest.approx(value):
                    sender.param_set_send(
                        1, 1, name.encode("ascii"), value, mavlink.MAV_PARAM_TYPE_REAL32
                    )
            self.next_write = now + 0.5
        super()._send_sensors(sender, state, ground_altitude_m)


def _dynamics_sensors(fault: str) -> type[_DynamicsSensors]:
    return type("DynamicsSensors", (_DynamicsSensors,), {"fault": fault})


@contextmanager
def _low_throttle_receiver():
    """Native simulated receiver arrival, without a production handoff request."""
    _require_loopback_namespace()
    # AP_RCProtocol_UDP.cpp consumes eight uint16 PWM channels on local 5501.
    # The test sets all switch positions to LOITER to isolate receiver topology.
    frame = struct.pack("<8H", 1500, 1500, 1000, 1500, 1700, 1000, 1000, 1800)
    stop = threading.Event()
    failures = []
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as connection:

        def transmit():
            try:
                while not stop.is_set():
                    connection.sendto(frame, ("127.0.0.1", 5501))
                    stop.wait(0.02)
            except Exception as error:
                failures.append(error)

        thread = threading.Thread(
            target=transmit, daemon=True, name="sitl-unrequested-rc"
        )
        thread.start()
        try:
            yield
        finally:
            stop.set()
            thread.join(timeout=2)
            assert not thread.is_alive(), "simulated receiver did not stop"
            assert not failures, failures


def _events(tmp_path: Path) -> list[dict]:
    paths = list((tmp_path / "artifacts/flights").glob("*/events.jsonl"))
    assert len(paths) == 1, paths
    return [json.loads(line) for line in paths[0].read_text().splitlines()]


def _land_attempts(audit: dict) -> list[dict]:
    return [
        row
        for row in audit["recent_attempts"]
        if row["kind"] == "set_mode" and row["mode"] == "LAND"
    ]


def _assert_failed_flight(tmp_path: Path, sensors, result: int, output: str) -> dict:
    assert result != 0, output
    sensors.assert_healthy()
    sensors.wait_for_disarm(timeout=15)
    assert True in sensors.armed_states, "fault did not exercise an armed FC"
    assert "LAND" in sensors.mode_transitions(), sensors.mode_transitions()
    events = _events(tmp_path)
    assert not any(
        event["event"] == "control_owner" and event.get("owner") == "human"
        for event in events
    ), "unrequested topology/mode change was treated as handoff"
    audit = next(event for event in events if event["event"] == "command_audit")
    assert audit["kinds"].get("Climb", 0) > 0
    assert audit["kinds"].get("Disarm", 0) == 0, "climb intent lost its LAND obligation"
    assert _land_attempts(audit), audit
    manifests = list((tmp_path / "artifacts/flights").glob("*/manifest.json"))
    assert len(manifests) == 1
    manifest = json.loads(manifests[0].read_text())
    assert manifest["completed"] is False and manifest["error"], manifest
    return audit


@contextmanager
def _fault_case(tmp_path: Path, fault: str, overlay: dict[str, float]):
    wall_offset = time.time() - time.monotonic()
    metadata: dict[str, Any] = {
        "fault": fault,
        "overlay": overlay,
        "wall_minus_monotonic_s": wall_offset,
        "hardware_qualification": False,
    }
    with _running_sitl(
        _ardupilot_root(),
        tmp_path,
        overlay=overlay,
        sensor_factory=_dynamics_sensors(fault),
    ) as sensors:
        try:
            _assert_running_sitl_configuration(sensors, overlay=overlay)
            # Configuration helper verified each overlay's actual FC readback.
            metadata["verified_initial_parameters"] = overlay
            sensors.reset_observations()
            receiver = (
                _low_throttle_receiver()
                if fault == "unexpected-low-throttle-rc"
                else nullcontext()
            )
            with (
                receiver,
                _running_cli(
                    tmp_path, fault, "control", *_production_hover_arguments(30)
                ) as (process, log),
            ):
                result = process.wait(timeout=100)
                metadata["exit_code"] = result
                metadata["diagnostic"] = log.read_text()[-4000:]
            audit = _assert_failed_flight(tmp_path, sensors, result, log.read_text())
            yield sensors, audit, log.read_text(), wall_offset
        finally:
            metadata["ekf_observations"] = sensors.ekf_observations
            metadata["rc_observations"] = sensors.rc_observations
            metadata["parameter_observations"] = sensors.parameter_observations
            armed = [row for row in sensors.truth if row["armed"]]
            if armed:
                metadata.update(
                    {
                        "maximum_height_m": max(row["height"] for row in armed),
                        "minimum_height_m": min(row["height"] for row in armed),
                        "maximum_downward_speed_m_s": max(row["vd"] for row in armed),
                        "armed_xy_displacement_m": _xy_displacement(armed),
                        "final_disarmed": sensors._current_armed is False,
                    }
                )
            if sensors.injections:
                injected = sensors.injections[0]
                after = [row for row in sensors.truth if row["at"] >= injected["at"]]
                descended = next(
                    (
                        row
                        for row in after
                        if row["height"] <= injected["height"] - 0.20
                    ),
                    None,
                )
                touched_down = next(
                    (row for row in after if row["height"] < 0.03), None
                )
                metadata["response_timing"] = {
                    "injected_monotonic": injected["at"],
                    "descended_20cm_monotonic": None
                    if descended is None
                    else descended["at"],
                    "touchdown_monotonic": None
                    if touched_down is None
                    else touched_down["at"],
                    "land_observed_monotonic": next(
                        (
                            at
                            for at, mode, _armed in sensors.flight_observations
                            if at >= injected["at"] and mode == "LAND"
                        ),
                        None,
                    ),
                }
            (tmp_path / "summary.json").write_text(json.dumps(metadata, indent=2))
            _save_evidence(tmp_path, sensors, metadata)


@pytest.mark.parametrize("fault", ["no-liftoff", "stalled-climb"])
def test_takeoff_without_progress_preserves_land_obligation(tmp_path: Path, fault: str):
    overlay = NO_THRUST if fault == "no-liftoff" else SHORT_TETHER
    with _fault_case(tmp_path, fault, overlay) as (sensors, audit, output, _offset):
        assert "takeoff altitude was not reached" in output, output
        first_climb = audit["first_attempts"]["Climb"]["attempted_monotonic"]
        land = _land_attempts(audit)[0]["attempted_monotonic"]
        # Production takeoff has a 15s deadline, plus bounded observation/write latency.
        assert 14.0 <= land - first_climb <= 16.5, land - first_climb
        ascent = [row for row in sensors.truth if first_climb <= row["at"] <= land]
        assert ascent
        maximum = max(row["height"] for row in ascent)
        if fault == "no-liftoff":
            assert maximum < 0.05, maximum
        else:
            assert 0.05 < maximum < 0.45, maximum
            stalled = [row["height"] for row in ascent if row["at"] >= land - 2]
            assert stalled and max(stalled) - min(stalled) < 0.10, stalled
        assert not any(row["event"] == "loiter_started" for row in _events(tmp_path))


@pytest.mark.parametrize("fault", ["missing-relative-aiding", "rejected-loiter-entry"])
def test_loiter_acquisition_failure_lands(tmp_path: Path, fault: str):
    overlay = BLOCK_LOITER if fault == "rejected-loiter-entry" else {}
    with _fault_case(tmp_path, fault, overlay) as (sensors, audit, output, offset):
        events = _events(tmp_path)
        acquired = next(
            row for row in events if row["event"] == "loiter_acquisition_started"
        )
        started = datetime.fromisoformat(acquired["timestamp_utc"]).timestamp() - offset
        land = _land_attempts(audit)[0]["attempted_monotonic"]
        assert "LOITER" not in sensors.mode_transitions()
        assert not any(row["event"] == "loiter_started" for row in events)
        assert max(sensors.altitudes_m) >= 0.45
        assert max(sensors.altitudes_m) < 0.8
        if fault == "missing-relative-aiding":
            assert sensors.injections
            assert "never established stable optical-flow relative position" in output
            assert 19.0 <= land - started <= 21.5, land - started
            flags = [
                row["flags"]
                for row in sensors.ekf_observations
                if land - 5 <= row["at"] <= land
            ]
            assert flags and not any(flag & mavlink.EKF_POS_HORIZ_REL for flag in flags)
            neutral = [
                row
                for row in audit["recent_attempts"]
                if row["kind"] == "climb" and row["fraction"] == 0
            ]
            assert len(neutral) >= 100, (
                "bounded relative-aiding wait did not maintain setpoints"
            )
        else:
            assert "did not confirm LOITER mode" in output
            assert 5.0 <= land - started <= 8.0, land - started
            assert any("GCS entry disabled" in text for text in sensors.status_texts)


@pytest.mark.parametrize("fault", ["unexpected-mode", "unexpected-low-throttle-rc"])
def test_unrequested_mode_or_receiver_change_lands(tmp_path: Path, fault: str):
    overlay = (
        {f"FLTMODE{index}": 5.0 for index in range(1, 7)}
        if fault == "unexpected-low-throttle-rc"
        else {}
    )
    with _fault_case(tmp_path, fault, overlay) as (sensors, audit, output, _offset):
        assert sensors.injections
        injection = sensors.injections[0]["at"]
        land = _land_attempts(audit)[0]["attempted_monotonic"]
        assert 0 <= land - injection <= 3.0, land - injection
        if fault == "unexpected-mode":
            assert "ALT_HOLD" in sensors.mode_transitions(), sensors.mode_transitions()
            assert "Loiter hold left LOITER mode for ALT_HOLD" in output, output
            assert set(sensors.rc_channel_counts) == {0}
        else:
            assert sensors.parameter_readback.get("SIM_RC_FAIL") == 0
            assert any(count > 0 for count in sensors.rc_channel_counts)
            assert any(
                row["count"] > 0 and row["throttle_pwm"] <= 1000
                for row in sensors.rc_observations
            )
            assert "receiver topology changed" in output, output


def test_uncommanded_descent_retains_failure_and_cleanup(tmp_path: Path):
    overlay = {"SIM_ENGINE_FAIL": 0.0, "SIM_ENGINE_MUL": 0.0}
    with _fault_case(tmp_path, "unexpected-descent", overlay) as (
        sensors,
        audit,
        output,
        _offset,
    ):
        assert sensors.injections
        assert sensors.parameter_readback.get("SIM_ENGINE_FAIL") == ALL_MOTORS
        injection = sensors.injections[0]
        assert injection["height"] >= 0.45, injection
        after = [row for row in sensors.truth if row["at"] >= injection["at"]]
        assert any(row["height"] <= injection["height"] - 0.20 for row in after)
        assert max(row["vd"] for row in after) > 0.2
        land = _land_attempts(audit)[0]["attempted_monotonic"]
        assert 0 <= land - injection["at"] <= 10.0, (land, injection, output)
        # Preserve actual diagnostic text (navigation/failsafe/mode/timeout); a
        # motor cut is not proof that the controller diagnosed descent directly.
        assert output.strip()
