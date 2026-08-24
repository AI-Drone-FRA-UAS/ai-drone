from __future__ import annotations

import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from contextlib import closing, contextmanager, suppress
from pathlib import Path
from typing import Any

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from ai_drone.cli import control as control_cli
from ai_drone.cli import record as inspect_cli
from ai_drone.mavlink.parameters import request_parameter
from ai_drone.mavlink.safety import heartbeat_is_armed
from ai_drone.recording import request_message_intervals

ARDUPILOT_COMMIT = "1511f27194f1dcc3728270883047bdf022b3fd53"
PARAMETERS = Path(__file__).parent / "sitl" / "copter.parm"
TARGET_ALTITUDE_M = 0.5
SENSOR_MAVLINK_PORT = 5762
SENSOR_RATE_HZ = 20.0
RANGE_MIN_CM = 2
RANGE_MAX_CM = 800
FLOW_QUALITY = 60

pytestmark = pytest.mark.sitl


def _ardupilot_root() -> Path:
    value = os.environ.get("ARDUPILOT_ROOT")
    if not value:
        pytest.skip("set ARDUPILOT_ROOT to the pinned ArduPilot checkout")
    root = Path(value).expanduser().resolve()
    binary = root / "build" / "sitl" / "bin" / "arducopter"
    if not binary.is_file():
        pytest.fail(
            "build ArduCopter SITL in the external checkout before running this test"
        )
    head = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True
    ).strip()
    if head != ARDUPILOT_COMMIT:
        pytest.fail(f"ARDUPILOT_ROOT must be checked out at {ARDUPILOT_COMMIT}")
    return root


def _wait_for_tcp(process: subprocess.Popen[bytes], timeout: float = 30.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            pytest.fail(f"SITL exited early with status {process.returncode}")
        with (
            suppress(OSError),
            closing(socket.create_connection(("127.0.0.1", 5760), timeout=0.25)),
        ):
            return
        time.sleep(0.25)
    pytest.fail("SITL did not listen on TCP port 5760 within 30 seconds")


def _stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def _connect() -> Any:
    connection = mavutil.mavlink_connection(
        "tcp:127.0.0.1:5760",
        source_system=255,
        source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )
    heartbeat = connection.wait_heartbeat(timeout=15)
    if heartbeat is None:
        pytest.fail("SITL did not send a heartbeat")
    connection.target_system = heartbeat.get_srcSystem()
    connection.target_component = heartbeat.get_srcComponent()
    return connection


class _ExternalMavlinkSensors:
    """Inject project-style MAVLink range and flow from SITL ground truth."""

    def __init__(self) -> None:
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="sitl-mavlink-sensors",
            daemon=True,
        )
        self._failure: BaseException | None = None
        self._condition = threading.Condition()
        self.sample_count = 0
        self.wire_protocol: str | None = None
        self.flight_modes: list[str] = []
        self.armed_states: list[bool] = []
        self.flight_states: list[tuple[str, bool]] = []
        self.altitudes_m: list[float] = []
        self.altitudes_by_mode: list[tuple[str | None, float]] = []
        self.ekf_by_mode: list[tuple[str | None, int]] = []
        self.positions_by_mode: list[tuple[str | None, float, float, float]] = []
        self.rc_channel_counts: list[int] = []
        self.status_texts: list[str] = []
        self.status_by_mode: list[tuple[str | None, str]] = []
        self._current_mode: str | None = None
        self._current_armed: bool | None = None

    def start(self, timeout: float = 15.0) -> None:
        self._thread.start()
        if not self._ready.wait(timeout) or self._failure is not None:
            self.stop(check=False)
            detail = "" if self._failure is None else f": {self._failure!r}"
            pytest.fail(f"external MAVLink sensors did not start{detail}")

    def stop(self, *, check: bool = True) -> None:
        self._stop.set()
        if self._thread.ident is None:
            return
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            pytest.fail("external MAVLink sensor feeder did not stop")
        if check and self._failure is not None:
            pytest.fail(f"external MAVLink sensor feeder failed: {self._failure!r}")

    def assert_healthy(self) -> None:
        if self._failure is not None:
            pytest.fail(f"external MAVLink sensor feeder failed: {self._failure!r}")
        assert self._thread.is_alive()
        assert self.sample_count > 0
        assert self.wire_protocol == "2.0"

    def reset_observations(self) -> None:
        """Start a fresh observation window without interrupting sensor injection."""

        with self._condition:
            self.flight_modes.clear()
            self.armed_states.clear()
            self.flight_states.clear()
            self.altitudes_m.clear()
            self.altitudes_by_mode.clear()
            self.ekf_by_mode.clear()
            self.positions_by_mode.clear()
            self.rc_channel_counts.clear()
            self.status_texts.clear()
            self.status_by_mode.clear()

    def wait_for_mode(
        self,
        requested: str,
        timeout: float,
        process: subprocess.Popen[Any] | None = None,
    ) -> None:
        requested = requested.upper()
        deadline = time.monotonic() + timeout
        with self._condition:
            while requested not in self.flight_modes:
                if self._failure is not None:
                    pytest.fail(
                        f"external MAVLink sensor feeder failed: {self._failure!r}"
                    )
                if process is not None and process.poll() is not None:
                    output = ""
                    if process.stdout is not None:
                        output = process.stdout.read()
                    pytest.fail(
                        f"production hover exited before {requested}: "
                        f"status={process.returncode}, output={output[-4000:]}"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    pytest.fail(
                        f"SITL did not enter {requested}; modes={self.mode_transitions()}, "
                        f"status={self.status_texts[-10:]}"
                    )
                self._condition.wait(timeout=min(remaining, 0.2))

    def wait_for_disarm(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not (True in self.armed_states and self._current_armed is False):
                if self._failure is not None:
                    pytest.fail(
                        f"external MAVLink sensor feeder failed: {self._failure!r}"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    pytest.fail(
                        "SITL did not disarm after flight; "
                        f"armed={self._current_armed}, modes={self.mode_transitions()}, "
                        f"status={self.status_texts[-10:]}"
                    )
                self._condition.wait(timeout=min(remaining, 0.2))

    def mode_transitions(self) -> list[str]:
        transitions: list[str] = []
        for mode in self.flight_modes:
            if not transitions or transitions[-1] != mode:
                transitions.append(mode)
        return transitions

    def assert_flight_result(self) -> dict[str, float]:
        """Validate externally observed GuidedNoGPS/Loiter/Land flight."""

        self.assert_healthy()
        transitions = self.mode_transitions()
        _assert_subsequence(transitions, ["GUIDED_NOGPS", "LOITER", "LAND"])
        assert True in self.armed_states, "production hover never armed SITL"
        assert self._current_armed is False, "production hover did not finish disarmed"
        loiter_armed = [armed for mode, armed in self.flight_states if mode == "LOITER"]
        assert loiter_armed and all(loiter_armed), (
            "vehicle disarmed before leaving Loiter",
            self.flight_states,
        )
        assert self.altitudes_m, "SITL produced no simulator altitude samples"
        assert max(self.altitudes_m) < 0.8, max(self.altitudes_m)
        assert max(self.altitudes_m) >= TARGET_ALTITUDE_M * 0.9, max(self.altitudes_m)
        loiter_altitudes = [
            altitude for mode, altitude in self.altitudes_by_mode if mode == "LOITER"
        ]
        assert loiter_altitudes, "Loiter produced no simulator altitude samples"
        assert min(loiter_altitudes) >= 0.3, (
            "Loiter descended below its bounded hold envelope",
            min(loiter_altitudes),
        )

        loiter_ekf = [flags for mode, flags in self.ekf_by_mode if mode == "LOITER"]
        assert loiter_ekf, "Loiter produced no externally observed EKF status"
        assert all(flags & mavlink.EKF_POS_HORIZ_REL for flags in loiter_ekf)
        assert all(not flags & mavlink.EKF_POS_HORIZ_ABS for flags in loiter_ekf)
        assert all(not flags & mavlink.EKF_CONST_POS_MODE for flags in loiter_ekf)
        assert self.rc_channel_counts, "SITL produced no RC_CHANNELS topology samples"
        assert set(self.rc_channel_counts) == {0}, (
            "the project topology requires no active RC receiver channels",
            sorted(set(self.rc_channel_counts)),
        )

        loiter_positions = [
            (x, y, altitude)
            for mode, x, y, altitude in self.positions_by_mode
            if mode == "LOITER"
        ]
        assert loiter_positions, "Loiter produced no local-position samples"
        start_x, start_y, _ = loiter_positions[0]
        max_drift = max(
            math.hypot(x - start_x, y - start_y) for x, y, _altitude in loiter_positions
        )
        return {
            "max_horizontal_drift_m": max_drift,
            "maximum_altitude_m": max(self.altitudes_m),
            "minimum_loiter_altitude_m": min(loiter_altitudes),
        }

    def _connect(self, timeout: float = 10.0) -> Any:
        deadline = time.monotonic() + timeout
        while not self._stop.is_set():
            try:
                return mavutil.mavlink_connection(
                    f"tcp:127.0.0.1:{SENSOR_MAVLINK_PORT}",
                    source_system=254,
                    source_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
                )
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                self._stop.wait(0.1)
        raise RuntimeError("sensor feeder stopped before connecting")

    @staticmethod
    def _sensor_values(state: Any, ground_altitude_m: float) -> tuple[float, ...]:
        roll = float(state.roll)
        pitch = float(state.pitch)
        yaw = float(state.yaw)
        sin_roll, cos_roll = math.sin(roll), math.cos(roll)
        sin_pitch, cos_pitch = math.sin(pitch), math.cos(pitch)
        sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)

        # This is the transpose of ArduPilot's body-to-NED DCM. It deliberately
        # mirrors AP_OpticalFlow_SITL instead of using the EKF's estimated state.
        velocity_body_x = (
            cos_pitch * cos_yaw * float(state.vn)
            + cos_pitch * sin_yaw * float(state.ve)
            - sin_pitch * float(state.vd)
        )
        velocity_body_y = (
            (sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw) * float(state.vn)
            + (sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw) * float(state.ve)
            + sin_roll * cos_pitch * float(state.vd)
        )

        height_agl_m = max(0.0, float(state.alt) - ground_altitude_m)
        downward_cosine = cos_roll * cos_pitch
        if height_agl_m > 0.0 and downward_cosine > 0.05:
            range_m = height_agl_m / downward_cosine
            translational_flow_x = -velocity_body_y / range_m
            translational_flow_y = velocity_body_x / range_m
        else:
            range_m = 0.0
            translational_flow_x = 0.0
            translational_flow_y = 0.0

        flow_rate_x = translational_flow_x + float(state.xgyro)
        flow_rate_y = translational_flow_y + float(state.ygyro)
        return range_m, flow_rate_x, flow_rate_y

    def _send_sensors(
        self,
        sender: Any,
        state: Any,
        ground_altitude_m: float,
    ) -> None:
        range_m, flow_rate_x, flow_rate_y = self._sensor_values(
            state, ground_altitude_m
        )
        now_us = time.monotonic_ns() // 1_000
        current_distance_cm = min(
            0xFFFF,
            max(RANGE_MIN_CM, round(range_m * 100.0)),
        )
        sender.distance_sensor_send(
            (now_us // 1_000) & 0xFFFFFFFF,
            RANGE_MIN_CM,
            RANGE_MAX_CM,
            current_distance_cm,
            mavlink2.MAV_DISTANCE_SENSOR_LASER,
            0,
            mavlink2.MAV_SENSOR_ROTATION_PITCH_270,
            0,
            0.0,
            0.0,
            (0.0, 0.0, 0.0, 0.0),
            0,
        )

        # A non-zero extension in the first packet selects ArduPilot 4.7's
        # high-precision rad/s path even when the simulated vehicle is still.
        if self.sample_count == 0 and flow_rate_x == 0.0 and flow_rate_y == 0.0:
            flow_rate_x = 1e-6
        sender.optical_flow_send(
            now_us,
            0,
            int(flow_rate_x),
            int(flow_rate_y),
            0.0,
            0.0,
            FLOW_QUALITY,
            range_m,
            flow_rate_x,
            flow_rate_y,
        )
        self.sample_count += 1

    def _observe_vehicle_message(self, message: Any) -> None:
        message_type = message.get_type()
        with self._condition:
            if message_type == "HEARTBEAT":
                mode = _mode_from_heartbeat(message)
                armed = heartbeat_is_armed(message)
                self._current_mode = mode
                self._current_armed = armed
                self.flight_modes.append(mode)
                self.armed_states.append(armed)
                self.flight_states.append((mode, armed))
            elif message_type == "EKF_STATUS_REPORT":
                self.ekf_by_mode.append((self._current_mode, int(message.flags)))
            elif message_type == "LOCAL_POSITION_NED":
                self.positions_by_mode.append(
                    (
                        self._current_mode,
                        float(message.x),
                        float(message.y),
                        -float(message.z),
                    )
                )
            elif message_type == "RC_CHANNELS":
                self.rc_channel_counts.append(int(message.chancount))
            elif message_type == "STATUSTEXT":
                text = str(message.text)
                self.status_texts.append(text)
                self.status_by_mode.append((self._current_mode, text))
            self._condition.notify_all()

    def _run(self) -> None:
        connection = None
        try:
            connection = self._connect()
            heartbeat = connection.wait_heartbeat(timeout=10)
            if heartbeat is None:
                raise TimeoutError("no heartbeat on SITL SERIAL1")
            self.wire_protocol = str(connection.WIRE_PROTOCOL_VERSION)
            if self.wire_protocol != "2.0":
                raise RuntimeError(
                    f"SITL SERIAL1 negotiated MAVLink {self.wire_protocol}, not 2.0"
                )
            connection.target_system = heartbeat.get_srcSystem()
            connection.target_component = heartbeat.get_srcComponent()
            request_message_intervals(
                connection,
                {
                    mavlink2.MAVLINK_MSG_ID_SIM_STATE: SENSOR_RATE_HZ,
                    mavlink2.MAVLINK_MSG_ID_HEARTBEAT: 10.0,
                    mavlink2.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 20.0,
                    mavlink2.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 10.0,
                    mavlink2.MAVLINK_MSG_ID_RC_CHANNELS: 10.0,
                },
            )
            sender = mavlink2.MAVLink(
                connection,
                srcSystem=254,
                srcComponent=mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER,
            )

            ground_altitude_m = None
            last_state_at = time.monotonic()
            while not self._stop.is_set():
                message = connection.recv_match(blocking=True, timeout=0.2)
                if message is None:
                    if self._ready.is_set() and time.monotonic() - last_state_at > 1.0:
                        raise TimeoutError("SIM_STATE stream stopped")
                    continue
                if message.get_srcSystem() != connection.target_system:
                    continue
                if message.get_type() != "SIM_STATE":
                    self._observe_vehicle_message(message)
                    continue
                state = message
                last_state_at = time.monotonic()
                if ground_altitude_m is None:
                    ground_altitude_m = float(state.alt)
                with self._condition:
                    altitude_m = max(0.0, float(state.alt) - ground_altitude_m)
                    self.altitudes_m.append(altitude_m)
                    self.altitudes_by_mode.append((self._current_mode, altitude_m))
                    self._condition.notify_all()
                self._send_sensors(sender, state, ground_altitude_m)
                self._ready.set()
        except BaseException as exc:
            self._failure = exc
            self._ready.set()
        finally:
            if connection is not None:
                connection.close()


def _mode_from_heartbeat(heartbeat: Any) -> str:
    return str(mavutil.mode_string_v10(heartbeat)).upper()


def _assert_subsequence(actual: list[str], expected: list[str]) -> None:
    next_index = 0
    for item in actual:
        if next_index < len(expected) and item == expected[next_index]:
            next_index += 1
    assert next_index == len(expected), f"expected {expected} in mode history {actual}"


def _assert_sitl_parameters(connection: Any) -> None:
    expected = {
        "AHRS_EKF_TYPE": 3.0,
        "AHRS_OPTIONS": 16.0,
        "ARMING_NEED_LOC": 0.0,
        "ARMING_SKIPCHK": 0.0,
        "AVOID_ENABLE": 2.0,
        "EK3_ENABLE": 1.0,
        "EK3_FLOW_USE": 1.0,
        "EK3_SRC1_POSXY": 0.0,
        "EK3_SRC1_POSZ": 1.0,
        "EK3_SRC1_VELXY": 5.0,
        "EK3_SRC1_VELZ": 0.0,
        "EK3_SRC1_YAW": 1.0,
        "EK3_SRC_OPTIONS": 0.0,
        "FLOW_TYPE": 5.0,
        "FRAME_CLASS": 1.0,
        "FRAME_TYPE": 1.0,
        "FS_CRASH_CHECK": 1.0,
        "FS_DR_ENABLE": 1.0,
        "FS_EKF_ACTION": 1.0,
        "FS_EKF_THRESH": 0.8,
        "FS_GCS_ENABLE": 5.0,
        "FS_GCS_TIMEOUT": 5.0,
        "FS_OPTIONS": 8.0,
        "FS_THR_ENABLE": 0.0,
        "FS_VIBE_ENABLE": 1.0,
        "GPS1_TYPE": 0.0,
        "GPS2_TYPE": 0.0,
        "GUID_OPTIONS": 0.0,
        "LAND_SPD_MS": 0.15,
        "MAV_GCS_SYSID": 255.0,
        "RNGFND1_MAX": 1.0,
        "RNGFND1_MIN": 0.0,
        "RNGFND1_ORIENT": 25.0,
        "RNGFND1_TYPE": 10.0,
        "RNGFND2_TYPE": 0.0,
        "SERIAL1_PROTOCOL": 2.0,
        "SIM_FLOW_ENABLE": 0.0,
        "SIM_GPS1_ENABLE": 0.0,
        "SIM_RC_FAIL": 1.0,
        "WP_SPD_UP": 0.25,
    }
    actual = {
        name: request_parameter(connection, name, timeout=5.0) for name in expected
    }
    mismatches = {
        name: {"expected": value, "actual": actual[name]}
        for name, value in expected.items()
        if not math.isclose(actual[name], value, rel_tol=0.0, abs_tol=1e-5)
    }
    assert not mismatches, mismatches


@contextmanager
def _running_sitl(root: Path, tmp_path: Path):
    with closing(socket.socket()) as probe:
        if probe.connect_ex(("127.0.0.1", 5760)) == 0:
            pytest.skip("TCP port 5760 is already in use")

    log_path = tmp_path / "sitl.log"
    binary = root / "build" / "sitl" / "bin" / "arducopter"
    defaults = root / "Tools" / "autotest" / "default_params" / "copter.parm"
    command = [
        str(binary),
        "-S",
        "-I0",
        "-w",
        "--home",
        "-35.363261,149.165230,584,353",
        "--model",
        "x",
        "--speedup",
        "1",
        "--defaults",
        f"{defaults},{PARAMETERS.resolve()}",
    ]
    sensors = _ExternalMavlinkSensors()
    sensors_started = False
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            command,
            cwd=tmp_path,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            _wait_for_tcp(process)
            # A full SERIAL0 heartbeat lets ArduPilot finish initializing the
            # other standard SITL MAVLink links before the feeder uses SERIAL1.
            bootstrap = _connect()
            bootstrap.close()
            sensors.start()
            sensors_started = True
            yield sensors
        finally:
            try:
                if sensors_started:
                    sensors.stop()
            finally:
                _stop_process(process)


def _assert_running_sitl_configuration(sensors: _ExternalMavlinkSensors) -> None:
    connection = _connect()
    try:
        _assert_sitl_parameters(connection)
        sensors.assert_healthy()
        request_message_intervals(
            connection,
            {mavlink.MAVLINK_MSG_ID_SYS_STATUS: 5.0},
        )
        deadline = time.monotonic() + 45.0
        last_health = 0
        next_prearm_request = 0.0
        status_texts: list[str] = []
        while (remaining := deadline - time.monotonic()) > 0:
            now = time.monotonic()
            if now >= next_prearm_request:
                connection.mav.command_long_send(
                    connection.target_system,
                    connection.target_component,
                    mavlink.MAV_CMD_RUN_PREARM_CHECKS,
                    0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
                next_prearm_request = now + 5.0
            message = connection.recv_match(
                type=["SYS_STATUS", "STATUSTEXT"],
                blocking=True,
                timeout=min(remaining, 0.5),
            )
            if message is None:
                continue
            if message.get_type() == "STATUSTEXT":
                text = str(message.text)
                if text not in status_texts:
                    status_texts.append(text)
                continue
            last_health = int(message.onboard_control_sensors_health)
            if last_health & mavlink.MAV_SYS_STATUS_PREARM_CHECK:
                break
        else:
            pytest.fail(
                f"SITL pre-arm checks did not settle; health={last_health:#x}, "
                f"status={status_texts[-15:]}"
            )
    finally:
        connection.close()


def _production_hover_arguments(duration: float) -> list[str]:
    return [
        "hover",
        "--device",
        "tcp:127.0.0.1:5760",
        "--max-alt",
        "0.8",
        "--takeoff-alt",
        str(TARGET_ALTITUDE_M),
        "--duration",
        str(duration),
        "--min-battery",
        "0",
        "--confirm-flight",
        control_cli.FLIGHT_CONFIRMATION,
    ]


def _assert_no_navigation_rejections(sensors: _ExternalMavlinkSensors) -> None:
    rejected = []
    for mode, status in sensors.status_by_mode:
        normalized = status.casefold()
        if any(
            text in normalized
            for text in (
                "stopped aiding",
                "ekf failsafe",
                "requires position",
                "mode change to loiter failed",
            )
        ) or ("sim hit ground" in normalized and mode != "LAND"):
            rejected.append(f"{mode}: {status}")
    assert not rejected, rejected


def test_production_hover_no_gps_loiter_in_pinned_sitl(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = _ardupilot_root()
    monkeypatch.chdir(tmp_path)
    with _running_sitl(root, tmp_path) as sensors:
        # Prove that the same sensor streams consumed by production code remain
        # available through the read-only inspection command before flight.
        inspection = tmp_path / "inspection"
        assert (
            inspect_cli.run(
                [
                    "--device",
                    "tcp:127.0.0.1:5760",
                    "--duration",
                    "5",
                    "--output-dir",
                    str(inspection),
                ]
            )
            == 0
        )
        inspection_manifest = json.loads((inspection / "manifest.json").read_text())
        assert inspection_manifest["components"]["camera"]["status"] == "unavailable"
        assert inspection_manifest["components"]["flight_controller"]["status"] == "ok"
        assert (
            inspection_manifest["components"]["downward_rangefinder"]["status"] == "ok"
        )
        assert inspection_manifest["components"]["optical_flow"]["status"] == "ok"
        assert (
            inspection_manifest["components"]["optical_flow"]["quality"] == FLOW_QUALITY
        )

        _assert_running_sitl_configuration(sensors)
        sensors.reset_observations()

        # This is the deployable command path.  The test does not reproduce its
        # mode, arm, takeoff, Loiter, or LAND commands with raw MAVLink helpers.
        assert control_cli.main(_production_hover_arguments(duration=5.0)) == 0

        sensors.wait_for_disarm(timeout=5.0)
        result = sensors.assert_flight_result()
        _assert_no_navigation_rejections(sensors)
        assert result["max_horizontal_drift_m"] <= 0.5, result

        flight_manifests = list(
            (tmp_path / "artifacts" / "flights").glob("*/manifest.json")
        )
        assert len(flight_manifests) == 1
        flight_manifest = json.loads(flight_manifests[0].read_text())
        assert flight_manifest["completed"] is True
        assert flight_manifest["metadata"]["command"] == "hover"
        print(
            "production no-GPS Loiter: "
            f"modes={sensors.mode_transitions()}, "
            f"max XY drift={result['max_horizontal_drift_m']:.3f} m, "
            f"Loiter min={result['minimum_loiter_altitude_m']:.3f} m, "
            f"max altitude={result['maximum_altitude_m']:.3f} m, "
            f"RC chancount={sorted(set(sensors.rc_channel_counts))}, "
            f"external MAVLink {sensors.wire_protocol} "
            f"sensor samples={sensors.sample_count}"
        )


def test_gcs_heartbeat_loss_in_loiter_lands_and_disarms(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = _ardupilot_root()
    monkeypatch.chdir(tmp_path)
    with _running_sitl(root, tmp_path) as sensors:
        _assert_running_sitl_configuration(sensors)
        sensors.reset_observations()

        repository_root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        prior_pythonpath = environment.get("PYTHONPATH")
        environment["PYTHONPATH"] = (
            str(repository_root)
            if not prior_pythonpath
            else os.pathsep.join((str(repository_root), prior_pythonpath))
        )
        command = [
            sys.executable,
            "-m",
            "ai_drone.cli.control",
            *_production_hover_arguments(duration=60.0),
        ]
        hover_process = subprocess.Popen(
            command,
            cwd=tmp_path,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            sensors.wait_for_mode("LOITER", timeout=60.0, process=hover_process)

            # SIGKILL deliberately bypasses the production cleanup path.  Only
            # ArduCopter's system-255 heartbeat failsafe can recover this flight.
            os.killpg(hover_process.pid, signal.SIGKILL)
            return_code = hover_process.wait(timeout=5.0)
            assert return_code == -signal.SIGKILL

            sensors.wait_for_mode("LAND", timeout=15.0)
            sensors.wait_for_disarm(timeout=40.0)
            result = sensors.assert_flight_result()
            _assert_no_navigation_rejections(sensors)
            assert result["max_horizontal_drift_m"] <= 0.5, result
            assert any(
                "gcs failsafe" in status.casefold() for status in sensors.status_texts
            ), sensors.status_texts
            print(
                "GCS-loss no-GPS Loiter recovery: "
                f"modes={sensors.mode_transitions()}, "
                f"max XY drift={result['max_horizontal_drift_m']:.3f} m, "
                f"Loiter min={result['minimum_loiter_altitude_m']:.3f} m, "
                f"max altitude={result['maximum_altitude_m']:.3f} m, "
                f"RC chancount={sorted(set(sensors.rc_channel_counts))}, "
                f"status={sensors.status_texts[-5:]}"
            )
        finally:
            if hover_process.poll() is None:
                os.killpg(hover_process.pid, signal.SIGKILL)
                hover_process.wait(timeout=5.0)
