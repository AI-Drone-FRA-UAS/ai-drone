from __future__ import annotations

import json
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import closing, contextmanager, suppress
from datetime import datetime
from itertools import pairwise
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
from ai_drone.operator import create_token, presence_server, read_token
from ai_drone.recording import request_message_intervals
from ai_drone.settings import load_settings

ARDUPILOT_COMMIT = "dbe792162d06cab66c3475fd5556bf7a120f119e"
PARAMETERS = Path(__file__).parent / "sitl" / "copter.parm"
TARGET_ALTITUDE_M = 0.5
SENSOR_MAVLINK_PORT = 5762
SENSOR_RATE_HZ = 20.0
RANGE_MIN_CM = 2
RANGE_MAX_CM = 800
FLOW_QUALITY = 60
FORWARD_RANGE_CM = 150
FORWARD_RANGE_PARAMETERS = {
    "RNGFND2_TYPE": 10.0,
    "RNGFND2_ORIENT": 0.0,
    "RNGFND2_MIN": 0.1,
    "RNGFND2_MAX": 15.0,
}

pytestmark = pytest.mark.sitl


@pytest.fixture(autouse=True)
def operator_presence(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    """Run the real authenticated endpoint independently of every CLI process."""
    _ardupilot_root()
    token = tmp_path / "operator.key"
    create_token(token)
    assert token.stat().st_mode & 0o777 == 0o600
    server = presence_server("127.0.0.1", 0, read_token(token))
    thread = threading.Thread(
        target=server.serve_forever,
        kwargs={"poll_interval": 0.05},
        name="sitl-operator-presence",
        daemon=True,
    )
    thread.start()

    def stop():
        if thread.is_alive():
            server.shutdown()
            thread.join(timeout=2)
        server.server_close()
        assert not thread.is_alive(), "operator presence server did not stop"

    # Linux Unix-socket paths are short; pytest's full test-name paths are not.
    with tempfile.TemporaryDirectory(prefix="drone-sitl-") as runtime_directory:
        directory = Path(runtime_directory)
        config = tmp_path / "drone.toml"
        config.write_text(
            "[operator]\n"
            f'endpoints = ["http://127.0.0.1:{server.server_port}"]\n'
            f"token_file = {json.dumps(str(token.resolve()))}\n"
            "interval = 0.2\ntimeout = 2.0\nrequest_timeout = 0.2\n"
            "[runtime]\n"
            f"socket = {json.dumps(str(directory / 'vehicle.sock'))}\n"
            f"status = {json.dumps(str(directory / 'status.json'))}\n"
            'device = "tcp:127.0.0.1:5760"\n'
        )
        monkeypatch.setenv("AI_DRONE_CONFIG", str(config.resolve()))
        try:
            yield stop
        finally:
            stop()


def _python_command(module: str, *arguments: str) -> list[str]:
    return [
        "uv",
        "run",
        "--no-sync",
        "--python",
        sys.executable,
        "python",
        "-m",
        module,
        *arguments,
    ]


def _child_environment() -> dict[str, str]:
    environment = os.environ.copy()
    root = str(Path(__file__).resolve().parents[1])
    previous = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = os.pathsep.join((root, previous)) if previous else root
    environment["PYTHONUNBUFFERED"] = "1"
    return environment


def _cli_pid(process: subprocess.Popen[Any]) -> int:
    """Signal the production Python CLI without terminating its uv supervisor."""
    pending = [process.pid]
    while pending:
        pid = pending.pop()
        try:
            command = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
            if (
                command
                and b"python" in Path(os.fsdecode(command[0])).name.encode()
                and b"ai_drone.cli.main" in command
            ):
                return pid
            for task in Path(f"/proc/{pid}/task").glob("*/children"):
                pending.extend(int(child) for child in task.read_text().split())
        except FileNotFoundError:
            continue
    pytest.fail("production Python CLI is not running under its uv process")


@contextmanager
def _running_cli(tmp_path: Path, name: str, *arguments: str):
    output = tmp_path / f"{name}.log"
    with output.open("w") as handle:
        process = subprocess.Popen(
            _python_command("ai_drone.cli.main", *arguments),
            cwd=tmp_path,
            env=_child_environment(),
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            yield process, output
        finally:
            _stop_process(process)


def _wait_for_cli(predicate, process, output: Path, *, timeout: float = 15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            pytest.fail(
                f"CLI exited with {process.returncode}: {output.read_text()[-4000:]}"
            )
        if predicate():
            return
        time.sleep(0.05)
    pytest.fail(f"CLI did not become ready: {output.read_text()[-4000:]}")


def _ardupilot_root() -> Path:
    value = os.environ.get("ARDUPILOT_ROOT")
    if not value:
        pytest.skip("set ARDUPILOT_ROOT to the pinned ArduPilot checkout")
    _require_loopback_namespace()
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


def _require_loopback_namespace() -> None:
    """No simulator or child process can route commands to a physical aircraft."""
    interfaces = {name for _index, name in socket.if_nameindex()}
    if os.environ.get("AI_DRONE_ISOLATED_SITL") != "1" or interfaces != {"lo"}:
        raise RuntimeError(
            "SITL requires a loopback-only network namespace; use scripts/run_sitl.py"
        )


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


def _stop_process(process: subprocess.Popen[Any]) -> None:
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

    def __init__(self, *, forward_range_enabled: bool = False) -> None:
        self.forward_range_enabled = forward_range_enabled
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
        self.flight_observations: list[tuple[float, str, bool]] = []
        self.altitudes_m: list[float] = []
        self.altitudes_by_mode: list[tuple[str | None, float]] = []
        self.ekf_by_mode: list[tuple[str | None, int]] = []
        self.positions_by_mode: list[tuple[str | None, float, float, float]] = []
        self.rc_channel_counts: list[int] = []
        self.status_texts: list[str] = []
        self.status_by_mode: list[tuple[str | None, str]] = []
        self.range_observations: list[dict[str, Any]] = []
        self.truth: list[dict[str, Any]] = []
        self.attitudes: list[dict[str, float]] = []
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
            self.flight_observations.clear()
            self.altitudes_m.clear()
            self.altitudes_by_mode.clear()
            self.ekf_by_mode.clear()
            self.positions_by_mode.clear()
            self.rc_channel_counts.clear()
            self.status_texts.clear()
            self.status_by_mode.clear()
            self.range_observations.clear()
            self.truth.clear()
            self.attitudes.clear()

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
        if self.forward_range_enabled:
            # Match the MT-15's native ID 0 even though the downward sensor also
            # uses ID 0. ArduPilot must separate backends by orientation and
            # republish the forward reading as instance 1. Its 1.5 m reading
            # deliberately exceeds the production hover's 0.8 m ceiling.
            sender.distance_sensor_send(
                (now_us // 1_000) & 0xFFFFFFFF,
                RANGE_MIN_CM,
                1500,
                FORWARD_RANGE_CM,
                mavlink2.MAV_DISTANCE_SENSOR_LASER,
                0,
                mavlink2.MAV_SENSOR_ROTATION_NONE,
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
            if message_type == "ATTITUDE":
                self.attitudes.append(
                    {"at": time.monotonic(), "yaw": float(message.yaw)}
                )
            elif message_type == "HEARTBEAT":
                mode = _mode_from_heartbeat(message)
                armed = heartbeat_is_armed(message)
                self._current_mode = mode
                self._current_armed = armed
                self.flight_modes.append(mode)
                self.armed_states.append(armed)
                self.flight_states.append((mode, armed))
                self.flight_observations.append((time.monotonic(), mode, armed))
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
            elif message_type == "DISTANCE_SENSOR":
                if (message.get_srcSystem(), message.get_srcComponent()) != (1, 1):
                    return
                self.range_observations.append(
                    {
                        "observed_monotonic": time.monotonic(),
                        "mode": self._current_mode,
                        "source_system": message.get_srcSystem(),
                        "source_component": message.get_srcComponent(),
                        "id": int(message.id),
                        "orientation": int(message.orientation),
                        "distance_cm": int(message.current_distance),
                        "min_cm": int(message.min_distance),
                        "max_cm": int(message.max_distance),
                    }
                )
            elif message_type == "STATUSTEXT":
                text = str(message.text)
                self.status_texts.append(text)
                self.status_by_mode.append((self._current_mode, text))
            self._condition.notify_all()

    def _configure_streams(self, connection: Any) -> Any:
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
        intervals = {
            mavlink2.MAVLINK_MSG_ID_SIM_STATE: SENSOR_RATE_HZ,
            mavlink2.MAVLINK_MSG_ID_HEARTBEAT: 10.0,
            mavlink2.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 20.0,
            mavlink2.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 10.0,
            mavlink2.MAVLINK_MSG_ID_RC_CHANNELS: 10.0,
            mavlink2.MAVLINK_MSG_ID_ATTITUDE: SENSOR_RATE_HZ,
            mavlink2.MAVLINK_MSG_ID_DISTANCE_SENSOR: SENSOR_RATE_HZ,
        }
        request_message_intervals(connection, intervals)
        return mavlink2.MAVLink(
            connection,
            srcSystem=254,
            srcComponent=mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER,
        )

    def _run(self) -> None:
        connection = None
        try:
            connection = self._connect()
            sender = self._configure_streams(connection)

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
                    self.truth.append(
                        {
                            "at": last_state_at,
                            "mode": self._current_mode,
                            "armed": self._current_armed,
                            "height": altitude_m,
                            "yaw": float(state.yaw),
                            "roll": float(state.roll),
                            "pitch": float(state.pitch),
                            "lat": int(state.lat_int),
                            "lon": int(state.lon_int),
                            "vn": float(state.vn),
                            "ve": float(state.ve),
                            "vd": float(state.vd),
                        }
                    )
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


def _assert_sitl_parameters(
    connection: Any,
    *,
    forward_range_enabled: bool = False,
    overlay: dict[str, float] | None = None,
) -> None:
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
    if forward_range_enabled:
        expected.update(FORWARD_RANGE_PARAMETERS)
    if overlay:
        expected.update(overlay)
    assert expected["ARMING_SKIPCHK"] == 0.0
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
def _running_sitl(
    root: Path,
    tmp_path: Path,
    *,
    forward_range_enabled: bool = False,
    heading: float = 353,
    overlay: dict[str, float] | None = None,
    sensor_factory: type[_ExternalMavlinkSensors] = _ExternalMavlinkSensors,
):
    _require_loopback_namespace()
    with closing(socket.socket()) as probe:
        if probe.connect_ex(("127.0.0.1", 5760)) == 0:
            pytest.fail(
                "TCP port 5760 is already in use; required SITL case cannot run"
            )

    log_path = tmp_path / "sitl.log"
    binary = root / "build" / "sitl" / "bin" / "arducopter"
    defaults = root / "Tools" / "autotest" / "default_params" / "copter.parm"
    defaults_paths = [str(defaults), str(PARAMETERS.resolve())]
    if overlay:
        experiment = tmp_path / "experiment.parm"
        experiment.write_text(
            "".join(f"{name},{value:g}\n" for name, value in overlay.items())
        )
        defaults_paths.append(str(experiment))
    if forward_range_enabled:
        forward_defaults = tmp_path / "forward-rangefinder.parm"
        forward_defaults.write_text(
            "".join(
                f"{name},{value:g}\n"
                for name, value in FORWARD_RANGE_PARAMETERS.items()
            )
        )
        defaults_paths.append(str(forward_defaults))
    command = [
        str(binary),
        "-S",
        "-I0",
        "-w",
        "--home",
        f"-35.363261,149.165230,584,{heading:g}",
        "--model",
        "x",
        "--speedup",
        "1",
        "--defaults",
        ",".join(defaults_paths),
    ]
    sensors = sensor_factory(forward_range_enabled=forward_range_enabled)
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


def _assert_running_sitl_configuration(
    sensors: _ExternalMavlinkSensors, *, overlay: dict[str, float] | None = None
) -> None:
    connection = _connect()
    try:
        _assert_sitl_parameters(
            connection,
            forward_range_enabled=sensors.forward_range_enabled,
            overlay=overlay,
        )
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


def _assert_forward_range_coexists(
    sensors: _ExternalMavlinkSensors, tmp_path: Path
) -> None:
    with sensors._condition:
        observations = list(sensors.range_observations)
    (tmp_path / "fc-range-observations.json").write_text(
        json.dumps(observations, indent=2) + "\n"
    )
    assert observations, "no FC-republished range observations"
    assert {
        (sample["source_system"], sample["source_component"]) for sample in observations
    } == {(1, 1)}
    loiter = [sample for sample in observations if sample["mode"] == "LOITER"]
    forward = [sample for sample in loiter if sample["orientation"] == 0]
    downward = [sample for sample in loiter if sample["orientation"] == 25]
    assert forward and downward, "both FC range backends must report during Loiter"
    assert {sample["id"] for sample in forward} == {1}
    assert {sample["id"] for sample in downward} == {0}
    assert {sample["distance_cm"] for sample in forward} == {FORWARD_RANGE_CM}
    assert {sample["min_cm"] for sample in forward} == {10}
    assert {sample["max_cm"] for sample in forward} == {1500}
    assert all(0 < sample["distance_cm"] < 80 for sample in downward)

    # A few startup packets are insufficient: require both FC streams across
    # the five-second Loiter hold, including its beginning and end. This fails
    # if the forward backend stops reporting while the downward-only flight
    # controller continues its otherwise successful hover.
    for samples in (forward, downward):
        times = [sample["observed_monotonic"] for sample in samples]
        assert times[-1] - times[0] >= 3.0, times
        assert max(later - earlier for earlier, later in pairwise(times)) < 1.0
    assert (
        abs(forward[0]["observed_monotonic"] - downward[0]["observed_monotonic"]) < 1.0
    )
    assert (
        abs(forward[-1]["observed_monotonic"] - downward[-1]["observed_monotonic"])
        < 1.0
    )


@pytest.mark.parametrize(
    "forward_range_enabled", [False, True], ids=["downward-only", "with-forward"]
)
def test_production_hover_no_gps_loiter_in_pinned_sitl(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, forward_range_enabled: bool
) -> None:
    root = _ardupilot_root()
    monkeypatch.chdir(tmp_path)
    with _running_sitl(
        root, tmp_path, forward_range_enabled=forward_range_enabled
    ) as sensors:
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
        if forward_range_enabled:
            forward_report = inspection_manifest["components"]["forward_rangefinder"]
            assert forward_report["status"] == "ok"
            assert forward_report["latest_m"] == FORWARD_RANGE_CM / 100.0

        _assert_running_sitl_configuration(sensors)
        sensors.reset_observations()

        # This is the deployable command path.  The test does not reproduce its
        # mode, arm, takeoff, Loiter, or LAND commands with raw MAVLink helpers.
        assert control_cli.main(_production_hover_arguments(duration=5.0)) == 0

        sensors.wait_for_disarm(timeout=5.0)
        result = sensors.assert_flight_result()
        _assert_no_navigation_rejections(sensors)
        assert result["max_horizontal_drift_m"] <= 0.5, result
        if forward_range_enabled:
            _assert_forward_range_coexists(sensors, tmp_path)

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
            f"forward range enabled={forward_range_enabled}, "
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

        command = _python_command(
            "ai_drone.cli.control",
            *_production_hover_arguments(duration=60.0),
        )
        hover_process = subprocess.Popen(
            command,
            cwd=tmp_path,
            env=_child_environment(),
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


def test_shared_recording_survives_hangup_and_operator_loss_lands(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, operator_presence
) -> None:
    """Exercise the production Unix service, recorder and controller together."""
    root = _ardupilot_root()
    monkeypatch.chdir(tmp_path)
    settings = load_settings()
    socket_path = Path(settings.runtime.socket)
    status_path = Path(settings.runtime.status)
    endpoint = f"unix:{socket_path}"
    dataset = tmp_path / "shared-recording"

    def runtime_ready():
        try:
            status = json.loads(status_path.read_text())
            return (
                socket_path.is_socket()
                and status["fresh"]
                and status["armed"] is False
                and status["operator_alive"]
                and status["wifi_required"] is False
            )
        except (OSError, ValueError, KeyError):
            return False

    with _running_sitl(root, tmp_path) as sensors:
        _assert_running_sitl_configuration(sensors)
        sensors.reset_observations()
        with _running_cli(
            tmp_path, "vehicle-runtime", "runtime", "serve", "--no-network"
        ) as (runtime_process, runtime_output):
            _wait_for_cli(runtime_ready, runtime_process, runtime_output)
            with _running_cli(
                tmp_path,
                "passive-recording",
                "record",
                "--device",
                endpoint,
                "--allow-flight",
                "--no-video",
                "--duration",
                "180",
                "--output-dir",
                str(dataset),
            ) as (record_process, record_output):
                _wait_for_cli(
                    lambda: "READY:" in record_output.read_text(),
                    record_process,
                    record_output,
                )
                arguments = _production_hover_arguments(duration=120.0)
                arguments[arguments.index("--device") + 1] = endpoint
                arguments.extend(["--runtime-status", str(status_path), "--foreground"])
                with _running_cli(
                    tmp_path, "shared-control", "control", *arguments
                ) as (control_process, control_output):
                    sensors.wait_for_mode(
                        "LOITER", timeout=60.0, process=control_process
                    )
                    hangup_at = time.monotonic()
                    os.kill(_cli_pid(control_process), signal.SIGHUP)

                    # Observe longer than the operator-loss timeout: closing the
                    # terminal must neither expire presence nor end the flight.
                    while time.monotonic() - hangup_at < settings.operator.timeout + 2:
                        assert control_process.poll() is None, (
                            control_output.read_text()
                        )
                        assert record_process.poll() is None, record_output.read_text()
                        assert runtime_process.poll() is None, (
                            runtime_output.read_text()
                        )
                        sensors.assert_healthy()
                        with sensors._condition:
                            assert sensors._current_armed is True
                            assert sensors._current_mode == "LOITER"
                        time.sleep(0.1)
                    assert json.loads(status_path.read_text())["operator_alive"]

                    operator_lost_at = time.monotonic()
                    operator_presence()
                    sensors.wait_for_mode("LAND", timeout=settings.operator.timeout + 3)
                    sensors.wait_for_disarm(timeout=40.0)
                    assert control_process.wait(timeout=10.0) == 1, (
                        control_output.read_text()
                    )
                    control_exited_utc = time.time()
                    assert "operator heartbeat lost" in control_output.read_text()
                    result = sensors.assert_flight_result()
                    assert result["max_horizontal_drift_m"] <= 0.5, result
                    _assert_no_navigation_rejections(sensors)
                    assert not any(
                        "gcs failsafe" in text.casefold()
                        for text in sensors.status_texts
                    ), sensors.status_texts

                    with sensors._condition:
                        observations = list(sensors.flight_observations)
                    landed_at = next(
                        observed
                        for observed, mode, _armed in observations
                        if mode == "LAND"
                    )
                    latency = landed_at - operator_lost_at
                    assert 0 <= latency <= settings.operator.timeout + 2.5
                    assert any(
                        hangup_at + settings.operator.timeout
                        <= observed
                        < operator_lost_at
                        and mode == "LOITER"
                        and armed
                        for observed, mode, armed in observations
                    )

                    # Controller cleanup closes only its subscription. The
                    # passive recorder must still receive newly arriving data.
                    events = dataset / "telemetry.jsonl"
                    size_at_control_exit = events.stat().st_size

                    def recording_advanced():
                        if events.stat().st_size <= size_at_control_exit:
                            return False
                        text = events.read_text()
                        lines = text.splitlines()
                        if not text.endswith("\n"):
                            lines = lines[:-1]
                        return bool(lines) and (
                            datetime.fromisoformat(
                                json.loads(lines[-1])["timestamp_utc"]
                            ).timestamp()
                            > control_exited_utc
                        )

                    _wait_for_cli(
                        recording_advanced,
                        record_process,
                        record_output,
                        timeout=3,
                    )
                    assert runtime_process.poll() is None
                    os.kill(_cli_pid(record_process), signal.SIGINT)
                    assert record_process.wait(timeout=10.0) == 1, (
                        record_output.read_text()
                    )

                    manifest = json.loads((dataset / "manifest.json").read_text())
                    assert manifest["ended_utc"]
                    assert manifest["error"] == "interrupted by user"
                    assert manifest["armed_abort"] is False
                    assert manifest["safety"]["allow_flight"] is True
                    assert manifest["safety"]["saw_armed"] is True
                    assert manifest["safety"]["saw_disarmed_after_arm"] is True
                    assert manifest["components"]["camera"]["status"] == "unavailable"
                    assert manifest["components"]["flight_controller"]["status"] == "ok"
                    records = [
                        json.loads(line) for line in events.read_text().splitlines()
                    ]
                    assert any(
                        datetime.fromisoformat(record["timestamp_utc"]).timestamp()
                        > control_exited_utc
                        for record in records
                    ), "recorder received no new packet after controller exit"
                    heartbeats = [
                        record
                        for record in records
                        if record["message"] == "HEARTBEAT"
                        and (record["source_system"], record["source_component"])
                        == (1, 1)
                    ]
                    assert any(
                        record["fields"]["base_mode"]
                        & mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        for record in heartbeats
                    )
                    assert (
                        not heartbeats[-1]["fields"]["base_mode"]
                        & mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    assert (dataset / "telemetry.tlog").stat().st_size > 0

                    flight_manifest_paths = list(
                        (tmp_path / "artifacts/flights").glob("*/manifest.json")
                    )
                    assert len(flight_manifest_paths) == 1
                    flight_manifest = json.loads(flight_manifest_paths[0].read_text())
                    assert flight_manifest["completed"] is False
                    assert "operator heartbeat lost" in flight_manifest["error"]
                    evidence = {
                        "hangup_monotonic": hangup_at,
                        "operator_lost_monotonic": operator_lost_at,
                        "land_observed_monotonic": landed_at,
                        "operator_loss_to_land_s": latency,
                        "operator_timeout_s": settings.operator.timeout,
                        "flight_observations": observations,
                        "recorded_heartbeats": len(heartbeats),
                        **result,
                    }
                    (tmp_path / "shared-operator-loss.json").write_text(
                        json.dumps(evidence, indent=2) + "\n"
                    )
                    print(
                        f"shared capture and operator-loss landing: {json.dumps(evidence)}"
                    )


def _set_sitl_parameter(connection: Any, name: str, value: float) -> None:
    connection.mav.param_set_send(
        connection.target_system,
        connection.target_component,
        name.encode("ascii"),
        value,
        mavlink.MAV_PARAM_TYPE_REAL32,
    )
    assert request_parameter(
        connection, name, timeout=5, require_disarmed=False
    ) == pytest.approx(value), name


@contextmanager
def _pilot_radio():
    """Feed the pinned simulator's real UDP receiver, with centered sticks."""
    # AP_RCProtocol_UDP.cpp accepts eight uint16 PWM values on port 5501.
    # The mode switch uses this test's FLTMODE2/4/5 configuration.
    mode_pwm = {"LOITER": 1700, "ALT_HOLD": 1500, "LAND": 1300}
    frame = [b""]

    def set_mode(mode: str) -> None:
        frame[0] = struct.pack(
            "<8H", 1500, 1500, 1500, 1500, mode_pwm[mode], 1000, 1000, 1800
        )

    set_mode("LOITER")
    stop = threading.Event()
    failures: list[Exception] = []
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as connection:

        def transmit() -> None:
            try:
                while not stop.is_set():
                    connection.sendto(frame[0], ("127.0.0.1", 5501))
                    stop.wait(0.02)
            except Exception as error:
                failures.append(error)

        thread = threading.Thread(target=transmit, name="sitl-pilot-radio", daemon=True)
        thread.start()
        try:
            yield set_mode
        finally:
            stop.set()
            thread.join(timeout=2)
            assert not thread.is_alive(), "SITL pilot radio did not stop"
            assert not failures, failures


def test_pilot_handoff_preserves_control_after_operator_loss(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, operator_presence
) -> None:
    """A real receiver owns the flight after explicit production handoff."""
    root = _ardupilot_root()
    monkeypatch.chdir(tmp_path)
    settings = load_settings()
    socket_path = Path(settings.runtime.socket)
    status_path = Path(settings.runtime.status)

    def runtime_ready():
        try:
            status = json.loads(status_path.read_text())
            return (
                socket_path.is_socket()
                and status["fresh"]
                and status["armed"] is False
                and status["operator_alive"]
            )
        except (OSError, ValueError, KeyError):
            return False

    def flight_events():
        paths = list((tmp_path / "artifacts/flights").glob("*/events.jsonl"))
        if not paths:
            return []
        assert len(paths) == 1
        text = paths[0].read_text()
        lines = text.splitlines()
        if not text.endswith("\n"):
            lines = lines[:-1]
        return [json.loads(line) for line in lines]

    with _running_sitl(root, tmp_path) as sensors:
        _assert_running_sitl_configuration(sensors)
        # SERIAL2 is independent of SERIAL0's production runtime and SERIAL1's
        # sensors. This link sends parameter commands only, never GCS heartbeats.
        with closing(
            mavutil.mavlink_connection(
                "tcp:127.0.0.1:5763",
                source_system=253,
                source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER,
            )
        ) as pilot:
            heartbeat = pilot.wait_heartbeat(timeout=10)
            assert heartbeat is not None, "no heartbeat on SITL SERIAL2"
            pilot.target_system = heartbeat.get_srcSystem()
            pilot.target_component = heartbeat.get_srcComponent()
            for name, value in {
                "FLTMODE2": 9,  # radio switch -> LAND
                "FLTMODE4": 2,  # radio switch -> ALT_HOLD
                "FLTMODE5": 5,  # centered receiver starts in LOITER
                "FLTMODE6": 5,  # SITL's first default receiver frame is also LOITER
            }.items():
                _set_sitl_parameter(pilot, name, value)
            gcs_timeout = request_parameter(pilot, "FS_GCS_TIMEOUT")
            sensors.reset_observations()
            with _running_cli(
                tmp_path, "pilot-runtime", "runtime", "serve", "--no-network"
            ) as (runtime_process, runtime_output):
                _wait_for_cli(runtime_ready, runtime_process, runtime_output)
                arguments = _production_hover_arguments(duration=120)
                arguments[arguments.index("--device") + 1] = f"unix:{socket_path}"
                arguments.extend(["--runtime-status", str(status_path), "--foreground"])
                with _running_cli(tmp_path, "pilot-control", "control", *arguments) as (
                    control_process,
                    control_output,
                ):
                    sensors.wait_for_mode("LOITER", timeout=60, process=control_process)
                    _wait_for_cli(
                        lambda: any(
                            event["event"] == "loiter_started"
                            for event in flight_events()
                        ),
                        control_process,
                        control_output,
                    )
                    with sensors._condition:
                        assert sensors.rc_channel_counts
                        assert set(sensors.rc_channel_counts) == {0}
                    requested_at = time.monotonic()
                    request = subprocess.run(
                        _python_command("ai_drone.cli.main", "control", "handoff"),
                        cwd=tmp_path,
                        env=_child_environment(),
                        capture_output=True,
                        text=True,
                        timeout=10,
                    )
                    assert request.returncode == 0, request.stdout + request.stderr
                    _wait_for_cli(
                        lambda: json.loads(status_path.read_text())["human_requested"],
                        control_process,
                        control_output,
                    )

                    # LOITER is already a pilot mode. Request first, then enable
                    # real RC; fresh RC must confirm ownership before changing mode.
                    with _pilot_radio() as pilot_mode:
                        _set_sitl_parameter(pilot, "SIM_RC_FAIL", 0)
                        _wait_for_cli(
                            lambda: any(
                                event["event"] == "control_owner"
                                and event.get("owner") == "human"
                                for event in flight_events()
                            ),
                            control_process,
                            control_output,
                            timeout=5,
                        )
                        handed_off_at = time.monotonic()
                        pilot_mode("ALT_HOLD")
                        sensors.wait_for_mode(
                            "ALT_HOLD", timeout=5, process=control_process
                        )
                        operator_lost_at = time.monotonic()
                        operator_presence()
                        # Exceed both loss deadlines. Only the production
                        # controller supplies the FC's GCS heartbeat here.
                        hold_seconds = max(settings.operator.timeout, gcs_timeout) + 2
                        while time.monotonic() - operator_lost_at < hold_seconds:
                            assert control_process.poll() is None, (
                                control_output.read_text()
                            )
                            assert runtime_process.poll() is None, (
                                runtime_output.read_text()
                            )
                            sensors.assert_healthy()
                            with sensors._condition:
                                assert sensors._current_armed is True
                                assert sensors._current_mode == "ALT_HOLD"
                            time.sleep(0.1)
                        assert not json.loads(status_path.read_text())["operator_alive"]
                        pilot_land_at = time.monotonic()
                        pilot_mode("LAND")
                        sensors.wait_for_mode(
                            "LAND", timeout=5, process=control_process
                        )
                        sensors.wait_for_disarm(timeout=40)
                        assert control_process.wait(timeout=10) == 0, (
                            control_output.read_text()
                        )

                    observations = list(sensors.flight_observations)
                    assert all(
                        mode != "LAND"
                        for observed, mode, _armed in observations
                        if requested_at <= observed < pilot_land_at
                    ), observations
                    _assert_subsequence(
                        sensors.mode_transitions(),
                        ["GUIDED_NOGPS", "LOITER", "ALT_HOLD", "LAND"],
                    )
                    assert any(count > 0 for count in sensors.rc_channel_counts)
                    assert 0.45 <= max(sensors.altitudes_m) < 0.8
                    pilot_altitudes = [
                        altitude
                        for mode, altitude in sensors.altitudes_by_mode
                        if mode == "ALT_HOLD"
                    ]
                    assert pilot_altitudes and min(pilot_altitudes) >= 0.3
                    _assert_no_navigation_rejections(sensors)
                    assert not any(
                        "gcs failsafe" in text.casefold()
                        for text in sensors.status_texts
                    ), sensors.status_texts
                    assert "operator heartbeat lost" not in control_output.read_text()
                    events = flight_events()
                    owners = [
                        event for event in events if event["event"] == "control_owner"
                    ]
                    assert [event["owner"] for event in owners] == [
                        "autonomous",
                        "human",
                    ]
                    assert owners[-1]["mode"] == "LOITER"
                    assert owners[-1]["rc_channels"] > 0
                    assert any(event["event"] == "pilot_disarmed" for event in events)
                    assert not any(
                        event["event"] == "landing_started" for event in events
                    )
                    manifests = list(
                        (tmp_path / "artifacts/flights").glob("*/manifest.json")
                    )
                    assert len(manifests) == 1
                    manifest = json.loads(manifests[0].read_text())
                    assert manifest["completed"] is True
                    assert manifest["error"] is None
                    assert manifest["ended_utc"]
                    assert (manifests[0].parent / "telemetry.tlog").stat().st_size > 0
                    evidence = {
                        "handoff_requested_monotonic": requested_at,
                        "handoff_confirmed_monotonic": handed_off_at,
                        "operator_lost_monotonic": operator_lost_at,
                        "pilot_land_monotonic": pilot_land_at,
                        "operator_timeout_s": settings.operator.timeout,
                        "gcs_timeout_s": gcs_timeout,
                        "operator_absent_hold_s": pilot_land_at - operator_lost_at,
                        "flight_observations": observations,
                        "control_owners": owners,
                        "maximum_altitude_m": max(sensors.altitudes_m),
                        "minimum_pilot_altitude_m": min(pilot_altitudes),
                    }
                    (tmp_path / "pilot-handoff.json").write_text(
                        json.dumps(evidence, indent=2) + "\n"
                    )
                    print(
                        f"pilot handoff survives operator loss: {json.dumps(evidence)}"
                    )
