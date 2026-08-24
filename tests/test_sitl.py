from __future__ import annotations

import json
import math
import os
import signal
import socket
import subprocess
import threading
import time
from contextlib import closing, suppress
from pathlib import Path
from typing import Any

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from ai_drone.cli import record as inspect_cli
from ai_drone.mavlink.parameters import request_parameter
from ai_drone.mavlink.safety import heartbeat_is_armed
from ai_drone.recording import request_message_intervals

ARDUPILOT_COMMIT = "1511f27194f1dcc3728270883047bdf022b3fd53"
PARAMETERS = Path(__file__).parent / "sitl" / "copter.parm"
TARGET_ALTITUDE_M = 1.0
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
        self.sample_count = 0
        self.wire_protocol: str | None = None

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
                {mavlink2.MAVLINK_MSG_ID_SIM_STATE: SENSOR_RATE_HZ},
            )
            sender = mavlink2.MAVLink(
                connection,
                srcSystem=254,
                srcComponent=mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER,
            )

            ground_altitude_m = None
            last_state_at = time.monotonic()
            while not self._stop.is_set():
                state = connection.recv_match(
                    type="SIM_STATE", blocking=True, timeout=0.2
                )
                if state is None:
                    if self._ready.is_set() and time.monotonic() - last_state_at > 1.0:
                        raise TimeoutError("SIM_STATE stream stopped")
                    continue
                last_state_at = time.monotonic()
                if ground_altitude_m is None:
                    ground_altitude_m = float(state.alt)
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


def _set_mode(connection: Any, mode: str, timeout: float = 10.0) -> None:
    requested = mode.upper()
    mapping = connection.mode_mapping()
    assert requested in mapping
    connection.mav.set_mode_send(
        connection.target_system,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[requested],
    )
    deadline = time.monotonic() + timeout
    while (remaining := deadline - time.monotonic()) > 0:
        heartbeat = connection.recv_match(
            type="HEARTBEAT", blocking=True, timeout=min(remaining, 0.5)
        )
        if heartbeat is not None and _mode_from_heartbeat(heartbeat) == requested:
            return
    pytest.fail(f"SITL did not enter {requested}")


def _wait_armed(connection: Any, armed: bool, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    status_texts: list[str] = []
    while (remaining := deadline - time.monotonic()) > 0:
        message = connection.recv_match(
            type=["HEARTBEAT", "STATUSTEXT"],
            blocking=True,
            timeout=min(remaining, 0.5),
        )
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            status_texts.append(str(message.text))
            continue
        if heartbeat_is_armed(message) is armed:
            return
    state = "arming" if armed else "disarming"
    pytest.fail(f"SITL did not confirm {state}; status={status_texts[-10:]}")


def _arm_when_ready(connection: Any, timeout: float = 45.0) -> None:
    deadline = time.monotonic() + timeout
    next_request = 0.0
    status_texts: list[str] = []
    while (remaining := deadline - time.monotonic()) > 0:
        now = time.monotonic()
        _send_rc(connection, 1000)
        if now >= next_request:
            connection.arducopter_arm()
            next_request = now + 2.0
        message = connection.recv_match(
            type=["HEARTBEAT", "STATUSTEXT"],
            blocking=True,
            timeout=min(remaining, 0.2),
        )
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            status_texts.append(str(message.text))
        elif heartbeat_is_armed(message):
            return
    pytest.fail(f"SITL never became ready to arm; status={status_texts[-10:]}")


def _wait_for_relative_position(connection: Any, timeout: float = 30.0) -> int:
    deadline = time.monotonic() + timeout
    last_flags = 0
    while (remaining := deadline - time.monotonic()) > 0:
        report = connection.recv_match(
            type="EKF_STATUS_REPORT", blocking=True, timeout=min(remaining, 0.5)
        )
        if report is None:
            continue
        last_flags = int(report.flags)
        if (
            last_flags & mavlink.EKF_POS_HORIZ_REL
            and not last_flags & mavlink.EKF_CONST_POS_MODE
        ):
            return last_flags
    pytest.fail(f"EKF never obtained relative horizontal position; flags={last_flags}")


def _send_rc(connection: Any, throttle_pwm: int) -> None:
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        1500,
        1500,
        throttle_pwm,
        1500,
        0,
        0,
        0,
        0,
    )


def _wait_for_takeoff(connection: Any, timeout: float = 30.0) -> Any:
    deadline = time.monotonic() + timeout
    last_position = None
    while (remaining := deadline - time.monotonic()) > 0:
        # A modest AltHold climb avoids carrying vertical momentum into Loiter.
        _send_rc(connection, 1700)
        position = connection.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=min(remaining, 0.2)
        )
        if position is None:
            continue
        last_position = position
        if -float(position.z) >= TARGET_ALTITUDE_M * 0.9:
            _send_rc(connection, 1500)
            return position
    altitude = None if last_position is None else -float(last_position.z)
    pytest.fail(f"SITL did not take off; last local altitude={altitude}")


def _settle_after_takeoff(connection: Any, duration: float = 3.0) -> None:
    deadline = time.monotonic() + duration
    while (remaining := deadline - time.monotonic()) > 0:
        _send_rc(connection, 1500)
        connection.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=min(remaining, 0.2)
        )


def _land_and_wait(connection: Any, timeout: float = 30.0) -> None:
    _set_mode(connection, "LAND")
    _wait_armed(connection, False, timeout)


def _assert_sitl_parameters(connection: Any) -> None:
    expected = {
        "AHRS_EKF_TYPE": 3.0,
        "ARMING_SKIPCHK": 0.0,
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
        "GPS1_TYPE": 0.0,
        "GPS2_TYPE": 0.0,
        "RNGFND1_MAX": 8.0,
        "RNGFND1_MIN": 0.0,
        "RNGFND1_ORIENT": 25.0,
        "RNGFND1_TYPE": 10.0,
        "SERIAL1_PROTOCOL": 2.0,
        "SIM_FLOW_ENABLE": 0.0,
        "SIM_GPS1_ENABLE": 0.0,
    }
    actual = {
        name: request_parameter(connection, name, timeout=5.0) for name in expected
    }
    assert actual == expected


def _hold_in_loiter(connection: Any, duration: float = 20.0) -> dict[str, float]:
    deadline = time.monotonic() + duration
    positions: list[tuple[float, float, float]] = []
    ranges_m: list[float] = []
    flow_qualities: list[int] = []
    ekf_flags: list[int] = []
    modes: list[str] = []
    rejected_statuses: list[str] = []
    next_rc = 0.0
    while (remaining := deadline - time.monotonic()) > 0:
        now = time.monotonic()
        if now >= next_rc:
            _send_rc(connection, 1500)
            next_rc = now + 0.2
        message = connection.recv_match(blocking=True, timeout=min(remaining, 0.1))
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            modes.append(_mode_from_heartbeat(message))
            assert heartbeat_is_armed(message)
        elif message_type == "LOCAL_POSITION_NED":
            positions.append((float(message.x), float(message.y), -float(message.z)))
        elif message_type == "DISTANCE_SENSOR" and int(message.orientation) == 25:
            if int(message.current_distance) > 0:
                ranges_m.append(float(message.current_distance) / 100.0)
        elif message_type == "OPTICAL_FLOW":
            flow_qualities.append(int(message.quality))
        elif message_type == "EKF_STATUS_REPORT":
            ekf_flags.append(int(message.flags))
        elif message_type == "STATUSTEXT":
            status = str(message.text)
            normalized = status.casefold()
            if any(
                rejected in normalized
                for rejected in (
                    "stopped aiding",
                    "ekf failsafe",
                    "requires position",
                    "mode change to loiter failed",
                )
            ):
                rejected_statuses.append(status)

    assert positions, "Loiter produced no local-position samples"
    assert ranges_m, "Loiter produced no downward-range samples"
    assert flow_qualities, "Loiter produced no optical-flow samples"
    assert ekf_flags, "Loiter produced no EKF status samples"
    assert not rejected_statuses, rejected_statuses
    assert modes and set(modes) == {"LOITER"}
    assert set(flow_qualities) == {FLOW_QUALITY}
    assert all(flags & mavlink.EKF_POS_HORIZ_REL for flags in ekf_flags)
    assert all(not flags & mavlink.EKF_POS_HORIZ_ABS for flags in ekf_flags)
    assert all(not flags & mavlink.EKF_CONST_POS_MODE for flags in ekf_flags)

    start_x, start_y, _ = positions[0]
    max_drift = max(
        math.hypot(x - start_x, y - start_y) for x, y, _altitude in positions
    )
    altitudes = [altitude for _x, _y, altitude in positions]
    return {
        "max_horizontal_drift_m": max_drift,
        "minimum_altitude_m": min(altitudes),
        "maximum_altitude_m": max(altitudes),
    }


def test_no_gps_optical_flow_loiter_in_pinned_sitl(tmp_path, monkeypatch) -> None:
    root = _ardupilot_root()
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
    monkeypatch.chdir(tmp_path)
    sensors = _ExternalMavlinkSensors()
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

            # Exercise the companion's read-only capture path before any simulated
            # arm or actuator command while the project-style external sensors run.
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
            manifest = json.loads((inspection / "manifest.json").read_text())
            assert manifest["components"]["camera"]["status"] == "unavailable"
            assert manifest["components"]["flight_controller"]["status"] == "ok"
            assert manifest["components"]["downward_rangefinder"]["status"] == "ok"
            assert manifest["components"]["optical_flow"]["status"] == "ok"
            assert manifest["components"]["optical_flow"]["quality"] == FLOW_QUALITY

            connection = _connect()
            armed = False
            try:
                _assert_sitl_parameters(connection)
                sensors.assert_healthy()
                request_message_intervals(
                    connection,
                    {
                        mavlink.MAVLINK_MSG_ID_HEARTBEAT: 4.0,
                        mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 20.0,
                        mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: 10.0,
                        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW: 10.0,
                        mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 10.0,
                    },
                )
                _set_mode(connection, "ALT_HOLD")
                _arm_when_ready(connection)
                armed = True
                _wait_for_takeoff(connection)
                _settle_after_takeoff(connection)

                relative_flags = _wait_for_relative_position(connection)
                assert relative_flags & mavlink.EKF_POS_HORIZ_REL
                assert not relative_flags & mavlink.EKF_POS_HORIZ_ABS
                _set_mode(connection, "LOITER")
                result = _hold_in_loiter(connection)
                assert result["max_horizontal_drift_m"] <= 0.5, result
                assert result["minimum_altitude_m"] >= 0.7, result
                assert result["maximum_altitude_m"] <= 1.3, result
                print(
                    "no-GPS Loiter: "
                    f"max XY drift={result['max_horizontal_drift_m']:.3f} m, "
                    f"altitude={result['minimum_altitude_m']:.3f}.."
                    f"{result['maximum_altitude_m']:.3f} m, "
                    f"external MAVLink {sensors.wire_protocol} "
                    f"sensor samples={sensors.sample_count}"
                )

                _land_and_wait(connection)
                armed = False
            finally:
                if armed:
                    with suppress(BaseException):
                        _land_and_wait(connection)
                connection.close()
        finally:
            try:
                sensors.stop()
            finally:
                _stop_process(process)
