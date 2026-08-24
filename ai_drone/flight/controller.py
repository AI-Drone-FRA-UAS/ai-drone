"""Canonical, safety-gated MAVLink flight controller for ArduPilot Copter."""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.mavlink.devices import resolve_mavlink_endpoint
from ai_drone.mavlink.parameters import request_parameter
from ai_drone.mavlink.safety import (
    heartbeat_is_armed,
    is_vehicle_message,
    require_fresh_disarmed_heartbeat,
)
from ai_drone.recording import request_message_intervals
from ai_drone.validation import finite_in_range

logger = logging.getLogger(__name__)

DOWNWARD_ORIENTATION = mavlink.MAV_SENSOR_ROTATION_PITCH_270
MAX_SAMPLE_AGE_MS = 1_000
FUTURE_TOLERANCE_MS = 250
GCS_HEARTBEAT_INTERVAL_S = 1.0
NAVIGATION_SAMPLE_MAX_AGE_S = 1.0
BATTERY_SAMPLE_MAX_AGE_S = 2.0
PARAMETER_ABS_TOLERANCE = 1e-5
ATTITUDE_TARGET_MASK = (
    mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)
GUIDED_HOLD_FIELD = 0.5
GUIDED_TAKEOFF_CLIMB_FRACTION = 0.3
MAX_PHYSICAL_ALTITUDE_M = 0.8

# These are the reviewed ArduCopter 4.7 invariants for this GPS-less aircraft.
# Hardware calibration and motor-output parameters deliberately do not belong here.
REQUIRED_NOGPS_LOITER_PARAMETERS: Mapping[str, float] = {
    "AHRS_EKF_TYPE": 3.0,
    "AHRS_OPTIONS": 16.0,
    "ARMING_NEED_LOC": 0.0,
    "AVOID_ENABLE": 2.0,
    "EK3_FLOW_USE": 1.0,
    "EK3_ENABLE": 1.0,
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
    # This aircraft intentionally has no RC receiver.  In 4.7, disabling this
    # check is what permits full-check GCS arming without reporting "RC not found".
    "FS_THR_ENABLE": 0.0,
    "FS_VIBE_ENABLE": 1.0,
    "GPS1_TYPE": 0.0,
    "GPS2_TYPE": 0.0,
    # Bit 3 would reinterpret SET_ATTITUDE_TARGET's field as raw thrust.
    "GUID_OPTIONS": 0.0,
    # Preserve the project's deliberately gentle final descent rate.
    "LAND_SPD_MS": 0.15,
    "MAV_GCS_SYSID": 255.0,
    "RNGFND1_MAX": 1.0,
    "RNGFND1_ORIENT": float(DOWNWARD_ORIENTATION),
    "RNGFND1_TYPE": 10.0,
    "RNGFND2_TYPE": 0.0,
    # Fractional Guided climb setpoints are scaled by this value in 4.7.
    "WP_SPD_UP": 0.25,
}

EXPECTED_FIRMWARE_VERSION = (4, 7, 0)
EXPECTED_FIRMWARE_COMMIT = b"1511f271"


class FlightSafetyError(RuntimeError):
    """Raised when a live-flight safety invariant is violated."""


class DroneController:
    """One controller for arm, takeoff, hover, and landing.

    The context manager only cleans up arming/flight initiated by this object.
    Merely observing an already-armed aircraft never sends LAND or DISARM.
    """

    def __init__(
        self,
        device: str | Path | None = None,
        baud: int = 115200,
        max_altitude: float = 0.8,
        min_battery_voltage: float = 0.0,
        target_system: int = 1,
        target_component: int = 1,
    ) -> None:
        if isinstance(baud, bool) or not 1 <= baud <= 4_000_000:
            raise ValueError("baud must be between 1 and 4000000")
        self.device = self.find_device(device)
        self.baud = baud
        self.max_altitude = finite_in_range(
            max_altitude,
            "max_altitude",
            minimum=0.1,
            maximum=MAX_PHYSICAL_ALTITUDE_M,
        )
        self.min_battery_voltage = finite_in_range(
            min_battery_voltage,
            "min_battery_voltage",
            minimum=0.0,
            maximum=60.0,
        )
        self.target_system = target_system
        self.target_component = target_component
        self.connection: Any | None = None
        self.current_altitude: float | None = None
        self.local_position_altitude: float | None = None
        self.local_position_altitude_aligned: float | None = None
        self.battery_voltage: float | None = None
        self.ekf_flags: int | None = None
        self.flow_quality: int | None = None
        self.rc_channel_count: int | None = None
        self.yaw_rad: float | None = None
        self.flight_mode: str | None = None
        self.is_armed = False
        self.is_flying = False
        self.last_telemetry_time = 0.0
        self.last_heartbeat_time = 0.0
        self.last_battery_time = 0.0
        self.last_ekf_time = 0.0
        self.last_flow_time = 0.0
        self.last_rc_channels_time = 0.0
        self.last_attitude_time = 0.0
        self.last_local_position_time = 0.0
        self.flight_sw_version: int | None = None
        self.flight_custom_version: bytes | None = None
        self._latest_boot_ms: int | None = None
        self._latest_boot_received = 0.0
        self._last_gcs_heartbeat_time: float | None = None
        self._arm_command_sent = False
        self._armed_by_controller = False
        self._flight_started_by_controller = False
        self._landing_commanded = False
        self._ground_reference: float | None = None
        self._local_altitude_offset: float | None = None
        # The CLI sets this to a non-blocking callback backed by its signal/event
        # handling.  Every long pre-landing loop checks it; landing deliberately
        # ignores it so a second signal cannot interrupt cleanup.
        self.stop_requested: Callable[[], bool] | None = None

    @staticmethod
    def find_device(requested: str | Path | None) -> str:
        try:
            return resolve_mavlink_endpoint(
                requested,
                include_pi_uart=True,
                missing_message="No ArduPilot device found; pass --device explicitly.",
            )
        except FileNotFoundError as error:
            if requested is not None:
                raise
            raise RuntimeError(str(error)) from error

    def __enter__(self) -> DroneController:
        self.connect()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        try:
            if self._flight_started_by_controller:
                # Once TAKEOFF has been sent, cleanup must never issue a disarm.
                # LAND may already have been requested or may have timed out; in
                # both cases keep monitoring and re-sending it until the vehicle
                # confirms disarm or this bounded attempt expires.
                self.land()
            elif self._arm_command_sent or self._armed_by_controller:
                self.disarm()
        except Exception as error:
            logger.error("Could not complete controller cleanup: %s", error)
        finally:
            self.close()

    def _connection(self) -> Any:
        if self.connection is None:
            raise RuntimeError("not connected")
        return self.connection

    def connect(self) -> None:
        connection = mavutil.mavlink_connection(
            self.device,
            baud=self.baud,
            source_system=255,
            source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        self.connection = connection
        try:
            heartbeat = connection.wait_heartbeat(timeout=15)
            if heartbeat is None:
                raise TimeoutError("no ArduPilot heartbeat received")
            self.target_system = int(heartbeat.get_srcSystem())
            self.target_component = int(heartbeat.get_srcComponent())
            connection.target_system = self.target_system
            connection.target_component = self.target_component
            self._process_message(heartbeat, time.monotonic())
            self._pump_gcs_heartbeat()
            self.request_telemetry_streams()
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        connection, self.connection = self.connection, None
        if connection is not None:
            connection.close()

    def request_telemetry_streams(self, rate_hz: int = 10) -> None:
        if isinstance(rate_hz, bool) or not 1 <= rate_hz <= 50:
            raise ValueError("rate_hz must be between 1 and 50")
        connection = self._connection()
        request_message_intervals(
            connection,
            {
                mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: float(rate_hz),
                mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: float(rate_hz),
                mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2.0,
                mavlink.MAVLINK_MSG_ID_ATTITUDE: float(rate_hz),
                mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: float(rate_hz),
                mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW: float(rate_hz),
                mavlink.MAVLINK_MSG_ID_RC_CHANNELS: float(rate_hz),
            },
        )

    def _pump_gcs_heartbeat(self) -> None:
        """Send the system-255 heartbeat used by ArduPilot's GCS failsafe."""

        if self.connection is None:
            return
        now = time.monotonic()
        if (
            self._last_gcs_heartbeat_time is not None
            and now - self._last_gcs_heartbeat_time < GCS_HEARTBEAT_INTERVAL_S
        ):
            return
        self.connection.mav.heartbeat_send(
            mavlink.MAV_TYPE_GCS,
            mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
        )
        self._last_gcs_heartbeat_time = now

    @staticmethod
    def _timestamp_delta(candidate: int, reference: int) -> int:
        return (candidate - reference + 2**31) % 2**32 - 2**31

    def _timestamp_is_fresh(self, raw: object, now: float) -> bool:
        if isinstance(raw, bool) or not isinstance(raw, int) or not 0 < raw < 2**32:
            return False
        if self._latest_boot_ms is None:
            self._latest_boot_ms = raw
            self._latest_boot_received = now
            return True
        elapsed_ms = round((now - self._latest_boot_received) * 1_000)
        delta = self._timestamp_delta(raw, self._latest_boot_ms)
        age_from_expected = delta - elapsed_ms
        if (
            age_from_expected < -MAX_SAMPLE_AGE_MS
            or age_from_expected > FUTURE_TOLERANCE_MS
        ):
            return False
        if delta > 0:
            self._latest_boot_ms = raw
            self._latest_boot_received = now
        return True

    def _matching_vehicle_message(self, message: Any) -> bool:
        return is_vehicle_message(
            message,
            system_id=self.target_system,
            component_id=self.target_component,
        )

    def _process_message(self, message: Any, now: float) -> None:
        if not self._matching_vehicle_message(message):
            return
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            self.is_armed = heartbeat_is_armed(message)
            self.last_heartbeat_time = now
            mode = getattr(self._connection(), "flightmode", None)
            if isinstance(mode, str):
                self.flight_mode = mode
            if not self.is_armed:
                self.is_flying = False
                self._arm_command_sent = False
                self._armed_by_controller = False
                self._flight_started_by_controller = False
                self._local_altitude_offset = None
                self.local_position_altitude_aligned = None
            return
        if message_type in {"ATTITUDE", "LOCAL_POSITION_NED"}:
            self._process_pose_message(message, message_type, now)
            return
        if message_type == "DISTANCE_SENSOR":
            if int(message.orientation) != DOWNWARD_ORIENTATION:
                return
            if not self._timestamp_is_fresh(
                getattr(message, "time_boot_ms", None), now
            ):
                return
            current = int(message.current_distance)
            minimum = int(message.min_distance)
            maximum = int(message.max_distance)
            quality = int(getattr(message, "signal_quality", 255))
            # MAVLink defines 0 as unknown/not supplied and 1 as invalid.
            if (
                current <= 0
                or (minimum > 0 and current < minimum)
                or (maximum > 0 and current > maximum)
                or quality == 1
            ):
                return
            altitude = current / 100.0
            if not math.isfinite(altitude):
                return
            self.current_altitude = altitude
            self.last_telemetry_time = now
            self._align_local_altitude(now)
        elif message_type == "EKF_STATUS_REPORT":
            self.ekf_flags = int(message.flags)
            self.last_ekf_time = now
        elif message_type in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}:
            quality = int(message.quality)
            self.flow_quality = quality
            if quality > 0:
                self.last_flow_time = now
        elif message_type == "RC_CHANNELS":
            if not self._timestamp_is_fresh(
                getattr(message, "time_boot_ms", None), now
            ):
                return
            channel_count = int(message.chancount)
            if 0 <= channel_count <= 18:
                self.rc_channel_count = channel_count
                self.last_rc_channels_time = now
        elif message_type == "SYS_STATUS":
            voltage = float(message.voltage_battery) / 1_000.0
            if math.isfinite(voltage) and voltage > 0.0:
                self.battery_voltage = voltage
                self.last_battery_time = now
        elif message_type == "AUTOPILOT_VERSION":
            self.flight_sw_version = int(message.flight_sw_version)
            custom = message.flight_custom_version
            self.flight_custom_version = (
                custom if isinstance(custom, bytes) else bytes(custom)
            )

    def _process_pose_message(
        self, message: Any, message_type: str, now: float
    ) -> None:
        if not self._timestamp_is_fresh(getattr(message, "time_boot_ms", None), now):
            return
        if message_type == "ATTITUDE":
            yaw = float(message.yaw)
            if math.isfinite(yaw):
                self.yaw_rad = yaw
                self.last_attitude_time = now
            return
        altitude = -float(message.z)
        if math.isfinite(altitude):
            self.local_position_altitude = altitude
            self.last_local_position_time = now
            self._align_local_altitude(now)

    def _align_local_altitude(self, now: float) -> None:
        """Align local NED altitude to the downward rangefinder's floor datum."""

        local = self.local_position_altitude
        distance = self.current_altitude
        if local is None or distance is None:
            return
        if now - self.last_local_position_time > NAVIGATION_SAMPLE_MAX_AGE_S:
            return
        if now - self.last_telemetry_time > NAVIGATION_SAMPLE_MAX_AGE_S:
            return
        if self._local_altitude_offset is None:
            self._local_altitude_offset = distance - local
        self.local_position_altitude_aligned = local + self._local_altitude_offset

    def update_telemetry(self, max_messages: int = 50) -> None:
        if isinstance(max_messages, bool) or not 1 <= max_messages <= 1_000:
            raise ValueError("max_messages must be between 1 and 1000")
        if self.connection is None:
            return
        self._pump_gcs_heartbeat()
        self._raise_if_stop_requested()
        for _ in range(max_messages):
            message = self.connection.recv_match(blocking=False)
            if message is None:
                break
            self._process_message(message, time.monotonic())
        if (
            self._flight_started_by_controller
            and not self._landing_commanded
            and (
                (
                    self.current_altitude is not None
                    and self.current_altitude > self.max_altitude
                )
                or (
                    self.local_position_altitude_aligned is not None
                    and self.local_position_altitude_aligned > self.max_altitude
                )
            )
        ):
            self.emergency_stop()
            measured = max(
                value
                for value in (
                    self.current_altitude,
                    self.local_position_altitude_aligned,
                )
                if value is not None
            )
            raise FlightSafetyError(
                f"altitude {measured:.2f} m exceeds {self.max_altitude:.2f} m"
            )
        if (
            self._flight_started_by_controller
            and not self._landing_commanded
            and self.flight_mode == "LOITER"
            and not self.navigation_is_healthy()
        ):
            self.emergency_stop()
            raise FlightSafetyError(
                "Loiter navigation became unhealthy (optical flow or relative EKF position)"
            )
        if (
            self._flight_started_by_controller
            and not self._landing_commanded
            and not self.no_rc_input_is_confirmed()
        ):
            self.emergency_stop()
            raise FlightSafetyError(
                "autonomous-flight receiver topology changed or RC_CHANNELS became stale"
            )
        if (
            self._flight_started_by_controller
            and not self._landing_commanded
            and self.min_battery_voltage > 0.0
            and (
                not self.battery_is_fresh()
                or self.battery_voltage is None
                or self.battery_voltage < self.min_battery_voltage
            )
        ):
            self.emergency_stop()
            if not self.battery_is_fresh() or self.battery_voltage is None:
                raise FlightSafetyError("battery telemetry became stale during flight")
            raise FlightSafetyError(
                f"battery {self.battery_voltage:.2f} V is below "
                f"{self.min_battery_voltage:.2f} V"
            )

    def _raise_if_stop_requested(self) -> None:
        callback = self.stop_requested
        if callback is None or self._landing_commanded:
            return
        try:
            requested = bool(callback())
        except Exception as error:
            logger.error("stop callback failed; ending the flight: %s", error)
            requested = True
        if not requested:
            return
        if self._flight_started_by_controller:
            self.emergency_stop()
        raise FlightSafetyError("flight stop requested")

    def altitude_is_fresh(self, max_age: float = 1.0) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.current_altitude is not None
            and time.monotonic() - self.last_telemetry_time <= max_age
        )

    def heartbeat_is_fresh(self, max_age: float = 2.5) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.last_heartbeat_time > 0
            and time.monotonic() - self.last_heartbeat_time <= max_age
        )

    def attitude_is_fresh(self, max_age: float = 1.0) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.yaw_rad is not None
            and self.last_attitude_time > 0
            and time.monotonic() - self.last_attitude_time <= max_age
        )

    def battery_is_fresh(self, max_age: float = BATTERY_SAMPLE_MAX_AGE_S) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.battery_voltage is not None
            and self.last_battery_time > 0
            and time.monotonic() - self.last_battery_time <= max_age
        )

    def optical_flow_is_fresh(
        self, max_age: float = NAVIGATION_SAMPLE_MAX_AGE_S
    ) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.flow_quality is not None
            and self.flow_quality > 0
            and self.last_flow_time > 0
            and time.monotonic() - self.last_flow_time <= max_age
        )

    def relative_position_is_fresh(
        self, max_age: float = NAVIGATION_SAMPLE_MAX_AGE_S
    ) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        flags = self.ekf_flags
        required = mavlink.EKF_VELOCITY_HORIZ | mavlink.EKF_POS_HORIZ_REL
        return (
            flags is not None
            and flags & required == required
            and not flags & mavlink.EKF_CONST_POS_MODE
            and self.last_ekf_time > 0
            and time.monotonic() - self.last_ekf_time <= max_age
        )

    def navigation_is_healthy(self) -> bool:
        """Return whether fresh optical flow backs a relative EKF position."""

        return self.optical_flow_is_fresh() and self.relative_position_is_fresh()

    def no_rc_input_is_confirmed(
        self, max_age: float = NAVIGATION_SAMPLE_MAX_AGE_S
    ) -> bool:
        """Return whether a fresh FC report confirms no receiver channels.

        ArduCopter 4.7 treats missing RC as neutral input in Loiter, while a
        valid low-throttle receiver commands descent.  This project has no RC
        receiver, so any nonzero channel count is an unsafe topology change.
        """

        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.rc_channel_count == 0
            and self.last_rc_channels_time > 0
            and time.monotonic() - self.last_rc_channels_time <= max_age
        )

    def wait_for_altitude(self, timeout: float = 3.0) -> float | None:
        """Wait for a newly received, downward DISTANCE_SENSOR sample."""
        finite_in_range(timeout, "timeout", minimum=0.05, maximum=30.0)
        connection = self._connection()
        while (queued := connection.recv_match(blocking=False)) is not None:
            self._process_message(queued, time.monotonic())
        started = time.monotonic()
        deadline = started + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            self._pump_gcs_heartbeat()
            self._raise_if_stop_requested()
            message = connection.recv_match(blocking=True, timeout=min(remaining, 0.25))
            if message is not None:
                self._process_message(message, time.monotonic())
            if (
                self.last_telemetry_time >= started
                and self.current_altitude is not None
            ):
                return self.current_altitude
        return None

    def wait_for_optical_flow(self, timeout: float = 3.0) -> None:
        """Wait for a fresh nonzero-quality optical-flow observation."""

        finite_in_range(timeout, "timeout", minimum=0.05, maximum=30.0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.update_telemetry()
            if self.optical_flow_is_fresh():
                return
            time.sleep(0.05)
        raise FlightSafetyError("no fresh valid optical-flow sample")

    def wait_for_attitude(self, timeout: float = 3.0) -> None:
        """Wait for the yaw used to construct a level attitude target."""

        finite_in_range(timeout, "timeout", minimum=0.05, maximum=30.0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.update_telemetry()
            if self.attitude_is_fresh():
                return
            time.sleep(0.05)
        raise FlightSafetyError("no fresh attitude sample")

    def wait_for_no_rc_input(self, timeout: float = 3.0) -> None:
        """Require the captured receiver-free topology before arming.

        A valid receiver can change both Loiter climb rate and the flight mode.
        This autonomous path therefore accepts only a fresh zero-channel report.
        """

        finite_in_range(timeout, "timeout", minimum=0.05, maximum=30.0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.update_telemetry()
            if self.no_rc_input_is_confirmed():
                return
            if (
                self.rc_channel_count is not None
                and self.last_rc_channels_time > 0
                and time.monotonic() - self.last_rc_channels_time
                <= NAVIGATION_SAMPLE_MAX_AGE_S
            ):
                raise FlightSafetyError(
                    "active RC receiver detected; autonomous flight requires zero channels"
                )
            time.sleep(0.05)
        raise FlightSafetyError(
            "no fresh RC_CHANNELS report confirming zero receiver channels"
        )

    def verify_battery_before_arming(self, timeout: float = 3.0) -> None:
        """Require a fresh pack voltage at or above the configured guard."""

        finite_in_range(timeout, "timeout", minimum=0.05, maximum=30.0)
        if self.min_battery_voltage <= 0.0:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.update_telemetry()
            if self.battery_is_fresh():
                voltage = self.battery_voltage
                if voltage is None:  # guarded above; keeps the comparison exact
                    break
                if voltage < self.min_battery_voltage:
                    raise FlightSafetyError(
                        f"battery {voltage:.2f} V is below the pre-arm minimum "
                        f"{self.min_battery_voltage:.2f} V"
                    )
                return
            time.sleep(0.05)
        raise FlightSafetyError("no fresh battery voltage before arming")

    def _fresh_disarmed(self, timeout: float = 2.5) -> None:
        heartbeat = require_fresh_disarmed_heartbeat(
            self._connection(),
            system_id=self.target_system,
            component_id=self.target_component,
            timeout=timeout,
        )
        self._process_message(heartbeat, time.monotonic())

    def verify_arming_checks(self) -> None:
        value = request_parameter(self._connection(), "ARMING_SKIPCHK")
        if value != 0.0:
            raise FlightSafetyError(
                f"ARMING_SKIPCHK={value:g}; flight requires exact value 0 "
                "(no checks skipped)"
            )

    def verify_nogps_loiter_parameters(self) -> None:
        """Require the reviewed ArduCopter 4.7 no-GPS flight invariants."""

        for name, expected in REQUIRED_NOGPS_LOITER_PARAMETERS.items():
            self._pump_gcs_heartbeat()
            actual = request_parameter(self._connection(), name)
            if not math.isclose(
                actual,
                expected,
                rel_tol=0.0,
                abs_tol=PARAMETER_ABS_TOLERANCE,
            ):
                raise FlightSafetyError(
                    f"{name}={actual:g}; no-GPS Loiter requires exact value {expected:g}"
                )

    def verify_firmware(self, timeout: float = 3.0) -> None:
        """Require official ArduCopter 4.7.0 at the captured project commit."""

        finite_in_range(timeout, "timeout", minimum=0.5, maximum=15.0)
        self.flight_sw_version = None
        self.flight_custom_version = None
        self._drain_messages()
        self._send_command_long_and_wait_ack(
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            (
                float(mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
            timeout=timeout,
        )
        deadline = time.monotonic() + timeout
        while self.flight_sw_version is None and time.monotonic() < deadline:
            self._pump_gcs_heartbeat()
            self._raise_if_stop_requested()
            message = self._connection().recv_match(blocking=True, timeout=0.25)
            if message is not None:
                self._process_message(message, time.monotonic())
        packed = self.flight_sw_version
        if packed is None:
            raise TimeoutError("flight controller did not return AUTOPILOT_VERSION")
        version = ((packed >> 24) & 0xFF, (packed >> 16) & 0xFF, (packed >> 8) & 0xFF)
        if version != EXPECTED_FIRMWARE_VERSION:
            rendered = ".".join(str(part) for part in version)
            raise FlightSafetyError(
                f"ArduCopter {rendered}; flight requires exact version 4.7.0"
            )
        if self.flight_custom_version != EXPECTED_FIRMWARE_COMMIT:
            actual = (self.flight_custom_version or b"").decode(
                "ascii", errors="replace"
            )
            raise FlightSafetyError(
                f"firmware commit {actual!r}; flight requires '1511f271'"
            )

    def verify_onboard_logging(self) -> None:
        backend = float(request_parameter(self._connection(), "LOG_BACKEND_TYPE"))
        bitmask = float(request_parameter(self._connection(), "LOG_BITMASK"))
        if (
            not math.isfinite(backend)
            or backend != int(backend)
            or not int(backend) & 5
        ):
            raise FlightSafetyError("onboard file/dataflash logging is disabled")
        if not math.isfinite(bitmask) or bitmask <= 0:
            raise FlightSafetyError("LOG_BITMASK must be nonzero")

    def set_mode(self, mode_name: str, timeout: float = 5.0) -> None:
        finite_in_range(timeout, "timeout", minimum=0.1, maximum=30.0)
        connection = self._connection()
        requested = mode_name.upper()
        mapping = connection.mode_mapping()
        if requested not in mapping:
            raise ValueError(f"flight mode {requested!r} is not supported")
        connection.mav.set_mode_send(
            self.target_system,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[requested],
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.update_telemetry()
            if self.flight_mode == requested:
                return
            time.sleep(0.05)
        raise TimeoutError(f"flight controller did not confirm {requested} mode")

    def arm(self, timeout: float = 10.0) -> None:
        finite_in_range(timeout, "timeout", minimum=0.5, maximum=30.0)
        self.update_telemetry()
        if self.is_armed and not self._armed_by_controller:
            raise FlightSafetyError(
                "refusing to take ownership of an already-armed vehicle"
            )
        if self.is_armed:
            return
        self.verify_firmware()
        self.verify_arming_checks()
        self.verify_nogps_loiter_parameters()
        self.verify_onboard_logging()
        if self.wait_for_altitude(timeout=min(timeout, 3.0)) is None:
            raise FlightSafetyError("no fresh downward DISTANCE_SENSOR altitude")
        self.wait_for_optical_flow(timeout=min(timeout, 3.0))
        self.wait_for_attitude(timeout=min(timeout, 3.0))
        self.wait_for_no_rc_input(timeout=min(timeout, 3.0))
        self.verify_battery_before_arming(timeout=min(timeout, 3.0))
        # AltHold's user-takeoff state machine still depends on positive pilot
        # throttle to spool up.  This companion-only aircraft has no RC input,
        # so use ArduPilot's position-free autonomous takeoff mode instead.
        self.set_mode("GUIDED_NOGPS")
        self._fresh_disarmed()
        if not self.altitude_is_fresh():
            raise FlightSafetyError("downward altitude became stale before arming")
        self._arm_command_sent = True
        self._landing_commanded = False
        self._connection().arducopter_arm()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump_gcs_heartbeat()
            self._raise_if_stop_requested()
            message = self._connection().recv_match(blocking=True, timeout=0.5)
            if message is not None:
                self._process_message(message, time.monotonic())
            if self.is_armed:
                self._armed_by_controller = True
                return
        raise TimeoutError("flight controller did not confirm arming")

    def _drain_messages(self) -> None:
        connection = self._connection()
        while (message := connection.recv_match(blocking=False)) is not None:
            self._process_message(message, time.monotonic())

    def _send_command_long_and_wait_ack(
        self,
        command: int,
        parameters: tuple[float, float, float, float, float, float, float],
        *,
        timeout: float = 3.0,
    ) -> None:
        """Send one command and require its source-filtered COMMAND_ACK."""

        finite_in_range(timeout, "timeout", minimum=0.1, maximum=30.0)
        self._connection().mav.command_long_send(
            self.target_system,
            self.target_component,
            command,
            0,
            *parameters,
        )
        status_text: list[str] = []
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            self._pump_gcs_heartbeat()
            self._raise_if_stop_requested()
            message = self._connection().recv_match(
                blocking=True, timeout=min(remaining, 0.25)
            )
            if message is None or not self._matching_vehicle_message(message):
                continue
            message_type = message.get_type()
            if message_type == "STATUSTEXT":
                text = str(message.text).strip()
                if text and text not in status_text:
                    status_text.append(text)
                continue
            if message_type != "COMMAND_ACK" or int(message.command) != command:
                self._process_message(message, time.monotonic())
                continue
            result = int(message.result)
            if result == mavlink.MAV_RESULT_ACCEPTED:
                return
            if result == mavlink.MAV_RESULT_IN_PROGRESS:
                continue
            detail = f": {'; '.join(status_text)}" if status_text else ""
            raise FlightSafetyError(
                f"flight controller rejected MAV_CMD {command} "
                f"with MAV_RESULT {result}{detail}"
            )
        detail = f": {'; '.join(status_text)}" if status_text else ""
        raise TimeoutError(
            f"flight controller did not acknowledge MAV_CMD {command}{detail}"
        )

    def _request_disarm(self) -> None:
        self._connection().arducopter_disarm()

    def disarm(self, timeout: float = 10.0) -> None:
        finite_in_range(timeout, "timeout", minimum=0.5, maximum=30.0)
        if self._flight_started_by_controller:
            raise FlightSafetyError("refusing to force-disarm a flight; use land()")
        self._request_disarm()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self._connection().recv_match(
                type="HEARTBEAT", blocking=True, timeout=0.5
            )
            if message is not None:
                self._process_message(message, time.monotonic())
            if not self.is_armed:
                return
        raise TimeoutError("flight controller did not confirm disarming")

    def _send_level_climb(self, climb_fraction: float) -> None:
        """Send a level GuidedNoGPS target with a bounded climb fraction.

        Because flight requires ``GUID_OPTIONS=0``, ArduCopter interprets the
        final SET_ATTITUDE_TARGET field as climb rate: 0.5 holds altitude and
        1.0 requests ``WP_SPD_UP``.  It is never raw motor thrust here.
        """

        fraction = finite_in_range(
            climb_fraction, "climb_fraction", minimum=-1.0, maximum=1.0
        )
        if not self.is_armed or self.flight_mode != "GUIDED_NOGPS":
            raise FlightSafetyError(
                "level climb setpoints require armed GUIDED_NOGPS mode"
            )
        if not self.attitude_is_fresh():
            raise FlightSafetyError("attitude became stale before climb setpoint")
        yaw = self.yaw_rad
        if yaw is None:  # guarded above; keeps the quaternion type exact
            raise FlightSafetyError("no yaw available for level climb setpoint")
        half_yaw = yaw * 0.5
        field = GUIDED_HOLD_FIELD + fraction * 0.5
        self._connection().mav.set_attitude_target_send(
            (time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            ATTITUDE_TARGET_MASK,
            [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)],
            0.0,
            0.0,
            0.0,
            field,
        )

    def takeoff(self, target_alt: float, timeout: float = 15.0) -> None:
        target = finite_in_range(
            target_alt, "target_alt", minimum=0.15, maximum=self.max_altitude
        )
        finite_in_range(timeout, "timeout", minimum=1.0, maximum=60.0)
        if not self.is_armed:
            self.arm()
        if not self._armed_by_controller:
            raise FlightSafetyError("takeoff requires arming by this controller")
        if self.wait_for_altitude(timeout=3.0) is None:
            raise FlightSafetyError("no fresh downward altitude before takeoff")
        if not self.optical_flow_is_fresh():
            raise FlightSafetyError("no fresh valid optical-flow sample before takeoff")
        if not self.attitude_is_fresh():
            self.wait_for_attitude(timeout=3.0)
        if self.flight_mode != "GUIDED_NOGPS":
            self.set_mode("GUIDED_NOGPS")
        self._ground_reference = self.current_altitude
        self._local_altitude_offset = None
        self.local_position_altitude_aligned = None
        self._align_local_altitude(time.monotonic())
        # Mark ownership before the first climb target leaves.  From this point
        # cleanup must LAND and never issue a force-disarm, even if no motion is
        # observed or the link fails immediately afterward.
        self._flight_started_by_controller = True
        self.is_flying = True
        started = time.monotonic()
        deadline = started + timeout
        while time.monotonic() < deadline:
            self.update_telemetry()
            now = time.monotonic()
            if now - started > 2.0 and (
                not self.altitude_is_fresh() or not self.heartbeat_is_fresh()
            ):
                self.emergency_stop()
                raise FlightSafetyError("telemetry became stale during takeoff")
            if (
                self.last_telemetry_time >= started
                and self.current_altitude is not None
                and self._ground_reference is not None
                and self.current_altitude - self._ground_reference >= target * 0.9
            ):
                self._send_level_climb(0.0)
                return
            self._send_level_climb(GUIDED_TAKEOFF_CLIMB_FRACTION)
            time.sleep(0.05)
        self.emergency_stop()
        raise TimeoutError("takeoff altitude was not reached")

    def wait_for_relative_position(
        self, timeout: float = 10.0, stable_for: float = 1.0
    ) -> None:
        """Wait for continuously healthy flow-backed relative navigation."""

        finite_in_range(timeout, "timeout", minimum=0.5, maximum=60.0)
        finite_in_range(stable_for, "stable_for", minimum=0.1, maximum=10.0)
        healthy_since: float | None = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.update_telemetry()
            self._send_level_climb(0.0)
            now = time.monotonic()
            if not self.altitude_is_fresh() or not self.heartbeat_is_fresh():
                self.emergency_stop()
                raise FlightSafetyError(
                    "telemetry became stale before the Loiter transition"
                )
            if self.navigation_is_healthy():
                if healthy_since is None:
                    healthy_since = now
                elif now - healthy_since >= stable_for:
                    return
            else:
                healthy_since = None
            time.sleep(0.05)
        if self._flight_started_by_controller:
            self.emergency_stop()
        raise FlightSafetyError(
            "EKF never established stable optical-flow relative position"
        )

    def enter_loiter(self, timeout: float = 10.0, stable_for: float = 1.0) -> None:
        """Gate and confirm the GuidedNoGPS-to-Loiter handoff."""

        if not self.is_armed or not self._flight_started_by_controller:
            raise FlightSafetyError("Loiter transition requires a controller takeoff")
        self.wait_for_relative_position(timeout=timeout, stable_for=stable_for)
        if not self.no_rc_input_is_confirmed():
            self.emergency_stop()
            raise FlightSafetyError(
                "Loiter requires a fresh RC_CHANNELS report with zero receiver channels"
            )
        self.set_mode("LOITER")
        if not self.navigation_is_healthy():
            self.emergency_stop()
            raise FlightSafetyError("navigation became unhealthy during Loiter entry")

    def hold_loiter(self, duration: float) -> None:
        """Hold confirmed Loiter while enforcing flow, EKF, range and link gates."""

        finite_in_range(duration, "duration", minimum=0.1, maximum=3_600.0)
        if self.flight_mode != "LOITER":
            raise FlightSafetyError("Loiter hold requires confirmed LOITER mode")
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.update_telemetry()
            if not self.altitude_is_fresh() or not self.heartbeat_is_fresh():
                self.emergency_stop()
                raise FlightSafetyError("telemetry became stale during Loiter")
            time.sleep(0.05)

    def land(self, timeout: float = 30.0) -> None:
        finite_in_range(timeout, "timeout", minimum=1.0, maximum=120.0)
        self._landing_commanded = True
        deadline = time.monotonic() + timeout
        next_request = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_request:
                self.emergency_stop()
                next_request = now + 1.0
            self.update_telemetry()
            if not self.is_armed:
                self.is_flying = False
                return
            time.sleep(0.1)
        raise TimeoutError(
            "LAND remains commanded but disarming was not confirmed; do not approach the vehicle"
        )

    def emergency_stop(self) -> None:
        """Command LAND without force-disarming a possibly airborne vehicle."""
        if self.connection is None:
            return
        self._landing_commanded = True
        mapping = self.connection.mode_mapping()
        if "LAND" not in mapping:
            raise FlightSafetyError("flight controller does not expose LAND mode")
        self.connection.mav.set_mode_send(
            self.target_system,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping["LAND"],
        )
