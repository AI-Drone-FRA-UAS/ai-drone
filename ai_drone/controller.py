"""Canonical, safety-gated MAVLink flight controller for ArduPilot Copter."""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.mavlink_devices import resolve_mavlink_endpoint
from ai_drone.mavlink_parameters import request_parameter
from ai_drone.mavlink_safety import (
    heartbeat_is_armed,
    is_vehicle_message,
    require_fresh_disarmed_heartbeat,
)
from ai_drone.recording import request_message_intervals

logger = logging.getLogger(__name__)

DOWNWARD_ORIENTATION = mavlink.MAV_SENSOR_ROTATION_PITCH_270
MAX_SAMPLE_AGE_MS = 1_000
FUTURE_TOLERANCE_MS = 250


class FlightSafetyError(RuntimeError):
    """Raised when a live-flight safety invariant is violated."""


def _finite(value: float, name: str, *, minimum: float, maximum: float) -> float:
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(
            f"{name} must be finite and between {minimum:g} and {maximum:g}"
        )
    return number


class DroneController:
    """One controller for arm, takeoff, body velocity, hover, and landing.

    The context manager only cleans up arming/flight initiated by this object.
    Merely observing an already-armed aircraft never sends LAND or DISARM.
    """

    def __init__(
        self,
        device: str | Path | None = None,
        baud: int = 115200,
        max_altitude: float = 0.8,
        target_system: int = 1,
        target_component: int = 1,
    ) -> None:
        if isinstance(baud, bool) or not 1 <= baud <= 4_000_000:
            raise ValueError("baud must be between 1 and 4000000")
        self.device = self.find_device(device)
        self.baud = baud
        self.max_altitude = _finite(
            max_altitude, "max_altitude", minimum=0.1, maximum=10.0
        )
        self.target_system = target_system
        self.target_component = target_component
        self.connection: Any | None = None
        self.current_altitude: float | None = None
        self.local_position_altitude: float | None = None
        self.battery_voltage: float | None = None
        self.flight_mode: str | None = None
        self.is_armed = False
        self.is_flying = False
        self.last_telemetry_time = 0.0
        self.last_heartbeat_time = 0.0
        self._latest_boot_ms: int | None = None
        self._latest_boot_received = 0.0
        self._arm_command_sent = False
        self._armed_by_controller = False
        self._flight_started_by_controller = False
        self._landing_commanded = False

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
                self.emergency_stop()
            elif self._arm_command_sent or self._armed_by_controller:
                self._request_disarm()
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
            },
        )

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
            return
        if message_type in {"ATTITUDE", "LOCAL_POSITION_NED"}:
            timestamp = getattr(message, "time_boot_ms", None)
            timestamp_ok = self._timestamp_is_fresh(timestamp, now)
            if message_type == "LOCAL_POSITION_NED" and timestamp_ok:
                altitude = -float(message.z)
                if math.isfinite(altitude):
                    self.local_position_altitude = altitude
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
        elif message_type == "SYS_STATUS":
            voltage = float(message.voltage_battery) / 1_000.0
            if math.isfinite(voltage) and voltage > 0.0:
                self.battery_voltage = voltage

    def update_telemetry(self, max_messages: int = 50) -> None:
        if isinstance(max_messages, bool) or not 1 <= max_messages <= 1_000:
            raise ValueError("max_messages must be between 1 and 1000")
        if self.connection is None:
            return
        for _ in range(max_messages):
            message = self.connection.recv_match(blocking=False)
            if message is None:
                break
            self._process_message(message, time.monotonic())
        if (
            self._flight_started_by_controller
            and not self._landing_commanded
            and self.current_altitude is not None
            and self.current_altitude > self.max_altitude
        ):
            self.emergency_stop()
            raise FlightSafetyError(
                f"altitude {self.current_altitude:.2f} m exceeds {self.max_altitude:.2f} m"
            )

    def altitude_is_fresh(self, max_age: float = 1.0) -> bool:
        _finite(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.current_altitude is not None
            and time.monotonic() - self.last_telemetry_time <= max_age
        )

    def heartbeat_is_fresh(self, max_age: float = 2.5) -> bool:
        _finite(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.last_heartbeat_time > 0
            and time.monotonic() - self.last_heartbeat_time <= max_age
        )

    def wait_for_altitude(self, timeout: float = 3.0) -> float | None:
        """Wait for a newly received, downward DISTANCE_SENSOR sample."""
        _finite(timeout, "timeout", minimum=0.05, maximum=30.0)
        connection = self._connection()
        while (queued := connection.recv_match(blocking=False)) is not None:
            self._process_message(queued, time.monotonic())
        started = time.monotonic()
        deadline = started + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            message = connection.recv_match(blocking=True, timeout=min(remaining, 0.25))
            if message is not None:
                self._process_message(message, time.monotonic())
            if (
                self.last_telemetry_time >= started
                and self.current_altitude is not None
            ):
                return self.current_altitude
        return None

    def _fresh_disarmed(self, timeout: float = 2.5) -> None:
        heartbeat = require_fresh_disarmed_heartbeat(
            self._connection(),
            system_id=self.target_system,
            component_id=self.target_component,
            timeout=timeout,
        )
        self._process_message(heartbeat, time.monotonic())

    def verify_arming_checks(self) -> None:
        value = request_parameter(self._connection(), "ARMING_CHECK")
        if value != 1.0:
            raise FlightSafetyError(
                f"ARMING_CHECK={value:g}; flight requires exact value 1 (all checks)"
            )

    def set_mode(self, mode_name: str, timeout: float = 5.0) -> None:
        _finite(timeout, "timeout", minimum=0.1, maximum=30.0)
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
        _finite(timeout, "timeout", minimum=0.5, maximum=30.0)
        self.update_telemetry()
        if self.is_armed and not self._armed_by_controller:
            raise FlightSafetyError(
                "refusing to take ownership of an already-armed vehicle"
            )
        if self.is_armed:
            return
        self.verify_arming_checks()
        if self.wait_for_altitude(timeout=min(timeout, 3.0)) is None:
            raise FlightSafetyError("no fresh downward DISTANCE_SENSOR altitude")
        self.set_mode("GUIDED")
        self._fresh_disarmed()
        if not self.altitude_is_fresh():
            raise FlightSafetyError("downward altitude became stale before arming")
        self._arm_command_sent = True
        self._connection().arducopter_arm()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self._connection().recv_match(
                type="HEARTBEAT", blocking=True, timeout=0.5
            )
            if message is not None:
                self._process_message(message, time.monotonic())
            if self.is_armed:
                self._armed_by_controller = True
                return
        raise TimeoutError("flight controller did not confirm arming")

    def _request_disarm(self) -> None:
        self._connection().arducopter_disarm()

    def disarm(self, timeout: float = 10.0) -> None:
        _finite(timeout, "timeout", minimum=0.5, maximum=30.0)
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

    def takeoff(self, target_alt: float, timeout: float = 15.0) -> None:
        target = _finite(
            target_alt, "target_alt", minimum=0.15, maximum=self.max_altitude
        )
        _finite(timeout, "timeout", minimum=1.0, maximum=60.0)
        if not self.is_armed:
            self.arm()
        if not self._armed_by_controller:
            raise FlightSafetyError("takeoff requires arming by this controller")
        if self.wait_for_altitude(timeout=3.0) is None:
            raise FlightSafetyError("no fresh downward altitude before takeoff")
        self._flight_started_by_controller = True
        self.is_flying = True
        started = time.monotonic()
        self._connection().mav.command_long_send(
            self.target_system,
            self.target_component,
            mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            target,
        )
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
                and self.current_altitude >= target * 0.95
            ):
                return
            time.sleep(0.05)
        self.emergency_stop()
        raise TimeoutError("takeoff altitude was not reached")

    def land(self, timeout: float = 30.0) -> None:
        _finite(timeout, "timeout", minimum=1.0, maximum=120.0)
        self._landing_commanded = True
        self.set_mode("LAND")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
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

    def send_velocity_body(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate_deg: float = 0.0,
    ) -> None:
        forward = _finite(vx, "vx", minimum=-1.0, maximum=1.0)
        right = _finite(vy, "vy", minimum=-1.0, maximum=1.0)
        down = _finite(vz, "vz", minimum=-0.5, maximum=0.5)
        yaw_rate = _finite(yaw_rate_deg, "yaw_rate_deg", minimum=-45.0, maximum=45.0)
        if not self.is_flying or not self.is_armed or self._landing_commanded:
            raise FlightSafetyError(
                "velocity commands require an active controller-owned flight"
            )
        self._connection().mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            mavlink.MAV_FRAME_BODY_NED,
            0x05C7,
            0,
            0,
            0,
            forward,
            right,
            down,
            0,
            0,
            0,
            0,
            math.radians(yaw_rate),
        )
