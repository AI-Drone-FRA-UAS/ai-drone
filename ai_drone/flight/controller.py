"""Canonical, safety-gated MAVLink flight controller for ArduPilot Copter."""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.mavlink import arming_checks
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


class FlightSafetyError(RuntimeError):
    """Raised when a live-flight safety invariant is violated."""


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
        self.max_altitude = finite_in_range(
            max_altitude, "max_altitude", minimum=0.1, maximum=10.0
        )
        self.target_system = target_system
        self.target_component = target_component
        self.connection: Any | None = None
        self.current_altitude: float | None = None
        self.local_position_altitude: float | None = None
        self.battery_voltage: float | None = None
        self.ekf_flags: int | None = None
        self._yaw_rad: float | None = None
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
                # This controller put the aircraft in the air, so it does not
                # get to hang up until the aircraft says it is disarmed.
                if not self.ensure_landed():
                    logger.error(
                        "VEHICLE DID NOT CONFIRM DISARM. It may still be flying. "
                        "LAND was requested repeatedly and never acknowledged. "
                        "Do not approach it; cut power only from a safe distance."
                    )
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
                mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2.0,
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
            if message_type == "ATTITUDE" and timestamp_ok:
                yaw = float(message.yaw)
                if math.isfinite(yaw):
                    self._yaw_rad = yaw
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
        elif message_type == "EKF_STATUS_REPORT":
            self.ekf_flags = int(message.flags)
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

    def wait_for_altitude(self, timeout: float = 3.0) -> float | None:
        """Wait for a newly received, downward DISTANCE_SENSOR sample."""
        finite_in_range(timeout, "timeout", minimum=0.05, maximum=30.0)
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
        value = request_parameter(self._connection(), arming_checks.PARAMETER)
        if not arming_checks.is_acceptable(value):
            raise FlightSafetyError(arming_checks.describe(value))

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

    def arm(self, timeout: float = 10.0, mode: str = "GUIDED") -> None:
        finite_in_range(timeout, "timeout", minimum=0.5, maximum=30.0)
        self.update_telemetry()
        if self.is_armed and not self._armed_by_controller:
            raise FlightSafetyError(
                "refusing to take ownership of an already-armed vehicle"
            )
        if self.is_armed:
            return
        self.verify_arming_checks()
        self.verify_onboard_logging()
        if self.wait_for_altitude(timeout=min(timeout, 3.0)) is None:
            raise FlightSafetyError("no fresh downward DISTANCE_SENSOR altitude")
        self.set_mode(mode)
        self._fresh_disarmed()
        if not self.altitude_is_fresh():
            raise FlightSafetyError("downward altitude became stale before arming")
        self._arm_command_sent = True
        self._connection().arducopter_arm()
        armed, refusals = self._await_arm_confirmation(timeout)
        if armed:
            self._armed_by_controller = True
            return
        if refusals:
            raise FlightSafetyError(
                "flight controller refused to arm: " + "; ".join(refusals)
            )
        raise TimeoutError(
            "flight controller did not confirm arming and gave no reason"
        )

    def _await_arm_confirmation(self, timeout: float) -> tuple[bool, list[str]]:
        """Wait for the armed heartbeat, collecting whatever the vehicle says.

        ArduPilot reports why it will not arm as STATUSTEXT.  Discarding those
        lines and raising a bare timeout throws away the only explanation that
        exists, which is exactly what an operator needs.

        Returns whether arming was confirmed, and every distinct line the
        vehicle sent while we waited.
        """

        refusals: list[str] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self._connection().recv_match(
                type=["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK"],
                blocking=True,
                timeout=0.5,
            )
            if message is not None and self._matching_vehicle_message(message):
                kind = message.get_type()
                if kind == "HEARTBEAT":
                    self._process_message(message, time.monotonic())
                elif kind == "STATUSTEXT":
                    line = str(message.text).strip()
                    if line and line not in refusals:
                        refusals.append(line)
                elif int(getattr(message, "command", 0)) == (
                    mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                ):
                    result = int(message.result)
                    if result != mavlink.MAV_RESULT_ACCEPTED:
                        line = f"arm command rejected (MAV_RESULT {result})"
                        if line not in refusals:
                            refusals.append(line)
            if self.is_armed:
                return True, refusals
        return False, refusals

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
        finite_in_range(timeout, "timeout", minimum=1.0, maximum=120.0)
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
        """Request LAND without force-disarming a possibly airborne vehicle.

        This is deliberately a single, fast, unverified request so a guard can
        call it the instant something is wrong.  It is *not* sufficient on its
        own: a request that was lost or refused leaves the aircraft flying.
        Every caller must follow it with ``ensure_landed`` before giving up the
        connection.
        """
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

    def ensure_landed(self, timeout: float = 60.0, retry_every: float = 1.0) -> bool:
        """Keep commanding LAND until the vehicle confirms it has disarmed.

        A single ``SET_MODE`` can be lost or refused, and an aircraft that never
        received it keeps doing whatever it was last told to do.  This re-sends
        the request on an interval and only reports success when the vehicle's
        own heartbeat says it is disarmed.

        Returns whether disarm was confirmed.  It never raises for a failed
        landing and never closes the connection: the caller is expected to stay
        connected and keep trying, because hanging up on an airborne aircraft
        is the one thing that must not happen.
        """

        finite_in_range(timeout, "timeout", minimum=1.0, maximum=300.0)
        finite_in_range(retry_every, "retry_every", minimum=0.1, maximum=10.0)
        if self.connection is None:
            return False

        deadline = time.monotonic() + timeout
        next_request = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_request:
                next_request = now + retry_every
                try:
                    self.emergency_stop()
                except (OSError, RuntimeError, FlightSafetyError) as error:
                    logger.error("could not request LAND: %s", error)
            try:
                self.update_telemetry()
            except FlightSafetyError:
                # An altitude-ceiling complaint while we are already landing
                # tells us nothing new and must not stop the landing.
                pass
            except (OSError, RuntimeError) as error:
                logger.error("telemetry failed while landing: %s", error)
            if not self.is_armed:
                self.is_flying = False
                return True
            time.sleep(0.1)
        return False

    # -- position-free flight ------------------------------------------

    # SET_ATTITUDE_TARGET type mask: use the attitude quaternion and the
    # thrust field, ignore the three body rate fields.
    ATTITUDE_TARGET_MASK = 0b0000_0111
    # With GUID_OPTIONS bit 3 clear -- the ArduPilot default -- the thrust
    # field of SET_ATTITUDE_TARGET is a *climb rate*, not a throttle: 0.5
    # holds altitude, 1.0 climbs at PILOT_SPEED_UP.  ArduPilot keeps the
    # altitude loop, so this never commands raw motor power.
    NEUTRAL_THRUST = 0.5
    # Hard bounds on what this class will ever send, whatever it is asked for.
    MIN_THRUST = 0.25
    MAX_THRUST = 0.80
    # The measured climb rate this class tolerates before it stops asking for a
    # climb at all.  The meaning of the thrust field depends on the vehicle's
    # GUID_OPTIONS; if that assumption is ever wrong, a "gentle" request becomes
    # a fast climb and nothing else here would notice.  Measuring the aircraft
    # instead of trusting the request is the only check that survives being
    # wrong about the protocol.
    MAX_MEASURED_CLIMB_MS = 0.45
    # Short on purpose.  If the thrust field turns out to mean throttle, the
    # climb is violent, and every extra tenth of a second of detection window
    # is another tenth of a metre of altitude before anything reacts.
    CLIMB_RATE_WINDOW_S = 0.2

    def send_level_climb(self, climb: float) -> None:
        """Command a level attitude and a normalized climb rate.

        ``climb`` is -1.0 to 1.0, where 0.0 holds altitude and 1.0 is a climb
        at ``PILOT_SPEED_UP``.  The vehicle must be in a GUIDED variant; the
        flight controller retains attitude stabilization and altitude control.
        """

        rate = finite_in_range(climb, "climb", minimum=-1.0, maximum=1.0)
        if not self.is_armed:
            raise FlightSafetyError("climb commands require an armed vehicle")
        thrust = min(
            self.MAX_THRUST, max(self.MIN_THRUST, self.NEUTRAL_THRUST + rate / 2.0)
        )
        half_yaw = (self._yaw_rad or 0.0) / 2.0
        self._connection().mav.set_attitude_target_send(
            0,
            self.target_system,
            self.target_component,
            self.ATTITUDE_TARGET_MASK,
            [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)],
            0.0,
            0.0,
            0.0,
            thrust,
        )

    def takeoff_without_position(
        self,
        target_alt: float,
        *,
        climb: float = 0.5,
        timeout: float = 30.0,
    ) -> None:
        """Climb to ``target_alt`` in GUIDED_NOGPS, using the downward range.

        This exists because a flow-only aircraft cannot arm in GUIDED at all:
        EKF3 does not begin optical-flow navigation until it has detected a
        takeoff, so the position estimate GUIDED requires does not exist while
        the aircraft is on the floor.  GUIDED_NOGPS arms without one, and the
        rangefinder is the altitude reference the whole way up.
        """

        target = finite_in_range(
            target_alt, "target_alt", minimum=0.15, maximum=self.max_altitude
        )
        finite_in_range(climb, "climb", minimum=0.05, maximum=1.0)
        finite_in_range(timeout, "timeout", minimum=1.0, maximum=120.0)

        if not self.is_armed:
            self.arm(mode="GUIDED_NOGPS")
        if not self._armed_by_controller:
            raise FlightSafetyError("takeoff requires arming by this controller")
        if self.wait_for_altitude(timeout=3.0) is None:
            raise FlightSafetyError("no fresh downward altitude before takeoff")

        self._flight_started_by_controller = True
        self.is_flying = True
        reference: tuple[float, float] | None = None
        started = time.monotonic()
        deadline = started + timeout
        try:
            while time.monotonic() < deadline:
                self.update_telemetry()
                now = time.monotonic()
                if now - started > 2.0 and (
                    not self.altitude_is_fresh() or not self.heartbeat_is_fresh()
                ):
                    self.emergency_stop()
                    raise FlightSafetyError("telemetry became stale during takeoff")
                altitude = self.current_altitude
                if (
                    self.last_telemetry_time >= started
                    and altitude is not None
                    and altitude >= target
                ):
                    return
                if altitude is not None:
                    if reference is None:
                        reference = (now, altitude)
                    elif now - reference[0] >= self.CLIMB_RATE_WINDOW_S:
                        measured = (altitude - reference[1]) / (now - reference[0])
                        if measured > self.MAX_MEASURED_CLIMB_MS:
                            self.emergency_stop()
                            raise FlightSafetyError(
                                f"climbing at {measured:.2f} m/s, faster than the "
                                f"{self.MAX_MEASURED_CLIMB_MS:.2f} m/s limit; the "
                                "commanded climb is not producing the expected motion"
                            )
                        reference = (now, altitude)
                self.send_level_climb(climb)
                time.sleep(0.1)
        except BaseException:
            self.emergency_stop()
            raise
        self.emergency_stop()
        raise TimeoutError("takeoff altitude was not reached")

    def hold_altitude(self, duration: float, on_sample: Any = None) -> None:
        """Hold the current altitude by commanding a zero climb rate."""

        finite_in_range(duration, "duration", minimum=0.1, maximum=600.0)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.update_telemetry()
            if on_sample is not None:
                on_sample(self)
            self.send_level_climb(0.0)
            time.sleep(0.1)

    def send_velocity_body(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate_deg: float = 0.0,
    ) -> None:
        forward = finite_in_range(vx, "vx", minimum=-1.0, maximum=1.0)
        right = finite_in_range(vy, "vy", minimum=-1.0, maximum=1.0)
        down = finite_in_range(vz, "vz", minimum=-0.5, maximum=0.5)
        yaw_rate = finite_in_range(
            yaw_rate_deg, "yaw_rate_deg", minimum=-45.0, maximum=45.0
        )
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
