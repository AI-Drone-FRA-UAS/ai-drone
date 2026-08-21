"""Canonical, safety-gated MAVLink flight controller for ArduPilot Copter."""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
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


@dataclass(frozen=True)
class ThrottleCalibration:
    """The vehicle's own throttle numbers, read from the vehicle.

    STABILIZE maps the throttle channel straight onto motor thrust, so "how
    much throttle is a hover" stops being a detail and becomes the whole
    flight.  The only defensible answer is the one ArduPilot learned for
    itself in ``MOT_THST_HOVER``, expressed against this receiver's own
    calibrated endpoints.  Nothing here is a constant in this repository.
    """

    minimum_pwm: int
    maximum_pwm: int
    roll_trim_pwm: int
    pitch_trim_pwm: int
    yaw_trim_pwm: int
    hover: float
    deadzone: int

    @property
    def span_pwm(self) -> int:
        return self.maximum_pwm - self.minimum_pwm

    @property
    def middle_pwm(self) -> int:
        return (self.maximum_pwm + self.minimum_pwm) // 2

    def pwm_for(self, normalized: float) -> int:
        """Convert a 0.0-1.0 throttle fraction to a calibrated PWM value."""

        clamped = min(1.0, max(0.0, normalized))
        return round(self.minimum_pwm + self.span_pwm * clamped)

    @property
    def deadzone_pwm(self) -> int:
        """THR_DZ against a 0-1000 stick, scaled onto this receiver's span."""

        return round(self.deadzone * self.span_pwm / 1_000.0)


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
        self.local_position_climb: float | None = None
        self._ground_reference: float | None = None
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
        self._manual_descent = False
        self._grounded_since: float | None = None
        # Set by the caller to put a human in the loop; see ai_drone.abort_key.
        self.abort_requested: Any | None = None
        self._rc_override: tuple[int, int, int, int] | None = None
        self._rc_override_sent = 0.0
        self._throttle_calibration: ThrottleCalibration | None = None

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
        # A standing RC override outlives this object on the vehicle, so it is
        # released here -- but only once the vehicle says it is disarmed.
        # Handing the throttle back on an armed aircraft with no receiver
        # fitted hands it to nothing at all.
        if self.connection is not None and self._rc_override is not None:
            try:
                self.clear_rc_override()
            except (OSError, RuntimeError, FlightSafetyError) as error:
                logger.error("could not release the RC override: %s", error)
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
                # This airframe does not publish LOCAL_POSITION_NED at all --
                # its EKF has no horizontal solution on the ground, so there
                # is no local frame to report.  VFR_HUD carries the vertical
                # estimate regardless, and it is the only place the -10000 m
                # divergence of 2026-08-21 is visible before a flight.
                mavlink.MAVLINK_MSG_ID_VFR_HUD: float(rate_hz),
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
                self._ground_reference = None
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
                # The EKF's own vertical rate.  This is the number LAND acts
                # on, so it is the number worth sanity-checking before asking
                # for LAND -- see ``vertical_estimate_is_sane``.
                climb = -float(message.vz)
                if math.isfinite(climb):
                    self.local_position_climb = climb
            return
        if message_type == "DISTANCE_SENSOR":
            self._process_distance_sensor(message, now)
        elif message_type == "VFR_HUD":
            # No timestamp on VFR_HUD to check, but it is the vertical
            # estimate this vehicle actually sends.  Only fill in what
            # LOCAL_POSITION_NED has not already provided.
            climb = float(message.climb)
            if math.isfinite(climb):
                self.local_position_climb = climb
            altitude = float(message.alt)
            if math.isfinite(altitude) and self.local_position_altitude is None:
                self.local_position_altitude = altitude
        elif message_type == "EKF_STATUS_REPORT":
            self.ekf_flags = int(message.flags)
        elif message_type == "SYS_STATUS":
            voltage = float(message.voltage_battery) / 1_000.0
            if math.isfinite(voltage) and voltage > 0.0:
                self.battery_voltage = voltage

    def _process_distance_sensor(self, message: Any, now: float) -> None:
        """Accept one downward rangefinder reading, or reject it and say nothing.

        Every altitude limit in this class is built on this value, so a
        reading that fails any of these tests must not become
        ``current_altitude`` -- and must not refresh ``last_telemetry_time``
        either, because that is what the staleness guards read.
        """

        if int(message.orientation) != DOWNWARD_ORIENTATION:
            return
        if not self._timestamp_is_fresh(getattr(message, "time_boot_ms", None), now):
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

    def update_telemetry(self, max_messages: int = 50) -> None:
        if isinstance(max_messages, bool) or not 1 <= max_messages <= 1_000:
            raise ValueError("max_messages must be between 1 and 1000")
        if self.connection is None:
            return
        self._pump_rc_override()
        if self._operator_asked_to_stop():
            # Checked before reading telemetry, not after: the operator is
            # reacting to the aircraft, and nothing this loop is about to
            # learn changes what they asked for.
            self.stop_now()
            raise FlightSafetyError("operator pressed the abort key")
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

    def _operator_asked_to_stop(self) -> bool:
        callback = self.abort_requested
        if callback is None:
            return False
        try:
            return bool(callback())
        except Exception as error:
            # A broken abort hook must not become a reason to keep flying, but
            # it also must not stop a flight that nobody asked to stop.
            logger.error("abort-key check failed: %s", error)
            return False

    def stop_now(self) -> None:
        """Force the motors off immediately, whatever the aircraft is doing.

        This is not ``emergency_stop``.  That one picks the gentlest ending the
        evidence supports and can end in LAND, an altitude-controlled mode.
        This one cuts the motors: ArduPilot refuses an ordinary disarm in
        flight, so the force magic number is required, and the aircraft will
        drop.  That is the intent.  It exists for the case the guards cannot
        see -- an operator watching the aircraft do something wrong -- and from
        the half-metre this airframe is flown at, a drop is a far better
        outcome than a climb nobody can stop.
        """

        if self.connection is None:
            return
        self._landing_commanded = True
        logger.error("FORCED DISARM: cutting the motors now")
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # 0 = disarm
            self.FORCE_DISARM_MAGIC,
            0,
            0,
            0,
            0,
            0,
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
        self._pump_rc_override()
        self._fresh_disarmed()
        self._pump_rc_override()
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
            self._pump_rc_override()
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
        self._ground_reference = self.current_altitude
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
            altitude = self.current_altitude
            if (
                not self._flight_started_by_controller
                and altitude is not None
                and self._ground_reference is not None
                and altitude > self._ground_reference + self.LIFTOFF_MARGIN_M
            ):
                # Only now is this a flight.  Declaring it earlier is what
                # made a climb that never lifted look like an airborne
                # emergency on 2026-08-21, and sent it to LAND.
                self._flight_started_by_controller = True
                self.is_flying = True
            if (
                self.last_telemetry_time >= started
                and altitude is not None
                and altitude >= target * 0.95
            ):
                return
            time.sleep(0.05)
        self.emergency_stop()
        raise TimeoutError("takeoff altitude was not reached")

    def land(self, timeout: float = 30.0) -> None:
        finite_in_range(timeout, "timeout", minimum=1.0, maximum=120.0)
        self._landing_commanded = True
        # Back an overridden throttle off before asking for LAND.  Just below
        # hover is a descent in whichever mode the vehicle is still in, so it
        # is safe to send first and does not depend on LAND being accepted.
        self._back_off_throttle_override()
        if self.never_left_the_ground():
            # Ending a flight that never began.  LAND would hand a grounded
            # aircraft to an altitude controller for no reason at all, and on
            # 2026-08-21 that controller answered a diverged estimate with
            # full throttle.
            logger.warning(
                "ending with a disarm rather than LAND: the aircraft never left "
                "the ground"
            )
            self._request_disarm()
        else:
            self.set_mode("LAND")
        # ensure_landed owns every wait for a disarm, so that the escape from a
        # LAND that climbs exists on this path too.  Waiting here separately is
        # what let a misbehaving LAND run unwatched for a full timeout.
        if not self.ensure_landed(timeout=timeout):
            raise TimeoutError(
                "LAND remains commanded but disarming was not confirmed; do not "
                "approach the vehicle"
            )
        self.is_flying = False
        if self._rc_override is not None:
            self.clear_rc_override()

    # A vertical rate this large is not motion on an indoor aircraft whose
    # climb rate is limited to 0.25 m/s; it is a diverged filter.  On
    # 2026-08-21 the EKF reported 38 m/s of descent while the aircraft sat on
    # the floor, and LAND answered that estimate with full throttle.
    MAX_PLAUSIBLE_CLIMB_MS = 5.0
    # How far above its arming reading the rangefinder must go before this
    # controller will say the aircraft is airborne.  ArduPilot's own takeoff
    # detector uses about the same margin.
    LIFTOFF_MARGIN_M = 0.05
    # ArduPilot refuses an ordinary disarm while flying.  This value in param2
    # of MAV_CMD_COMPONENT_ARM_DISARM is its documented override, and it is the
    # only way to cut the motors of an aircraft that is doing something wrong.
    FORCE_DISARM_MAGIC = 21196
    # How far a LAND may climb before it stops being treated as a landing.
    # Larger than the rangefinder's centimetre rounding and than the settle a
    # real touchdown produces, small enough to act inside a metre.
    LAND_CLIMB_ESCAPE_M = 0.15
    # How long a LAND gets to start working before an impossible vertical
    # estimate is treated as proof that it never will.
    LAND_ESCAPE_AFTER_S = 2.0

    def vertical_estimate_is_sane(self) -> bool:
        """Whether the EKF's vertical rate is physically possible for this aircraft.

        Reported as sane when there is no estimate at all: the absence of a
        number is not evidence against LAND, and refusing to stop an aircraft
        because a message is missing would be worse than the failure this
        guards against.
        """

        climb = self.local_position_climb
        if climb is None or not math.isfinite(climb):
            return True
        return abs(climb) <= self.MAX_PLAUSIBLE_CLIMB_MS

    def _rangefinder_says_grounded(self) -> bool:
        """Whether a fresh rangefinder puts the aircraft back at its reference.

        False on a stale reading: a number nobody can vouch for is not
        evidence of anything, least of all of it being safe to disarm.
        """

        if not self.altitude_is_fresh():
            return False
        altitude = self.current_altitude
        reference = self._ground_reference
        if altitude is None or reference is None:
            return False
        return altitude <= reference + self.LIFTOFF_MARGIN_M

    def never_left_the_ground(self) -> bool:
        """Whether the aircraft demonstrably never became airborne.

        Deliberately narrow.  It is false the moment this controller has seen
        a liftoff, so it can never disarm an aircraft that is flying.
        """

        if self._flight_started_by_controller:
            return False
        return self._rangefinder_says_grounded()

    def settled_on_the_ground(self, for_seconds: float = 0.5) -> bool:
        """Whether the rangefinder has read ground level for long enough to act on.

        Used only during a manual descent, where the altitude estimate is
        already known to be untrustworthy and the rangefinder is all there is.
        A single low sample is not enough -- an aircraft passing over an
        obstacle produces one -- so this requires the reading to hold.
        """

        if not self._rangefinder_says_grounded():
            self._grounded_since = None
            return False
        now = time.monotonic()
        if self._grounded_since is None:
            self._grounded_since = now
            return False
        return now - self._grounded_since >= for_seconds

    def emergency_stop(self) -> None:
        """Stop the aircraft by the safest route the evidence supports.

        This is deliberately a single, fast, unverified request so a guard can
        call it the instant something is wrong.  It is *not* sufficient on its
        own: a request that was lost or refused leaves the aircraft flying.
        Every caller must follow it with ``ensure_landed`` before giving up the
        connection.

        LAND is an altitude-controlled mode, so it is only ever as good as the
        vehicle's vertical estimate.  On 2026-08-21 this method requested LAND
        on an aircraft that had never left the floor, whose EKF reported
        -10000 m and a 38 m/s descent; the altitude controller went to full
        throttle in a single log sample and flew the aircraft into a ceiling.
        An aircraft that demonstrably never became airborne is therefore ended
        with a disarm, which no altitude controller can misread.
        """
        if self.connection is None:
            return
        self._landing_commanded = True
        if self.never_left_the_ground():
            logger.warning(
                "stopping with a disarm rather than LAND: the rangefinder still "
                "reports %.2f m against a %.2f m ground reference, so this "
                "aircraft never became airborne",
                self.current_altitude
                if self.current_altitude is not None
                else float("nan"),
                self._ground_reference
                if self._ground_reference is not None
                else float("nan"),
            )
            self._request_disarm()
            return
        if not self.vertical_estimate_is_sane():
            logger.error(
                "requesting LAND while the EKF reports %.1f m/s of vertical "
                "motion, which this aircraft cannot do. LAND is altitude "
                "controlled and may answer that estimate with full throttle. "
                "Be ready to cut power.",
                self.local_position_climb,
            )
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

        started = time.monotonic()
        deadline = started + timeout
        next_request = 0.0
        reference = self.current_altitude if self.altitude_is_fresh() else None
        escaped = self._manual_descent
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_request:
                next_request = now + retry_every
                try:
                    if self._operator_asked_to_stop():
                        # update_telemetry raises for this, and this loop
                        # deliberately swallows that exception so a landing
                        # cannot be interrupted.  Re-asserting it here is what
                        # keeps the operator's decision from being swallowed
                        # with it.
                        self.stop_now()
                    elif escaped:
                        # No longer asking LAND for anything: keep the manual
                        # descent throttle alive instead.
                        self._send_rc_override()
                    else:
                        self.emergency_stop()
                except (OSError, RuntimeError, FlightSafetyError) as error:
                    logger.error("could not command the vehicle: %s", error)
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

            altitude = self.current_altitude
            measured = altitude if self.altitude_is_fresh() else None
            # Whatever route brought it down, an aircraft the rangefinder has
            # held at ground level is one this loop can finish.  A manual
            # descent has no landing detector behind it, and a vehicle that
            # refuses LAND never runs its own.
            if self.settled_on_the_ground():
                self._request_disarm()
            if not escaped:
                climbed = (
                    measured is not None
                    and reference is not None
                    and measured > reference + self.LAND_CLIMB_ESCAPE_M
                )
                # The rangefinder runs out of range long before a runaway
                # climb does, so a measured climb cannot be the only trigger.
                # An estimate the aircraft cannot possibly be producing, still
                # armed a couple of seconds after LAND was asked for, is the
                # 2026-08-21 signature and does not depend on range at all.
                impossible_estimate = (
                    not self.vertical_estimate_is_sane()
                    and now - started > self.LAND_ESCAPE_AFTER_S
                )
                if climbed or impossible_estimate:
                    escaped = self._escape_to_manual_descent(
                        measured if measured is not None else float("nan"),
                        reference if reference is not None else float("nan"),
                    )
                    if escaped:
                        next_request = time.monotonic()
                elif measured is not None:
                    reference = (
                        measured if reference is None else min(reference, measured)
                    )
            time.sleep(0.1)
        return False

    def _escape_to_manual_descent(self, altitude: float, reference: float) -> bool:
        """Abandon LAND for STABILIZE and descend on a manual throttle.

        LAND is altitude controlled, so a diverged vertical estimate can make
        it answer a landing request with a climb -- on 2026-08-21 with full
        throttle.  Asking again gets the same answer from the same broken
        controller.  STABILIZE ignores the altitude estimate entirely: its
        attitude loop runs on the IMU, and a throttle below hover descends
        whatever the EKF believes.

        Returns whether the escape could be made.  Without a throttle
        calibration and a standing override there is no manual throttle to
        escape to, and LAND, however badly it is behaving, is still the only
        thing left to ask for.
        """

        if self._throttle_calibration is None or self._rc_override is None:
            logger.error(
                "LAND has climbed from %.2f m to %.2f m and there is no throttle "
                "override to take over with. Cut power from a safe distance.",
                reference,
                altitude,
            )
            return False
        logger.error(
            "LAND is climbing (%.2f m to %.2f m). Abandoning it for STABILIZE on "
            "a below-hover throttle, which no altitude estimate can affect.",
            reference,
            altitude,
        )
        # Throttle first, mode second.  The other order hands STABILIZE
        # whatever stick happens to be standing for as long as the mode change
        # takes to confirm.
        self._back_off_throttle_override()
        try:
            connection = self._connection()
            connection.mav.set_mode_send(
                self.target_system,
                mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                connection.mode_mapping()["STABILIZE"],
            )
        except (OSError, RuntimeError, KeyError) as error:
            logger.error("could not leave LAND for STABILIZE: %s", error)
            return False
        # _landing_commanded deliberately stays set.  Clearing it re-arms the
        # ceiling check in update_telemetry, which answers an over-height
        # aircraft by calling emergency_stop -- putting it straight back into
        # the LAND this method just escaped.
        self._manual_descent = True
        return True

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

        self._ground_reference = self.current_altitude
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
                    not self._flight_started_by_controller
                    and altitude is not None
                    and self._ground_reference is not None
                    and altitude > self._ground_reference + self.LIFTOFF_MARGIN_M
                ):
                    # Only now is this a flight.  Declaring it earlier is what
                    # made a climb that never lifted look like an airborne
                    # emergency on 2026-08-21, and sent it to LAND.
                    self._flight_started_by_controller = True
                    self.is_flying = True
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

    # -- STABILIZE climb ------------------------------------------------

    # ArduPilot lets an RC override lapse after RC_OVERRIDE_TIME, 3 s on this
    # vehicle.  When it lapses control returns to the receiver, and this
    # airframe has none, so a lapsed override leaves STABILIZE with no
    # throttle source at all while the aircraft is in the air.  Refresh far
    # inside that window.
    RC_OVERRIDE_REFRESH_S = 0.4
    # The most throttle above the learned hover value this class will ever
    # command in STABILIZE.  In a mode with no altitude loop this number *is*
    # the climb authority: every metre of altitude comes from it, and nothing
    # in the autopilot will take it back.
    MAX_THROTTLE_ABOVE_HOVER = 0.10
    # The ramp starts below hover so that the first seconds of an armed
    # aircraft cannot lift it, and any surprise in the thrust curve shows up
    # as motion before there is enough throttle to climb on.
    STABILIZE_RAMP_START_BELOW_HOVER = 0.10
    STABILIZE_RAMP_SECONDS = 2.0
    # Backing the throttle off by this much below hover is the one command
    # that is safe under every reading of the stick: a gentle descent in
    # STABILIZE, a bounded descent in ALT_HOLD, and ignored in LAND.
    ABORT_THROTTLE_BELOW_HOVER = 0.04
    # STABILIZE gives away no altitude control whatsoever, so the measured
    # climb is the only thing that can report that the commanded throttle is
    # producing more thrust than expected.
    #
    # The limit is deliberately not tight.  The rangefinder reports whole
    # centimetres, so over this window rounding alone moves the measurement by
    # about 0.10 m/s, and a guard that aborts a healthy climb is a guard that
    # gets turned off.  What this catches is the 2026-08-20 failure -- a
    # commanded value producing a multiple of the expected thrust -- not a
    # thirty percent error.  A slow runaway is the altitude ceiling's job, and
    # ``update_telemetry`` enforces that on every message.
    MAX_STABILIZE_CLIMB_MS = 0.60
    STABILIZE_CLIMB_WINDOW_S = 0.2

    def read_throttle_calibration(self) -> ThrottleCalibration:
        """Read the vehicle's own throttle mapping before commanding any of it.

        Refuses anything it cannot make sense of.  A guessed hover throttle in
        STABILIZE is a guessed climb rate, and a wrong one is a flyaway.
        """

        connection = self._connection()
        names = (
            "RCMAP_ROLL",
            "RCMAP_PITCH",
            "RCMAP_THROTTLE",
            "RCMAP_YAW",
            "RC1_TRIM",
            "RC2_TRIM",
            "RC3_MIN",
            "RC3_MAX",
            "RC4_TRIM",
            "MOT_THST_HOVER",
            "THR_DZ",
        )
        values = {name: request_parameter(connection, name) for name in names}

        # This class sends the first four override channels positionally, so a
        # vehicle that maps its sticks anywhere else must be refused rather
        # than quietly flown with roll on the throttle.
        for name, channel in (
            ("RCMAP_ROLL", 1),
            ("RCMAP_PITCH", 2),
            ("RCMAP_THROTTLE", 3),
            ("RCMAP_YAW", 4),
        ):
            if int(values[name]) != channel:
                raise FlightSafetyError(
                    f"{name} is {values[name]:.0f}, not {channel}; this command sends "
                    "the first four override channels in the default RCMAP order"
                )

        minimum = int(values["RC3_MIN"])
        maximum = int(values["RC3_MAX"])
        if not 800 <= minimum < maximum <= 2_200 or maximum - minimum < 400:
            raise FlightSafetyError(
                f"throttle channel is not calibrated: RC3_MIN={minimum}, "
                f"RC3_MAX={maximum}"
            )
        hover = float(values["MOT_THST_HOVER"])
        if not math.isfinite(hover) or not 0.05 <= hover <= 0.7:
            raise FlightSafetyError(
                f"MOT_THST_HOVER is {hover:.3f}, outside the range this command will "
                "build a STABILIZE throttle from"
            )
        return ThrottleCalibration(
            minimum_pwm=minimum,
            maximum_pwm=maximum,
            roll_trim_pwm=int(values["RC1_TRIM"]),
            pitch_trim_pwm=int(values["RC2_TRIM"]),
            yaw_trim_pwm=int(values["RC4_TRIM"]),
            hover=hover,
            deadzone=max(0, int(values["THR_DZ"])),
        )

    def _calibration(self) -> ThrottleCalibration:
        if self._throttle_calibration is None:
            raise FlightSafetyError("throttle calibration has not been read")
        return self._throttle_calibration

    def _set_rc_override(self, throttle_pwm: int) -> None:
        """Hold the sticks level and the throttle at ``throttle_pwm``.

        Roll, pitch and yaw are overridden alongside the throttle on purpose.
        With no receiver fitted, a channel left un-overridden has no source,
        and STABILIZE takes its entire attitude command from those sticks.
        """

        calibration = self._calibration()
        if isinstance(throttle_pwm, bool) or not (
            calibration.minimum_pwm <= int(throttle_pwm) <= calibration.maximum_pwm
        ):
            raise FlightSafetyError(
                f"throttle {throttle_pwm} is outside the calibrated range "
                f"{calibration.minimum_pwm}-{calibration.maximum_pwm}"
            )
        self._rc_override = (
            calibration.roll_trim_pwm,
            calibration.pitch_trim_pwm,
            int(throttle_pwm),
            calibration.yaw_trim_pwm,
        )
        self._send_rc_override()

    def _send_rc_override(self) -> None:
        override = self._rc_override
        if override is None or self.connection is None:
            return
        roll, pitch, throttle, yaw = override
        self.connection.mav.rc_channels_override_send(
            self.target_system,
            self.target_component,
            roll,
            pitch,
            throttle,
            yaw,
            0,
            0,
            0,
            0,
        )
        self._rc_override_sent = time.monotonic()

    def _pump_rc_override(self) -> None:
        """Re-send the standing override before ArduPilot lets it lapse.

        Every loop that can block for more than a moment goes through here, so
        that no wait -- for a mode confirmation, for an arm confirmation, for a
        landing -- can silently drop the only throttle source the aircraft has.
        """

        if self._rc_override is None or self.connection is None:
            return
        if time.monotonic() - self._rc_override_sent >= self.RC_OVERRIDE_REFRESH_S:
            self._send_rc_override()

    def clear_rc_override(self) -> None:
        """Release every overridden channel, once the vehicle is disarmed."""

        if self.connection is None:
            return
        if self.is_armed:
            raise FlightSafetyError(
                "refusing to release the RC override while the vehicle is armed"
            )
        self._rc_override = None
        self.connection.mav.rc_channels_override_send(
            self.target_system, self.target_component, 0, 0, 0, 0, 0, 0, 0, 0
        )

    def command_stabilize_throttle(self, above_hover: float) -> None:
        """Command a STABILIZE throttle, expressed relative to learned hover.

        The mode is verified from the vehicle's own heartbeat before anything
        is sent.  The same stick position means motor thrust in STABILIZE and
        a climb rate in ALT_HOLD, so a value that is gentle in one mode is
        violent in the other; checking which mode the vehicle reports it is in
        is what keeps the two apart.
        """

        finite_in_range(above_hover, "above_hover", minimum=-1.0, maximum=1.0)
        if not self.is_armed:
            raise FlightSafetyError("throttle commands require an armed vehicle")
        if self.flight_mode != "STABILIZE":
            raise FlightSafetyError(
                f"refusing to send a STABILIZE throttle while the vehicle reports "
                f"{self.flight_mode or 'an unknown mode'}"
            )
        calibration = self._calibration()
        bounded = min(self.MAX_THROTTLE_ABOVE_HOVER, above_hover)
        self._set_rc_override(calibration.pwm_for(calibration.hover + bounded))

    def command_alt_hold_climb(self, climb: float) -> None:
        """Command an ALT_HOLD climb rate, ``0.0`` meaning hold this altitude.

        ArduPilot owns the altitude loop here and bounds the result by
        PILOT_SPEED_UP and PILOT_SPEED_DN; this only places the stick.
        """

        rate = finite_in_range(climb, "climb", minimum=-1.0, maximum=1.0)
        if not self.is_armed:
            raise FlightSafetyError("throttle commands require an armed vehicle")
        if self.flight_mode != "ALT_HOLD":
            raise FlightSafetyError(
                f"refusing to send an ALT_HOLD climb rate while the vehicle reports "
                f"{self.flight_mode or 'an unknown mode'}"
            )
        calibration = self._calibration()
        middle = calibration.middle_pwm
        deadzone = calibration.deadzone_pwm
        if rate == 0.0:
            self._set_rc_override(middle)
            return
        if rate > 0.0:
            span = calibration.maximum_pwm - middle - deadzone
            self._set_rc_override(round(middle + deadzone + rate * span))
            return
        span = middle - deadzone - calibration.minimum_pwm
        self._set_rc_override(round(middle - deadzone + rate * span))

    def _ramped_throttle(self, elapsed: float, climb: float) -> float:
        """Ramp from below hover up to ``climb`` over ``STABILIZE_RAMP_SECONDS``."""

        fraction = min(1.0, max(0.0, elapsed / self.STABILIZE_RAMP_SECONDS))
        start = -self.STABILIZE_RAMP_START_BELOW_HOVER
        return start + fraction * (climb - start)

    def abort(self) -> None:
        """Stop the aircraft and back the throttle off to a value safe in any mode.

        An abort cannot assume its own mode change was accepted, so the
        throttle it leaves behind has to be safe whether the vehicle is still
        in STABILIZE, already in ALT_HOLD, or in LAND.  Just below hover is
        that value; centring the stick would not be.

        Whether this ends in LAND or in a disarm is ``emergency_stop``'s
        decision, and it turns on whether the aircraft ever actually left the
        ground.  This was ``abort_to_land`` until 2026-08-21, when requesting
        LAND unconditionally flew a grounded aircraft into a ceiling.
        """

        try:
            self.emergency_stop()
        except (OSError, RuntimeError, FlightSafetyError) as error:
            logger.error("could not stop the aircraft: %s", error)
        self._back_off_throttle_override()

    def _back_off_throttle_override(self) -> None:
        """Move a standing override to the one throttle safe in every mode."""

        if self._throttle_calibration is None or self._rc_override is None:
            return
        calibration = self._throttle_calibration
        try:
            self._set_rc_override(
                calibration.pwm_for(
                    max(0.0, calibration.hover - self.ABORT_THROTTLE_BELOW_HOVER)
                )
            )
        except (OSError, RuntimeError, FlightSafetyError) as error:
            logger.error("could not back the throttle override off: %s", error)

    def climb_in_stabilize(
        self,
        target_alt: float,
        *,
        climb: float = 0.06,
        timeout: float = 12.0,
    ) -> None:
        """Climb to ``target_alt`` in STABILIZE on an overridden throttle.

        STABILIZE has no altitude controller: the throttle stick is motor
        thrust, and the aircraft accelerates upward for as long as that thrust
        exceeds its weight.  Everything protective here is therefore in this
        loop -- a bounded throttle built from the vehicle's learned hover, a
        ramp onto it, a measured climb-rate limit, and the altitude ceiling in
        ``update_telemetry`` -- because the autopilot contributes none of it.
        """

        target = finite_in_range(
            target_alt, "target_alt", minimum=0.15, maximum=self.max_altitude
        )
        finite_in_range(
            climb, "climb", minimum=0.01, maximum=self.MAX_THROTTLE_ABOVE_HOVER
        )
        # Short on purpose.  An aircraft still on the floor after this long is
        # an aircraft holding hover throttle plus a climb margin against the
        # ground, and the longer that lasts the worse whatever is stuck about
        # it gets.
        finite_in_range(timeout, "timeout", minimum=1.0, maximum=60.0)

        self._throttle_calibration = self.read_throttle_calibration()
        calibration = self._throttle_calibration
        # Hold the throttle at its calibrated minimum *before* arming.  With
        # RC_OPTIONS bit 5 set the vehicle refuses to arm otherwise, and an
        # override that only starts after arming leaves a window in which
        # STABILIZE has no throttle source.
        self._set_rc_override(calibration.minimum_pwm)
        if not self.is_armed:
            self.arm(mode="STABILIZE")
        if not self._armed_by_controller:
            raise FlightSafetyError("takeoff requires arming by this controller")
        if self.wait_for_altitude(timeout=3.0) is None:
            raise FlightSafetyError("no fresh downward altitude before takeoff")

        self._ground_reference = self.current_altitude
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
                    raise FlightSafetyError("telemetry became stale during the climb")
                altitude = self.current_altitude
                if (
                    not self._flight_started_by_controller
                    and altitude is not None
                    and self._ground_reference is not None
                    and altitude > self._ground_reference + self.LIFTOFF_MARGIN_M
                ):
                    # Only now is this a flight.  Declaring it earlier is what
                    # made a climb that never lifted look like an airborne
                    # emergency on 2026-08-21, and sent it to LAND.
                    self._flight_started_by_controller = True
                    self.is_flying = True
                if (
                    self.last_telemetry_time >= started
                    and altitude is not None
                    and altitude >= target
                ):
                    return
                if altitude is not None:
                    if reference is None:
                        reference = (now, altitude)
                    elif now - reference[0] >= self.STABILIZE_CLIMB_WINDOW_S:
                        measured = (altitude - reference[1]) / (now - reference[0])
                        if measured > self.MAX_STABILIZE_CLIMB_MS:
                            raise FlightSafetyError(
                                f"climbing at {measured:.2f} m/s, faster than the "
                                f"{self.MAX_STABILIZE_CLIMB_MS:.2f} m/s limit; the "
                                "commanded throttle is producing more thrust than "
                                "the learned hover value predicts"
                            )
                        reference = (now, altitude)
                self.command_stabilize_throttle(
                    self._ramped_throttle(now - started, climb)
                )
                time.sleep(0.05)
        except BaseException:
            self.abort()
            raise
        self.abort()
        raise TimeoutError("the STABILIZE climb did not reach the target altitude")

    # ArduPilot bounds an ALT_HOLD climb by PILOT_SPEED_UP, which is 0.25 m/s
    # on this aircraft.  A measured climb well past that means the stick is
    # not being read as a climb rate at all -- the mode confusion that makes
    # this airframe dangerous -- rather than a slightly brisk takeoff.
    MAX_ALT_HOLD_CLIMB_MS = 0.50
    ALT_HOLD_CLIMB_WINDOW_S = 0.2

    def climb_in_alt_hold(
        self,
        target_alt: float,
        *,
        climb: float = 0.5,
        timeout: float = 20.0,
    ) -> None:
        """Climb to ``target_alt`` in ALT_HOLD, letting ArduPilot fly the climb.

        This exists because the two things that made the earlier routes up
        dangerous are both absent here.  There is no thrust mapping to get
        wrong: the stick is a climb rate, bounded by ``PILOT_SPEED_UP``, and
        ArduPilot owns the altitude loop.  And ALT_HOLD needs no position
        estimate, so unlike GUIDED it can arm on the floor -- the vehicle's own
        pre-arm verdict says ``Need Position Estimate`` for GUIDED and says
        nothing of the sort here.

        What it does depend on is the vehicle's vertical estimate, which is
        why this is only safe now: with ``EK3_SRC1_POSZ`` on the barometer the
        aircraft reports +0.00 m/s standing still, where it reported -17.78 m/s
        before.  The guards below assume nothing about that and measure the
        climb anyway.
        """

        target = finite_in_range(
            target_alt, "target_alt", minimum=0.15, maximum=self.max_altitude
        )
        finite_in_range(climb, "climb", minimum=0.05, maximum=1.0)
        finite_in_range(timeout, "timeout", minimum=1.0, maximum=60.0)

        self._throttle_calibration = self.read_throttle_calibration()
        calibration = self._throttle_calibration
        # The throttle has to be at its calibrated minimum before arming, and
        # an override that only starts afterwards leaves a window in which the
        # vehicle has no throttle source at all.
        self._set_rc_override(calibration.minimum_pwm)
        if not self.is_armed:
            self.arm(mode="ALT_HOLD")
        if not self._armed_by_controller:
            raise FlightSafetyError("takeoff requires arming by this controller")
        if self.wait_for_altitude(timeout=3.0) is None:
            raise FlightSafetyError("no fresh downward altitude before takeoff")

        self._ground_reference = self.current_altitude
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
                    raise FlightSafetyError("telemetry became stale during the climb")
                altitude = self.current_altitude
                if (
                    not self._flight_started_by_controller
                    and altitude is not None
                    and self._ground_reference is not None
                    and altitude > self._ground_reference + self.LIFTOFF_MARGIN_M
                ):
                    self._flight_started_by_controller = True
                    self.is_flying = True
                if (
                    self.last_telemetry_time >= started
                    and altitude is not None
                    and altitude >= target
                ):
                    return
                if altitude is not None:
                    if reference is None:
                        reference = (now, altitude)
                    elif now - reference[0] >= self.ALT_HOLD_CLIMB_WINDOW_S:
                        measured = (altitude - reference[1]) / (now - reference[0])
                        if measured > self.MAX_ALT_HOLD_CLIMB_MS:
                            raise FlightSafetyError(
                                f"climbing at {measured:.2f} m/s against a "
                                f"{self.MAX_ALT_HOLD_CLIMB_MS:.2f} m/s limit; "
                                "ArduPilot is not bounding this by PILOT_SPEED_UP, "
                                "so the stick is not being read as a climb rate"
                            )
                        reference = (now, altitude)
                self.command_alt_hold_climb(climb)
                time.sleep(0.05)
        except BaseException:
            self.abort()
            raise
        self.abort()
        raise TimeoutError("the ALT_HOLD climb did not reach the target altitude")

    def handover_to_alt_hold(self, timeout: float = 5.0) -> None:
        """Hand altitude control to ArduPilot, in the one order that is safe.

        The mode change goes first.  Centring the throttle stick while still in
        STABILIZE would command roughly half throttle -- close to twice hover
        on this airframe -- for however long the mode change takes to confirm.
        Doing it in this order costs a brief descent at PILOT_SPEED_DN instead,
        and nothing else.
        """

        try:
            self.set_mode("ALT_HOLD", timeout=timeout)
            self.command_alt_hold_climb(0.0)
        except BaseException:
            self.abort()
            raise

    def hold_in_alt_hold(self, duration: float, on_sample: Any = None) -> None:
        """Hold altitude in ALT_HOLD with the throttle stick centred."""

        finite_in_range(duration, "duration", minimum=0.1, maximum=600.0)
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline:
                self.update_telemetry()
                if on_sample is not None:
                    on_sample(self)
                self.command_alt_hold_climb(0.0)
                time.sleep(0.1)
        except BaseException:
            self.abort()
            raise
