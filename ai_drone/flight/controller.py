"""Canonical, safety-gated MAVLink flight controller for ArduPilot Copter."""

from __future__ import annotations

import logging
import math
import os
import time
from collections import deque
from collections.abc import Callable, Iterator
from dataclasses import replace
from pathlib import Path
from typing import Any, assert_never

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight.limits import (
    MAX_PHYSICAL_ALTITUDE_M as MAX_PHYSICAL_ALTITUDE_M,
)
from ai_drone.flight.limits import (
    FlightEvidence,
    FlightLimits,
    relative_position_ready,
    takeoff_ceiling_violation,
    takeoff_reached,
    violation,
)
from ai_drone.flight.ownership import OwnershipPolicy, human_takeover_allowed
from ai_drone.flight.params import (
    EXPECTED_FIRMWARE_COMMIT as EXPECTED_FIRMWARE_COMMIT,
)
from ai_drone.flight.params import (
    EXPECTED_FIRMWARE_VERSION as EXPECTED_FIRMWARE_VERSION,
)
from ai_drone.flight.params import (
    FORWARD_RANGEFINDER_PARAMETERS as FORWARD_RANGEFINDER_PARAMETERS,
)
from ai_drone.flight.params import (
    PARAMETER_ABS_TOLERANCE as PARAMETER_ABS_TOLERANCE,
)
from ai_drone.flight.params import (
    REQUIRED_NOGPS_LOITER_PARAMETERS as REQUIRED_NOGPS_LOITER_PARAMETERS,
)
from ai_drone.flight.params import (
    select_navigation_profile,
)
from ai_drone.flight.phase import (
    Arm,
    Armed,
    ArmPending,
    Climb,
    Command,
    CommandAttempt,
    CommandLong,
    Disarm,
    Flight,
    Human,
    Landed,
    Landing,
    Phase,
    SetMode,
    Unclaimed,
    cleanup,
    ground_reference,
    observed_disarm,
    owns_control,
    request_landing,
)
from ai_drone.flight.state import (
    VehicleState,
    align_altitude,
    decode,
    fresh,
    observe,
)
from ai_drone.mavlink.connection import open_ardupilot_connection
from ai_drone.mavlink.devices import resolve_mavlink_endpoint
from ai_drone.mavlink.parameters import request_parameter
from ai_drone.mavlink.safety import (
    is_vehicle_message,
    require_ardupilot_heartbeat,
    require_fresh_disarmed_heartbeat,
)
from ai_drone.mavlink.shared import SharedMavlinkError, received_monotonic
from ai_drone.recording import request_message_intervals
from ai_drone.validation import finite_in_range

logger = logging.getLogger(__name__)

DOWNWARD_ORIENTATION = mavlink.MAV_SENSOR_ROTATION_PITCH_270
GCS_HEARTBEAT_INTERVAL_S = 1.0
NAVIGATION_SAMPLE_MAX_AGE_S = 1.0
BATTERY_SAMPLE_MAX_AGE_S = 2.0
ATTITUDE_TARGET_MASK = (
    mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)
GUIDED_HOLD_FIELD = 0.5
GUIDED_TAKEOFF_CLIMB_FRACTION = 0.3

COPTER_MODES = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR)


class FlightSafetyError(RuntimeError):
    """Raised when a live-flight safety invariant is violated."""


class HumanControlTaken(FlightSafetyError):
    """Autonomous work must yield; the caller must keep telemetry alive until disarm."""


def _validate_takeoff_ceiling(
    target: float, ground_reference: float | None, max_altitude: float
) -> None:
    reason = takeoff_ceiling_violation(target, ground_reference, max_altitude)
    if reason is not None:
        raise FlightSafetyError(reason)


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
        *,
        operator_alive: Callable[[], bool] | None = None,
        human_takeover_requested: Callable[[], bool] | None = None,
        ownership_policy: OwnershipPolicy | None = None,
        navigation_profile: str = "flow-compass",
    ) -> None:
        if isinstance(baud, bool) or not 1 <= baud <= 4_000_000:
            raise ValueError("baud must be between 1 and 4000000")
        self.device = self.find_device(device)
        self.navigation_profile = select_navigation_profile(
            navigation_profile,
            self.device,
            isolated_sitl=os.environ.get("AI_DRONE_ISOLATED_SITL") == "1",
        )
        self.profile_initialized_at = time.monotonic()
        if self.navigation_profile.experimental:
            raw_reference = os.environ.get("AI_DRONE_PROFILE_REFERENCE_MONOTONIC")
            if raw_reference is not None:
                self.profile_initialized_at = finite_in_range(
                    float(raw_reference),
                    "profile initialization",
                    minimum=0.0,
                    maximum=self.profile_initialized_at,
                )
        self.baud = baud
        self.limits = FlightLimits(max_altitude, min_battery_voltage)
        self.target_system = target_system
        self.target_component = target_component
        self.connection: Any | None = None
        self.state = VehicleState()
        self.phase: Phase = Unclaimed()
        self.command_attempts: deque[CommandAttempt] = deque(maxlen=256)
        self._command_sequence = 0
        self._last_gcs_heartbeat_time: float | None = None
        # The CLI sets this to a non-blocking callback backed by its signal/event
        # handling.  Every long pre-landing loop checks it; landing deliberately
        # ignores it so a second signal cannot interrupt cleanup.
        self.stop_requested: Callable[[], bool] | None = None
        self.operator_alive = operator_alive
        self.human_takeover_requested = human_takeover_requested
        self.ownership_policy = ownership_policy or OwnershipPolicy()

    @property
    def max_altitude(self) -> float:
        return self.limits.ceiling_m

    @property
    def min_battery_voltage(self) -> float:
        return self.limits.min_battery_voltage

    @property
    def is_flying(self) -> bool:
        airborne = (
            self.phase.flight_started
            if isinstance(self.phase, Human)
            else cleanup(self.phase) == "land"
        )
        return airborne and self.is_armed

    @property
    def _human_control(self) -> bool:
        return isinstance(self.phase, Human)

    @property
    def _arm_command_sent(self) -> bool:
        phase = self.phase.previous if isinstance(self.phase, Landing) else self.phase
        return isinstance(phase, ArmPending)

    @property
    def _armed_by_controller(self) -> bool:
        phase = self.phase.previous if isinstance(self.phase, Landing) else self.phase
        return isinstance(phase, Armed | Flight)

    @property
    def _flight_started_by_controller(self) -> bool:
        return cleanup(self.phase) == "land"

    @property
    def _landing_commanded(self) -> bool:
        return isinstance(self.phase, Landing) or (
            isinstance(self.phase, Landed) and self.phase.landing_commanded
        )

    @property
    def _ground_reference(self) -> float | None:
        return ground_reference(self.phase)

    @property
    def current_altitude(self) -> float | None:
        sample = self.state.altitude
        return None if sample is None else sample.value

    @property
    def local_position_altitude(self) -> float | None:
        sample = self.state.local_altitude
        return None if sample is None else sample.value

    @property
    def battery_voltage(self) -> float | None:
        sample = self.state.battery
        return None if sample is None else sample.value

    @property
    def ekf_flags(self) -> int | None:
        sample = self.state.ekf_flags
        return None if sample is None else sample.value

    @property
    def flow_quality(self) -> int | None:
        sample = self.state.flow_quality
        return None if sample is None else sample.value

    @property
    def rc_channel_count(self) -> int | None:
        sample = self.state.rc_channels
        return None if sample is None else sample.value

    @property
    def yaw_rad(self) -> float | None:
        sample = self.state.yaw
        return None if sample is None else sample.value

    @property
    def last_telemetry_time(self) -> float:
        sample = self.state.altitude
        return 0.0 if sample is None else sample.received_at

    @property
    def last_local_position_time(self) -> float:
        sample = self.state.local_altitude
        return 0.0 if sample is None else sample.received_at

    @property
    def last_battery_time(self) -> float:
        sample = self.state.battery
        return 0.0 if sample is None else sample.received_at

    @property
    def last_ekf_time(self) -> float:
        sample = self.state.ekf_flags
        return 0.0 if sample is None else sample.received_at

    @property
    def last_flow_time(self) -> float:
        sample = self.state.flow_quality
        return 0.0 if sample is None else sample.received_at

    @property
    def last_rc_channels_time(self) -> float:
        sample = self.state.rc_channels
        return 0.0 if sample is None else sample.received_at

    @property
    def last_attitude_time(self) -> float:
        sample = self.state.yaw
        return 0.0 if sample is None else sample.received_at

    @property
    def last_heartbeat_time(self) -> float:
        sample = self.state.heartbeat
        return 0.0 if sample is None else sample.received_at

    @property
    def is_armed(self) -> bool:
        return self.state.heartbeat is not None and self.state.heartbeat.value.armed

    @property
    def flight_mode(self) -> str | None:
        return None if self.state.heartbeat is None else self.state.heartbeat.value.mode

    @property
    def local_position_altitude_aligned(self) -> float | None:
        return (
            None if self.state.alignment is None else self.state.alignment.local.value
        )

    @property
    def flight_sw_version(self) -> int | None:
        return (
            None if self.state.firmware is None else self.state.firmware.value.version
        )

    @property
    def flight_custom_version(self) -> bytes | None:
        return None if self.state.firmware is None else self.state.firmware.value.commit

    @property
    def owns_control(self) -> bool:
        return owns_control(self.phase)

    @property
    def control_owner(self) -> str:
        if self._human_control:
            return "human"
        return "autonomous" if self.owns_control else "unclaimed"

    def _require_autonomous_control(self) -> None:
        if self._human_control:
            raise HumanControlTaken("control was handed to the radio pilot")

    def _write_command(self, command: Command) -> None:
        """The autonomous write boundary; heartbeat and passive requests are separate.

        Intent precedes a potentially ambiguous transport write. A written command
        is not an FC acknowledgement or an observed arming/landing confirmation.
        """
        self._resolve_control_ownership()
        self._require_autonomous_control()
        if isinstance(command, Disarm) and cleanup(self.phase) == "land":
            raise FlightSafetyError("refusing to force-disarm a flight; use land()")
        if (
            isinstance(command, CommandLong)
            and command.command != mavlink.MAV_CMD_REQUEST_MESSAGE
        ):
            raise FlightSafetyError(
                "command-ACK helper only authorizes passive message requests"
            )
        if isinstance(command, Arm):
            self.phase = ArmPending()
        elif isinstance(command, Climb):
            if not isinstance(self.phase, Armed | Flight):
                raise FlightSafetyError("climb requires arming by this controller")
            if not isinstance(self.phase, Flight):
                self.phase = Flight("taking_off", self._ground_reference)
        elif isinstance(command, SetMode) and command.name == "LAND":
            self.phase = request_landing(self.phase)
        self._command_sequence += 1
        attempt = CommandAttempt(
            self._command_sequence, command, time.monotonic(), "attempting"
        )
        self.command_attempts.append(attempt)
        try:
            self._perform_command(command)
        except BaseException as error:
            self.command_attempts[-1] = replace(
                attempt, outcome="failed", error=f"{type(error).__name__}: {error}"
            )
            raise
        self.command_attempts[-1] = replace(attempt, outcome="written")

    def _perform_command(self, command: Command) -> None:
        connection = self._connection()
        match command:
            case Arm():
                connection.arducopter_arm()
            case Disarm():
                connection.arducopter_disarm()
            case SetMode(name=name):
                connection.mav.set_mode_send(
                    self.target_system,
                    mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    COPTER_MODES[name],
                )
            case Climb(fraction=fraction, yaw=yaw):
                half_yaw = yaw * 0.5
                connection.mav.set_attitude_target_send(
                    (time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF,
                    self.target_system,
                    self.target_component,
                    ATTITUDE_TARGET_MASK,
                    [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)],
                    0.0,
                    0.0,
                    0.0,
                    GUIDED_HOLD_FIELD + fraction * 0.5,
                )
            case CommandLong(command=identifier, parameters=parameters):
                connection.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    identifier,
                    0,
                    *parameters,
                )
            case _ as impossible:
                assert_never(impossible)

    def _resolve_control_ownership(self) -> None:
        callback = self.human_takeover_requested
        if not self.owns_control or callback is None:
            return
        try:
            requested = bool(callback())
        except Exception as error:
            logger.error("human takeover callback failed: %s", error)
            return
        if not human_takeover_allowed(
            self.ownership_policy,
            requested=requested,
            armed=self.is_armed,
            mode=self.flight_mode,
            heartbeat_received=self.last_heartbeat_time,
            rc_received=self.last_rc_channels_time,
            rc_channels=self.rc_channel_count,
            now=time.monotonic(),
        ):
            return
        self.phase = Human(self.is_flying)
        logger.warning("control handed to the radio pilot in %s", self.flight_mode)

    def _require_operator(self) -> None:
        callback = self.operator_alive
        if callback is None or self._human_control:
            return
        try:
            alive = bool(callback())
        except Exception as error:
            logger.error("operator heartbeat callback failed: %s", error)
            alive = False
        if not alive:
            if self._flight_started_by_controller:
                self.emergency_stop()
            raise FlightSafetyError("operator heartbeat lost")

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
            if self._human_control:
                self.supervise_human()
                return
            if self._flight_started_by_controller:
                # Once TAKEOFF has been sent, cleanup must never issue a disarm.
                # LAND may already have been requested or may have timed out; in
                # both cases keep monitoring and re-sending it until the vehicle
                # confirms disarm or this bounded attempt expires.
                self.land()
            elif self._arm_command_sent or self._armed_by_controller:
                self.disarm()
        except HumanControlTaken:
            self.supervise_human()
        except Exception as error:
            logger.error("Could not complete controller cleanup: %s", error)
        finally:
            self.close()

    def supervise_human(self) -> None:
        """Keep the onboard heartbeat until the pilot confirms disarm."""
        if not self._human_control:
            raise FlightSafetyError(
                "pilot supervision requires confirmed human ownership"
            )
        while self.is_armed or not self.heartbeat_is_fresh():
            try:
                self.update_telemetry()
                time.sleep(0.05)
            except KeyboardInterrupt:
                logger.warning(
                    "pilot retains control; maintaining heartbeat until disarm"
                )

    def _connection(self) -> Any:
        if self.connection is None:
            raise RuntimeError("not connected")
        return self.connection

    def connect(self) -> None:
        connection = open_ardupilot_connection(
            self.device,
            baud=self.baud,
            source_system=255,
            source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        self.connection = connection
        try:
            heartbeat = require_ardupilot_heartbeat(
                connection,
                system_id=self.target_system,
                component_id=self.target_component,
                timeout=15.0,
            )
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

    def _matching_vehicle_message(self, message: Any) -> bool:
        return is_vehicle_message(
            message,
            system_id=self.target_system,
            component_id=self.target_component,
        )

    def _process_message(self, message: Any, now: float) -> None:
        if not self._matching_vehicle_message(message):
            return
        observation = decode(
            message,
            received=received_monotonic(message, default=now),
            fallback_mode=getattr(self.connection, "flightmode", None),
        )
        if observation is None:
            return
        previous = self.state
        self.state = observe(
            previous,
            observation,
            now=now,
            target_system=self.target_system,
            target_component=self.target_component,
            heartbeat_max_age=self.ownership_policy.heartbeat_max_age,
        )
        if self.state.heartbeat is not previous.heartbeat and not self.is_armed:
            self.phase = observed_disarm(self.phase)

    def update_telemetry(self, max_messages: int = 50) -> None:
        if isinstance(max_messages, bool) or not 1 <= max_messages <= 1_000:
            raise ValueError("max_messages must be between 1 and 1000")
        if self.connection is None:
            return
        self._pump_gcs_heartbeat()
        for _ in range(max_messages):
            message = self.connection.recv_match(blocking=False)
            if message is None:
                break
            self._process_message(message, time.monotonic())
        self._resolve_control_ownership()
        self._raise_if_stop_requested()
        if self._flight_started_by_controller and not self._landing_commanded:
            self._enforce_flight_limits()

    def _enforce_flight_limits(self) -> None:
        self._require_profile_time()
        evidence = FlightEvidence(
            self.current_altitude,
            self.local_position_altitude_aligned,
            self.navigation_is_healthy(),
            self.no_rc_input_is_confirmed(),
            self.battery_voltage,
            self.battery_is_fresh(),
            self.altitude_is_fresh(),
            self.heartbeat_is_fresh(),
        )
        reason = violation(
            self.limits,
            evidence,
            in_loiter=self.flight_mode == "LOITER",
            require_flight_telemetry=self._ground_reference is not None,
        )
        if reason is not None:
            self.emergency_stop()
            raise FlightSafetyError(reason)

    def _require_profile_time(self, requested_duration: float = 0.0) -> None:
        profile = self.navigation_profile
        if profile.maximum_sequence_s is None:
            return
        remaining = profile.maximum_sequence_s - (
            time.monotonic() - self.profile_initialized_at
        )
        if remaining < requested_duration + profile.landing_reserve_s:
            if self._flight_started_by_controller:
                self.emergency_stop()
            raise FlightSafetyError(
                f"{profile.name} experiment duration budget exhausted; {profile.landing_reserve_s:g} s reserved for LAND"
            )

    def _poll(self, timeout: float) -> Iterator[None]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.update_telemetry()
            yield
            time.sleep(0.05)

    def _raise_if_stop_requested(self) -> None:
        if self._human_control:
            return
        if self.owns_control and not self._landing_commanded:
            self._require_operator()
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
        return fresh(self.state.altitude, time.monotonic(), max_age) is not None

    def heartbeat_is_fresh(self, max_age: float = 2.5) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.last_heartbeat_time > 0
            and fresh(self.state.heartbeat, time.monotonic(), max_age) is not None
        )

    def attitude_is_fresh(self, max_age: float = 1.0) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.last_attitude_time > 0
            and fresh(self.state.yaw, time.monotonic(), max_age) is not None
        )

    def battery_is_fresh(self, max_age: float = BATTERY_SAMPLE_MAX_AGE_S) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return (
            self.last_battery_time > 0
            and fresh(self.state.battery, time.monotonic(), max_age) is not None
        )

    def optical_flow_is_fresh(
        self, max_age: float = NAVIGATION_SAMPLE_MAX_AGE_S
    ) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        quality = fresh(self.state.flow_quality, time.monotonic(), max_age)
        return self.last_flow_time > 0 and quality is not None and quality > 0

    def relative_position_is_fresh(
        self, max_age: float = NAVIGATION_SAMPLE_MAX_AGE_S
    ) -> bool:
        finite_in_range(max_age, "max_age", minimum=0.05, maximum=10.0)
        return self.last_ekf_time > 0 and relative_position_ready(
            self.state.ekf_flags, time.monotonic(), max_age
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
            self.last_rc_channels_time > 0
            and fresh(self.state.rc_channels, time.monotonic(), max_age) == 0
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
        for _ in self._poll(timeout):
            if self.optical_flow_is_fresh():
                return
        raise FlightSafetyError("no fresh valid optical-flow sample")

    def wait_for_attitude(self, timeout: float = 3.0) -> None:
        """Wait for the yaw used to construct a level attitude target."""

        finite_in_range(timeout, "timeout", minimum=0.05, maximum=30.0)
        for _ in self._poll(timeout):
            if self.attitude_is_fresh():
                return
        raise FlightSafetyError("no fresh attitude sample")

    def wait_for_no_rc_input(self, timeout: float = 3.0) -> None:
        """Require the captured receiver-free topology before arming.

        A valid receiver can change both Loiter climb rate and the flight mode.
        This autonomous path therefore accepts only a fresh zero-channel report.
        """

        finite_in_range(timeout, "timeout", minimum=0.05, maximum=30.0)
        for _ in self._poll(timeout):
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
        raise FlightSafetyError(
            "no fresh RC_CHANNELS report confirming zero receiver channels"
        )

    def verify_battery_before_arming(self, timeout: float = 3.0) -> None:
        """Require a fresh pack voltage at or above the configured guard."""

        finite_in_range(timeout, "timeout", minimum=0.05, maximum=30.0)
        if self.min_battery_voltage <= 0.0:
            return
        for _ in self._poll(timeout):
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
        raise FlightSafetyError("no fresh battery voltage before arming")

    def _fresh_disarmed(self, timeout: float = 2.5) -> None:
        heartbeat = require_fresh_disarmed_heartbeat(
            self._connection(),
            system_id=self.target_system,
            component_id=self.target_component,
            timeout=timeout,
            observe=lambda message: self._process_message(message, time.monotonic()),
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

        expected_parameters = dict(self.navigation_profile.parameters)
        self._pump_gcs_heartbeat()
        forward_type = request_parameter(self._connection(), "RNGFND2_TYPE")
        if forward_type == 10.0:
            expected_parameters.update(FORWARD_RANGEFINDER_PARAMETERS)
        elif forward_type != 0.0:
            raise FlightSafetyError(
                f"RNGFND2_TYPE={forward_type:g}; no-GPS Loiter requires disabled "
                "(0) or the reviewed forward MAVLink rangefinder (10)"
            )

        for name, expected in expected_parameters.items():
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
        """Require official ArduCopter 4.7.1 at the captured project commit."""

        finite_in_range(timeout, "timeout", minimum=0.5, maximum=15.0)
        self.state = replace(self.state, firmware=None)
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
                f"ArduCopter {rendered}; flight requires exact version 4.7.1"
            )
        if self.flight_custom_version != EXPECTED_FIRMWARE_COMMIT:
            actual = (self.flight_custom_version or b"").decode(
                "ascii", errors="replace"
            )
            raise FlightSafetyError(
                f"firmware commit {actual!r}; flight requires 'dbe79216'"
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
        self._require_autonomous_control()
        finite_in_range(timeout, "timeout", minimum=0.1, maximum=30.0)
        self._connection()
        requested = mode_name.upper()
        mapping = COPTER_MODES
        if requested not in mapping:
            raise ValueError(f"flight mode {requested!r} is not supported")
        self._write_command(SetMode(requested))
        for _ in self._poll(timeout):
            self._require_autonomous_control()
            if self.flight_mode == requested:
                return
        raise TimeoutError(f"flight controller did not confirm {requested} mode")

    def arm(
        self, timeout: float = 10.0, *, takeoff_gain_m: float | None = None
    ) -> None:
        self._require_autonomous_control()
        self._require_operator()
        finite_in_range(timeout, "timeout", minimum=0.5, maximum=30.0)
        self.update_telemetry()
        self._require_autonomous_control()
        if self.is_armed and not self._armed_by_controller:
            raise FlightSafetyError(
                "refusing to take ownership of an already-armed vehicle"
            )
        if self.is_armed:
            return
        if isinstance(self.phase, Landed):
            self.phase = Unclaimed()  # arm() explicitly starts a new mission
        self.verify_firmware()
        self.verify_arming_checks()
        self.verify_nogps_loiter_parameters()
        self.verify_onboard_logging()
        if self.wait_for_altitude(timeout=min(timeout, 3.0)) is None:
            raise FlightSafetyError("no fresh downward DISTANCE_SENSOR altitude")
        self._verify_takeoff_reference(takeoff_gain_m)
        self.wait_for_optical_flow(timeout=min(timeout, 3.0))
        self.wait_for_attitude(timeout=min(timeout, 3.0))
        self.wait_for_no_rc_input(timeout=min(timeout, 3.0))
        self.verify_battery_before_arming(timeout=min(timeout, 3.0))
        # AltHold's user-takeoff state machine still depends on positive pilot
        # throttle to spool up.  This companion-only aircraft has no RC input,
        # so use ArduPilot's position-free autonomous takeoff mode instead.
        self.set_mode("GUIDED_NOGPS")
        self._fresh_disarmed()
        self._require_operator()
        if not self.altitude_is_fresh():
            raise FlightSafetyError("downward altitude became stale before arming")
        self._verify_takeoff_reference(takeoff_gain_m)
        self._write_command(Arm())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump_gcs_heartbeat()
            message = self._connection().recv_match(blocking=True, timeout=0.5)
            if message is not None:
                self._process_message(message, time.monotonic())
            self._resolve_control_ownership()
            self._require_autonomous_control()
            self._raise_if_stop_requested()
            if self.is_armed:
                self.phase = Armed()
                return
        raise TimeoutError("flight controller did not confirm arming")

    def _verify_takeoff_reference(self, takeoff_gain_m: float | None) -> None:
        if takeoff_gain_m is not None:
            gain = finite_in_range(
                takeoff_gain_m,
                "takeoff_gain_m",
                minimum=0.15,
                maximum=self.max_altitude,
            )
            if not self.altitude_is_fresh():
                raise FlightSafetyError("takeoff requires a fresh ground reference")
            _validate_takeoff_ceiling(gain, self.current_altitude, self.max_altitude)

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

        self._require_autonomous_control()
        finite_in_range(timeout, "timeout", minimum=0.1, maximum=30.0)
        self._write_command(CommandLong(command, parameters))
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
        self._write_command(Disarm())

    def _drain_cleanup_evidence(self) -> None:
        # Bound the drain and isolate transport errors: queued pilot evidence
        # takes precedence, while a broken receive path cannot veto LAND.
        try:
            for _ in range(50):
                message = self._connection().recv_match(blocking=False)
                if message is None:
                    break
                self._process_message(message, time.monotonic())
        except (OSError, SharedMavlinkError) as error:
            logger.warning("cleanup receive failed: %s", error)
        self._resolve_control_ownership()
        self._require_autonomous_control()

    def _best_effort_disarm(self) -> None:
        try:
            self._request_disarm()
        except (OSError, SharedMavlinkError) as error:
            logger.warning("DISARM write failed; retrying: %s", error)

    def disarm(self, timeout: float = 10.0) -> None:
        self._require_autonomous_control()
        finite_in_range(timeout, "timeout", minimum=0.5, maximum=30.0)
        if self._flight_started_by_controller:
            raise FlightSafetyError("refusing to force-disarm a flight; use land()")
        self._drain_cleanup_evidence()
        started = max(time.monotonic(), self.last_heartbeat_time)
        deadline = time.monotonic() + timeout
        self._best_effort_disarm()
        next_request = time.monotonic() + 1.0
        while (remaining := deadline - time.monotonic()) > 0:
            now = time.monotonic()
            if now >= next_request:
                self._drain_cleanup_evidence()
                self._best_effort_disarm()
                next_request = now + 1.0
            try:
                self._pump_gcs_heartbeat()
                message = self._connection().recv_match(
                    type="HEARTBEAT", blocking=True, timeout=min(remaining, 0.5)
                )
                if message is not None:
                    self._process_message(message, time.monotonic())
                self._resolve_control_ownership()
                self._require_autonomous_control()
                if self.last_heartbeat_time > started and not self.is_armed:
                    self.phase = Landed(landing_commanded=False)
                    return
            except (OSError, SharedMavlinkError) as error:
                logger.warning("DISARM receive failed; retrying: %s", error)
                time.sleep(min(remaining, 0.1))
        raise TimeoutError("flight controller did not confirm disarming")

    def _send_level_climb(self, climb_fraction: float) -> None:
        """Send a level GuidedNoGPS target with a bounded climb fraction.

        Because flight requires ``GUID_OPTIONS=0``, ArduCopter interprets the
        final SET_ATTITUDE_TARGET field as climb rate: 0.5 holds altitude and
        1.0 requests ``WP_SPD_UP``.  It is never raw motor thrust here.
        """

        self._require_autonomous_control()
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
        self._write_command(Climb(fraction, yaw))

    def takeoff(self, target_alt: float, timeout: float = 15.0) -> None:
        self._require_autonomous_control()
        target = finite_in_range(
            target_alt, "target_alt", minimum=0.15, maximum=self.max_altitude
        )
        finite_in_range(timeout, "timeout", minimum=1.0, maximum=60.0)
        self._require_profile_time(timeout)
        if not self.is_armed:
            self.arm(takeoff_gain_m=target)
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
        ground = self.current_altitude
        _validate_takeoff_ceiling(target, ground, self.max_altitude)
        self.state = align_altitude(
            replace(self.state, alignment=None), time.monotonic()
        )
        # Mark ownership before the first climb target leaves.  From this point
        # cleanup must LAND and never issue a force-disarm, even if no motion is
        # observed or the link fails immediately afterward.
        self.phase = Flight("taking_off", ground)
        started = time.monotonic()
        deadline = started + timeout
        while time.monotonic() < deadline:
            self.update_telemetry()
            self._require_autonomous_control()
            now = time.monotonic()
            if now - started > 2.0 and (
                not self.altitude_is_fresh() or not self.heartbeat_is_fresh()
            ):
                self.emergency_stop()
                raise FlightSafetyError("telemetry became stale during takeoff")
            if takeoff_reached(
                self.state.altitude,
                started=started,
                ground=self._ground_reference,
                target=target,
            ):
                self._send_level_climb(0.0)
                self.phase = Flight("holding_altitude", ground)
                return
            self._send_level_climb(GUIDED_TAKEOFF_CLIMB_FRACTION)
            time.sleep(0.05)
        self.emergency_stop()
        raise TimeoutError("takeoff altitude was not reached")

    def wait_for_relative_position(
        self, timeout: float = 10.0, stable_for: float = 1.0
    ) -> None:
        """Wait for continuously healthy flow-backed relative navigation."""

        self._require_autonomous_control()
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

        self._require_autonomous_control()
        if not self.is_armed or not self._flight_started_by_controller:
            raise FlightSafetyError("Loiter transition requires a controller takeoff")
        self.phase = Flight("awaiting_loiter", self._ground_reference)
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
        self.phase = Flight("loitering", self._ground_reference)

    def hold_loiter(self, duration: float) -> None:
        """Hold confirmed Loiter while enforcing flow, EKF, range and link gates."""

        self._require_autonomous_control()
        finite_in_range(duration, "duration", minimum=0.1, maximum=30.0)
        self._require_profile_time(duration)
        if self.flight_mode != "LOITER":
            raise FlightSafetyError("Loiter hold requires confirmed LOITER mode")
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.update_telemetry()
            self._require_autonomous_control()
            if self.flight_mode != "LOITER":
                self.emergency_stop()
                raise FlightSafetyError(
                    f"Loiter hold left LOITER mode for {self.flight_mode or 'unknown'}"
                )
            if not self.altitude_is_fresh() or not self.heartbeat_is_fresh():
                self.emergency_stop()
                raise FlightSafetyError("telemetry became stale during Loiter")
            time.sleep(0.05)

    def hold_altitude(self, duration: float) -> None:
        """Bounded autonomous vertical hold in GuidedNoGPS; XY is not controlled.

        Retain the takeoff pre-arm prerequisites. Once airborne, the vertical
        operation requires range, attitude, heartbeat, battery and receiver
        topology; loss of horizontal aiding alone does not establish a vertical
        failure. Loiter retains its independent flow/relative-position gate.
        """
        self._require_autonomous_control()
        finite_in_range(duration, "duration", minimum=0.1, maximum=30.0)
        self._require_profile_time(duration)
        if not isinstance(self.phase, Flight) or not self.is_armed:
            raise FlightSafetyError("altitude hold requires a controller takeoff")
        if self.flight_mode != "GUIDED_NOGPS":
            raise FlightSafetyError(
                "altitude hold requires confirmed GUIDED_NOGPS mode"
            )
        self.phase = Flight("holding_altitude", self._ground_reference)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.update_telemetry()
            self._require_autonomous_control()
            if self.flight_mode != "GUIDED_NOGPS":
                self.emergency_stop()
                raise FlightSafetyError(
                    f"altitude hold left GUIDED_NOGPS mode for {self.flight_mode or 'unknown'}"
                )
            if not self.altitude_is_fresh() or not self.heartbeat_is_fresh():
                self.emergency_stop()
                raise FlightSafetyError("telemetry became stale during altitude hold")
            try:
                self._send_level_climb(0.0)
            except HumanControlTaken:
                raise
            except Exception:
                self.emergency_stop()
                raise
            time.sleep(0.05)

    def land(self, timeout: float = 30.0) -> None:
        self._resolve_control_ownership()
        self._require_autonomous_control()
        finite_in_range(timeout, "timeout", minimum=1.0, maximum=120.0)
        self._drain_cleanup_evidence()
        self.phase = request_landing(self.phase)
        started = max(time.monotonic(), self.last_heartbeat_time)
        deadline = time.monotonic() + timeout
        self.emergency_stop()
        next_request = started + 1.0
        while time.monotonic() < deadline:
            try:
                self.update_telemetry()
            except (OSError, SharedMavlinkError) as error:
                logger.warning("LAND receive failed; retrying: %s", error)
            self._resolve_control_ownership()
            self._require_autonomous_control()
            if self.last_heartbeat_time > started and not self.is_armed:
                self.phase = Landed()
                return
            now = time.monotonic()
            if now >= next_request:
                self._drain_cleanup_evidence()
                self.emergency_stop()
                next_request = now + 1.0
            time.sleep(0.1)
        raise TimeoutError(
            "LAND remains commanded but disarming was not confirmed; do not approach the vehicle"
        )

    def emergency_stop(self) -> None:
        """Best-effort LAND; cleanup remains safe even if the link is broken."""
        try:
            self._resolve_control_ownership()
            if self.connection is None or self._human_control:
                return
            self._write_command(SetMode("LAND"))
        except Exception:
            logger.exception("could not request emergency LAND")
