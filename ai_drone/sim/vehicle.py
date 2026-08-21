"""A MAVLink copter that behaves enough like ArduPilot to rehearse a flight.

The point of this module is that ``drone-control hover`` and
``drone-control velocity-test`` can be exercised end to end -- arming,
GUIDED mode, ``MAV_CMD_NAV_TAKEOFF``, the altitude/battery/staleness guards,
and the LAND cleanup path -- without a vehicle, propellers, or a flight
controller anywhere near the operator.

It is deliberately *not* a flight-dynamics model.  It is a protocol and
sequencing double: it answers the same messages ArduPilot answers, in the same
order, and it can be told to misbehave.  ``Fault`` is the reason this exists at
all -- a guard that has never been seen to stop an aircraft is a guard nobody
should trust, and the only safe place to watch one trip is here.

Nothing in this module ever touches a serial port.  It binds a UDP socket and
speaks to whatever connects to it.
"""

from __future__ import annotations

import enum
import math
import time
from dataclasses import dataclass, field
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.validation import finite_in_range

# ArduPilot Copter custom mode numbers.  Only the modes this double answers to
# are listed; pymavlink maps the rest for us when it decodes the heartbeat.
COPTER_MODES: dict[str, int] = {
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "LOITER": 5,
    "RTL": 6,
    "CIRCLE": 7,
    "LAND": 9,
    "POSHOLD": 16,
    "GUIDED_NOGPS": 20,
    "FLOWHOLD": 22,
}
MODE_NAMES: dict[int, str] = {number: name for name, number in COPTER_MODES.items()}

DOWNWARD = mavlink.MAV_SENSOR_ROTATION_PITCH_270
RANGEFINDER_MIN_CM = 2
RANGEFINDER_MAX_CM = 800
CLIMB_RATE_MS = 0.5
DESCENT_RATE_MS = 0.35
TOUCHDOWN_ALTITUDE_M = 0.05

# ArduPilot lets an RC override lapse after RC_OVERRIDE_TIME seconds.  This
# double models the lapse as the aircraft losing its throttle input entirely,
# which is what happens on an airframe with no receiver fitted -- so a
# rehearsal fails if the flight code stops refreshing the override.
RC_OVERRIDE_TIME_S = 3.0
# How hard a STABILIZE throttle above the learned hover value pushes the
# aircraft up.  This is a sequencing double, not a thrust model: the number
# only has to be steep enough that a wrong throttle is visible as motion.
STABILIZE_THRUST_GAIN = 5.0
# What a runaway looks like: the same commanded throttle, four times the
# thrust.  This is the 2026-08-20 failure -- a commanded value producing far
# more lift than the number predicted -- expressed as something a guard can
# be watched catching.
RUNAWAY_THRUST_MULTIPLIER = 4.0
# PILOT_SPEED_UP and PILOT_SPEED_DN on this airframe, in m/s.
PILOT_SPEED_UP_MS = 0.25
PILOT_SPEED_DN_MS = 0.20

# The parameters DroneController reads before it will arm.  These are the
# values a *correctly configured* aircraft reports: all pre-arm checks on,
# onboard logging enabled.  The rehearsal is worthless if the double is more
# permissive than the real gate.
DEFAULT_PARAMETERS: dict[str, float] = {
    "ARMING_CHECK": 1.0,
    "LOG_BACKEND_TYPE": 4.0,
    "LOG_BITMASK": 180222.0,
    "SYSID_THISMAV": 1.0,
    "FRAME_CLASS": 1.0,
    "FRAME_TYPE": 1.0,
    "RNGFND1_TYPE": 10.0,
    "RNGFND1_ORIENT": float(DOWNWARD),
    "EK3_SRC1_POSXY": 3.0,
    "BATT_MONITOR": 4.0,
    # The throttle mapping a STABILIZE climb reads before it commands any of
    # it.  These are the live values from the 2026-08-20 capture, including
    # the hover throttle ArduPilot learned during the flight itself.
    "RCMAP_ROLL": 1.0,
    "RCMAP_PITCH": 2.0,
    "RCMAP_THROTTLE": 3.0,
    "RCMAP_YAW": 4.0,
    "RC1_TRIM": 1501.0,
    "RC2_TRIM": 1500.0,
    "RC3_MIN": 988.0,
    "RC3_MAX": 2011.0,
    "RC4_TRIM": 1500.0,
    "MOT_THST_HOVER": 0.263,
    "THR_DZ": 40.0,
}


class Fault(enum.Enum):
    """A misbehavior the double can be told to exhibit.

    Each value names the guard or failure path it exists to exercise.
    """

    NONE = "none"
    REFUSE_ARM = "refuse-arm"
    NO_TAKEOFF = "no-takeoff"
    ALTITUDE_RUNAWAY = "altitude-runaway"
    BATTERY_SAG = "battery-sag"
    STALE_ALTITUDE = "stale-altitude"
    HEARTBEAT_LOSS = "heartbeat-loss"
    REFUSE_LAND = "refuse-land"
    THROTTLE_RUNAWAY = "throttle-runaway"
    EKF_DIVERGENCE = "ekf-divergence"
    LAND_CLIMBS = "land-climbs"


# What the vehicle reported on 2026-08-21 while sitting on the floor: an
# altitude ten kilometres underground and a 38 m/s descent.  LAND is altitude
# controlled, so it answered that estimate with full throttle.
DIVERGED_ALTITUDE_M = -10_000.0
DIVERGED_CLIMB_MS = -38.0
DIVERGED_LAND_CLIMB_MS = 3.8
# param2 of MAV_CMD_COMPONENT_ARM_DISARM: ArduPilot's documented override for
# disarming an aircraft that is flying.
FORCE_DISARM_MAGIC = 21196


# Both faults report a diverged vertical estimate and make LAND climb.  They
# differ in whether the aircraft ever leaves the ground: EKF_DIVERGENCE is
# 2026-08-21 end to end, where the commanded throttle never lifted it and the
# abort flew it; LAND_CLIMBS is the same broken LAND under an aircraft that
# did take off normally.
_DIVERGED_FAULTS = frozenset({Fault.EKF_DIVERGENCE, Fault.LAND_CLIMBS})


@dataclass
class VehicleState:
    """Everything the double knows about itself."""

    mode: int = COPTER_MODES["STABILIZE"]
    armed: bool = False
    altitude_m: float = 0.0
    north_m: float = 0.0
    east_m: float = 0.0
    yaw_rad: float = 0.0
    battery_v: float = 16.0
    takeoff_target_m: float | None = None
    velocity: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    velocity_deadline: float = 0.0
    climb_rate_ms: float = 0.0
    climb_deadline: float = 0.0
    rc_throttle_pwm: int | None = None
    rc_override_deadline: float = 0.0
    parameters: dict[str, float] = field(
        default_factory=lambda: dict(DEFAULT_PARAMETERS)
    )

    @property
    def mode_name(self) -> str:
        return MODE_NAMES.get(self.mode, f"MODE{self.mode}")


class SimulatedVehicle:
    """A UDP MAVLink endpoint that answers like an ArduPilot Copter.

    ``run`` blocks.  Callers that want to stop it early set ``stop`` from
    another thread or raise ``KeyboardInterrupt``; ``close`` is always reached
    through the ``finally`` in ``run``.
    """

    def __init__(
        self,
        endpoint: str = "tcpin:127.0.0.1:5760",
        *,
        fault: Fault = Fault.NONE,
        battery_v: float = 16.0,
        system_id: int = 1,
        component_id: int = 1,
        clock: Any = time.monotonic,
    ) -> None:
        self.endpoint = endpoint
        self.fault = fault
        self.system_id = system_id
        self.component_id = component_id
        self._clock = clock
        self.state = VehicleState(
            battery_v=finite_in_range(battery_v, "battery_v", minimum=0.0, maximum=60.0)
        )
        self.connection: Any | None = None
        self.stop = False
        self.started = clock()
        self._boot = self.started
        self._last: dict[str, float] = {}
        self._flight_started = 0.0
        self.log: list[str] = []
        self.failure: BaseException | None = None

    # -- lifecycle ------------------------------------------------------

    def __enter__(self) -> SimulatedVehicle:
        self.open()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()

    def open(self) -> None:
        self.connection = mavutil.mavlink_connection(
            self.endpoint,
            source_system=self.system_id,
            source_component=self.component_id,
            dialect="ardupilotmega",
        )

    def close(self) -> None:
        connection, self.connection = self.connection, None
        if connection is not None:
            connection.close()

    def _link(self) -> Any:
        if self.connection is None:
            raise RuntimeError("simulated vehicle is not open")
        return self.connection

    def run(self, max_seconds: float = 600.0) -> None:
        """Serve the simulated vehicle until stopped or ``max_seconds`` elapse."""

        finite_in_range(max_seconds, "max_seconds", minimum=0.1, maximum=86_400.0)
        if self.connection is None:
            self.open()
        deadline = self._clock() + max_seconds
        previous = self._clock()
        try:
            while not self.stop and self._clock() < deadline:
                self.pump()
                now = self._clock()
                self.advance(now - previous)
                previous = now
                self.emit(now)
                time.sleep(0.01)
        except Exception as error:
            self.failure = error
            self._note(f"simulator stopped: {type(error).__name__}: {error}")
        finally:
            self.close()

    # -- physics --------------------------------------------------------

    def advance(self, dt: float) -> None:
        """Move the simulated aircraft forward by ``dt`` seconds."""

        if dt <= 0.0 or not math.isfinite(dt):
            return
        state = self.state

        if state.armed:
            # Hovering costs far more than sitting armed on the ground.
            drain = 0.02 if state.altitude_m > TOUCHDOWN_ALTITUDE_M else 0.004
            if self.fault is Fault.BATTERY_SAG:
                drain = 1.2
            state.battery_v = max(0.1, state.battery_v - drain * dt)

        if state.mode == COPTER_MODES["LAND"] and self.fault in _DIVERGED_FAULTS:
            # The accident: the altitude controller reads a 38 m/s descent that
            # is not happening, applies full throttle to arrest it, and flies
            # the aircraft upward.  Nothing about being on the ground stops it.
            if state.armed:
                state.altitude_m += DIVERGED_LAND_CLIMB_MS * dt
            return

        if state.mode == COPTER_MODES["LAND"] and self.fault is not Fault.REFUSE_LAND:
            state.altitude_m = max(0.0, state.altitude_m - DESCENT_RATE_MS * dt)
            if state.altitude_m <= TOUCHDOWN_ALTITUDE_M:
                state.altitude_m = 0.0
                if state.armed:
                    self._note("landed and disarmed")
                state.armed = False
                state.takeoff_target_m = None
            return

        if not state.armed:
            return

        if state.mode in {COPTER_MODES["STABILIZE"], COPTER_MODES["ALT_HOLD"]}:
            self._advance_on_sticks(dt)
            return

        if self.fault is Fault.ALTITUDE_RUNAWAY and state.altitude_m > 0.05:
            # A stuck climb: the aircraft keeps going up regardless of target.
            state.altitude_m += CLIMB_RATE_MS * dt
            return

        if self._clock() < state.climb_deadline and self.fault is not Fault.NO_TAKEOFF:
            state.altitude_m = max(0.0, state.altitude_m + state.climb_rate_ms * dt)

        target = state.takeoff_target_m
        if target is not None and self.fault is not Fault.NO_TAKEOFF:
            step = CLIMB_RATE_MS * dt
            if state.altitude_m < target:
                state.altitude_m = min(target, state.altitude_m + step)

        if self._clock() < state.velocity_deadline:
            vx, vy, vz, yaw_rate = state.velocity
            state.yaw_rad += yaw_rate * dt
            state.north_m += (
                vx * math.cos(state.yaw_rad) - vy * math.sin(state.yaw_rad)
            ) * dt
            state.east_m += (
                vx * math.sin(state.yaw_rad) + vy * math.cos(state.yaw_rad)
            ) * dt
            state.altitude_m = max(0.0, state.altitude_m - vz * dt)

    def _stick_throttle(self) -> float | None:
        """The overridden throttle as a 0.0-1.0 fraction, or None if lapsed.

        A lapsed override is not "hold the last value": ArduPilot hands the
        channel back to the receiver, and this airframe has none.  Reporting
        it as no throttle at all is what makes a flight command that stops
        refreshing its override fail here instead of over a floor.
        """

        state = self.state
        if state.rc_throttle_pwm is None:
            return None
        if self._clock() > state.rc_override_deadline:
            if self._last.get("override_lapsed") != state.rc_override_deadline:
                self._last["override_lapsed"] = state.rc_override_deadline
                self._note("RC override lapsed; no throttle source")
            return None
        minimum = state.parameters["RC3_MIN"]
        maximum = state.parameters["RC3_MAX"]
        return (state.rc_throttle_pwm - minimum) / (maximum - minimum)

    def _advance_on_sticks(self, dt: float) -> None:
        """Move the aircraft the way the overridden throttle stick asks.

        The same stick means two different things: motor thrust in STABILIZE,
        a climb rate in ALT_HOLD.  Modelling both is the point -- a flight
        command that confuses them is exactly what this double exists to
        catch, and the difference is a flyaway on the real aircraft.
        """

        state = self.state
        throttle = self._stick_throttle()
        if throttle is None:
            # No throttle source: the aircraft comes down.
            state.altitude_m = max(0.0, state.altitude_m - DESCENT_RATE_MS * dt)
            return

        if state.mode == COPTER_MODES["STABILIZE"]:
            gain = STABILIZE_THRUST_GAIN
            if self.fault is Fault.THROTTLE_RUNAWAY:
                gain *= RUNAWAY_THRUST_MULTIPLIER
            rate = (throttle - state.parameters["MOT_THST_HOVER"]) * gain
            if self.fault is Fault.EKF_DIVERGENCE:
                # 2026-08-21: the commanded throttle was about half of hover,
                # so no stick this command sends ever lifts the aircraft.
                rate = min(rate, 0.0)
        else:
            deadzone = state.parameters["THR_DZ"] / 1_000.0
            offset = throttle - 0.5
            if abs(offset) <= deadzone:
                rate = 0.0
            elif offset > 0.0:
                rate = (offset - deadzone) / (0.5 - deadzone) * PILOT_SPEED_UP_MS
            else:
                rate = (offset + deadzone) / (0.5 - deadzone) * PILOT_SPEED_DN_MS

        if self.fault is Fault.NO_TAKEOFF:
            # The throttle is commanded and nothing happens.  In a stick-driven
            # mode this is the aircraft that never leaves the floor, and the
            # only thing that ends it is the climb's own timeout.
            return
        if self.fault is Fault.ALTITUDE_RUNAWAY and state.altitude_m > 0.05:
            # A climb slow enough to pass the climb-rate guard and keep going
            # anyway.  This is what leaves the altitude ceiling as the only
            # thing between the aircraft and the roof, so the rehearsal has to
            # be able to produce it.
            rate = 0.30
        rate = min(2.0, max(-DESCENT_RATE_MS, rate))
        if state.altitude_m <= 0.0 and rate <= 0.0:
            return
        if self._flight_started == 0.0 and rate > 0.0:
            self._flight_started = self._clock()
        state.altitude_m = max(0.0, state.altitude_m + rate * dt)

    # -- outbound -------------------------------------------------------

    def _boot_ms(self, now: float) -> int:
        return int((now - self._boot) * 1_000) & 0xFFFFFFFF

    def _due(self, name: str, now: float, period: float) -> bool:
        if now - self._last.get(name, -1e9) < period:
            return False
        self._last[name] = now
        return True

    def emit(self, now: float) -> None:
        """Send whichever periodic messages are due."""

        link = self._link()
        state = self.state
        flying = state.altitude_m > TOUCHDOWN_ALTITUDE_M
        airborne_for = now - self._flight_started if self._flight_started else 0.0

        heartbeat_lost = self.fault is Fault.HEARTBEAT_LOSS and airborne_for > 1.0
        if not heartbeat_lost and self._due("heartbeat", now, 0.25):
            base = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if state.armed:
                base |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            link.mav.heartbeat_send(
                mavlink.MAV_TYPE_QUADROTOR,
                mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base,
                state.mode,
                mavlink.MAV_STATE_ACTIVE if state.armed else mavlink.MAV_STATE_STANDBY,
            )

        altitude_lost = (
            self.fault is Fault.STALE_ALTITUDE and flying and airborne_for > 1.0
        )
        if not altitude_lost and self._due("distance", now, 0.1):
            centimetres = max(RANGEFINDER_MIN_CM, round(state.altitude_m * 100.0))
            link.mav.distance_sensor_send(
                self._boot_ms(now),
                RANGEFINDER_MIN_CM,
                RANGEFINDER_MAX_CM,
                int(min(centimetres, 0xFFFF)),
                mavlink.MAV_DISTANCE_SENSOR_LASER,
                1,
                DOWNWARD,
                0,
            )

        if self._due("local_position", now, 0.1):
            diverged = self.fault in _DIVERGED_FAULTS
            link.mav.local_position_ned_send(
                self._boot_ms(now),
                state.north_m,
                state.east_m,
                DIVERGED_ALTITUDE_M if diverged else -state.altitude_m,
                0.0,
                0.0,
                -DIVERGED_CLIMB_MS if diverged else 0.0,
            )

        if self._due("attitude", now, 0.1):
            link.mav.attitude_send(
                self._boot_ms(now), 0.0, 0.0, state.yaw_rad, 0.0, 0.0, 0.0
            )

        if self._due("ekf_status", now, 0.5):
            # Attitude, both velocities, vertical position, and a predicted
            # relative horizontal position: a healthy flow-navigating vehicle.
            link.mav.ekf_status_report_send(
                1 | 2 | 4 | 8 | 32 | 256, 0.1, 0.1, 0.1, 0.1, 0.1
            )

        if self._due("sys_status", now, 0.5):
            link.mav.sys_status_send(
                0, 0, 0, 500, int(state.battery_v * 1_000), -1, -1, 0, 0, 0, 0, 0, 0
            )

    # -- inbound --------------------------------------------------------

    def pump(self, budget: int = 40) -> None:
        """Handle every queued inbound message, up to ``budget`` of them."""

        link = self._link()
        for _ in range(budget):
            message = link.recv_match(blocking=False)
            if message is None:
                return
            self.handle(message)

    def handle(self, message: Any) -> None:
        handler = getattr(self, f"_on_{message.get_type().lower()}", None)
        if handler is not None:
            handler(message)

    def _on_param_request_read(self, message: Any) -> None:
        name = str(message.param_id).split("\0", 1)[0]
        value = self.state.parameters.get(name)
        if value is None:
            return
        names = sorted(self.state.parameters)
        self._link().mav.param_value_send(
            name.encode("ascii"),
            value,
            mavlink.MAV_PARAM_TYPE_REAL32,
            len(names),
            names.index(name),
        )

    def _on_set_mode(self, message: Any) -> None:
        requested = int(message.custom_mode)
        if requested not in MODE_NAMES:
            return
        if self.fault is Fault.REFUSE_LAND and requested == COPTER_MODES["LAND"]:
            self._note("ignored LAND request (injected fault)")
            return
        self.state.mode = requested
        self._note(f"mode -> {self.state.mode_name}")

    def _on_command_long(self, message: Any) -> None:
        command = int(message.command)
        result = mavlink.MAV_RESULT_ACCEPTED

        if command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            result = self._arm_or_disarm(
                bool(message.param1 >= 0.5), force=float(message.param2)
            )
        elif command == mavlink.MAV_CMD_NAV_TAKEOFF:
            result = self._takeoff(float(message.param7))
        elif command in {
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            401,  # MAV_CMD_RUN_PREARM_CHECKS
        }:
            result = mavlink.MAV_RESULT_ACCEPTED
        else:
            result = mavlink.MAV_RESULT_UNSUPPORTED

        self._link().mav.command_ack_send(command, result)

    def _arm_or_disarm(self, arm: bool, force: float = 0.0) -> int:
        state = self.state
        if not arm:
            if int(force) == FORCE_DISARM_MAGIC:
                # ArduPilot's documented override.  Modelling it matters: the
                # abort key exists precisely for an aircraft that is airborne
                # and doing something wrong, so a double that only accepts
                # disarms on the ground can never test it.
                if state.altitude_m > TOUCHDOWN_ALTITUDE_M:
                    self._note("FORCED disarm while airborne; the aircraft drops")
                else:
                    self._note("forced disarm")
                state.armed = False
                state.altitude_m = 0.0
                state.takeoff_target_m = None
                return mavlink.MAV_RESULT_ACCEPTED
            if state.altitude_m > TOUCHDOWN_ALTITUDE_M:
                self._note("refused disarm while airborne")
                return mavlink.MAV_RESULT_DENIED
            state.armed = False
            state.takeoff_target_m = None
            self._note("disarmed")
            return mavlink.MAV_RESULT_ACCEPTED
        if self.fault is Fault.REFUSE_ARM:
            self._note("refused arm (injected fault)")
            return mavlink.MAV_RESULT_DENIED
        if state.mode == COPTER_MODES["STABILIZE"]:
            # RC_OPTIONS bit 5 is set on this airframe: the vehicle refuses to
            # arm unless the throttle channel reads its calibrated minimum.
            # Enforcing it here is what makes a rehearsal prove that the
            # override is already running before the arm request goes out.
            throttle = state.rc_throttle_pwm
            if throttle is None or throttle > state.parameters["RC3_MIN"] + 20:
                self._note("refused arm in STABILIZE: throttle is not at minimum")
                return mavlink.MAV_RESULT_DENIED
        elif state.mode not in {
            COPTER_MODES["GUIDED"],
            COPTER_MODES["ALT_HOLD"],
            COPTER_MODES["GUIDED_NOGPS"],
        }:
            self._note(f"refused arm in {state.mode_name}")
            return mavlink.MAV_RESULT_DENIED
        state.armed = True
        self._note(f"armed in {state.mode_name}")
        return mavlink.MAV_RESULT_ACCEPTED

    def _takeoff(self, target_m: float) -> int:
        state = self.state
        if not state.armed or state.mode != COPTER_MODES["GUIDED"]:
            return mavlink.MAV_RESULT_DENIED
        if not math.isfinite(target_m) or target_m <= 0.0:
            return mavlink.MAV_RESULT_DENIED
        state.takeoff_target_m = target_m
        self._flight_started = self._clock()
        self._note(f"takeoff to {target_m:.2f} m")
        return mavlink.MAV_RESULT_ACCEPTED

    def _on_rc_channels_override(self, message: Any) -> None:
        state = self.state
        throttle = int(message.chan3_raw)
        if throttle == 0:
            state.rc_throttle_pwm = None
            state.rc_override_deadline = 0.0
            self._note("RC override released")
            return
        state.rc_throttle_pwm = throttle
        state.rc_override_deadline = self._clock() + RC_OVERRIDE_TIME_S

    def _on_set_position_target_local_ned(self, message: Any) -> None:
        state = self.state
        if not state.armed:
            return
        state.velocity = (
            float(message.vx),
            float(message.vy),
            float(message.vz),
            float(message.yaw_rate),
        )
        # A real autopilot stops on an expired setpoint; so does this one.
        state.velocity_deadline = self._clock() + 1.0

    def _on_set_attitude_target(self, message: Any) -> None:
        """With GUID_OPTIONS bit 3 clear, thrust is a climb rate: 0.5 holds."""

        if not self.state.armed:
            return
        thrust = float(message.thrust)
        # PILOT_SPEED_UP is 25 cm/s on this airframe.
        self.state.climb_rate_ms = (thrust - 0.5) * 2.0 * 0.25
        self.state.climb_deadline = self._clock() + 1.0
        if self._flight_started == 0.0 and thrust > 0.5:
            self._flight_started = self._clock()

    def _on_log_request_list(self, message: Any) -> None:
        self._link().mav.log_entry_send(1, 1, 1, int(time.time()), 4096)

    def _note(self, line: str) -> None:
        self.log.append(f"{self._clock() - self.started:7.2f}s  {line}")
