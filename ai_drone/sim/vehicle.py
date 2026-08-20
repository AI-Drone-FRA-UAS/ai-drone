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
            link.mav.local_position_ned_send(
                self._boot_ms(now),
                state.north_m,
                state.east_m,
                -state.altitude_m,
                0.0,
                0.0,
                0.0,
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
            result = self._arm_or_disarm(bool(message.param1 >= 0.5))
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

    def _arm_or_disarm(self, arm: bool) -> int:
        state = self.state
        if not arm:
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
        if state.mode not in {
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
