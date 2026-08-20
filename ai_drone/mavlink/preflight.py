"""A read-only go/no-go assessment of whether this aircraft could fly.

``drone-control status`` answers "is the link alive".  This module answers the
question that actually blocks a flight test: *if we commanded a takeoff right
now, what would stop it?*  It reads parameters and telemetry and sends nothing
that can move the aircraft -- no arm, no mode change, no setpoint, and no
parameter write.

The assessment is split in two on purpose.  ``gather`` does the I/O; ``assess``
is a pure function over a ``Snapshot``, so every verdict below is testable
without a vehicle.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.mavlink import arming_checks
from ai_drone.mavlink.parameters import decode_parameter_name
from ai_drone.mavlink.safety import heartbeat_is_armed, is_vehicle_message

DOWNWARD = mavlink.MAV_SENSOR_ROTATION_PITCH_270

# EK3_SRC1_VELXY selects the horizontal velocity source; 5 is OpticalFlow.
VELXY_OPTICAL_FLOW = 5.0
# ArduPilot ignores flow frames below FLOW_QUAL_MIN; its default is 10.
MINIMUM_FLOW_QUALITY = 10

# EKF_STATUS_REPORT.flags bits, named so a report can explain itself.
EKF_ATTITUDE = 1
EKF_VELOCITY_HORIZ = 2
EKF_VELOCITY_VERT = 4
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
EKF_POS_VERT_ABS = 32
EKF_POS_VERT_AGL = 64
EKF_CONST_POS_MODE = 128
EKF_PRED_POS_HORIZ_REL = 256
EKF_PRED_POS_HORIZ_ABS = 512

EKF_FLAG_NAMES: tuple[tuple[str, int], ...] = (
    ("attitude", EKF_ATTITUDE),
    ("velocity_horiz", EKF_VELOCITY_HORIZ),
    ("velocity_vert", EKF_VELOCITY_VERT),
    ("pos_horiz_rel", EKF_POS_HORIZ_REL),
    ("pos_horiz_abs", EKF_POS_HORIZ_ABS),
    ("pos_vert_abs", EKF_POS_VERT_ABS),
    ("pos_vert_agl", EKF_POS_VERT_AGL),
    ("const_pos_mode", EKF_CONST_POS_MODE),
    ("pred_pos_horiz_rel", EKF_PRED_POS_HORIZ_REL),
    ("pred_pos_horiz_abs", EKF_PRED_POS_HORIZ_ABS),
)

# Parameters worth reading before a flight test.  Kept short: every extra name
# is another request/response round trip on a 115200 baud telemetry link.
PARAMETERS: tuple[str, ...] = (
    "ARMING_CHECK",
    "LOG_BACKEND_TYPE",
    "LOG_BITMASK",
    "BATT_MONITOR",
    "BATT_LOW_VOLT",
    "RNGFND1_TYPE",
    "RNGFND1_ORIENT",
    "EK3_SRC1_POSXY",
    "EK3_SRC1_VELXY",
    "EK3_SRC1_POSZ",
    "FLOW_TYPE",
    "FRAME_CLASS",
)


def describe_ekf_flags(flags: int) -> str:
    """Render an EKF_STATUS_REPORT bitmask as the names it sets."""

    present = [name for name, bit in EKF_FLAG_NAMES if flags & bit]
    return " ".join(present) if present else "none"


@dataclass(frozen=True)
class Snapshot:
    """What one read-only pass over the vehicle observed."""

    mode: str | None = None
    armed: bool = False
    parameters: dict[str, float] = field(default_factory=dict)
    ekf_flags: int | None = None
    gps_fix: int | None = None
    gps_satellites: int | None = None
    rangefinder_cm: int | None = None
    rangefinder_orientation: int | None = None
    battery_v: float | None = None
    local_position: bool = False
    flow_samples: int = 0
    flow_quality: int | None = None
    modes_available: tuple[str, ...] = ()
    statustexts: tuple[str, ...] = ()


@dataclass(frozen=True)
class Check:
    """One named verdict, with the reason a human needs to act on it."""

    name: str
    passed: bool | None
    detail: str

    @property
    def marker(self) -> str:
        return {True: "PASS", False: "FAIL", None: "UNKNOWN"}[self.passed]


def _parameter(snapshot: Snapshot, name: str) -> float | None:
    value = snapshot.parameters.get(name)
    return value if value is not None and math.isfinite(value) else None


def _check_arming_checks(snapshot: Snapshot) -> Check:
    value = _parameter(snapshot, arming_checks.PARAMETER)
    if value is None:
        return Check("arming_checks", None, f"{arming_checks.PARAMETER} did not answer")
    return Check(
        "arming_checks",
        arming_checks.is_acceptable(value),
        arming_checks.describe(value),
    )


def _check_logging(snapshot: Snapshot) -> Check:
    backend = _parameter(snapshot, "LOG_BACKEND_TYPE")
    bitmask = _parameter(snapshot, "LOG_BITMASK")
    if backend is None or bitmask is None:
        return Check("onboard_logging", None, "logging parameters did not answer")
    if int(backend) & 5 and bitmask > 0:
        return Check(
            "onboard_logging",
            True,
            f"LOG_BACKEND_TYPE={backend:g} LOG_BITMASK={bitmask:g}",
        )
    return Check(
        "onboard_logging",
        False,
        "onboard dataflash logging is off; a flight test would leave no log to review",
    )


def _check_rangefinder(snapshot: Snapshot) -> Check:
    if snapshot.rangefinder_cm is None:
        return Check(
            "downward_rangefinder",
            False,
            "no downward DISTANCE_SENSOR received; altitude guards cannot work",
        )
    if snapshot.rangefinder_orientation != DOWNWARD:
        return Check(
            "downward_rangefinder",
            False,
            f"rangefinder orientation is {snapshot.rangefinder_orientation}, "
            f"expected {DOWNWARD} (downward)",
        )
    return Check(
        "downward_rangefinder",
        True,
        f"{snapshot.rangefinder_cm / 100.0:.2f} m downward",
    )


def _check_optical_flow(snapshot: Snapshot) -> Check:
    """Is the downward flow sensor delivering frames the EKF would accept?"""

    if _parameter(snapshot, "FLOW_TYPE") in (None, 0.0):
        return Check("optical_flow", None, "FLOW_TYPE is 0; no flow sensor configured")
    if snapshot.flow_samples == 0:
        return Check(
            "optical_flow",
            False,
            "FLOW_TYPE is set but no OPTICAL_FLOW messages arrived; the sensor is "
            "not reaching the flight controller",
        )
    quality = snapshot.flow_quality
    if quality is None or quality < MINIMUM_FLOW_QUALITY:
        return Check(
            "optical_flow",
            False,
            f"{snapshot.flow_samples} frames but median quality {quality} is below "
            f"{MINIMUM_FLOW_QUALITY}; the EKF will discard them. Check floor "
            "texture and lighting.",
        )
    return Check(
        "optical_flow",
        True,
        f"{snapshot.flow_samples} frames, median quality {quality}/255",
    )


def _check_horizontal_position(snapshot: Snapshot) -> Check:
    """Does the EKF currently have a horizontal estimate, and if not, why not?

    A flow-only aircraft is *correctly* configured with ``EK3_SRC1_POSXY=0``
    and ``EK3_SRC1_VELXY=5``: optical flow supplies velocity, not position.  So
    a missing estimate on such an aircraft is not automatically a
    misconfiguration -- it is usually the EKF not yet fusing flow, which is
    normal while the aircraft sits on the floor below the sensor's usable
    range.  Saying otherwise would send someone off to "fix" a correct setup.
    """

    flags = snapshot.ekf_flags
    if flags is None:
        return Check("horizontal_position", None, "no EKF_STATUS_REPORT received")
    if flags & (EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS):
        return Check(
            "horizontal_position", True, f"EKF reports {describe_ekf_flags(flags)}"
        )

    velxy = _parameter(snapshot, "EK3_SRC1_VELXY")
    posxy = _parameter(snapshot, "EK3_SRC1_POSXY")
    reason = "EKF has no horizontal estimate yet"
    if flags & EKF_CONST_POS_MODE:
        reason += " and is in constant-position mode"

    if velxy == VELXY_OPTICAL_FLOW:
        if (
            snapshot.flow_samples
            and (snapshot.flow_quality or 0) >= MINIMUM_FLOW_QUALITY
        ):
            return Check(
                "horizontal_position",
                False,
                reason + ". This is a flow-based setup (EK3_SRC1_VELXY=5) and the "
                "flow sensor is healthy, so this is expected on the ground: the "
                "EKF starts fusing flow once the aircraft is clear of the floor. "
                "Only the vehicle's own PreArm verdict settles whether GUIDED "
                "will arm.",
            )
        return Check(
            "horizontal_position",
            False,
            reason + "; the configured flow velocity source is not delivering "
            "usable frames (see the optical_flow check)",
        )

    if posxy == 0.0 and (velxy in (None, 0.0)):
        return Check(
            "horizontal_position",
            False,
            reason + "; neither EK3_SRC1_POSXY nor EK3_SRC1_VELXY names a "
            "horizontal source, so nothing can supply one",
        )
    if snapshot.gps_fix is not None and snapshot.gps_fix < 2:
        reason += f"; GPS fix type {snapshot.gps_fix} with {snapshot.gps_satellites} satellites"
    return Check("horizontal_position", False, reason)


def _check_vertical_position(snapshot: Snapshot) -> Check:
    flags = snapshot.ekf_flags
    if flags is None:
        return Check("vertical_position", None, "no EKF_STATUS_REPORT received")
    if flags & (EKF_POS_VERT_ABS | EKF_POS_VERT_AGL):
        return Check("vertical_position", True, "EKF has a vertical position estimate")
    return Check("vertical_position", False, "EKF has no vertical position estimate")


def _check_battery(snapshot: Snapshot) -> Check:
    voltage = snapshot.battery_v
    if voltage is None:
        return Check("battery", None, "no SYS_STATUS battery voltage received")
    low = _parameter(snapshot, "BATT_LOW_VOLT")
    if low is not None and low > 0.0 and voltage <= low:
        return Check(
            "battery", False, f"{voltage:.2f} V is at or below BATT_LOW_VOLT={low:g}"
        )
    return Check("battery", True, f"{voltage:.2f} V")


def _check_disarmed(snapshot: Snapshot) -> Check:
    if snapshot.armed:
        return Check(
            "disarmed", False, "vehicle is ARMED; do not approach it or run a test"
        )
    return Check("disarmed", True, f"disarmed in {snapshot.mode or 'unknown mode'}")


def assess(snapshot: Snapshot) -> list[Check]:
    """Turn one observation into the ordered list of things that block a flight."""

    return [
        _check_disarmed(snapshot),
        _check_arming_checks(snapshot),
        _check_logging(snapshot),
        _check_rangefinder(snapshot),
        _check_optical_flow(snapshot),
        _check_vertical_position(snapshot),
        _check_horizontal_position(snapshot),
        _check_battery(snapshot),
    ]


def guided_takeoff_blockers(checks: list[Check]) -> list[Check]:
    """The subset of ``assess`` results that stop a GUIDED takeoff outright."""

    required = {
        "disarmed",
        "arming_checks",
        "onboard_logging",
        "downward_rangefinder",
        "optical_flow",
        "horizontal_position",
        "battery",
    }
    return [
        check for check in checks if check.name in required and check.passed is not True
    ]


class _Collector:
    """Accumulates one read-only pass so ``gather`` stays a plain loop."""

    def __init__(self) -> None:
        self.parameters: dict[str, float] = {}
        self.statustexts: list[str] = []
        self.mode: str | None = None
        self.armed = False
        self.ekf_flags: int | None = None
        self.gps_fix: int | None = None
        self.gps_satellites: int | None = None
        self.rangefinder_cm: int | None = None
        self.rangefinder_orientation: int | None = None
        self.battery_v: float | None = None
        self.local_position = False
        self.flow_qualities: list[int] = []

    def apply(self, message: Any, connection: Any) -> None:
        handler = getattr(self, f"_on_{message.get_type().lower()}", None)
        if handler is not None:
            handler(message, connection)

    def _on_heartbeat(self, message: Any, connection: Any) -> None:
        self.armed = heartbeat_is_armed(message)
        candidate = getattr(connection, "flightmode", None)
        if isinstance(candidate, str):
            self.mode = candidate

    def _on_param_value(self, message: Any, connection: Any) -> None:
        name = decode_parameter_name(message.param_id)
        if name in PARAMETERS:
            self.parameters[name] = float(message.param_value)

    def _on_ekf_status_report(self, message: Any, connection: Any) -> None:
        self.ekf_flags = int(message.flags)

    def _on_gps_raw_int(self, message: Any, connection: Any) -> None:
        self.gps_fix = int(message.fix_type)
        self.gps_satellites = int(message.satellites_visible)

    def _on_distance_sensor(self, message: Any, connection: Any) -> None:
        if int(message.orientation) == DOWNWARD or self.rangefinder_cm is None:
            self.rangefinder_cm = int(message.current_distance)
            self.rangefinder_orientation = int(message.orientation)

    def _on_sys_status(self, message: Any, connection: Any) -> None:
        voltage = float(message.voltage_battery) / 1_000.0
        if math.isfinite(voltage) and voltage > 0.0:
            self.battery_v = voltage

    def _on_local_position_ned(self, message: Any, connection: Any) -> None:
        self.local_position = True

    def _on_optical_flow(self, message: Any, connection: Any) -> None:
        self.flow_qualities.append(int(message.quality))

    def _on_statustext(self, message: Any, connection: Any) -> None:
        line = str(message.text).strip()
        if line and line not in self.statustexts:
            self.statustexts.append(line)

    def snapshot(self, modes_available: tuple[str, ...]) -> Snapshot:
        return Snapshot(
            mode=self.mode,
            armed=self.armed,
            parameters=dict(self.parameters),
            ekf_flags=self.ekf_flags,
            gps_fix=self.gps_fix,
            gps_satellites=self.gps_satellites,
            rangefinder_cm=self.rangefinder_cm,
            rangefinder_orientation=self.rangefinder_orientation,
            battery_v=self.battery_v,
            local_position=self.local_position,
            flow_samples=len(self.flow_qualities),
            flow_quality=(
                sorted(self.flow_qualities)[len(self.flow_qualities) // 2]
                if self.flow_qualities
                else None
            ),
            modes_available=modes_available,
            statustexts=tuple(self.statustexts),
        )


def _request_observations(connection: Any, system: int, component: int) -> None:
    """Ask for the streams and parameters this assessment reads.

    Every request here is read-only: bounded message intervals and parameter
    reads.  Nothing in this function can move the aircraft.
    """

    for message_id, rate_hz in (
        (mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2.0),
        (mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2.0),
        (mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 5.0),
        (mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2.0),
        (mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5.0),
        (mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW, 10.0),
    ):
        connection.mav.command_long_send(
            system,
            component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            round(1_000_000 / rate_hz),
            0,
            0,
            0,
            0,
            0,
        )
    for name in PARAMETERS:
        connection.mav.param_request_read_send(system, component, name.encode(), -1)


def gather(connection: Any, *, timeout: float = 12.0) -> Snapshot:
    """Observe the vehicle once.  Sends only stream and parameter requests."""

    if not math.isfinite(timeout) or not 1.0 <= timeout <= 60.0:
        raise ValueError("timeout must be between 1 and 60 seconds")

    system = int(connection.target_system)
    component = int(connection.target_component)
    _request_observations(connection, system, component)

    collector = _Collector()
    deadline = time.monotonic() + timeout
    retried = False
    while (remaining := deadline - time.monotonic()) > 0.0:
        if not retried and remaining < timeout / 2.0:
            # One retry, halfway through, for parameters the link dropped.
            retried = True
            for name in PARAMETERS:
                if name not in collector.parameters:
                    connection.mav.param_request_read_send(
                        system, component, name.encode(), -1
                    )
        message = connection.recv_match(blocking=True, timeout=min(remaining, 0.4))
        if message is None:
            continue
        if not is_vehicle_message(message, system_id=system, component_id=component):
            continue
        collector.apply(message, connection)

    return collector.snapshot(tuple(sorted(connection.mode_mapping() or {})))
