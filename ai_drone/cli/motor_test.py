"""Strictly guarded, short ArduPilot bench motor test.

This utility deliberately uses MAV_CMD_DO_MOTOR_TEST instead of normal vehicle
arming or throttle control. ArduPilot temporarily soft-arms the selected motor
outputs while the command is active and restores them when its timeout expires.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.config_snapshot import decode_parameter_name, heartbeat_is_armed
from ai_drone.mavlink_devices import resolve_mavlink_endpoint

MAX_THROTTLE_PERCENT = 10.0
MAX_DURATION_SECONDS = 1.0
MOTOR_FUNCTIONS = frozenset(range(33, 41))
ACCEPTED = mavlink.MAV_RESULT_ACCEPTED


def _request_parameter(connection: Any, name: str, timeout: float = 3.0) -> float:
    connection.mav.param_request_read_send(
        connection.target_system,
        connection.target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["PARAM_VALUE", "HEARTBEAT"], blocking=True, timeout=0.25
        )
        if message is None:
            continue
        if message.get_type() == "HEARTBEAT":
            if heartbeat_is_armed(message):
                raise RuntimeError("Vehicle became ARMED before the motor test")
            continue
        if decode_parameter_name(message.param_id) == name:
            return float(message.param_value)
    raise TimeoutError(f"Flight controller did not return {name}")


def _configured_motor_count(connection: Any) -> int:
    functions = {
        round(_request_parameter(connection, f"SERVO{index}_FUNCTION"))
        for index in range(1, 9)
    }
    configured = sorted(functions & MOTOR_FUNCTIONS)
    if not configured:
        raise RuntimeError("No Motor1..Motor8 output functions are configured")
    expected = list(range(33, max(configured) + 1))
    if configured != expected:
        raise RuntimeError(f"Motor output functions are not contiguous: {configured}")
    return len(configured)


def _send_motor_test(
    connection: Any,
    *,
    first_motor: int,
    throttle_percent: float,
    duration: float,
    motor_count: int,
) -> None:
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        first_motor,
        mavlink.MOTOR_TEST_THROTTLE_PERCENT,
        throttle_percent,
        duration,
        motor_count,
        mavlink.MOTOR_TEST_ORDER_SEQUENCE,
        0,
    )


def _wait_for_ack(connection: Any, timeout: float = 4.0) -> Any:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT", "HEARTBEAT"],
            blocking=True,
            timeout=0.25,
        )
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "STATUSTEXT":
            print(f"ArduPilot: {message.text}")
        elif (
            message_type == "COMMAND_ACK"
            and int(message.command) == mavlink.MAV_CMD_DO_MOTOR_TEST
        ):
            return message
    raise TimeoutError("No acknowledgement for MAV_CMD_DO_MOTOR_TEST")


def _result_name(result: int) -> str:
    entry = mavlink.enums.get("MAV_RESULT", {}).get(result)
    return entry.name if entry is not None else str(result)


def _stop_motor_test(connection: Any, motor: int) -> None:
    """Request an immediate zero-output motor-test timeout."""

    _send_motor_test(
        connection,
        first_motor=motor,
        throttle_percent=0.0,
        duration=0.0,
        motor_count=1,
    )


def _wait_until_disarmed(connection: Any, timeout: float = 4.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None and not heartbeat_is_armed(heartbeat):
            return True
    return False


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Briefly test selected motor outputs on a propeller-free bench."
    )
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--motor", type=int, help="motor sequence number")
    selection.add_argument(
        "--all-motors", action="store_true", help="test all configured motors in order"
    )
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--throttle-percent", type=float, default=7.0)
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--countdown", type=int, default=5)
    parser.add_argument("--confirm-props-removed", required=True)
    parser.add_argument("--confirm-vehicle-secured", required=True)
    args = parser.parse_args(arguments)

    if args.confirm_props_removed != "PROPS_REMOVED":
        parser.error("--confirm-props-removed must be exactly PROPS_REMOVED")
    if args.confirm_vehicle_secured != "VEHICLE_SECURED":
        parser.error("--confirm-vehicle-secured must be exactly VEHICLE_SECURED")
    if not 1.0 <= args.throttle_percent <= MAX_THROTTLE_PERCENT:
        parser.error(
            f"--throttle-percent must be between 1 and {MAX_THROTTLE_PERCENT:g}"
        )
    if not 0.1 <= args.duration <= MAX_DURATION_SECONDS:
        parser.error(f"--duration must be between 0.1 and {MAX_DURATION_SECONDS:g}")
    if not 3 <= args.countdown <= 10:
        parser.error("--countdown must be between 3 and 10 seconds")

    endpoint = resolve_mavlink_endpoint(args.device, include_pi_uart=True)
    print(f"Connecting to {endpoint} at {args.baud} baud ...")
    connection = mavutil.mavlink_connection(
        endpoint,
        baud=args.baud,
        source_system=255,
        source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )
    started = False
    first_motor = 1
    try:
        heartbeat = connection.wait_heartbeat(timeout=15)
        if heartbeat is None:
            raise SystemExit("No ArduPilot heartbeat received.")
        if heartbeat_is_armed(heartbeat):
            raise SystemExit("Vehicle is already ARMED; refusing motor test.")
        connection.target_system = heartbeat.get_srcSystem()
        connection.target_component = heartbeat.get_srcComponent()

        arming_check = round(_request_parameter(connection, "ARMING_CHECK"))
        if arming_check == 0:
            raise SystemExit(
                "ARMING_CHECK=0. Restore and pass the pre-arm safety checks before "
                "using this motor-test utility."
            )

        motor_count = _configured_motor_count(connection)
        if args.motor is not None:
            if not 1 <= args.motor <= motor_count:
                parser.error(f"--motor must be between 1 and {motor_count}")
            first_motor = args.motor
            requested_count = 1
        else:
            requested_count = motor_count

        pwm_min = _request_parameter(connection, "MOT_PWM_MIN")
        pwm_max = _request_parameter(connection, "MOT_PWM_MAX")
        spin_min = _request_parameter(connection, "MOT_SPIN_MIN") * 100.0
        estimated_pwm = pwm_min + (pwm_max - pwm_min) * args.throttle_percent / 100.0

        print("\nBENCH MOTOR TEST — PROPELLERS MUST BE REMOVED")
        print(f"Configured motors: {motor_count}")
        print(
            f"Selection: {'all in sequence' if args.all_motors else first_motor}; "
            f"{args.duration:.2f} s each at {args.throttle_percent:.1f}% "
            f"(approximately {estimated_pwm:.0f} us)"
        )
        print(f"Configured MOT_SPIN_MIN is approximately {spin_min:.1f}%.")
        print("ArduPilot temporarily soft-arms outputs during MAV_CMD_DO_MOTOR_TEST.")
        print("Press Ctrl-C now to cancel.")
        for remaining in range(args.countdown, 0, -1):
            print(f"Starting in {remaining} ...", flush=True)
            time.sleep(1)

        _send_motor_test(
            connection,
            first_motor=first_motor,
            throttle_percent=args.throttle_percent,
            duration=args.duration,
            motor_count=requested_count,
        )
        started = True
        acknowledgement = _wait_for_ack(connection)
        result = int(acknowledgement.result)
        if result != ACCEPTED:
            raise RuntimeError(f"Motor test rejected: {_result_name(result)}")
        print("Motor-test command accepted.")

        expected = args.duration * (requested_count + 0.5 * (requested_count - 1))
        deadline = time.monotonic() + expected + 1.0
        while time.monotonic() < deadline:
            message = connection.recv_match(
                type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=0.25
            )
            if message is not None and message.get_type() == "STATUSTEXT":
                print(f"ArduPilot: {message.text}")
    except KeyboardInterrupt:
        print("Cancelled; requesting immediate motor-test stop.", file=sys.stderr)
        return 130
    finally:
        if started:
            _stop_motor_test(connection, first_motor)
            if not _wait_until_disarmed(connection):
                print(
                    "WARNING: no disarmed heartbeat was observed. Disconnect the LiPo "
                    "before approaching the vehicle.",
                    file=sys.stderr,
                )
        connection.close()

    print("Motor test complete; a disarmed heartbeat was observed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
