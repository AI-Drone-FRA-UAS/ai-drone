"""Strictly guarded, short ArduPilot bench motor test.

This utility deliberately uses MAV_CMD_DO_MOTOR_TEST instead of normal vehicle
arming or throttle control. ArduPilot temporarily soft-arms the selected motor
outputs while the command is active and restores them when its timeout expires.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
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

MAX_THROTTLE_PERCENT = 10.0
MAX_DURATION_SECONDS = 1.0
MOTOR_FUNCTIONS = frozenset(range(33, 41))
ACCEPTED = mavlink.MAV_RESULT_ACCEPTED


# Private compatibility name retained for focused tests and downstream imports;
# the implementation is shared with the flight controller.
_request_parameter = request_parameter


def _configured_motor_count(connection: Any) -> int:
    assignments: dict[int, int] = {}
    for output in range(1, 9):
        raw_function = float(_request_parameter(connection, f"SERVO{output}_FUNCTION"))
        if not math.isfinite(raw_function) or not raw_function.is_integer():
            raise RuntimeError(
                f"SERVO{output}_FUNCTION is not a finite integer: {raw_function!r}"
            )
        function = int(raw_function)
        if function not in MOTOR_FUNCTIONS:
            continue
        previous_output = assignments.get(function)
        if previous_output is not None:
            motor = function - min(MOTOR_FUNCTIONS) + 1
            raise RuntimeError(
                f"Motor{motor} is assigned to both SERVO{previous_output} and "
                f"SERVO{output} outputs"
            )
        assignments[function] = output

    configured = sorted(assignments)
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
        if not is_vehicle_message(
            message,
            system_id=int(connection.target_system),
            component_id=int(connection.target_component),
        ):
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
    while (remaining := deadline - time.monotonic()) > 0:
        try:
            require_fresh_disarmed_heartbeat(
                connection,
                system_id=int(connection.target_system),
                component_id=int(connection.target_component),
                timeout=remaining,
            )
        except RuntimeError:
            # A fresh armed heartbeat is expected while ArduPilot is stopping
            # the temporary motor test; keep waiting within the same deadline.
            continue
        except TimeoutError:
            return False
        else:
            return True
    return False


def _cleanup_motor_test(
    connection: Any,
    *,
    started: bool,
    first_motor: int,
) -> bool:
    """Stop a started test, verify disarm, and always close the connection."""

    disarmed_observed = not started
    if started:
        try:
            _stop_motor_test(connection, first_motor)
        except Exception as error:
            print(
                f"WARNING: could not send the motor-test stop command: {error}",
                file=sys.stderr,
            )
        else:
            try:
                disarmed_observed = _wait_until_disarmed(connection)
            except Exception as error:
                print(
                    f"WARNING: could not verify that the vehicle disarmed: {error}",
                    file=sys.stderr,
                )

    try:
        connection.close()
    except Exception as error:
        disarmed_observed = False
        print(
            f"WARNING: could not close the MAVLink connection: {error}", file=sys.stderr
        )

    if started and not disarmed_observed:
        print(
            "WARNING: no disarmed heartbeat was observed. Disconnect the LiPo "
            "before approaching the vehicle.",
            file=sys.stderr,
        )
    return disarmed_observed


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

    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
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
    cleanup_confirmed = False
    try:
        heartbeat = connection.wait_heartbeat(timeout=15)
        if heartbeat is None:
            raise SystemExit("No ArduPilot heartbeat received.")
        if heartbeat_is_armed(heartbeat):
            raise SystemExit("Vehicle is already ARMED; refusing motor test.")
        connection.target_system = heartbeat.get_srcSystem()
        connection.target_component = heartbeat.get_srcComponent()

        arming_check = float(_request_parameter(connection, "ARMING_CHECK"))
        if arming_check != 1.0:
            raise SystemExit(
                f"ARMING_CHECK={arming_check:g}. Set ARMING_CHECK=1 (all checks) and "
                "resolve every pre-arm failure before using this utility."
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

        require_fresh_disarmed_heartbeat(
            connection,
            system_id=int(connection.target_system),
            component_id=int(connection.target_component),
            timeout=2.5,
        )

        # Treat the command as active before writing it: a serial write can fail
        # after the controller received enough bytes to start the bounded test.
        started = True
        _send_motor_test(
            connection,
            first_motor=first_motor,
            throttle_percent=args.throttle_percent,
            duration=args.duration,
            motor_count=requested_count,
        )
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
        cleanup_confirmed = _cleanup_motor_test(
            connection,
            started=started,
            first_motor=first_motor,
        )

    if cleanup_confirmed:
        print("Motor test complete; a disarmed heartbeat was observed.")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
