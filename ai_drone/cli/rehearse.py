"""Fly a real flight command against a simulated vehicle instead of an aircraft.

``drone-rehearse`` starts ``ai_drone.sim.vehicle`` on the loopback interface and
points the ordinary ``drone-control`` command at it.  The flight code under test
is the same code that flies the aircraft: same arming gate, same guards, same
LAND cleanup, same flight recording.  Only the vehicle is fake.

The endpoint is always built here and never taken from the operator, so a
rehearsal cannot be aimed at a serial port by accident.  A rehearsal that
reached the real flight controller would be the one bug this command must not
have.
"""

from __future__ import annotations

import argparse
import logging
import threading
import time
from collections.abc import Sequence

from ai_drone.cli.control import ARM_TEST_CONFIRMATION, FLIGHT_CONFIRMATION
from ai_drone.cli.control import main as control_main
from ai_drone.sim.vehicle import Fault, SimulatedVehicle
from ai_drone.validation import finite_in_range

logger = logging.getLogger(__name__)

BANNER = (
    "=== SIMULATED VEHICLE ===  no aircraft is involved; the endpoint is loopback only"
)


def _endpoint(port: int, *, listening: bool) -> str:
    if isinstance(port, bool) or not 1_024 <= port <= 65_535:
        raise ValueError("--port must be between 1024 and 65535")
    prefix = "tcpin" if listening else "tcp"
    return f"{prefix}:127.0.0.1:{port}"


def _fault(name: str) -> Fault:
    try:
        return Fault(name)
    except ValueError as error:
        choices = ", ".join(item.value for item in Fault)
        raise ValueError(
            f"unknown --fault {name!r}; choose one of {choices}"
        ) from error


def cmd_serve(args: argparse.Namespace) -> int:
    """Run only the simulated vehicle, for a manually driven drone-control."""

    vehicle = SimulatedVehicle(
        _endpoint(args.port, listening=True),
        fault=_fault(args.fault),
        battery_v=args.battery,
    )
    print(BANNER)
    print(f"listening on {vehicle.endpoint}  fault={vehicle.fault.value}")
    print(
        f"connect with: drone-control status --device {_endpoint(args.port, listening=False)}"
    )
    try:
        vehicle.run(max_seconds=args.seconds)
    except KeyboardInterrupt:
        print("\nsimulated vehicle stopped")
    _print_log(vehicle)
    return 0


def _print_log(vehicle: SimulatedVehicle) -> None:
    if vehicle.failure is not None:
        print(f"\nsimulator itself failed: {vehicle.failure!r}")
    print("\n--- what the simulated vehicle did ---")
    for line in vehicle.log:
        print(" ", line)
    if not vehicle.log:
        print("  (nothing; the flight command never reached the arming stage)")


def _rehearse(
    args: argparse.Namespace,
    command: list[str],
    *,
    confirmation: tuple[str, str] = ("--confirm-flight", FLIGHT_CONFIRMATION),
) -> int:
    finite_in_range(args.seconds, "--seconds", minimum=5.0, maximum=600.0)
    finite_in_range(args.drain, "--drain", minimum=0.0, maximum=60.0)
    vehicle = SimulatedVehicle(
        _endpoint(args.port, listening=True),
        fault=_fault(args.fault),
        battery_v=args.battery,
    )
    vehicle.open()
    worker = threading.Thread(target=vehicle.run, args=(args.seconds,), daemon=True)

    print(BANNER)
    print(f"fault={vehicle.fault.value}  battery={args.battery:.2f} V")
    print(f"running: drone-control {' '.join(command)}\n")

    worker.start()
    try:
        code = control_main(
            [
                *command,
                "--device",
                _endpoint(args.port, listening=False),
                *confirmation,
            ]
        )
    finally:
        # Keep serving after the command returns.  A guard sends LAND and then
        # raises, so stopping the vehicle the instant the command exits would
        # discard the very recovery this rehearsal exists to demonstrate.
        time.sleep(args.drain)
        vehicle.stop = True
        worker.join(timeout=5.0)

    _print_log(vehicle)
    state = vehicle.state
    print(
        f"\nfinal state: mode={state.mode_name} armed={state.armed} "
        f"altitude={state.altitude_m:.2f} m battery={state.battery_v:.2f} V"
    )
    expected_failure = vehicle.fault is not Fault.NONE
    if expected_failure:
        verdict = (
            "the injected fault was caught and the flight was stopped"
            if code != 0
            else "WARNING: the injected fault did NOT stop the flight"
        )
    else:
        verdict = (
            "clean rehearsal" if code == 0 else "the rehearsal failed without a fault"
        )
    print(f"drone-control exit code {code}: {verdict}")
    if expected_failure and code != 0:
        return 0
    return code


def cmd_arm_test(args: argparse.Namespace) -> int:
    return _rehearse(
        args,
        [
            "arm-test",
            "--max-alt",
            str(args.max_alt),
            "--duration",
            str(args.duration),
            "--min-battery",
            str(args.min_battery),
        ],
        confirmation=("--confirm-props-off", ARM_TEST_CONFIRMATION),
    )


def cmd_nogps(args: argparse.Namespace) -> int:
    return _rehearse(
        args,
        [
            "nogps-takeoff",
            "--takeoff-alt",
            str(args.takeoff_alt),
            "--max-alt",
            str(args.max_alt),
            "--duration",
            str(args.duration),
            "--min-battery",
            str(args.min_battery),
        ],
    )


def cmd_stabilize(args: argparse.Namespace) -> int:
    return _rehearse(
        args,
        [
            "stabilize-takeoff",
            "--takeoff-alt",
            str(args.takeoff_alt),
            "--max-alt",
            str(args.max_alt),
            "--duration",
            str(args.duration),
            "--min-battery",
            str(args.min_battery),
            "--climb",
            str(args.climb),
        ],
    )


def cmd_hover(args: argparse.Namespace) -> int:
    return _rehearse(
        args,
        [
            "hover",
            "--takeoff-alt",
            str(args.takeoff_alt),
            "--max-alt",
            str(args.max_alt),
            "--duration",
            str(args.duration),
            "--min-battery",
            str(args.min_battery),
        ],
    )


def cmd_velocity(args: argparse.Namespace) -> int:
    return _rehearse(
        args,
        [
            "velocity-test",
            "--takeoff-alt",
            str(args.takeoff_alt),
            "--max-alt",
            str(args.max_alt),
            "--duration",
            str(args.duration),
            "--min-battery",
            str(args.min_battery),
            "--vx",
            str(args.vx),
        ],
    )


def cmd_preflight(args: argparse.Namespace) -> int:
    vehicle = SimulatedVehicle(
        _endpoint(args.port, listening=True),
        fault=_fault(args.fault),
        battery_v=args.battery,
    )
    vehicle.open()
    worker = threading.Thread(target=vehicle.run, args=(args.seconds,), daemon=True)
    print(BANNER)
    worker.start()
    try:
        return control_main(
            [
                "preflight",
                "--device",
                _endpoint(args.port, listening=False),
                "--duration",
                str(min(args.seconds - 2.0, 12.0)),
            ]
        )
    finally:
        vehicle.stop = True
        worker.join(timeout=5.0)


def _add_shared(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", type=int, default=5760)
    parser.add_argument(
        "--fault",
        default="none",
        help="misbehavior to inject: " + ", ".join(item.value for item in Fault),
    )
    parser.add_argument("--battery", type=float, default=16.0)
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument(
        "--drain",
        type=float,
        default=4.0,
        help="keep the simulated vehicle running this long after the command exits, "
        "so a commanded LAND can be seen through",
    )


def _add_flight(parser: argparse.ArgumentParser) -> None:
    _add_shared(parser)
    parser.add_argument("--takeoff-alt", type=float, default=0.4)
    parser.add_argument("--max-alt", type=float, default=0.8)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--min-battery", type=float, default=14.4)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Rehearse drone-control against a simulated vehicle",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    commands = parser.add_subparsers(dest="command", required=True)

    serve = commands.add_parser("serve", help="run only the simulated vehicle")
    _add_shared(serve)
    serve.set_defaults(handler=cmd_serve)

    preflight = commands.add_parser(
        "preflight", help="run drone-control preflight against the simulator"
    )
    _add_shared(preflight)
    preflight.set_defaults(handler=cmd_preflight)

    arm_test = commands.add_parser(
        "arm-test", help="rehearse arming in GUIDED without a takeoff"
    )
    _add_flight(arm_test)
    arm_test.set_defaults(handler=cmd_arm_test)

    nogps = commands.add_parser(
        "nogps-takeoff", help="rehearse a position-free climb, hold and landing"
    )
    _add_flight(nogps)
    nogps.set_defaults(handler=cmd_nogps)

    stabilize = commands.add_parser(
        "stabilize-takeoff",
        help="rehearse a STABILIZE climb, an ALT_HOLD hold and a landing",
    )
    _add_flight(stabilize)
    stabilize.add_argument("--climb", type=float, default=0.06)
    stabilize.set_defaults(handler=cmd_stabilize)

    hover = commands.add_parser("hover", help="rehearse a guarded takeoff and landing")
    _add_flight(hover)
    hover.set_defaults(handler=cmd_hover)

    velocity = commands.add_parser(
        "velocity-test", help="rehearse a bounded body-velocity test"
    )
    _add_flight(velocity)
    velocity.add_argument("--vx", type=float, default=0.2)
    velocity.set_defaults(handler=cmd_velocity)

    return parser


def main(arguments: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(arguments)
    try:
        return int(args.handler(args))
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        logger.error("%s", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
