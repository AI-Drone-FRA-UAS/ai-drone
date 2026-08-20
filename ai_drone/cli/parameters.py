"""Command adapter for reading and writing one flight-controller parameter."""

from __future__ import annotations

import argparse
import logging
from collections.abc import Sequence

from ai_drone.flight.controller import DroneController
from ai_drone.mavlink.parameter_write import (
    PROTECTED_PARAMETERS,
    ParameterWriteError,
    set_parameter,
)
from ai_drone.mavlink.parameters import request_parameter

CONFIRMATION = "WRITE_FLIGHT_CONTROLLER_PARAMETER"
logger = logging.getLogger(__name__)


def _controller(args: argparse.Namespace) -> DroneController:
    return DroneController(device=args.device, baud=args.baud)


def cmd_get(args: argparse.Namespace) -> int:
    with _controller(args) as drone:
        for name in args.names:
            value = request_parameter(drone._connection(), name)
            # Bitmask parameters must print as integers: ARMING_CHECK=1.04396e+06
            # is not a value anyone can act on.
            shown = f"{int(value)}" if float(value).is_integer() else f"{value:g}"
            print(f"{name} = {shown}")
    return 0


def cmd_set(args: argparse.Namespace) -> int:
    if args.confirm != CONFIRMATION:
        raise ValueError(
            f"--confirm must be exactly {CONFIRMATION}; this writes "
            f"{args.name}={args.value:g} to the flight controller"
        )
    with _controller(args) as drone:
        result = set_parameter(drone._connection(), args.name, args.value)
    print(result.describe())
    print(
        "\nThis change is live but not proven. Re-run 'drone-control preflight' "
        "before arming."
    )
    return 0


def _parser() -> argparse.ArgumentParser:
    guarded = ", ".join(sorted(PROTECTED_PARAMETERS))
    parser = argparse.ArgumentParser(
        description=(
            "Read or write one ArduPilot parameter. Writes require a fresh "
            "disarmed heartbeat and are verified by readback. These parameters "
            f"accept only the values this project permits: {guarded}."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    commands = parser.add_subparsers(dest="command", required=True)

    get = commands.add_parser("get", help="read one or more parameters")
    get.add_argument("names", nargs="+")
    get.set_defaults(handler=cmd_get)

    setter = commands.add_parser("set", help="write one parameter and verify it")
    setter.add_argument("name")
    setter.add_argument("value", type=float)
    setter.add_argument("--confirm")
    setter.set_defaults(handler=cmd_set)

    for sub in (get, setter):
        sub.add_argument("--device", help="MAVLink serial path or network endpoint")
        sub.add_argument("--baud", type=int, default=115200)

    return parser


def main(arguments: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(arguments)
    try:
        return int(args.handler(args))
    except KeyboardInterrupt:
        return 130
    except (ParameterWriteError, OSError, RuntimeError, ValueError) as error:
        logger.error("%s", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
