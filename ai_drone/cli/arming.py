"""Command adapter for reading and setting the pre-arm check configuration."""

from __future__ import annotations

import argparse
import logging
from collections.abc import Sequence

from ai_drone.flight.controller import DroneController
from ai_drone.mavlink.arming_checks import (
    ALL_CHECKS,
    ALL_EXCEPT_GPS,
    PARAMETER,
    ArmingCheckError,
    RestoreResult,
    describe,
    is_acceptable,
    read_arming_checks,
)
from ai_drone.mavlink.parameter_write import ParameterWriteError, set_parameter


def restore_arming_checks(
    connection, *, without_gps: bool = False, timeout: float = 5.0
) -> RestoreResult:
    """Write a permitted pre-arm configuration and verify the vehicle took it.

    Both targets are fixed constants.  Neither this function nor its caller can
    assemble an arbitrary mask, and there is no path to 0.
    """

    target = ALL_EXCEPT_GPS if without_gps else ALL_CHECKS
    try:
        result = set_parameter(connection, PARAMETER, target, timeout=timeout)
    except ParameterWriteError as error:
        raise ArmingCheckError(f"{error}. Do not arm.") from error
    return RestoreResult(previous=result.previous, current=result.current)


CONFIRMATION = "RESTORE_PREARM_CHECKS"
logger = logging.getLogger(__name__)


def _controller(args: argparse.Namespace) -> DroneController:
    return DroneController(device=args.device, baud=args.baud)


def cmd_show(args: argparse.Namespace) -> int:
    with _controller(args) as drone:
        value = read_arming_checks(drone._connection())
    print(describe(value))
    if is_acceptable(value):
        return 0
    print(f"\nFix it with: drone-arming-checks restore --confirm {CONFIRMATION}")
    print(
        "Add --without-gps on an aircraft with no GPS receiver: that clears the "
        "two GPS checks and leaves every other check enabled."
    )
    return 2


def cmd_restore(args: argparse.Namespace) -> int:
    if args.confirm != CONFIRMATION:
        target = ALL_EXCEPT_GPS if args.without_gps else ALL_CHECKS
        raise ValueError(
            f"--confirm must be exactly {CONFIRMATION}; this writes "
            f"ARMING_CHECK={int(target)} to the flight controller"
        )
    with _controller(args) as drone:
        result = restore_arming_checks(
            drone._connection(), without_gps=args.without_gps
        )
    print(result.describe())
    if args.without_gps:
        print(
            "\nThe GPS lock and GPS configuration checks are now off because this "
            "aircraft has no GPS receiver. Every other check still runs and still "
            "reports. Set --without-gps off again if a receiver is ever fitted."
        )
    print("\nRun 'drone-control preflight' and resolve what it reports before arming.")
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read or set ArduPilot's pre-arm check configuration. Only two "
            f"values are reachable: {int(ALL_CHECKS)} (every check) and "
            f"{int(ALL_EXCEPT_GPS)} (every check except the two GPS checks). "
            "Disabling the checks entirely is not possible through this command."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    commands = parser.add_subparsers(dest="command", required=True)

    show = commands.add_parser("show", help="read the live ARMING_CHECK value")
    show.set_defaults(handler=cmd_show)

    restore = commands.add_parser(
        "restore", help="write a permitted pre-arm check configuration"
    )
    restore.add_argument("--confirm")
    restore.add_argument(
        "--without-gps",
        action="store_true",
        help="clear only the GPS lock and GPS configuration checks",
    )
    restore.set_defaults(handler=cmd_restore)

    for sub in (show, restore):
        sub.add_argument("--device", help="MAVLink serial path or network endpoint")
        sub.add_argument("--baud", type=int, default=115200)

    return parser


def main(arguments: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(arguments)
    try:
        return int(args.handler(args))
    except KeyboardInterrupt:
        return 130
    except (ArmingCheckError, OSError, RuntimeError, ValueError) as error:
        logger.error("%s", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
