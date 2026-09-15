"""Command-line adapter for payload mount servo control."""

from __future__ import annotations

import argparse
import sys

from ai_drone.mount import (
    DEFAULT_SETTLE_S,
    MOUNT_CLOSE_VALUE,
    MOUNT_OPEN_VALUE,
    MountController,
    parse_servo_input,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="drone-mount",
        description="Direct payload mount control (open / close / set).",
    )
    subparsers = parser.add_subparsers(
        dest="action",
        required=True,
        help="Mount action to perform",
    )

    open_parser = subparsers.add_parser(
        "open",
        help="Open the payload mount (servo -> 0.0)",
    )
    open_parser.add_argument(
        "--settle",
        "-s",
        type=float,
        default=DEFAULT_SETTLE_S,
        help=f"Settle time in seconds (default: {DEFAULT_SETTLE_S})",
    )
    open_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Test without moving physical hardware",
    )

    close_parser = subparsers.add_parser(
        "close",
        help="Close the payload mount (servo -> -1.0)",
    )
    close_parser.add_argument(
        "--settle",
        "-s",
        type=float,
        default=DEFAULT_SETTLE_S,
        help=f"Settle time in seconds (default: {DEFAULT_SETTLE_S})",
    )
    close_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Test without moving physical hardware",
    )

    set_parser = subparsers.add_parser(
        "set",
        help="Set the servo to a specific normalized position or pulse width",
    )
    set_parser.add_argument(
        "value",
        help="Target position (-1.0 to 1.0, e.g. '0.0', '1500us', '-30deg')",
    )
    set_parser.add_argument(
        "--settle",
        "-s",
        type=float,
        default=DEFAULT_SETTLE_S,
        help=f"Settle time in seconds (default: {DEFAULT_SETTLE_S})",
    )
    set_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Test without moving physical hardware",
    )

    return parser


def main(arguments: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(arguments)

    try:
        with MountController(dry_run=args.dry_run) as controller:
            if args.action == "open":
                print(f"Opening payload mount (servo value -> {MOUNT_OPEN_VALUE}) ...")
                controller.open_mount(settle_s=args.settle)
                print("Payload mount is OPEN.")
            elif args.action == "close":
                print(f"Closing payload mount (servo value -> {MOUNT_CLOSE_VALUE}) ...")
                controller.close_mount(settle_s=args.settle)
                print("Payload mount is CLOSED.")
            elif args.action == "set":
                target = parse_servo_input(args.value)
                print(f"Setting payload mount position to {target:.2f} ...")
                controller.set_position(target, settle_s=args.settle)
                print(f"Payload mount position set to {target:.2f}.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
