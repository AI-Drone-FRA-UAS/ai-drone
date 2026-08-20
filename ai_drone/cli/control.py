"""Single command surface for passive status and guarded flight tests."""

from __future__ import annotations

import argparse
import logging
import time
from collections.abc import Sequence
from contextlib import contextmanager

from ai_drone.flight.controller import DroneController, FlightSafetyError
from ai_drone.flight.dataflash import latest_dataflash_log
from ai_drone.flight.guards import FlightGuardError, check_safety_guardrails
from ai_drone.flight.recording import FlightRecorder
from ai_drone.validation import finite_in_range

FLIGHT_CONFIRMATION = "FLIGHT_TEST_READY"
logger = logging.getLogger(__name__)


def _validate_common(args: argparse.Namespace) -> None:
    if isinstance(args.baud, bool) or not 1 <= args.baud <= 4_000_000:
        raise ValueError("--baud must be between 1 and 4000000")
    finite_in_range(args.max_alt, "--max-alt", minimum=0.1, maximum=10.0)
    if hasattr(args, "takeoff_alt"):
        finite_in_range(
            args.takeoff_alt, "--takeoff-alt", minimum=0.15, maximum=args.max_alt
        )
    finite_in_range(args.duration, "--duration", minimum=0.1, maximum=3_600.0)
    if hasattr(args, "min_battery"):
        finite_in_range(args.min_battery, "--min-battery", minimum=0.0, maximum=60.0)


def _require_flight_confirmation(args: argparse.Namespace) -> None:
    if args.confirm_flight != FLIGHT_CONFIRMATION:
        raise ValueError(
            f"--confirm-flight must be exactly {FLIGHT_CONFIRMATION}; this confirms "
            "the aircraft is complete, props are secure, the area is clear, and a pilot can take over"
        )


def _controller(args: argparse.Namespace) -> DroneController:
    return DroneController(
        device=args.device,
        baud=args.baud,
        max_altitude=args.max_alt,
    )


@contextmanager
def _flight_session(args: argparse.Namespace):
    with _controller(args) as drone:
        metadata = {
            key: value
            for key, value in vars(args).items()
            if key not in {"confirm_flight", "handler"}
            and isinstance(value, str | int | float | bool | type(None))
        }
        record = FlightRecorder(drone._connection(), metadata)
        try:
            yield drone, record
        except BaseException as error:
            record.finish(error)
            if drone.is_flying:
                try:
                    drone.emergency_stop()
                except Exception:
                    logger.exception("could not request emergency LAND")
            raise
        else:
            try:
                dataflash_log = latest_dataflash_log(drone._connection())
                record.set_dataflash_log(dataflash_log)
                record.event(
                    "dataflash_identified",
                    found=dataflash_log is not None,
                    **(dataflash_log or {}),
                )
            except Exception as error:
                record.event("dataflash_unavailable", error=str(error))
            record.finish()
        finally:
            record.close()


def _monitor(drone: DroneController, duration: float, min_battery_v: float) -> None:
    """Hold for ``duration`` seconds, aborting the moment a guard trips."""

    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        drone.update_telemetry()
        try:
            check_safety_guardrails(drone, min_battery_v)
        except FlightGuardError as error:
            raise FlightSafetyError(str(error)) from error
        time.sleep(0.05)


def cmd_status(args: argparse.Namespace) -> int:
    with _controller(args) as drone:
        deadline = time.monotonic() + args.duration
        next_report = 0.0
        while time.monotonic() < deadline:
            drone.update_telemetry()
            now = time.monotonic()
            if now >= next_report:
                altitude = (
                    f"{drone.current_altitude:.2f} m downward"
                    if drone.current_altitude is not None
                    else "unavailable"
                )
                battery = (
                    f"{drone.battery_voltage:.2f} V"
                    if drone.battery_voltage is not None
                    else "unavailable"
                )
                print(
                    f"mode={drone.flight_mode or 'unknown'} armed={drone.is_armed} "
                    f"altitude={altitude} battery={battery}"
                )
                next_report = now + 1.0
            time.sleep(0.05)
    return 0


def cmd_hover(args: argparse.Namespace) -> int:
    _require_flight_confirmation(args)
    with _flight_session(args) as (drone, record):
        record.event("takeoff_started", target_alt_m=args.takeoff_alt)
        drone.takeoff(args.takeoff_alt)
        record.event("hover_started")
        _monitor(drone, args.duration, args.min_battery)
        record.event("landing_started")
        drone.land()
        record.event("landed")
    return 0


def cmd_velocity_test(args: argparse.Namespace) -> int:
    _require_flight_confirmation(args)
    finite_in_range(args.vx, "--vx", minimum=-1.0, maximum=1.0)
    finite_in_range(args.vy, "--vy", minimum=-1.0, maximum=1.0)
    finite_in_range(args.vz, "--vz", minimum=-0.5, maximum=0.5)
    finite_in_range(args.yaw_rate, "--yaw-rate", minimum=-45.0, maximum=45.0)
    with _flight_session(args) as (drone, record):
        record.event("takeoff_started", target_alt_m=args.takeoff_alt)
        drone.takeoff(args.takeoff_alt)
        record.event(
            "velocity_started",
            vx=args.vx,
            vy=args.vy,
            vz=args.vz,
            yaw_rate_deg=args.yaw_rate,
        )
        deadline = time.monotonic() + args.duration
        try:
            while time.monotonic() < deadline:
                drone.update_telemetry()
                try:
                    check_safety_guardrails(drone, args.min_battery)
                except FlightGuardError as error:
                    raise FlightSafetyError(str(error)) from error
                drone.send_velocity_body(args.vx, args.vy, args.vz, args.yaw_rate)
                time.sleep(0.2)
        finally:
            try:
                drone.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            except (OSError, RuntimeError):
                logger.warning("could not send final zero-velocity setpoint")
        record.event("landing_started")
        drone.land()
        record.event("landed")
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Canonical ArduPilot status and guarded flight-test command",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    commands = parser.add_subparsers(dest="command", required=True)

    status = commands.add_parser("status", help="passively monitor vehicle state")
    _add_common_options(status, include_takeoff=False)
    status.add_argument("--duration", type=float, default=5.0)
    status.set_defaults(handler=cmd_status)

    hover = commands.add_parser(
        "hover",
        aliases=["takeoff"],
        help="take off, hold GUIDED position, then land",
    )
    _add_common_options(hover, include_takeoff=True)
    hover.add_argument("--duration", type=float, default=5.0)
    hover.add_argument("--confirm-flight")
    hover.set_defaults(handler=cmd_hover)

    velocity = commands.add_parser(
        "velocity-test", help="take off, send bounded body velocity, then land"
    )
    _add_common_options(velocity, include_takeoff=True)
    velocity.add_argument("--duration", type=float, default=4.0)
    velocity.add_argument("--vx", type=float, default=0.2)
    velocity.add_argument("--vy", type=float, default=0.0)
    velocity.add_argument("--vz", type=float, default=0.0)
    velocity.add_argument("--yaw-rate", type=float, default=10.0)
    velocity.add_argument("--confirm-flight")
    velocity.set_defaults(handler=cmd_velocity_test)

    return parser


def _add_common_options(
    parser: argparse.ArgumentParser, *, include_takeoff: bool
) -> None:
    """Add one shared set of connection/flight-limit options to a subcommand."""

    parser.add_argument("--device", help="MAVLink serial path or network endpoint")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--max-alt", type=float, default=0.8)
    if include_takeoff:
        parser.add_argument("--takeoff-alt", type=float, default=0.4)
        parser.add_argument(
            "--min-battery",
            type=float,
            default=14.4,
            help="abort and stop below this pack voltage",
        )


def main(arguments: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(arguments)
    try:
        _validate_common(args)
        return int(args.handler(args))
    except KeyboardInterrupt:
        logger.warning(
            "operator interrupted command; controller cleanup requested LAND"
        )
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        logger.error("%s", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
