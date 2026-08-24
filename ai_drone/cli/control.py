"""Guarded takeoff, hover, and landing command."""

from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
from collections.abc import Sequence
from contextlib import contextmanager
from types import FrameType
from typing import Any

from ai_drone.flight.controller import DroneController, FlightSafetyError
from ai_drone.flight.dataflash import latest_dataflash_log
from ai_drone.flight.guards import FlightGuardError, check_safety_guardrails
from ai_drone.flight.recording import FlightRecorder
from ai_drone.validation import finite_in_range

FLIGHT_CONFIRMATION = "FLIGHT_TEST_READY"
MAX_AUTONOMOUS_CEILING_M = 0.8
MAX_AUTONOMOUS_TAKEOFF_M = 0.6
logger = logging.getLogger(__name__)


def _validate_common(args: argparse.Namespace) -> None:
    if isinstance(args.baud, bool) or not 1 <= args.baud <= 4_000_000:
        raise ValueError("--baud must be between 1 and 4000000")
    finite_in_range(
        args.max_alt,
        "--max-alt",
        minimum=0.1,
        maximum=MAX_AUTONOMOUS_CEILING_M,
    )
    if hasattr(args, "takeoff_alt"):
        finite_in_range(
            args.takeoff_alt,
            "--takeoff-alt",
            minimum=0.15,
            maximum=min(args.max_alt, MAX_AUTONOMOUS_TAKEOFF_M),
        )
    finite_in_range(args.duration, "--duration", minimum=0.1, maximum=3_600.0)
    if hasattr(args, "min_battery"):
        finite_in_range(args.min_battery, "--min-battery", minimum=0.0, maximum=60.0)
    if hasattr(args, "navigation_timeout"):
        finite_in_range(
            args.navigation_timeout,
            "--navigation-timeout",
            minimum=1.0,
            maximum=60.0,
        )


def _require_flight_confirmation(args: argparse.Namespace) -> None:
    if args.confirm_flight != FLIGHT_CONFIRMATION:
        raise ValueError(
            f"--confirm-flight must be exactly {FLIGHT_CONFIRMATION}; this confirms "
            "the aircraft is complete, props are secure, the area is clear, and an "
            "independent emergency LAND method is ready"
        )


def _controller(args: argparse.Namespace) -> DroneController:
    return DroneController(
        device=args.device,
        baud=args.baud,
        max_altitude=args.max_alt,
        min_battery_voltage=args.min_battery,
    )


@contextmanager
def _termination_event():
    """Turn service-stop signals into a guarded LAND request.

    The handler only sets an event.  Controller polling notices it and starts
    LAND; once landing begins, later signals cannot interrupt that cleanup.
    """

    requested = threading.Event()
    if threading.current_thread() is not threading.main_thread():
        yield requested
        return

    previous: dict[signal.Signals, Any] = {}

    def request_stop(signum: int, _frame: FrameType | None) -> None:
        logger.warning("received signal %s; requesting guarded LAND", signum)
        requested.set()

    handled = [signal.SIGTERM]
    if hasattr(signal, "SIGHUP"):
        handled.append(signal.SIGHUP)
    try:
        for handled_signal in handled:
            previous[handled_signal] = signal.getsignal(handled_signal)
            signal.signal(handled_signal, request_stop)
        yield requested
    finally:
        for handled_signal, old_handler in previous.items():
            signal.signal(handled_signal, old_handler)


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
        if drone.flight_mode != "LOITER":
            drone.emergency_stop()
            raise FlightSafetyError(
                f"Loiter hold left LOITER mode for {drone.flight_mode or 'unknown'}"
            )
        try:
            check_safety_guardrails(drone, min_battery_v)
        except FlightGuardError as error:
            raise FlightSafetyError(str(error)) from error
        time.sleep(0.05)


def cmd_hover(args: argparse.Namespace) -> int:
    _require_flight_confirmation(args)
    with (
        _termination_event() as stop_requested,
        _flight_session(args) as (drone, record),
    ):
        drone.stop_requested = stop_requested.is_set
        record.event("guided_nogps_takeoff_started", target_alt_m=args.takeoff_alt)
        drone.takeoff(args.takeoff_alt)
        record.event("loiter_acquisition_started")
        drone.enter_loiter(timeout=args.navigation_timeout)
        record.event("loiter_started", ekf_flags=drone.ekf_flags)
        _monitor(drone, args.duration, args.min_battery)
        record.event("landing_started")
        drone.land()
        record.event("landed")
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Guarded ArduPilot takeoff, hover, and landing command",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    commands = parser.add_subparsers(dest="command", required=True)

    hover = commands.add_parser(
        "hover",
        aliases=["takeoff"],
        help="take off in GuidedNoGPS, hold no-GPS Loiter, then land",
    )
    _add_common_options(hover)
    hover.add_argument("--duration", type=float, default=5.0)
    hover.add_argument(
        "--navigation-timeout",
        type=float,
        default=20.0,
        help="maximum time to establish stable optical-flow relative position",
    )
    hover.add_argument("--confirm-flight")
    hover.set_defaults(handler=cmd_hover)

    return parser


def _add_common_options(parser: argparse.ArgumentParser) -> None:
    """Add one shared set of connection/flight-limit options to a subcommand."""

    parser.add_argument("--device", help="MAVLink serial path or network endpoint")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--max-alt", type=float, default=MAX_AUTONOMOUS_CEILING_M)
    parser.add_argument("--takeoff-alt", type=float, default=0.5)
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
