"""Single command surface for passive status and guarded flight tests."""

from __future__ import annotations

import argparse
import logging
import math
import time
from collections.abc import Sequence

from ai_drone.flight.controller import DroneController, FlightSafetyError
from ai_drone.flight.follower import AutonomousFollower

FLIGHT_CONFIRMATION = "FLIGHT_TEST_READY"
FOLLOW_CONFIRMATION = "CAMERA_RIGID_AND_CALIBRATED"
logger = logging.getLogger(__name__)


class _SimulationController:
    """No-I/O target for the pure person-follow simulation."""

    battery_voltage: float | None = 16.0
    current_altitude: float | None = 0.0
    max_altitude: float = 0.8
    is_flying: bool = False
    is_armed: bool = False

    def update_telemetry(self) -> None:
        return

    def emergency_stop(self) -> None:
        return

    def send_velocity_body(
        self, vx: float, vy: float, vz: float, yaw_rate_deg: float = 0.0
    ) -> None:
        return


def _bounded(value: float, name: str, minimum: float, maximum: float) -> float:
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(
            f"{name} must be finite and between {minimum:g} and {maximum:g}"
        )
    return number


def _validate_common(args: argparse.Namespace) -> None:
    if isinstance(args.baud, bool) or not 1 <= args.baud <= 4_000_000:
        raise ValueError("--baud must be between 1 and 4000000")
    _bounded(args.max_alt, "--max-alt", 0.1, 10.0)
    if hasattr(args, "takeoff_alt"):
        _bounded(args.takeoff_alt, "--takeoff-alt", 0.15, args.max_alt)
    _bounded(args.duration, "--duration", 0.1, 3_600.0)


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


def _monitor(drone: DroneController, duration: float) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        drone.update_telemetry()
        if not drone.altitude_is_fresh() or not drone.heartbeat_is_fresh():
            raise FlightSafetyError("flight telemetry became stale")
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
    with _controller(args) as drone:
        drone.takeoff(args.takeoff_alt)
        _monitor(drone, args.duration)
        drone.land()
    return 0


def cmd_velocity_test(args: argparse.Namespace) -> int:
    _require_flight_confirmation(args)
    _bounded(args.vx, "--vx", -1.0, 1.0)
    _bounded(args.vy, "--vy", -1.0, 1.0)
    _bounded(args.vz, "--vz", -0.5, 0.5)
    _bounded(args.yaw_rate, "--yaw-rate", -45.0, 45.0)
    with _controller(args) as drone:
        drone.takeoff(args.takeoff_alt)
        deadline = time.monotonic() + args.duration
        try:
            while time.monotonic() < deadline:
                drone.update_telemetry()
                if not drone.altitude_is_fresh() or not drone.heartbeat_is_fresh():
                    raise FlightSafetyError("flight telemetry became stale")
                drone.send_velocity_body(args.vx, args.vy, args.vz, args.yaw_rate)
                time.sleep(0.2)
        finally:
            try:
                drone.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            except (OSError, RuntimeError):
                logger.warning("could not send final zero-velocity setpoint")
        drone.land()
    return 0


def cmd_follow(args: argparse.Namespace) -> int:
    follower = AutonomousFollower(
        _SimulationController(),
        target_dist_m=args.target_dist,
        max_vx=args.max_speed,
        max_yaw_rate_deg=args.max_yaw,
    )
    if args.simulate:
        follower.run_simulated_tracking(args.duration)
        return 0

    _require_flight_confirmation(args)
    if args.confirm_live_follow != FOLLOW_CONFIRMATION:
        raise ValueError(
            f"--confirm-live-follow must be exactly {FOLLOW_CONFIRMATION}; the current "
            "loosely cable-held camera does not satisfy this gate"
        )
    if args.focal_length_px is None:
        raise ValueError("live follow requires measured --focal-length-px")
    _bounded(args.confidence, "--confidence", 0.0, 1.0)
    _bounded(args.focal_length_px, "--focal-length-px", 1.0, 100_000.0)
    _bounded(args.person_height, "--person-height", 0.5, 2.5)
    with _controller(args) as drone:
        live_follower = AutonomousFollower(
            drone,
            target_dist_m=args.target_dist,
            max_vx=args.max_speed,
            max_yaw_rate_deg=args.max_yaw,
        )
        drone.takeoff(args.takeoff_alt)
        live_follower.run_live_tracking(
            confidence=args.confidence,
            max_duration_s=args.duration,
            focal_length_px=args.focal_length_px,
            person_height_m=args.person_height,
        )
        drone.land()
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

    follow = commands.add_parser(
        "follow", help="simulate or run experimental live person following"
    )
    _add_common_options(follow, include_takeoff=True)
    follow.add_argument("--duration", type=float, default=15.0)
    follow.add_argument("--target-dist", type=float, default=2.0)
    follow.add_argument("--max-speed", type=float, default=0.3)
    follow.add_argument("--max-yaw", type=float, default=20.0)
    follow.add_argument("--confidence", type=float, default=0.4)
    follow.add_argument("--focal-length-px", type=float)
    follow.add_argument("--person-height", type=float, default=1.7)
    follow.add_argument(
        "--simulate", "--sim-target", dest="simulate", action="store_true"
    )
    follow.add_argument("--confirm-flight")
    follow.add_argument("--confirm-live-follow")
    follow.set_defaults(handler=cmd_follow)
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
