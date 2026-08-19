"""Retired ``drone-control follow`` adapter, kept for reference.

Extracted verbatim from ``ai_drone/cli/control.py`` when person-following
was retired.  It depends on ``attic/ai_drone/flight/follower.py`` and on
helpers that remain in the live ``control.py`` (``_controller``,
``_bounded``, ``_require_flight_confirmation``, ``_add_common_options``).
This file is reference material: it is not importable as written.
"""

FOLLOW_CONFIRMATION = "CAMERA_RIGID_AND_CALIBRATED"


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


# Subparser wiring removed from _parser():

#     follow = commands.add_parser(
#         "follow", help="simulate or run experimental live person following"
#     )
#     _add_common_options(follow, include_takeoff=True)
#     follow.add_argument("--duration", type=float, default=15.0)
#     follow.add_argument("--target-dist", type=float, default=2.0)
#     follow.add_argument("--max-speed", type=float, default=0.3)
#     follow.add_argument("--max-yaw", type=float, default=20.0)
#     follow.add_argument("--confidence", type=float, default=0.4)
#     follow.add_argument("--focal-length-px", type=float)
#     follow.add_argument("--person-height", type=float, default=1.7)
#     follow.add_argument(
#         "--simulate", "--sim-target", dest="simulate", action="store_true"
#     )
#     follow.add_argument("--confirm-flight")
#     follow.add_argument("--confirm-live-follow")
#     follow.set_defaults(handler=cmd_follow)
