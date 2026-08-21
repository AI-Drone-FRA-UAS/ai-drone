"""Single command surface for passive status and guarded flight tests."""

from __future__ import annotations

import argparse
import logging
import time
from collections.abc import Sequence
from contextlib import contextmanager

from ai_drone.abort_key import AbortKey, AbortOnHangUp
from ai_drone.flight.controller import DroneController, FlightSafetyError
from ai_drone.flight.dataflash import latest_dataflash_log
from ai_drone.flight.guards import FlightGuardError, check_safety_guardrails
from ai_drone.flight.recording import FlightRecorder
from ai_drone.mavlink.preflight import (
    assess,
    describe_ekf_flags,
    gather,
    guided_takeoff_blockers,
)
from ai_drone.validation import finite_in_range

FLIGHT_CONFIRMATION = "FLIGHT_TEST_READY"
# A separate phrase from FLIGHT_CONFIRMATION on purpose: this one asserts a
# physical fact about the aircraft that nothing in software can verify.
ARM_TEST_CONFIRMATION = "PROPELLERS_REMOVED"
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


def _require_arm_test_confirmation(args: argparse.Namespace) -> None:
    if args.confirm_props_off != ARM_TEST_CONFIRMATION:
        raise ValueError(
            f"--confirm-props-off must be exactly {ARM_TEST_CONFIRMATION}; this "
            "spins the motors, and no software check can see whether the "
            "propellers are still fitted"
        )


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
    with (
        AbortOnHangUp() as hang_up,
        _controller(args) as drone,
        AbortKey() as abort_key,
    ):
        # Printed rather than assumed.  A watcher that quietly did nothing --
        # no terminal, stdin a pipe -- would be worse than none at all,
        # because the operator would be counting on it.
        print(abort_key.describe())
        print(hang_up.describe())
        drone.abort_requested = abort_key.requested
        metadata = {
            key: value
            for key, value in vars(args).items()
            if key not in {"confirm_flight", "handler"}
            and isinstance(value, str | int | float | bool | type(None))
        }
        metadata["abort_key"] = abort_key.reason
        metadata["hang_up_abort"] = ",".join(hang_up.installed) or "none"
        record = FlightRecorder(drone._connection(), metadata)
        try:
            yield drone, record
        except BaseException as error:
            record.finish(error)
            if drone.is_flying:
                # A guard already requested LAND, but a request is not a
                # landing.  Stay connected and keep asking until the vehicle
                # says it is disarmed; letting go here is what leaves an
                # aircraft in the air with nobody talking to it.
                landed = drone.ensure_landed()
                record.event("emergency_landing", confirmed=landed)
                if not landed:
                    logger.error(
                        "VEHICLE DID NOT CONFIRM DISARM. It may still be flying. "
                        "Do not approach it; cut power only from a safe distance."
                    )
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


def cmd_preflight(args: argparse.Namespace) -> int:
    """Report every reason a guarded takeoff would refuse to start.

    Read-only: this sends stream and parameter requests and nothing else.
    """

    with _controller(args) as drone:
        snapshot = gather(drone._connection(), timeout=args.duration)

    checks = assess(snapshot)
    print(f"vehicle: {snapshot.mode or 'unknown mode'}, armed={snapshot.armed}")
    if snapshot.ekf_flags is not None:
        print(f"EKF: {describe_ekf_flags(snapshot.ekf_flags)}")
    print()
    for check in checks:
        print(f"[{check.marker:7s}] {check.name}: {check.detail}")
    for line in snapshot.statustexts:
        print(f"[vehicle ] {line}")

    blockers = guided_takeoff_blockers(checks)
    print()
    if not blockers:
        print(
            "GUIDED takeoff: no blocker found. Re-read the checks above before flying."
        )
        return 0
    print("GUIDED takeoff: BLOCKED by " + ", ".join(check.name for check in blockers))
    return 2


def cmd_arm_test(args: argparse.Namespace) -> int:
    """Arm in GUIDED, hold at idle, then disarm. Never commands a takeoff.

    This is the rung between ``preflight`` and ``hover``: it exercises the exact
    arming path a flight uses -- check verification, GUIDED mode, fresh
    disarmed heartbeat, arm confirmation -- and stops there.  A vehicle that
    cannot arm in GUIDED has no business being asked to take off, and finding
    that out with the propellers off costs nothing.
    """

    _require_arm_test_confirmation(args)
    with _flight_session(args) as (drone, record):
        record.event("arm_test_started", mode=args.mode)
        drone.arm(mode=args.mode)
        record.event("armed", ekf_flags=drone.ekf_flags)
        print(f"armed in {drone.flight_mode}; holding for {args.duration:.1f} s")
        _watch_navigation(drone, args.duration, args.min_battery, record)
        record.event("disarming", ekf_flags=drone.ekf_flags)
        drone.disarm()
        record.event("disarmed")
        print("disarmed")
    return 0


def _watch_navigation(
    drone: DroneController,
    duration: float,
    min_battery_v: float,
    record: FlightRecorder,
) -> None:
    """Hold under the guards, reporting what the EKF makes of the world.

    ``pred_horiz_pos_rel`` is the flag ArduPilot consults when a disarmed
    vehicle asks to enter a mode that needs a position, so watching it appear
    (or fail to) is the whole point of a bench arm test on this airframe.
    """

    deadline = time.monotonic() + duration
    next_report = 0.0
    while time.monotonic() < deadline:
        drone.update_telemetry()
        try:
            check_safety_guardrails(drone, min_battery_v)
        except FlightGuardError as error:
            raise FlightSafetyError(str(error)) from error
        now = time.monotonic()
        if now >= next_report:
            next_report = now + 0.5
            flags = drone.ekf_flags
            altitude = (
                f"{drone.current_altitude:.2f} m"
                if drone.current_altitude is not None
                else "n/a"
            )
            print(
                f"  alt={altitude} ekf={describe_ekf_flags(flags) if flags is not None else 'n/a'}"
            )
            record.event("navigation_sample", ekf_flags=flags)
        time.sleep(0.05)


def cmd_nogps_takeoff(args: argparse.Namespace) -> int:
    """Take off, hold, and land without ever needing a position estimate.

    The aircraft this exists for has no GPS and navigates from optical flow,
    which EKF3 only starts fusing once it has detected a takeoff.  That makes a
    GUIDED takeoff from the floor circular, so this climbs in GUIDED_NOGPS on
    the downward rangefinder instead.  ArduPilot keeps attitude and altitude
    control throughout; this only ever asks for a bounded climb rate.
    """

    _require_flight_confirmation(args)
    with _flight_session(args) as (drone, record):
        record.event("nogps_takeoff_started", target_alt_m=args.takeoff_alt)
        print(f"climbing to {args.takeoff_alt:.2f} m in GUIDED_NOGPS")
        drone.takeoff_without_position(args.takeoff_alt, climb=args.climb)
        record.event("hover_started", ekf_flags=drone.ekf_flags)
        print(f"holding for {args.duration:.1f} s")

        def sample(controller: DroneController) -> None:
            try:
                check_safety_guardrails(controller, args.min_battery)
            except FlightGuardError as error:
                raise FlightSafetyError(str(error)) from error

        drone.hold_altitude(args.duration, on_sample=sample)
        record.event("landing_started", ekf_flags=drone.ekf_flags)
        print("landing")
        drone.land()
        record.event("landed")
        print("landed")
    return 0


def cmd_stabilize_takeoff(args: argparse.Namespace) -> int:
    """Climb in STABILIZE, hold in ALT_HOLD, then land in LAND.

    STABILIZE is the one mode in this CLI where the autopilot contributes
    nothing to holding the aircraft up: the throttle stick is motor thrust,
    and the climb ends only when this loop ends it.  It exists because
    ALT_HOLD has never lifted this airframe cleanly off the floor and GUIDED
    cannot arm without a position estimate it only gets after taking off.

    The three phases are three different meanings of the same stick, so the
    controller verifies the vehicle's reported mode before every value it
    sends, and hands altitude control back to ArduPilot as soon as there is
    altitude to hold.
    """

    _require_flight_confirmation(args)
    with _flight_session(args) as (drone, record):
        record.event(
            "stabilize_takeoff_started",
            target_alt_m=args.takeoff_alt,
            throttle_above_hover=args.climb,
        )
        print(f"climbing to {args.takeoff_alt:.2f} m in STABILIZE")
        drone.climb_in_stabilize(args.takeoff_alt, climb=args.climb)
        record.event("handover_started", ekf_flags=drone.ekf_flags)
        print("handing altitude control to ALT_HOLD")
        drone.handover_to_alt_hold()
        record.event("hold_started", ekf_flags=drone.ekf_flags)
        print(f"holding in ALT_HOLD for {args.duration:.1f} s")

        def sample(controller: DroneController) -> None:
            try:
                check_safety_guardrails(controller, args.min_battery)
            except FlightGuardError as error:
                raise FlightSafetyError(str(error)) from error

        drone.hold_in_alt_hold(args.duration, on_sample=sample)
        record.event("landing_started", ekf_flags=drone.ekf_flags)
        print("landing")
        drone.land()
        record.event("landed")
        print("landed")
    return 0


def _live_line(drone: DroneController, climb: float | None) -> str:
    """One line of everything the guards are about to act on."""

    def metres(value: float | None) -> str:
        return f"{value:+6.2f}" if value is not None else "   n/a"

    believed = measured = None
    if drone.local_position_altitude is not None and drone._ekf_reference is not None:
        believed = drone.local_position_altitude - drone._ekf_reference
    if drone.current_altitude is not None and drone._ground_reference is not None:
        measured = drone.current_altitude - drone._ground_reference
    disagreement = (
        f"{believed - measured:+5.2f}"
        if believed is not None and measured is not None
        else "  n/a"
    )
    return (
        f"  {drone.flight_mode or '?':<9} rng={metres(drone.current_altitude)} m "
        f"climb={metres(climb)} m/s  ekf={metres(drone.local_position_altitude)} m "
        f"ekfvz={metres(drone.local_position_climb)} m/s  "
        f"gap={disagreement} m  batt={metres(drone.battery_voltage)} V"
    )


def _reporter(period: float = 0.25):
    """Print the live line, rate limited so the console stays readable."""

    state = {"next": 0.0}

    def report(drone: DroneController, climb: float | None = None) -> None:
        now = time.monotonic()
        if now < state["next"]:
            return
        state["next"] = now + period
        print(_live_line(drone, climb), flush=True)

    return report


def cmd_alt_hold_takeoff(args: argparse.Namespace) -> int:
    """Climb, hold and land entirely in modes ArduPilot controls the altitude in.

    The one route up this aircraft can actually fly. GUIDED is refused by the
    vehicle itself -- ``PreArm: Need Position Estimate``, because EKF3 does not
    start optical-flow navigation until it detects a takeoff -- and STABILIZE
    puts raw motor thrust on the stick through a mapping that two flights have
    failed to pin down. ALT_HOLD needs no position estimate and reads the stick
    as a climb rate bounded by ``PILOT_SPEED_UP``, so ArduPilot flies the climb
    and there is no thrust curve left to guess at.

    It depends instead on the vehicle's vertical estimate, which is why it
    would have been a bad idea this morning and is a reasonable one now.
    """

    _require_flight_confirmation(args)
    with _flight_session(args) as (drone, record):
        record.event(
            "alt_hold_takeoff_started",
            target_alt_m=args.takeoff_alt,
            climb_rate_fraction=args.climb,
        )
        report = _reporter()
        print(f"climbing to {args.takeoff_alt:.2f} m in ALT_HOLD")
        print(
            "  mode      rangefinder  measured climb   EKF height   EKF rate   "
            "gap between them   battery"
        )

        def climbing(controller: DroneController, climb: float | None) -> None:
            report(controller, climb)
            record.event(
                "climb_sample",
                rangefinder_m=controller.current_altitude,
                measured_climb_ms=climb,
                ekf_altitude_m=controller.local_position_altitude,
                ekf_climb_ms=controller.local_position_climb,
            )

        drone.climb_in_alt_hold(args.takeoff_alt, climb=args.climb, on_sample=climbing)
        record.event("hold_started", ekf_flags=drone.ekf_flags)
        print(f"holding in ALT_HOLD for {args.duration:.1f} s")

        def sample(controller: DroneController) -> None:
            report(controller)
            try:
                check_safety_guardrails(controller, args.min_battery)
            except FlightGuardError as error:
                raise FlightSafetyError(str(error)) from error

        drone.hold_in_alt_hold(args.duration, on_sample=sample)
        record.event("landing_started", ekf_flags=drone.ekf_flags)
        print("landing")
        drone.land()
        record.event("landed")
        print("landed")
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

    preflight = commands.add_parser(
        "preflight",
        help="read-only: report what would block a guarded takeoff",
    )
    _add_common_options(preflight, include_takeoff=False)
    preflight.add_argument("--duration", type=float, default=12.0)
    preflight.set_defaults(handler=cmd_preflight)

    arm_test = commands.add_parser(
        "arm-test",
        help="propellers off: arm in GUIDED, hold at idle, disarm. No takeoff.",
    )
    _add_common_options(arm_test, include_takeoff=True)
    arm_test.add_argument("--duration", type=float, default=3.0)
    arm_test.add_argument(
        "--mode",
        default="GUIDED",
        help="mode to arm in; ALT_HOLD needs no position estimate",
    )
    arm_test.add_argument("--confirm-props-off")
    arm_test.set_defaults(handler=cmd_arm_test)

    nogps = commands.add_parser(
        "nogps-takeoff",
        help="climb, hold and land without a position estimate (GUIDED_NOGPS)",
    )
    _add_common_options(nogps, include_takeoff=True)
    nogps.add_argument("--duration", type=float, default=3.0)
    nogps.add_argument(
        "--climb",
        type=float,
        default=0.5,
        help="normalized climb rate, 1.0 being PILOT_SPEED_UP",
    )
    nogps.add_argument("--confirm-flight")
    nogps.set_defaults(handler=cmd_nogps_takeoff)

    stabilize = commands.add_parser(
        "stabilize-takeoff",
        help="climb in STABILIZE, hold in ALT_HOLD, then land",
    )
    _add_common_options(stabilize, include_takeoff=True)
    stabilize.add_argument("--duration", type=float, default=3.0)
    stabilize.add_argument(
        "--climb",
        type=float,
        default=0.06,
        help="throttle above the vehicle's learned hover, 0.0-0.10",
    )
    stabilize.add_argument("--confirm-flight")
    stabilize.set_defaults(handler=cmd_stabilize_takeoff)

    alt_hold = commands.add_parser(
        "alt-hold-takeoff",
        help="climb, hold and land in ALT_HOLD; ArduPilot flies the climb",
    )
    _add_common_options(alt_hold, include_takeoff=True)
    alt_hold.add_argument("--duration", type=float, default=3.0)
    alt_hold.add_argument(
        "--climb",
        type=float,
        default=0.5,
        help="climb rate as a fraction of PILOT_SPEED_UP, 0.05-1.0",
    )
    alt_hold.add_argument("--confirm-flight")
    alt_hold.set_defaults(handler=cmd_alt_hold_takeoff)

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
