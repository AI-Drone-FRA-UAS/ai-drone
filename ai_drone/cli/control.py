"""Guarded takeoff, hover, and landing command."""

from __future__ import annotations

import argparse
import logging
import os
import shutil
import signal
import subprocess
import threading
import time
from collections.abc import Sequence
from contextlib import contextmanager
from pathlib import Path
from types import FrameType
from typing import Any

from ai_drone.flight.controller import (
    DroneController,
    FlightSafetyError,
    HumanControlTaken,
)
from ai_drone.flight.dataflash import latest_dataflash_log
from ai_drone.flight.guards import FlightGuardError, check_safety_guardrails
from ai_drone.flight.recording import FlightRecorder
from ai_drone.mavlink.remote import runtime_request
from ai_drone.mavlink.shared import SharedMavlink
from ai_drone.operator import OperatorMonitor
from ai_drone.platform import is_raspberry_pi
from ai_drone.runtime import operator_has_control_link, read_runtime_status
from ai_drone.settings import load_settings
from ai_drone.validation import finite_in_range

FLIGHT_CONFIRMATION = "FLIGHT_TEST_READY"
MAX_AUTONOMOUS_CEILING_M = 0.8
MAX_AUTONOMOUS_TAKEOFF_M = 0.6
logger = logging.getLogger(__name__)


def _validate_common(args: argparse.Namespace) -> None:
    if isinstance(args.baud, bool) or not 1 <= args.baud <= 4_000_000:
        raise ValueError("--baud must be between 1 and 4000000")
    limits = (
        (args.max_alt, "--max-alt", 0.1, MAX_AUTONOMOUS_CEILING_M),
        (
            args.takeoff_alt,
            "--takeoff-alt",
            0.15,
            min(args.max_alt, MAX_AUTONOMOUS_TAKEOFF_M),
        ),
        (args.duration, "--duration", 0.1, 3_600.0),
        (args.min_battery, "--min-battery", 0.0, 60.0),
        (args.navigation_timeout, "--navigation-timeout", 1.0, 60.0),
    )
    for value, name, minimum, maximum in limits:
        finite_in_range(value, name, minimum=minimum, maximum=maximum)


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
def _operator_link(args):
    settings = load_settings()
    path = Path(args.runtime_status or settings.runtime.status)
    if path.is_file():

        def status():
            return read_runtime_status(path)

        if not operator_has_control_link(status()):
            raise FlightSafetyError(
                "independent operator heartbeat or Wi-Fi link is unavailable"
            )
        yield (
            lambda: operator_has_control_link(status()),
            lambda: bool(status().get("human_requested")),
        )
        return
    if is_raspberry_pi():
        raise FlightSafetyError(
            "start the shared vehicle runtime before autonomous control"
        )
    monitor = OperatorMonitor(settings.operator)
    if not monitor.configured:
        raise FlightSafetyError(
            "configure operator endpoints and start drone operator before autonomous control"
        )
    monitor.start()
    try:
        deadline = time.monotonic() + settings.operator.timeout
        while not monitor.alive() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not monitor.alive():
            raise FlightSafetyError("independent operator heartbeat is unavailable")
        yield monitor.alive, lambda: False
    finally:
        monitor.close()


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
    try:
        for handled_signal in handled:
            previous[handled_signal] = signal.getsignal(handled_signal)
            signal.signal(handled_signal, request_stop)
        if hasattr(signal, "SIGHUP"):
            previous[signal.SIGHUP] = signal.getsignal(signal.SIGHUP)
            signal.signal(signal.SIGHUP, signal.SIG_IGN)
        yield requested
    finally:
        for handled_signal, old_handler in previous.items():
            signal.signal(handled_signal, old_handler)


@contextmanager
def _shared_controller(args: argparse.Namespace):
    hub = None
    try:
        with _controller(args) as drone:
            hub = SharedMavlink(drone._connection())
            drone.connection = hub.subscribe("flight-control")
            yield drone, hub
    finally:
        if hub is not None:
            hub.close()


@contextmanager
def _flight_session(args: argparse.Namespace):
    with _shared_controller(args) as (drone, hub):
        metadata = {
            key: value
            for key, value in vars(args).items()
            if key not in {"confirm_flight", "handler"}
            and isinstance(value, str | int | float | bool | type(None))
        }
        record = FlightRecorder(hub.subscribe("flight-recording"), metadata)
        drone.request_telemetry_streams()
        try:
            yield drone, record
        except BaseException as error:
            record.finish(error)
            if drone.is_flying and drone.control_owner != "human":
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
        if drone.control_owner == "human":
            raise HumanControlTaken("control was handed to the radio pilot")
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
        _operator_link(args) as (operator_alive, human_requested),
        _termination_event() as stop_requested,
        _flight_session(args) as (drone, record),
    ):
        drone.stop_requested = stop_requested.is_set
        drone.operator_alive = operator_alive
        drone.human_takeover_requested = human_requested
        try:
            record.event("control_owner", owner="autonomous")
            record.event("guided_nogps_takeoff_started", target_alt_m=args.takeoff_alt)
            drone.takeoff(args.takeoff_alt)
            record.event("loiter_acquisition_started")
            drone.enter_loiter(timeout=args.navigation_timeout)
            record.event("loiter_started", ekf_flags=drone.ekf_flags)
            _monitor(drone, args.duration, args.min_battery)
            record.event("landing_started")
            drone.land()
            record.event("landed")
        except HumanControlTaken:
            record.event(
                "control_owner",
                owner="human",
                mode=drone.flight_mode,
                rc_channels=drone.rc_channel_count,
            )
            drone.supervise_human()
            record.event("pilot_disarmed")
    return 0


def _launch_hover(args: argparse.Namespace) -> int:
    unit = "ai-drone-control.service"
    state = subprocess.run(
        ["systemctl", "show", unit, "--property=LoadState"],
        capture_output=True,
        text=True,
        check=False,
        timeout=5,
    )
    if state.returncode not in (0, 1) or state.stdout.strip() != "LoadState=not-found":
        raise RuntimeError("a control service exists or its state is unknown")
    uid, gid = os.getuid(), os.getgid()
    if uid == 0:
        raise ValueError("start flight control as the normal Pi user")
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError("uv is required to launch flight control")
    forwarded = []
    for name, value in vars(args).items():
        if name in {"handler", "command", "worker", "foreground"} or value is None:
            continue
        forwarded.extend([f"--{name.replace('_', '-')}", str(value)])
    command = [
        "sudo",
        "-n",
        "systemd-run",
        f"--unit={unit}",
        "--collect",
        "--service-type=exec",
        "--expand-environment=no",
        f"--uid={uid}",
        f"--gid={gid}",
        f"--working-directory={Path(__file__).resolve().parents[2]}",
        f"--setenv=PATH={Path(uv).parent}:{os.defpath}",
        "--property=KillSignal=SIGTERM",
        "--property=KillMode=mixed",
        "--property=TimeoutStopSec=infinity",
        *(
            [
                f"--setenv=AI_DRONE_CONFIG={Path(os.environ['AI_DRONE_CONFIG']).absolute()}"
            ]
            if os.environ.get("AI_DRONE_CONFIG")
            else []
        ),
        uv,
        "run",
        "--no-sync",
        "python",
        "-m",
        "ai_drone.cli.control",
        "hover",
        "--worker",
        *forwarded,
    ]
    return subprocess.run(command, check=False).returncode


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
    hover.add_argument("--device", help="MAVLink serial path or network endpoint")
    hover.add_argument("--baud", type=int, default=115200)
    hover.add_argument("--max-alt", type=float, default=MAX_AUTONOMOUS_CEILING_M)
    hover.add_argument("--takeoff-alt", type=float, default=0.5)
    hover.add_argument(
        "--min-battery",
        type=float,
        default=14.4,
        help="abort and stop below this pack voltage",
    )
    hover.add_argument("--duration", type=float, default=5.0)
    hover.add_argument(
        "--navigation-timeout",
        type=float,
        default=20.0,
        help="maximum time to establish stable optical-flow relative position",
    )
    hover.add_argument("--confirm-flight")
    hover.add_argument("--runtime-status", help="shared runtime status file")
    hover.add_argument(
        "--foreground",
        action="store_true",
        help="run here; Pi defaults to a detached service",
    )
    hover.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    hover.set_defaults(handler=cmd_hover)

    handoff = commands.add_parser(
        "handoff",
        help="request explicit handoff to a radio pilot in a confirmed pilot mode",
    )
    handoff.set_defaults(
        handler=lambda _args: (
            runtime_request(load_settings().runtime.socket, {"human": True}) and 0
        )
    )

    return parser


def main(arguments: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(arguments)
    try:
        if args.command in {"hover", "takeoff"}:
            _validate_common(args)
            _require_flight_confirmation(args)
            if is_raspberry_pi() and not args.worker and not args.foreground:
                return _launch_hover(args)
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
