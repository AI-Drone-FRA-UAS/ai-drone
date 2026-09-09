"""Inspect drone connections and prepare a disarmed drone for cable removal."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from ai_drone.link.targets import (
    preferred_pi_addresses,
    resolve_connection_target,
    resolve_deploy_target,
    ssh_base_command,
)
from ai_drone.mavlink.devices import STABLE_FLIGHT_CONTROLLER_DEVICE
from ai_drone.mavlink.ownership import SerialDeviceBusyError, require_available_serial
from ai_drone.mavlink.safety import heartbeat_is_armed, is_vehicle_message
from ai_drone.platform import is_raspberry_pi

WALK_UNIT = "ai-drone-walk.service"
REMOVALS = ("battery", "fc-usb", "pi-usb", "all")
_ACTIVE = {"active", "activating", "deactivating", "reloading"}

# Runs with existing sudo permission to inspect root-owned descriptors too.
# Never returns process arguments, which might contain credentials.
_OWNERS_SCRIPT = r"""
import json, os
from pathlib import Path
hardware, packages, errors = [], [], []
prefixes = ('/dev/ttyAMA', '/dev/ttyACM', '/dev/ttyUSB', '/dev/ttyS',
            '/dev/video', '/dev/media', '/dev/vchiq', '/dev/gpio', '/dev/dma_heap/')
locks = {'/var/lib/dpkg/lock', '/var/lib/dpkg/lock-frontend',
         '/var/lib/apt/lists/lock', '/var/cache/apt/archives/lock'}
for process in Path('/proc').iterdir():
    if not process.name.isdigit() or int(process.name) == os.getpid():
        continue
    try:
        name = (process / 'comm').read_text().strip()
        arguments = (process / 'cmdline').read_bytes().split(b'\0')
        package_job = name in ('apt', 'apt-get', 'dpkg', 'unattended-upgr') or any(
            arg.endswith(b'/pi-safe-upgrade.sh') for arg in arguments)
        walk = any(line.endswith('/' + 'ai-drone-walk.service')
                   for line in (process / 'cgroup').read_text().splitlines())
        for descriptor in (process / 'fd').iterdir():
            try:
                target = os.readlink(descriptor)
            except FileNotFoundError:
                continue
            if target in locks:
                package_job = True
            if target.startswith(prefixes):
                hardware.append({'pid': int(process.name), 'name': name,
                                 'device': target, 'walk': walk})
        if package_job:
            packages.append({'pid': int(process.name), 'name': name})
    except (FileNotFoundError, ProcessLookupError):
        continue
    except (PermissionError, OSError):
        errors.append(int(process.name))
print(json.dumps({'complete': not errors, 'hardware': hardware,
                  'packages': packages, 'unreadable_pids': errors}))
"""


def _run(
    command: list[str], *, timeout: float = 10
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command, capture_output=True, text=True, check=False, timeout=timeout
    )


def _remaining(deadline: float | None, maximum: float = 10) -> float:
    if deadline is None:
        return maximum
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise RuntimeError("FC confirmation expired; keep power connected")
    return min(remaining, maximum)


def _owners(*, deadline: float | None = None) -> dict[str, Any]:
    if not sys.platform.startswith("linux"):
        raise RuntimeError("hardware-owner inspection currently requires Linux")
    command = ["/usr/bin/python3", "-c", _OWNERS_SCRIPT]
    if os.geteuid() != 0:
        command = ["sudo", "-n", *command]
    result = _run(command, timeout=_remaining(deadline))
    if result.returncode:
        raise RuntimeError(
            "cannot inspect all hardware owners (sudo permission required)"
        )
    value = json.loads(result.stdout)
    if not isinstance(value, dict) or value.get("complete") is not True:
        raise RuntimeError("hardware-owner inspection was incomplete")
    return value


def _walk_state(*, deadline: float | None = None) -> dict[str, str]:
    result = _run(
        [
            "systemctl",
            "show",
            WALK_UNIT,
            "--property=LoadState",
            "--property=ActiveState",
            "--property=Result",
            "--property=InvocationID",
            "--property=KillSignal",
            "--property=KillMode",
        ],
        timeout=_remaining(deadline),
    )
    value = dict(
        line.split("=", 1) for line in result.stdout.splitlines() if "=" in line
    )
    if value.get("LoadState") == "not-found" and result.returncode in (0, 1):
        return {"LoadState": "not-found", "ActiveState": "inactive"}
    if result.returncode or value.get("LoadState") != "loaded":
        raise RuntimeError("cannot establish recorder service state")
    return value


def _pi_snapshot(*, deadline: float | None = None) -> dict[str, Any]:
    result = _owners(deadline=deadline)
    result["walk"] = _walk_state(deadline=deadline)
    audit = _run(["dpkg", "--audit"], timeout=_remaining(deadline))
    result["package_database_ok"] = audit.returncode == 0 and not audit.stdout.strip()
    return result


def _guard_idle(snapshot: dict[str, Any], *, allow_walk: bool = False) -> None:
    if snapshot.get("complete") is not True:
        raise RuntimeError("hardware-owner status is unknown")
    if snapshot.get("packages") or snapshot.get("package_database_ok") is not True:
        raise RuntimeError("package maintenance is active or dpkg is incomplete")
    owners = [
        owner for owner in snapshot["hardware"] if not (allow_walk and owner["walk"])
    ]
    if owners:
        raise RuntimeError(
            "hardware is busy; close other camera, serial or GPIO tools first"
        )
    if not allow_walk and snapshot["walk"].get("ActiveState") in _ACTIVE:
        raise RuntimeError("recorder has not finished")


def _quiesce() -> dict[str, Any]:
    snapshot = _pi_snapshot()
    _guard_idle(snapshot, allow_walk=True)
    state = snapshot["walk"]
    report: str | None = None
    if state.get("ActiveState") in _ACTIVE:
        if state.get("KillSignal") != "2" or state.get("KillMode") != "mixed":
            raise RuntimeError(
                "recorder lacks the expected graceful SIGINT stop settings"
            )
        invocation = state.get("InvocationID")
        if not invocation or not all(c in "0123456789abcdef" for c in invocation):
            raise RuntimeError("cannot identify the active recorder invocation")
        stopped = _run(["sudo", "-n", "systemctl", "stop", WALK_UNIT], timeout=190)
        if stopped.returncode:
            raise RuntimeError("recorder stop failed; power must remain connected")
        journal = _run(
            [
                "journalctl",
                f"_SYSTEMD_INVOCATION_ID={invocation}",
                "--no-pager",
                "-o",
                "cat",
            ]
        )
        reports = [
            line.removeprefix("Report: ")
            for line in journal.stdout.splitlines()
            if line.startswith("Report: ")
        ]
        if journal.returncode or not reports or not Path(reports[-1]).is_file():
            raise RuntimeError(
                "recorder stopped without a confirmed report; review its journal"
            )
        report = reports[-1]
    final = _pi_snapshot()
    _guard_idle(final)
    return {"quiesced": True, "report": report}


def _configure_receiver(fd: int) -> None:
    import termios

    settings = termios.tcgetattr(fd)
    settings[0] = 0
    settings[1] = 0
    settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    settings[3] = 0
    settings[4] = settings[5] = termios.B115200
    settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    termios.tcflush(fd, termios.TCIFLUSH)


def _receive_fc(fd: int, device: str, *, duration: float = 4) -> dict[str, Any]:
    import select

    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    parser = mavlink.MAVLink(None)
    parser.robust_parsing = True
    end = time.monotonic() + duration
    last_heartbeat: float | None = None
    heartbeats = 0
    armed = False
    result: dict[str, Any] = {"device": device, "status": "unavailable"}
    while (remaining := end - time.monotonic()) > 0:
        if not select.select([fd], [], [], min(remaining, 0.2))[0]:
            continue
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not data:
            break
        for message in parser.parse_buffer(data) or []:
            if not is_vehicle_message(message, system_id=1, component_id=1):
                continue
            if (
                isinstance(message, mavlink.MAVLink_heartbeat_message)
                and message.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                and message.type == mavlink.MAV_TYPE_QUADROTOR
            ):
                armed = armed or heartbeat_is_armed(message)
                last_heartbeat = time.monotonic()
                heartbeats += 1
            elif isinstance(message, mavlink.MAVLink_sys_status_message):
                result["battery_voltage_v"] = (
                    message.voltage_battery / 1000
                    if message.voltage_battery != 65535
                    else None
                )
                result["battery_remaining_percent"] = (
                    message.battery_remaining
                    if message.battery_remaining >= 0
                    else None
                )
            elif isinstance(message, mavlink.MAVLink_power_status_message):
                result.update(
                    board_voltage_v=message.Vcc / 1000,
                    servo_voltage_v=message.Vservo / 1000,
                    power_flags=message.flags,
                )
    age = None if last_heartbeat is None else time.monotonic() - last_heartbeat
    result.update(heartbeats=heartbeats, heartbeat_age_s=age)
    if armed:
        result["status"] = "armed"
    elif heartbeats >= 2 and age is not None and age <= 2:
        result["status"] = "disarmed"
    return result


def _probe_fc(device: Path) -> dict[str, Any]:
    if not device.exists():
        return {"status": "absent", "device": str(device), "detected": False}
    try:
        snapshot = _owners()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        # Detection does not authorize opening a port whose owners are unknown.
        # A same-user serial client must still block the Pi fallback when the
        # complete inspection needs permissions the laptop does not have.
        unavailable = {
            "status": "unavailable",
            "device": str(device),
            "detected": True,
            "reason": str(error),
        }
        try:
            require_available_serial(str(device))
        except SerialDeviceBusyError as busy:
            unavailable.update(status="busy", serial_owner_check=str(busy))
        except (
            OSError,
            ValueError,
            RuntimeError,
            subprocess.SubprocessError,
        ) as unknown:
            unavailable.update(status="unknown", serial_owner_check=str(unknown))
        else:
            unavailable["serial_owner_check"] = "no visible owner"
        return unavailable
    actual = str(device.resolve())
    if any(owner["device"] == actual for owner in snapshot["hardware"]):
        return {"status": "busy", "device": str(device), "detected": True}
    import fcntl

    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        _configure_receiver(fd)
        return {**_receive_fc(fd, str(device)), "detected": True}
    finally:
        os.close(fd)


def _remote_action(action: str) -> dict[str, Any]:
    if not is_raspberry_pi():
        raise RuntimeError("remote power actions require the Raspberry Pi")
    if action == "snapshot":
        return _pi_snapshot()
    if action == "quiesce":
        return _quiesce()
    if action == "fc":
        snapshot = _pi_snapshot()
        _guard_idle(snapshot)
        return _probe_fc(Path("/dev/serial0"))
    if action not in {"idle", "shutdown"}:
        raise ValueError("unknown Pi action")
    _guard_idle(_pi_snapshot())
    # An earlier laptop USB observation cannot authorize a later remote action.
    # Check the idle Pi UART here, then include all final checks and the action
    # within the selected heartbeat's remaining two-second freshness window.
    fc = _probe_fc(Path("/dev/serial0"))
    verified = time.monotonic()
    _require_disarmed(fc)
    deadline = verified + 2 - fc["heartbeat_age_s"]
    _guard_idle(_pi_snapshot(deadline=deadline))
    remaining = _remaining(deadline)
    if action == "idle":
        return {"idle": True}
    result = _run(
        ["sudo", "-n", "systemctl", "poweroff", "--no-block"], timeout=remaining
    )
    if result.returncode:
        raise RuntimeError("Pi rejected the shutdown request")
    return {"shutdown_requested": True, "halt_confirmed": False}


def _ssh_commands(action: str) -> list[list[str]]:
    connection = resolve_connection_target()
    target = resolve_deploy_target(ping=lambda _host: False)
    hosts = (
        [target.ssh_target]
        if os.environ.get("PI_HOST")
        else [
            f"{connection.pi_user}@{host}"
            for host in preferred_pi_addresses(connection)
        ]
    )
    remote = (
        f"cd {shlex.quote(target.project_dir)} && "
        "uv run --no-sync python -m ai_drone.cli.power --remote " + action
    )
    return [
        [
            *ssh_base_command(target.ssh_config),
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            "-o",
            "StrictHostKeyChecking=yes",
            host,
            remote,
        ]
        for host in hosts
    ]


def _remote(command: list[str], action: str) -> dict[str, Any]:
    # Retry only read-only discovery. A failed mutation may already have run.
    actual = [*command[:-1], command[-1].rsplit(" ", 1)[0] + " " + action]
    result = _run(actual, timeout=210 if action == "quiesce" else 20)
    if result.returncode:
        raise RuntimeError(
            f"Pi {action} failed or its result is unknown; keep power connected"
        )
    data = json.loads(result.stdout)
    if not isinstance(data, dict):
        raise RuntimeError("invalid Pi response")
    if data.get("error"):
        raise RuntimeError(str(data["error"]))
    return data


def _connect_pi() -> tuple[list[str], dict[str, Any]]:
    errors = []
    for command in _ssh_commands("snapshot"):
        try:
            return command, _remote(command, "snapshot")
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            errors.append(str(error))
    raise RuntimeError("Pi status unavailable: " + "; ".join(errors))


def _fresh_fc(command: list[str], *, fallback: bool = True) -> dict[str, Any]:
    fc = _probe_fc(STABLE_FLIGHT_CONTROLLER_DEVICE)
    if fallback and fc["status"] in {"absent", "unavailable"}:
        # Keep USB detection/inspection failures visible even if the Pi fallback
        # also fails, and distinguish connection evidence from FC telemetry.
        print("FC USB: " + json.dumps(fc), flush=True)
        return {
            **_remote(command, "fc"),
            "telemetry_source": "Pi UART",
            "usb_connection": fc,
        }
    return {**fc, "telemetry_source": "FC USB"}


def _require_disarmed(fc: dict[str, Any]) -> None:
    age = fc.get("heartbeat_age_s")
    if (
        fc.get("status") != "disarmed"
        or not isinstance(age, int | float)
        or not 0 <= age <= 2
    ):
        raise RuntimeError(
            "fresh selected-FC disarmed state is not established; do not disconnect"
        )


def _print_preparations(blocked: str | None = None) -> None:
    print(
        "Pi USB supply and remaining power sources: unknown; USB networking is not power proof."
    )
    print(
        "These prepare commands recheck safety; status alone does not authorize removal."
    )
    for removal in REMOVALS:
        state = (
            f"blocked ({blocked})"
            if blocked
            else "needs Pi shutdown; prepare rechecks the idle Pi UART first"
        )
        print(f"{removal}: {state}")
        print(f"  uv run --locked drone-power prepare {removal}")


def _status() -> int:
    command = None
    blocked = []
    try:
        command, snapshot = _connect_pi()
        print("Pi SSH: reachable")
        print("Recorder: " + snapshot["walk"].get("ActiveState", "unknown"))
        print(
            f"Package jobs: {len(snapshot['packages'])}; hardware owners: {len(snapshot['hardware'])}"
        )
        print(
            "Pi package database: "
            + ("consistent" if snapshot["package_database_ok"] else "blocked")
        )
        try:
            _guard_idle(snapshot, allow_walk=True)
        except RuntimeError as error:
            blocked.append(str(error))
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"Pi: unknown ({error})")
        blocked.append(f"Pi status unavailable: {error}")
    try:
        fc = (
            _probe_fc(STABLE_FLIGHT_CONTROLLER_DEVICE)
            if command is None
            else _fresh_fc(command)
        )
        print("FC: " + json.dumps(fc))
        _require_disarmed(fc)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"FC: unknown ({error})")
        blocked.append(str(error))
    _print_preparations("; ".join(blocked) or None)
    return 1 if blocked else 0


def _prepare(args: argparse.Namespace) -> int:
    if args.removal == "all" and args.pi_power_independent:
        raise ValueError(
            "an independent supply cannot be attested when removing all power"
        )
    if args.dry_run:
        print(
            "Would inspect package jobs and hardware owners, finish the known recorder,"
        )
        print(
            "require fresh disarmed FC telemetry, then recheck idle Pi UART telemetry."
        )
        print(
            "Pi stays powered by your attested independent supply."
            if args.pi_power_independent
            else "Would request Pi shutdown, then require physical halt confirmation."
        )
        print(shlex.join(_ssh_commands("snapshot")[0]))
        return 0
    command, snapshot = _connect_pi()
    _guard_idle(snapshot, allow_walk=True)
    print(
        "Checking idle state and finishing the known recorder, if active...", flush=True
    )
    quiet = _remote(command, "quiesce")
    if quiet.get("report"):
        print(f"Finalized report: {quiet['report']}")
    fc = _fresh_fc(command)
    verified = time.monotonic()
    _require_disarmed(fc)
    print("FC: " + json.dumps(fc))
    if time.monotonic() - verified + fc["heartbeat_age_s"] > 2:
        raise RuntimeError("FC confirmation expired; keep power connected")
    final = _remote(command, "idle" if args.pi_power_independent else "shutdown")
    if args.pi_power_independent:
        if final.get("idle") is not True:
            raise RuntimeError("Pi idle state was not confirmed")
        print(
            f"Prepared for {args.removal}; keep your attested independent Pi supply connected."
        )
    else:
        if final.get("shutdown_requested") is not True:
            raise RuntimeError("Pi shutdown was not acknowledged")
        print("Pi shutdown requested; clean physical halt is NOT yet confirmed.")
        print(
            "Keep power connected until the Pi has physically finished shutting down."
        )
        print(
            f"Then remove {args.removal}. SSH loss alone does not establish a clean halt."
        )
    return 0


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--remote",
        choices=("snapshot", "quiesce", "fc", "idle", "shutdown"),
        help=argparse.SUPPRESS,
    )
    subparsers = parser.add_subparsers(dest="action")
    subparsers.add_parser("status")
    prepare = subparsers.add_parser("prepare")
    prepare.add_argument("removal", choices=REMOVALS)
    prepare.add_argument(
        "--pi-power-independent",
        action="store_true",
        help="confirm another Pi supply remains after this specific removal",
    )
    prepare.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(arguments)
    try:
        if args.remote:
            print(json.dumps(_remote_action(args.remote)))
            return 0
        if args.action is None:
            parser.error("choose status or prepare")
        if args.action == "status":
            return _status()
        return _prepare(args)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        if args.remote:
            print(json.dumps({"error": str(error)}))
        else:
            print(f"Blocked: {error}", file=sys.stderr)
            _print_preparations(str(error))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
