"""Inspect drone connections and prepare a disarmed drone for cable removal."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from ai_drone.link.targets import (
    preferred_pi_addresses,
    remote_uv_command,
    resolve_connection_target,
    resolve_deploy_target,
    ssh_base_command,
)
from ai_drone.mavlink.devices import STABLE_FLIGHT_CONTROLLER_DEVICE
from ai_drone.mavlink.ownership import SerialDeviceBusyError, require_available_serial
from ai_drone.mavlink.remote import runtime_request
from ai_drone.mavlink.safety import heartbeat_is_armed, is_vehicle_message
from ai_drone.platform import is_raspberry_pi
from ai_drone.power_state import FcObserved, PiSnapshot, parse_fc_probe
from ai_drone.runtime_status import RuntimeStatus
from ai_drone.settings import load_settings
from ai_drone.system import unit_state

WALK_UNIT = "ai-drone-walk.service"
RUNTIME_UNIT = "ai-drone-runtime.service"
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
        groups = (process / 'cgroup').read_text().splitlines()
        walk = any(line.endswith('/ai-drone-walk.service') for line in groups)
        runtime = any(line.endswith('/ai-drone-runtime.service') for line in groups)
        for descriptor in (process / 'fd').iterdir():
            try:
                target = os.readlink(descriptor)
            except FileNotFoundError:
                continue
            if target in locks:
                package_job = True
            if target.startswith(prefixes):
                hardware.append({'pid': int(process.name), 'name': name,
                                 'device': target, 'walk': walk, 'runtime': runtime})
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
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError("uv is required for hardware-owner inspection")
    command = [
        uv,
        "run",
        "--no-project",
        "--no-config",
        "--offline",
        "--python",
        "/usr/bin/python3",
        "python",
        "-I",
        "-c",
        _OWNERS_SCRIPT,
    ]
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
    state = unit_state(result)
    if state.absent:
        return {"LoadState": "not-found", "ActiveState": "inactive"}
    if result.returncode or state.load != "loaded":
        raise RuntimeError("cannot establish recorder service state")
    return value


def _pi_snapshot(*, deadline: float | None = None) -> dict[str, Any]:
    result = _owners(deadline=deadline)
    result["walk"] = _walk_state(deadline=deadline)
    audit = _run(["dpkg", "--audit"], timeout=_remaining(deadline))
    result["package_database_ok"] = audit.returncode == 0 and not audit.stdout.strip()
    result["runtime"] = _runtime_status(deadline=deadline)
    return result


def _runtime_status(*, deadline: float | None = None) -> dict[str, Any] | None:
    socket = load_settings().runtime.socket
    if not Path(socket).exists():
        return None
    return {
        **runtime_request(socket, {"status": True}, timeout=_remaining(deadline, 3)),
        "socket": socket,
    }


def _runtime_fc(status: dict[str, Any]) -> dict[str, Any]:
    parsed = RuntimeStatus.parse(status)
    age = parsed.effective_heartbeat_age(time.monotonic())
    if not parsed.selected_source or age is None:
        raise RuntimeError("runtime selected-FC status is stale or unknown")
    fc = {**status, "heartbeat_age_s": age}
    if parsed.armed is not False:
        raise RuntimeError("runtime FC is armed or unknown")
    _require_disarmed(fc)
    return fc


def _guard_idle(snapshot: dict[str, Any], *, allow_walk: bool = False) -> None:
    parsed = PiSnapshot.parse(snapshot)
    if not parsed.complete:
        raise RuntimeError("hardware-owner status is unknown")
    if parsed.packages_active or not parsed.package_database_ok:
        raise RuntimeError("package maintenance is active or dpkg is incomplete")
    runtime = parsed.runtime
    if runtime is not None:
        _runtime_fc(runtime)
        if (
            "control_client" not in runtime
            or runtime["control_client"] is not None
            or runtime.get("network_busy") is not False
        ):
            raise RuntimeError(
                "runtime control or network operation is active or unknown"
            )
        clients = runtime.get("clients")
        allowed = 2 if allow_walk and parsed.walk_active else 1
        if type(clients) is not int or not 1 <= clients <= allowed:
            raise RuntimeError(
                "recorder or another runtime client is active or unknown"
            )
    owners = [
        owner
        for owner in parsed.hardware
        if not ((allow_walk and owner.walk) or (runtime is not None and owner.runtime))
    ]
    if owners:
        raise RuntimeError(
            "hardware is busy; close other camera, serial or GPIO tools first"
        )
    if not allow_walk and parsed.walk_active:
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


def _observe_power(message: Any, result: dict[str, Any]) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    if isinstance(message, mavlink.MAVLink_sys_status_message):
        result["battery_voltage_v"] = (
            message.voltage_battery / 1000 if message.voltage_battery != 65535 else None
        )
        result["battery_remaining_percent"] = (
            message.battery_remaining if message.battery_remaining >= 0 else None
        )
    elif isinstance(message, mavlink.MAVLink_power_status_message):
        result.update(
            board_voltage_v=message.Vcc / 1000,
            servo_voltage_v=message.Vservo / 1000,
            power_flags=message.flags,
        )


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
            else:
                _observe_power(message, result)
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
        return _pi_fc(snapshot)
    if action not in {"idle", "shutdown"}:
        raise ValueError("unknown Pi action")
    snapshot = _pi_snapshot()
    _guard_idle(snapshot)
    runtime = snapshot.get("runtime")
    if runtime is not None:
        return _shared_final_action(action, runtime)
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


def _pi_fc(snapshot: dict[str, Any]) -> dict[str, Any]:
    runtime = snapshot.get("runtime")
    return (
        _runtime_fc(runtime) if runtime is not None else _probe_fc(Path("/dev/serial0"))
    )


def _shared_final_action(action: str, runtime: dict[str, Any]) -> dict[str, Any]:
    """Keep new clients blocked through cable removal or accepted shutdown."""
    socket = runtime["socket"]
    confirmed = False
    acquired = False
    try:
        result = runtime_request(socket, {"maintenance": True})
        if result.get("maintenance") is not True:
            raise RuntimeError("runtime did not enter maintenance")
        acquired = True
        snapshot = _pi_snapshot()
        _guard_idle(snapshot)
        fc = _pi_fc(snapshot)
        _require_disarmed(fc)
        deadline = time.monotonic() + 2 - fc["heartbeat_age_s"]
        if action == "shutdown":
            result = _run(
                ["sudo", "-n", "systemctl", "poweroff", "--no-block"],
                timeout=_remaining(deadline),
            )
            if result.returncode:
                raise RuntimeError("Pi rejected the shutdown request")
        else:
            _remaining(deadline)
        confirmed = True
        return (
            {"idle": True, "maintenance": True}
            if action == "idle"
            else {"shutdown_requested": True, "halt_confirmed": False}
        )
    finally:
        if acquired and not confirmed:
            runtime_request(socket, {"maintenance": False})


def _ssh_commands(action: str) -> list[list[str]]:
    connection = resolve_connection_target()
    target = resolve_deploy_target()
    hosts = (
        [target.ssh_target]
        if os.environ.get("PI_HOST")
        else [
            f"{connection.pi_user}@{host}"
            for host in preferred_pi_addresses(connection)
        ]
    )
    remote = remote_uv_command(target, "ai_drone.cli.power", ["--remote", action])[-1]
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
    observation = parse_fc_probe(fc)
    if not isinstance(observation, FcObserved) or not observation.fresh_disarmed:
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
        print(f"  uv run --locked python scripts/power.py prepare {removal}")


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
        if final.get("maintenance"):
            print(
                "Runtime maintenance remains active; restart the runtime before the next flight."
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
