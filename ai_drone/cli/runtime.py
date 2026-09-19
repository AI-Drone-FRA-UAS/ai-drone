"""Run or inspect the Pi's shared vehicle/access service."""

from __future__ import annotations

import argparse
import json
import os
import signal
import threading
from dataclasses import replace
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from ai_drone.mavlink.connection import open_ardupilot_connection
from ai_drone.mavlink.devices import is_network_endpoint
from ai_drone.mavlink.ownership import require_available_serial
from ai_drone.mavlink.remote import runtime_request
from ai_drone.mavlink.shared import SharedMavlink
from ai_drone.runtime import VehicleAccess
from ai_drone.settings import load_settings


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("serve", "status"))
    parser.add_argument("--device")
    parser.add_argument("--socket", type=Path)
    parser.add_argument("--status-file", type=Path)
    parser.add_argument(
        "--no-network",
        action="store_true",
        help="run telemetry/operator access without managing Wi-Fi",
    )
    args = parser.parse_args(arguments)
    settings = load_settings()
    settings = replace(
        settings,
        runtime=replace(
            settings.runtime,
            device=args.device or settings.runtime.device,
            socket=str(args.socket or settings.runtime.socket),
            status=str(args.status_file or settings.runtime.status),
        ),
    )
    if args.action == "status":
        print(
            json.dumps(
                runtime_request(settings.runtime.socket, {"status": True}), indent=2
            )
        )
        return 0
    if os.name != "posix":
        parser.error("vehicle runtime requires the Pi's Unix sockets")
    if settings.runtime.device.startswith("unix:"):
        parser.error(
            "the vehicle runtime must own the physical FC or a simulator endpoint"
        )
    if not is_network_endpoint(settings.runtime.device):
        require_available_serial(settings.runtime.device)
    directory = Path(settings.runtime.socket).parent
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    stop = threading.Event()
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda _sig, _frame: stop.set())
    try:
        raw = open_ardupilot_connection(
            settings.runtime.device,
            baud=settings.runtime.baud,
            source_system=255,
            source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        with SharedMavlink(raw) as hub:
            VehicleAccess(hub, settings, manage_network=not args.no_network).run(stop)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
