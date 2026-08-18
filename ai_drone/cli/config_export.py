"""Export the complete current ArduPilot configuration without changing it."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.config_snapshot import (
    download_all_parameters,
    heartbeat_is_armed,
    parameter_sha256,
    records_to_json,
)
from ai_drone.mavlink_devices import resolve_mavlink_endpoint


def build_bundle(
    records: list[Any],
    heartbeat: Any,
    *,
    endpoint: str,
    baud: int,
    captured_at: str | None = None,
) -> dict[str, Any]:
    """Build the validated JSON object transferred back to the developer host."""

    return {
        "schema_version": 1,
        "captured_at": captured_at or datetime.now(timezone.utc).isoformat(),
        "source": {
            "endpoint": endpoint,
            "baud": baud,
            "target_system": int(getattr(heartbeat, "get_srcSystem", lambda: 0)()),
            "target_component": int(
                getattr(heartbeat, "get_srcComponent", lambda: 0)()
            ),
        },
        "vehicle": {
            "type": int(heartbeat.type),
            "autopilot": int(heartbeat.autopilot),
            "base_mode": int(heartbeat.base_mode),
            "custom_mode": int(heartbeat.custom_mode),
            "system_status": int(heartbeat.system_status),
            "armed": heartbeat_is_armed(heartbeat),
        },
        "parameter_count": len(records),
        "parameter_sha256": parameter_sha256(records),
        "parameters": records_to_json(records),
    }


def _write_bundle(bundle: dict[str, Any], output: Path | None) -> None:
    payload = json.dumps(bundle, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if output is None:
        sys.stdout.write(payload)
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(payload)
    print(f"Saved snapshot bundle to {output}", file=sys.stderr)


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read every ArduPilot parameter over MAVLink; never write parameters."
    )
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--heartbeat-timeout", type=float, default=15.0)
    parser.add_argument("--download-timeout", type=float, default=180.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(arguments)

    endpoint = resolve_mavlink_endpoint(args.device, include_pi_uart=True)
    print(
        f"Connecting read-only to {endpoint} at {args.baud} baud ...", file=sys.stderr
    )
    connection = mavutil.mavlink_connection(
        endpoint,
        baud=args.baud,
        source_system=255,
        source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )
    try:
        heartbeat = connection.wait_heartbeat(timeout=args.heartbeat_timeout)
        if heartbeat is None:
            raise SystemExit("No ArduPilot heartbeat received.")
        if heartbeat_is_armed(heartbeat):
            raise SystemExit("Vehicle is ARMED; refusing a full parameter download.")
        connection.target_system = heartbeat.get_srcSystem()
        connection.target_component = heartbeat.get_srcComponent()

        print("Vehicle is DISARMED. Downloading all parameters ...", file=sys.stderr)
        records = download_all_parameters(
            connection,
            timeout=args.download_timeout,
        )
        bundle = build_bundle(
            records,
            heartbeat,
            endpoint=endpoint,
            baud=args.baud,
        )
        _write_bundle(bundle, args.output)
        print(
            f"Downloaded {len(records)}/{len(records)} parameters; "
            f"SHA-256 {bundle['parameter_sha256']}",
            file=sys.stderr,
        )
        return 0
    finally:
        connection.close()


if __name__ == "__main__":
    raise SystemExit(main())
