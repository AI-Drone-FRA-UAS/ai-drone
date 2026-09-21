"""Inspect networking or ask the shared Pi runtime for an explicit Wi-Fi switch."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys

from ai_drone.link.targets import remote_python_command, resolve_deploy_target
from ai_drone.mavlink.remote import runtime_request
from ai_drone.platform import is_raspberry_pi
from ai_drone.settings import load_settings


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="SSH hostname/IP for an explicit remote route")
    commands = parser.add_subparsers(dest="action", required=True)
    commands.add_parser("status")
    commands.add_parser("list")
    commands.add_parser("connect").add_argument("profile")
    commands.add_parser("hotspot").add_argument("state", choices=("on", "off"))
    args = parser.parse_args(arguments)
    action = [args.action]
    if args.action == "connect":
        action.append(args.profile)
    elif args.action == "hotspot":
        action.append(args.state)
    try:
        if args.host or not is_raspberry_pi():
            values = dict(os.environ)
            if args.host:
                values["PI_HOST"] = args.host
            target = resolve_deploy_target(values)
            command = remote_python_command(target, ["scripts/network.py", *action])
            return subprocess.run(command, check=False).returncode
        if args.action == "list":
            return subprocess.run(
                [
                    "nmcli",
                    "-f",
                    "NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY",
                    "connection",
                    "show",
                ],
                check=False,
            ).returncode
        request = {"status": True} if args.action == "status" else {"network": action}
        print(
            json.dumps(
                runtime_request(load_settings().runtime.socket, request), indent=2
            )
        )
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"Network request failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
