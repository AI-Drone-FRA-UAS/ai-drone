from __future__ import annotations

import argparse
import os
from importlib import import_module
from pathlib import Path

from ai_drone.settings import load_settings

COMMANDS = {
    "connect": ("ai_drone.link.connect", "run"),
    "deploy": ("ai_drone.link.deploy", "run"),
    "record": ("ai_drone.cli.record", "run"),
    "walk": ("ai_drone.cli.walk", "main"),
    "report": ("ai_drone.cli.report", "main"),
    "check": ("ai_drone.cli.check", "main"),
    "control": ("ai_drone.cli.control", "main"),
    "config": ("ai_drone.config.sync", "main"),
    "tag-servo-record": ("ai_drone.cli.tag_servo_record", "main"),
    "operator": ("ai_drone.cli.operator", "main"),
    "runtime": ("ai_drone.cli.runtime", "main"),
}


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="drone", description="Drone operation tools.")
    parser.add_argument(
        "--config", type=Path, help="local TOML defaults (otherwise drone.toml)"
    )
    parser.add_argument("command", choices=COMMANDS)
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args(arguments)
    previous = os.environ.get("AI_DRONE_CONFIG")
    if args.config is not None:
        os.environ["AI_DRONE_CONFIG"] = str(args.config.expanduser().absolute())
    try:
        load_settings()
        module, function = COMMANDS[args.command]
        return getattr(import_module(module), function)(args.arguments)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    finally:
        if previous is None:
            os.environ.pop("AI_DRONE_CONFIG", None)
        else:
            os.environ["AI_DRONE_CONFIG"] = previous


if __name__ == "__main__":
    raise SystemExit(main())
