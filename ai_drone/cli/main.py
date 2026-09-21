from __future__ import annotations

import argparse
from importlib import import_module
from pathlib import Path

from ai_drone.cli.harness import invoke
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
    "power": ("ai_drone.cli.power", "main"),
    "servo": ("ai_drone.cli.servo", "main"),
    "mount": ("ai_drone.cli.mount", "main"),
    "motor-test": ("ai_drone.cli.motor_test", "main"),
    "config-export": ("ai_drone.cli.config_export", "main"),
}


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="drone", description="Drone operation tools.")
    parser.add_argument(
        "--config", type=Path, help="local TOML defaults (otherwise drone.toml)"
    )
    parser.add_argument("command", choices=COMMANDS)
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args(arguments)
    try:
        settings = load_settings(args.config)
        module, function = COMMANDS[args.command]
        return invoke(
            getattr(import_module(module), function), args.arguments, settings
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
