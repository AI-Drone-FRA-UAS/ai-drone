#!/usr/bin/env python3
"""Apply ai-drone's skill settings as launch overrides for Codex 0.154."""

import json
import os
import sys
import tomllib
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
BINARY = Path("/home/abaris/.codex/packages/standalone/current/bin/codex")


def launch_directory(arguments, cwd):
    """Honor the CLI's working-directory option before applying project scope."""
    result = cwd
    i = 0
    while i < len(arguments):
        argument = arguments[i]
        if argument == "--":
            break
        if argument in ("-c", "--config", "-m", "--model", "-p", "--profile"):
            i += 2
            continue
        if argument in ("-C", "--cd") and i + 1 < len(arguments):
            result = Path(arguments[i + 1]).expanduser()
            i += 2
            continue
        if argument.startswith("--cd="):
            result = Path(argument.split("=", 1)[1]).expanduser()
        elif argument.startswith("-C") and len(argument) > 2:
            result = Path(argument[2:]).expanduser()
        i += 1
    if not result.is_absolute():
        result = cwd / result
    return result.resolve()


def launch_arguments(arguments, cwd):
    directory = launch_directory(arguments, cwd)
    if not directory.is_relative_to(PROJECT):
        return arguments
    config = tomllib.loads((PROJECT / ".codex/config.toml").read_text())
    settings = config.get("skills", {}).get("config", [])
    entries = []
    for setting in settings:
        fields = []
        for selector in ("name", "path"):
            if selector in setting:
                fields.append(selector + "=" + json.dumps(setting[selector]))
        fields.append("enabled=" + str(setting.get("enabled", True)).lower())
        entries.append("{" + ",".join(fields) + "}")
    # Explicit caller overrides appear later and retain precedence.
    return ["-c", "skills.config=[" + ",".join(entries) + "]", *arguments]


if __name__ == "__main__":
    arguments = launch_arguments(sys.argv[1:], Path.cwd())
    os.execv(BINARY, [str(BINARY), *arguments])
