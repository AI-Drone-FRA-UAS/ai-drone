"""Open SSH through the laptop's current network."""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
from collections.abc import Mapping, Sequence

from ai_drone.link.targets import resolve_deploy_target, ssh_base_command


def run(
    arguments: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "host", nargs="?", help="explicit hostname/IP, optionally user@host"
    )
    parser.add_argument("--dry-run", action="store_true", help="print the SSH command")
    args = parser.parse_args(arguments)
    values = dict(os.environ if environ is None else environ)
    if args.host:
        values["PI_HOST"] = args.host
    target = resolve_deploy_target(values)
    command = [*ssh_base_command(target.ssh_config), "-t", target.ssh_target]
    if args.dry_run:
        print(shlex.join(command))
        return 0
    return subprocess.run(command, check=False).returncode


def main() -> None:
    raise SystemExit(run())
