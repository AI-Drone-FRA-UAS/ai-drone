"""Run pinned SITL in a Linux network namespace that cannot reach an aircraft."""

from __future__ import annotations

import os
import subprocess
import sys


def main() -> int:
    environment = {**os.environ, "AI_DRONE_ISOLATED_SITL": "1"}
    return subprocess.run(
        [
            "unshare",
            "--user",
            "--map-root-user",
            "--net",
            "sh",
            "-c",
            'ip link set lo up && exec "$@"',
            "sitl",
            "uv",
            "run",
            "--no-sync",
            "--python",
            sys.executable,
            "python",
            "-m",
            "pytest",
            "-m",
            "sitl",
            *sys.argv[1:],
        ],
        env=environment,
        check=False,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
