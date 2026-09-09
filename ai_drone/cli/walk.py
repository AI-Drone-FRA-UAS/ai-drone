"""Launch one timed, disarmed Pi recording independently of the SSH session."""

from __future__ import annotations

import argparse
import os
import re
import shlex
import signal
import subprocess
import sys
import uuid
from contextlib import suppress
from datetime import UTC, datetime
from pathlib import Path

from ai_drone.platform import is_raspberry_pi
from ai_drone.validation import positive_finite

UNIT = "ai-drone-walk.service"


def _runtime_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _user_ids() -> tuple[int, int]:
    if not hasattr(os, "getuid"):
        raise ValueError("drone-walk requires a Linux user account")
    uid, gid = os.getuid(), os.getgid()
    if uid == 0 or os.geteuid() == 0:
        raise ValueError("run drone-walk as the normal Pi user, without sudo")
    return uid, gid


def _output_path(requested: Path | None, root: Path) -> Path:
    output = requested or (
        root
        / "artifacts"
        / f"walk-{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid.uuid4().hex[:12]}"
    )
    output = output.expanduser().absolute()
    if any(character in str(output) for character in "\0\r\n"):
        raise ValueError("output directory must not contain control characters")
    if os.path.lexists(output):
        raise ValueError(f"output directory already exists: {output}")
    return output


def _validate_runtime(root: Path, python: Path) -> None:
    if not is_raspberry_pi():
        raise ValueError(
            "drone-walk must run on the Raspberry Pi; use --dry-run to preview"
        )
    for path in (
        python,
        root / "ai_drone/cli/record.py",
        root / "ai_drone/cli/report.py",
    ):
        if not path.is_file():
            raise ValueError(f"required runtime file is missing: {path}")
    if not Path("/dev/serial0").exists():
        raise ValueError("the Pi flight-controller port /dev/serial0 is missing")


def _launch_command(
    root: Path, python: Path, output: Path, duration: float, uid: int, gid: int
) -> list[str]:
    return [
        "sudo",
        "-n",
        "systemd-run",
        f"--unit={UNIT}",
        "--collect",
        "--service-type=exec",
        "--expand-environment=no",
        f"--uid={uid}",
        f"--gid={gid}",
        f"--working-directory={root}",
        "--setenv=PYTHONUNBUFFERED=1",
        "--property=Restart=no",
        "--property=KillSignal=SIGINT",
        "--property=KillMode=mixed",
        "--property=TimeoutStopSec=180",
        str(python),
        "-m",
        "ai_drone.cli.walk",
        "--worker",
        "--duration",
        str(duration),
        "--output-dir",
        str(output),
    ]


def _require_free_unit() -> None:
    result = subprocess.run(
        ["systemctl", "show", UNIT, "--property=LoadState", "--property=ActiveState"],
        capture_output=True,
        text=True,
        check=False,
        timeout=10,
    )
    properties = dict(
        line.split("=", 1) for line in result.stdout.splitlines() if "=" in line
    )
    if properties.get("LoadState") == "not-found" and result.returncode in (0, 1):
        return
    if properties.get("LoadState") == "loaded":
        raise ValueError(
            f"{UNIT} already exists ({properties.get('ActiveState', 'unknown')}); "
            "the existing job will not be stopped or replaced"
        )
    raise ValueError(
        f"cannot establish whether {UNIT} is free: {result.stderr.strip()}"
    )


def _reported_dataset(line: str, requested: Path) -> Path | None:
    prefix = "Manifest: "
    if not line.startswith(prefix):
        return None
    manifest = Path(line[len(prefix) :].strip())
    dataset = manifest.parent
    if (
        manifest.name != "manifest.json"
        or dataset.parent != requested.parent
        or re.fullmatch(re.escape(requested.name) + r"(?:-[1-9][0-9]*)?", dataset.name)
        is None
    ):
        return None
    return dataset


def _run_worker(root: Path, python: Path, output: Path, duration: float) -> int:
    child: subprocess.Popen[str] | None = None
    interrupted = False
    dataset: Path | None = None

    def stop(_signum: int, _frame: object) -> None:
        nonlocal interrupted
        interrupted = True
        if child is not None and child.poll() is None:
            with suppress(ProcessLookupError):
                child.send_signal(signal.SIGINT)

    def execute(command: list[str], *, recording: bool) -> int:
        nonlocal child, dataset
        with subprocess.Popen(
            command,
            cwd=root,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        ) as process:
            child = process
            if recording and interrupted:
                stop(signal.SIGINT, None)
            assert process.stdout is not None
            for line in process.stdout:
                print(line, end="", flush=True)
                if recording:
                    dataset = _reported_dataset(line, output) or dataset
            result = process.wait()
        child = None
        return result

    previous = {
        signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        for signum in previous:
            signal.signal(signum, stop)
        # Recheck immediately before capture; the launcher never reserves the
        # final directory because the recorder's exclusive mkdir owns it.
        _output_path(output, root)
        capture_result = execute(
            [
                str(python),
                "-m",
                "ai_drone.cli.record",
                "--device",
                "/dev/serial0",
                "--baud",
                "115200",
                "--backend",
                "native",
                "--duration",
                str(duration),
                "--output-dir",
                str(output),
            ],
            recording=True,
        )
        if dataset is None:
            print(
                "Recorder did not publish its dataset path; automatic report skipped.",
                flush=True,
            )
            return capture_result or 1
        print(f"Dataset: {dataset}", flush=True)
        report_result = execute(
            [str(python), "-m", "ai_drone.cli.report", str(dataset)], recording=False
        )
        return capture_result or report_result
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Start a detached, timed disarmed walkthrough recording and report."
    )
    parser.add_argument("--duration", type=float, default=300.0, metavar="SECONDS")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--dry-run", action="store_true", help="print the launch command only"
    )
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(arguments)
    try:
        duration = positive_finite(args.duration, "--duration")
        root = _runtime_root()
        # Preserve the venv path: resolving its interpreter symlink would lose
        # the Pi environment that includes the camera's system packages.
        python = Path(sys.executable).absolute()
        output = _output_path(args.output_dir, root)
        uid, gid = _user_ids()
        command = _launch_command(root, python, output, duration, uid, gid)
        if args.dry_run:
            print(shlex.join(command))
        else:
            _validate_runtime(root, python)
            if args.worker:
                return _run_worker(root, python, output, duration)
            _require_free_unit()
            # systemd also rejects an existing unit atomically if another
            # launcher wins the race after our read-only status check.
            subprocess.run(command, check=True, timeout=20)
        print(f"Dataset: {output}")
        print(f"Status: systemctl status {UNIT}")
        print(f"Logs: journalctl -u {UNIT} -f")
        print(f"Stop and finalize: sudo systemctl stop {UNIT}")
        print("Wait for manifest.json and the report before disconnecting power.")
        return 0
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Failed: {error}", file=sys.stderr)
        print(
            f"Check systemctl status {UNIT}; no existing job was stopped.",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
