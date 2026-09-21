"""Fetch and validate the live drone configuration through the Pi."""

from __future__ import annotations

import argparse
import json
import math
import shlex
import subprocess
from collections.abc import Mapping
from datetime import datetime
from pathlib import Path
from typing import Any

from ai_drone.config.snapshot import (
    ParameterRecord,
    parameter_sha256,
    records_from_json,
    render_parameter_file,
)
from ai_drone.durability import atomic_write_text
from ai_drone.link import deploy
from ai_drone.link.targets import remote_uv_command


def _run(
    command: list[str], *, capture: bool = False
) -> subprocess.CompletedProcess[str]:
    print(f"  {shlex.join(command)}", flush=True)
    try:
        return subprocess.run(
            command,
            check=True,
            capture_output=capture,
            text=True,
        )
    except subprocess.CalledProcessError as error:
        if capture and error.stderr:
            print(error.stderr, end="", flush=True)
        raise


def remote_export_command(
    plan: deploy.DeployPlan,
    *,
    device: str | None,
    baud: int,
    timeout: float,
) -> list[str]:
    """Build the non-interactive SSH command that emits one JSON bundle."""

    arguments = (["--device", device] if device else []) + [
        "--baud",
        str(baud),
        "--download-timeout",
        str(timeout),
    ]
    return remote_uv_command(plan.target, "ai_drone.cli.config_export", arguments)


def _captured_at(value: object) -> datetime:
    captured_value = value
    if not isinstance(captured_value, str):
        raise ValueError("Snapshot captured_at must be an ISO-8601 string")
    try:
        captured = datetime.fromisoformat(captured_value)
    except ValueError as error:
        raise ValueError("Snapshot captured_at is not valid ISO-8601") from error
    if captured.tzinfo is None:
        raise ValueError("Snapshot captured_at must include a timezone")

    return captured


def _validated_bundle(
    bundle: Mapping[str, Any],
) -> tuple[list[ParameterRecord], datetime, Mapping[str, Any]]:
    """Validate the untrusted JSON object returned over SSH."""

    if bundle.get("schema_version") != 1:
        raise ValueError("Unsupported or missing snapshot schema_version")

    captured = _captured_at(bundle.get("captured_at"))

    source = bundle.get("source")
    if not isinstance(source, Mapping):
        raise ValueError("Snapshot source must be an object")
    endpoint = source.get("endpoint")
    if not isinstance(endpoint, str) or not endpoint:
        raise ValueError("Snapshot source endpoint must be a non-empty string")
    if len(endpoint) > 512 or any(
        ord(character) < 32 or ord(character) == 127 for character in endpoint
    ):
        raise ValueError(
            "Snapshot source endpoint contains control characters or is too long"
        )
    baud = source.get("baud")
    if not isinstance(baud, int) or isinstance(baud, bool) or baud <= 0:
        raise ValueError("Snapshot source baud must be a positive integer")

    vehicle = bundle.get("vehicle")
    if not isinstance(vehicle, Mapping) or vehicle.get("armed") is not False:
        raise ValueError("Snapshot must explicitly report a disarmed vehicle")

    items = bundle.get("parameters")
    if not isinstance(items, list) or not all(isinstance(item, dict) for item in items):
        raise ValueError("Snapshot parameters must be a list of objects")
    records = records_from_json(items)

    count = bundle.get("parameter_count")
    if not isinstance(count, int) or isinstance(count, bool) or count != len(records):
        raise ValueError("Snapshot parameter_count does not match its payload")
    calculated_hash = parameter_sha256(records)
    if bundle.get("parameter_sha256") != calculated_hash:
        raise ValueError("Remote parameter payload failed its SHA-256 validation")
    return records, captured, source


def snapshot_paths(bundle: Mapping[str, Any], repo_root: Path) -> tuple[Path, Path]:
    """Choose stable tracked paths for a remote snapshot bundle."""

    captured = datetime.fromisoformat(str(bundle["captured_at"]))
    date = captured.date().isoformat()
    return (
        repo_root / "params" / f"flywoo-f745-live-{date}.param",
        repo_root / "state" / date / "drone-config.json",
    )


def write_snapshot(bundle: dict[str, Any], repo_root: Path) -> tuple[Path, Path]:
    """Validate the transfer and write the parameter file plus metadata."""

    records, captured, source = _validated_bundle(bundle)
    calculated_hash = parameter_sha256(records)

    parameter_path, metadata_path = snapshot_paths(bundle, repo_root)
    header = (
        f"# Captured from {source['endpoint']} at {source['baud']} baud\n"
        f"# UTC: {captured.isoformat()}\n"
        "# Vehicle was DISARMED\n"
        f"# Received {len(records)} of {len(records)} advertised "
        "parameters\n"
        f"# Parameter SHA-256: {calculated_hash}\n"
    )
    parameter_content = header + render_parameter_file(records)

    metadata = dict(bundle)
    metadata.pop("parameters", None)
    metadata["parameter_file"] = str(parameter_path.relative_to(repo_root))
    metadata_content = (
        json.dumps(
            metadata,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    )

    atomic_write_text(parameter_path, parameter_content)
    atomic_write_text(metadata_path, metadata_content)
    return parameter_path, metadata_path


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Read the full flight-controller configuration through the Pi, save it "
            "locally without deploying, changing parameters, or publishing to Git."
        )
    )
    parser.add_argument("--device")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args(arguments)

    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and greater than zero")
    plan = deploy.build_plan([])

    completed = _run(
        remote_export_command(
            plan,
            device=args.device,
            baud=args.baud,
            timeout=args.timeout,
        ),
        capture=True,
    )
    if completed.stderr:
        print(completed.stderr, end="")
    bundle = json.loads(completed.stdout)
    paths = write_snapshot(bundle, deploy.REPO_ROOT)
    print(f"Saved {bundle['parameter_count']} parameters to {paths[0]}")
    print(f"Saved snapshot metadata to {paths[1]}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
