#!/usr/bin/env python3
"""Verify the linked features and APJ limits of the FlywooF745 firmware build."""

from __future__ import annotations

import argparse
import base64
import binascii
import hashlib
import json
import re
import subprocess
import sys
import zlib
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import cast

EXPECTED_BOARD_ID = 1027
EXPECTED_ARDUPILOT_COMMIT = "1511f27194f1dcc3728270883047bdf022b3fd53"
EXPECTED_GIT_IDENTITY = "1511f271"
EXPECTED_MAGIC = "APJFWv1"
EXPECTED_SUMMARY = "FlywooF745"
EXPECTED_RUNTIME_VERSION_BANNER = b"ArduCopter V4.7.0 (1511f271)"
FORBIDDEN_RUNTIME_VERSION_BANNER = b"ArduCopter V4.7.0 (abcdef)"
EXPECTED_BASELINE_ROMFS_HWDEF_SHA256 = (
    "67404e7f31d096a010d810db2600617e9b4b8c287f017472136e299f5781e225"
)
EXPECTED_INTENTIONAL_FEATURE_CHANGES = {
    "EK3_FEATURE_OPTFLOW_FUSION": {"from": 0, "to": 1},
    "MODE_GUIDED_NOGPS_ENABLED": {"from": 0, "to": 1},
}
MAX_IMAGE_SIZE = 950_272
DEFAULT_MANIFEST_PATH = (
    Path(__file__).resolve().parents[1]
    / "firmware"
    / "FlywooF745-nogps-loiter.manifest.json"
)

MANIFEST_ARTIFACT_KEYS = (
    "arducopter",
    "arducopter.bin",
    "arducopter.apj",
    "build/FlywooF745/hw.dat",
    "firmware/FlywooF745-nogps-loiter-extra.hwdef",
)

REQUIRED_FEATURES = frozenset(
    {
        "AP_OPTICALFLOW_ENABLED",
        "AP_OPTICALFLOW_MAV_ENABLED",
        "AP_RANGEFINDER_ENABLED",
        "AP_RANGEFINDER_MAVLINK_ENABLED",
        "EK3_FEATURE_OPTFLOW_FUSION",
        "HAL_NAVEKF3_AVAILABLE",
        "MODE_GUIDED_NOGPS_ENABLED",
    }
)
FORBIDDEN_FEATURES = frozenset({"EK3_FEATURE_OPTFLOW_SRTM"})

_FEATURE_LINE = re.compile(r"(?P<disabled>!?)(?P<name>[A-Z][A-Z0-9_]*)")


class VerificationError(RuntimeError):
    """A firmware artifact failed a required invariant."""


@dataclass(frozen=True)
class VerificationResult:
    """Metadata returned after all build invariants pass."""

    elf_path: Path
    bin_path: Path
    apj_path: Path
    manifest_path: Path
    board_id: int
    git_identity: str
    image_size: int


@dataclass(frozen=True)
class ApjMetadata:
    """Validated metadata and uncompressed firmware payload from an APJ."""

    board_id: int
    git_identity: str
    image_size: int
    image_maxsize: int
    flash_total: int
    summary: str
    image: bytes


@dataclass(frozen=True)
class ManifestClaims:
    """Strictly validated reviewed claims and artifact hashes."""

    image_size: int
    hashes: dict[str, str]


def parse_feature_report(output: str) -> dict[str, bool]:
    """Parse the official extract_features.py one-feature-per-line output."""
    statuses: dict[str, bool] = {}
    for raw_line in output.splitlines():
        match = _FEATURE_LINE.fullmatch(raw_line.strip())
        if match is None:
            continue
        name = match.group("name")
        enabled = not bool(match.group("disabled"))
        previous = statuses.get(name)
        if previous is not None and previous != enabled:
            raise VerificationError(
                f"feature extractor reported conflicting states for {name}"
            )
        statuses[name] = enabled
    return statuses


def feature_errors(statuses: dict[str, bool]) -> list[str]:
    """Return every missing, disabled, or unexpectedly linked feature."""
    errors: list[str] = []
    for name in sorted(REQUIRED_FEATURES):
        if name not in statuses:
            errors.append(f"feature extractor did not report required feature {name}")
        elif not statuses[name]:
            errors.append(f"required feature is not linked: {name}")

    for name in sorted(FORBIDDEN_FEATURES):
        if name not in statuses:
            errors.append(f"feature extractor did not report forbidden feature {name}")
        elif statuses[name]:
            errors.append(f"forbidden feature is linked: {name}")
    return errors


def extract_feature_statuses(
    ardupilot_root: Path,
    elf_path: Path,
    *,
    nm: str | None = None,
) -> dict[str, bool]:
    """Run ArduPilot's own linked-symbol feature extractor for an ELF."""
    extractor = ardupilot_root / "Tools" / "scripts" / "extract_features.py"
    if not extractor.is_file():
        raise VerificationError(f"ArduPilot feature extractor not found: {extractor}")

    command = [sys.executable, str(extractor), str(elf_path)]
    if nm is not None:
        command.extend(("--nm", nm))
    try:
        completed = subprocess.run(
            command,
            cwd=ardupilot_root,
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError as exc:
        raise VerificationError(
            f"could not run ArduPilot feature extractor: {exc}"
        ) from exc

    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip() or "no output"
        raise VerificationError(
            f"ArduPilot feature extractor exited {completed.returncode}: {detail}"
        )
    return parse_feature_report(completed.stdout)


def load_apj_metadata(apj_path: Path) -> ApjMetadata:
    """Load, type-check, and decompress the safety-critical APJ fields."""
    try:
        document = json.loads(apj_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise VerificationError(
            f"could not read APJ metadata from {apj_path}: {exc}"
        ) from exc

    if not isinstance(document, dict):
        raise VerificationError(f"APJ document is not a JSON object: {apj_path}")

    board_id = document.get("board_id")
    git_identity = document.get("git_identity")
    image = document.get("image")
    image_maxsize = document.get("image_maxsize")
    image_size = document.get("image_size")
    flash_total = document.get("flash_total")
    magic = document.get("magic")
    summary = document.get("summary")
    if type(board_id) is not int:  # bool is not valid even though it subclasses int
        raise VerificationError("APJ board_id must be an integer")
    if not isinstance(git_identity, str):
        raise VerificationError("APJ git_identity must be a string")
    if not isinstance(image, str):
        raise VerificationError("APJ image must be a base64 string")
    if type(image_maxsize) is not int:
        raise VerificationError("APJ image_maxsize must be an integer")
    if type(image_size) is not int:
        raise VerificationError("APJ image_size must be an integer")
    if type(flash_total) is not int:
        raise VerificationError("APJ flash_total must be an integer")
    if not isinstance(summary, str):
        raise VerificationError("APJ summary must be a string")
    if image_size <= 0:
        raise VerificationError("APJ image_size must be positive")
    if magic != EXPECTED_MAGIC:
        raise VerificationError(f"APJ magic is {magic!r}, expected {EXPECTED_MAGIC!r}")

    try:
        compressed_image = base64.b64decode(image, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise VerificationError(f"APJ image is not valid base64: {exc}") from exc
    try:
        decoded_image = zlib.decompress(compressed_image)
    except zlib.error as exc:
        raise VerificationError(f"APJ image is not valid zlib data: {exc}") from exc
    if len(decoded_image) != image_size:
        raise VerificationError(
            "APJ decompressed image length is "
            f"{len(decoded_image)} bytes, image_size is {image_size}"
        )

    return ApjMetadata(
        board_id=board_id,
        git_identity=git_identity,
        image_size=image_size,
        image_maxsize=image_maxsize,
        flash_total=flash_total,
        summary=summary,
        image=decoded_image,
    )


def _require_exact_keys(
    value: object,
    expected_keys: set[str] | frozenset[str],
    field: str,
) -> dict[str, object]:
    if not isinstance(value, dict):
        raise VerificationError(f"firmware manifest {field} must be a JSON object")
    actual_keys = set(value)
    if actual_keys != expected_keys:
        missing = sorted(expected_keys - actual_keys)
        extra = sorted(actual_keys - expected_keys)
        raise VerificationError(
            f"firmware manifest {field} keys differ; missing={missing}, extra={extra}"
        )
    return cast("dict[str, object]", value)


def _validate_manifest_identity(document: dict[str, object]) -> None:
    ardupilot_commit = document["ardupilot_commit"]
    if not isinstance(ardupilot_commit, str):
        raise VerificationError("firmware manifest ardupilot_commit must be a string")
    if ardupilot_commit != EXPECTED_ARDUPILOT_COMMIT:
        raise VerificationError(
            f"firmware manifest ardupilot_commit is {ardupilot_commit!r}, "
            f"expected {EXPECTED_ARDUPILOT_COMMIT!r}"
        )

    board = document["board"]
    if not isinstance(board, str):
        raise VerificationError("firmware manifest board must be a string")
    if board != EXPECTED_SUMMARY:
        raise VerificationError(
            f"firmware manifest board is {board!r}, expected {EXPECTED_SUMMARY!r}"
        )

    board_id = document["board_id"]
    if type(board_id) is not int:
        raise VerificationError("firmware manifest board_id must be an integer")
    if board_id != EXPECTED_BOARD_ID:
        raise VerificationError(
            f"firmware manifest board_id is {board_id}, expected {EXPECTED_BOARD_ID}"
        )


def _validate_manifest_build(value: object) -> int:
    build = _require_exact_keys(
        value,
        {
            "consistent_builds",
            "image_max_size",
            "image_size",
            "runtime_custom_version",
        },
        "build",
    )
    consistent_builds = build["consistent_builds"]
    if type(consistent_builds) is not bool:
        raise VerificationError(
            "firmware manifest build.consistent_builds must be a boolean"
        )
    if consistent_builds is not False:
        raise VerificationError(
            "firmware manifest build.consistent_builds must be false"
        )

    image_max_size = build["image_max_size"]
    if type(image_max_size) is not int:
        raise VerificationError(
            "firmware manifest build.image_max_size must be an integer"
        )
    if image_max_size != MAX_IMAGE_SIZE:
        raise VerificationError(
            f"firmware manifest build.image_max_size is {image_max_size}, "
            f"expected {MAX_IMAGE_SIZE}"
        )

    image_size = build["image_size"]
    if type(image_size) is not int:
        raise VerificationError("firmware manifest build.image_size must be an integer")
    if image_size <= 0:
        raise VerificationError("firmware manifest build.image_size must be positive")

    runtime_custom_version = build["runtime_custom_version"]
    if not isinstance(runtime_custom_version, str):
        raise VerificationError(
            "firmware manifest build.runtime_custom_version must be a string"
        )
    if runtime_custom_version != EXPECTED_GIT_IDENTITY:
        raise VerificationError(
            "firmware manifest build.runtime_custom_version is "
            f"{runtime_custom_version!r}, expected {EXPECTED_GIT_IDENTITY!r}"
        )
    return image_size


def _validate_manifest_baseline(value: object) -> None:
    installed_baseline = _require_exact_keys(
        value,
        {"romfs_hwdef_sha256"},
        "installed_baseline",
    )
    baseline_hash = installed_baseline["romfs_hwdef_sha256"]
    if not isinstance(baseline_hash, str):
        raise VerificationError(
            "firmware manifest installed_baseline.romfs_hwdef_sha256 must be a string"
        )
    if baseline_hash != EXPECTED_BASELINE_ROMFS_HWDEF_SHA256:
        raise VerificationError(
            "firmware manifest installed_baseline.romfs_hwdef_sha256 is "
            f"{baseline_hash!r}, expected {EXPECTED_BASELINE_ROMFS_HWDEF_SHA256!r}"
        )


def _validate_intentional_feature_changes(value: object) -> None:
    changes = _require_exact_keys(
        value,
        set(EXPECTED_INTENTIONAL_FEATURE_CHANGES),
        "intentional_feature_changes",
    )
    for feature, expected_change in EXPECTED_INTENTIONAL_FEATURE_CHANGES.items():
        change = _require_exact_keys(
            changes[feature],
            {"from", "to"},
            f"intentional_feature_changes.{feature}",
        )
        for direction, expected_value in expected_change.items():
            actual_value = change[direction]
            if type(actual_value) is not int or actual_value != expected_value:
                raise VerificationError(
                    "firmware manifest intentional_feature_changes."
                    f"{feature}.{direction} is {actual_value!r}, "
                    f"expected integer {expected_value}"
                )


def _load_manifest_hashes(value: object) -> dict[str, str]:
    files = _require_exact_keys(
        value,
        set(MANIFEST_ARTIFACT_KEYS),
        "files",
    )

    hashes: dict[str, str] = {}
    for key in MANIFEST_ARTIFACT_KEYS:
        entry = _require_exact_keys(files[key], {"sha256"}, f"files.{key}")
        expected_hash = entry["sha256"]
        if (
            not isinstance(expected_hash, str)
            or re.fullmatch(r"[0-9a-f]{64}", expected_hash) is None
        ):
            raise VerificationError(
                f"firmware manifest SHA-256 for {key} must be 64 lowercase hex digits"
            )
        hashes[key] = expected_hash
    return hashes


def load_manifest_claims(manifest_path: Path) -> ManifestClaims:
    """Strictly parse every reviewed manifest claim and artifact hash."""
    try:
        document = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise VerificationError(
            f"could not read firmware manifest {manifest_path}: {exc}"
        ) from exc
    document = _require_exact_keys(
        document,
        {
            "ardupilot_commit",
            "board",
            "board_id",
            "build",
            "files",
            "installed_baseline",
            "intentional_feature_changes",
        },
        "top-level",
    )

    _validate_manifest_identity(document)
    image_size = _validate_manifest_build(document["build"])
    _validate_manifest_baseline(document["installed_baseline"])
    _validate_intentional_feature_changes(document["intentional_feature_changes"])
    hashes = _load_manifest_hashes(document["files"])
    return ManifestClaims(image_size=image_size, hashes=hashes)


def manifest_errors(
    manifest_path: Path,
    build_dir: Path,
    apj_image_size: int,
) -> list[str]:
    """Compare every reviewed manifest hash with its exact build input/output."""
    claims = load_manifest_claims(manifest_path)
    targets = {
        "arducopter": build_dir / "bin" / "arducopter",
        "arducopter.bin": build_dir / "bin" / "arducopter.bin",
        "arducopter.apj": build_dir / "bin" / "arducopter.apj",
        "build/FlywooF745/hw.dat": build_dir / "hw.dat",
        "firmware/FlywooF745-nogps-loiter-extra.hwdef": (
            manifest_path.parent / "FlywooF745-nogps-loiter-extra.hwdef"
        ),
    }

    errors: list[str] = []
    if claims.image_size != apj_image_size:
        errors.append(
            f"manifest image_size is {claims.image_size}, "
            f"APJ image_size is {apj_image_size}"
        )
    for key in MANIFEST_ARTIFACT_KEYS:
        path = targets[key]
        try:
            with path.open("rb") as artifact:
                actual_hash = hashlib.file_digest(artifact, "sha256").hexdigest()
        except OSError as exc:
            errors.append(f"could not hash manifest artifact {key} at {path}: {exc}")
            continue
        if actual_hash != claims.hashes[key]:
            errors.append(
                f"SHA-256 mismatch for {key}: got {actual_hash}, "
                f"expected {claims.hashes[key]}"
            )
    return errors


def verify_build(
    ardupilot_root: Path,
    build_dir: Path,
    *,
    manifest_path: Path | None = None,
    nm: str | None = None,
) -> VerificationResult:
    """Verify one FlywooF745 ArduCopter ELF/APJ artifact pair."""
    root = ardupilot_root.expanduser().resolve()
    resolved_build_dir = build_dir.expanduser()
    if not resolved_build_dir.is_absolute():
        resolved_build_dir = root / resolved_build_dir
    resolved_build_dir = resolved_build_dir.resolve()
    resolved_manifest_path = (
        DEFAULT_MANIFEST_PATH if manifest_path is None else manifest_path.expanduser()
    ).resolve()

    elf_path = resolved_build_dir / "bin" / "arducopter"
    bin_path = resolved_build_dir / "bin" / "arducopter.bin"
    apj_path = resolved_build_dir / "bin" / "arducopter.apj"
    if not elf_path.is_file():
        raise VerificationError(f"linked ArduCopter ELF not found: {elf_path}")
    if not bin_path.is_file():
        raise VerificationError(f"ArduCopter BIN not found: {bin_path}")
    if not apj_path.is_file():
        raise VerificationError(f"ArduCopter APJ not found: {apj_path}")

    statuses = extract_feature_statuses(root, elf_path, nm=nm)
    metadata = load_apj_metadata(apj_path)
    reviewed_manifest_errors = manifest_errors(
        resolved_manifest_path,
        resolved_build_dir,
        metadata.image_size,
    )
    try:
        binary_image = bin_path.read_bytes()
    except OSError as exc:
        raise VerificationError(
            f"could not read ArduCopter BIN {bin_path}: {exc}"
        ) from exc

    errors = [*reviewed_manifest_errors, *feature_errors(statuses)]
    if metadata.board_id != EXPECTED_BOARD_ID:
        errors.append(
            f"APJ board_id is {metadata.board_id}, "
            f"expected FlywooF745 id {EXPECTED_BOARD_ID}"
        )
    if metadata.git_identity != EXPECTED_GIT_IDENTITY:
        errors.append(
            f"APJ git_identity is {metadata.git_identity!r}, "
            f"expected {EXPECTED_GIT_IDENTITY!r}"
        )
    if metadata.image_maxsize != MAX_IMAGE_SIZE:
        errors.append(
            f"APJ image_maxsize is {metadata.image_maxsize}, expected {MAX_IMAGE_SIZE}"
        )
    if metadata.flash_total != MAX_IMAGE_SIZE:
        errors.append(
            f"APJ flash_total is {metadata.flash_total}, expected {MAX_IMAGE_SIZE}"
        )
    if metadata.summary != EXPECTED_SUMMARY:
        errors.append(
            f"APJ summary is {metadata.summary!r}, expected {EXPECTED_SUMMARY!r}"
        )
    if metadata.image_size > MAX_IMAGE_SIZE:
        errors.append(
            f"APJ image_size is {metadata.image_size} bytes, limit is {MAX_IMAGE_SIZE}"
        )
    if metadata.image != binary_image:
        errors.append("APJ decompressed image does not match arducopter.bin")
    if EXPECTED_RUNTIME_VERSION_BANNER not in binary_image:
        errors.append(
            "arducopter.bin does not contain runtime version banner "
            f"{EXPECTED_RUNTIME_VERSION_BANNER.decode('ascii')!r}"
        )
    if EXPECTED_RUNTIME_VERSION_BANNER not in metadata.image:
        errors.append(
            "APJ payload does not contain runtime version banner "
            f"{EXPECTED_RUNTIME_VERSION_BANNER.decode('ascii')!r}"
        )
    if FORBIDDEN_RUNTIME_VERSION_BANNER in binary_image:
        errors.append(
            "arducopter.bin contains forbidden 'abcdef' runtime version banner"
        )
    if FORBIDDEN_RUNTIME_VERSION_BANNER in metadata.image:
        errors.append("APJ payload contains forbidden 'abcdef' runtime version banner")
    if errors:
        raise VerificationError(
            "firmware verification failed:\n- " + "\n- ".join(errors)
        )

    return VerificationResult(
        elf_path=elf_path,
        bin_path=bin_path,
        apj_path=apj_path,
        manifest_path=resolved_manifest_path,
        board_id=metadata.board_id,
        git_identity=metadata.git_identity,
        image_size=metadata.image_size,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Verify that a FlywooF745 ArduCopter build contains the no-GPS "
            "Loiter dependencies and fits in internal flash."
        )
    )
    parser.add_argument(
        "--ardupilot-root",
        required=True,
        type=Path,
        help="ArduPilot source checkout containing Tools/scripts/extract_features.py",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=Path("build/FlywooF745"),
        help=(
            "board build directory, absolute or relative to --ardupilot-root "
            "(default: build/FlywooF745)"
        ),
    )
    parser.add_argument(
        "--nm",
        help=(
            "nm executable passed to extract_features.py "
            "(default: its arm-none-eabi-nm default)"
        ),
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=DEFAULT_MANIFEST_PATH,
        help=(f"reviewed firmware hash manifest (default: {DEFAULT_MANIFEST_PATH})"),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = verify_build(
            args.ardupilot_root,
            args.build_dir,
            manifest_path=args.manifest,
            nm=args.nm,
        )
    except VerificationError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    free_bytes = MAX_IMAGE_SIZE - result.image_size
    print("PASS: FlywooF745 no-GPS Loiter firmware build verified")
    print(f"  ELF: {result.elf_path}")
    print(f"  BIN: {result.bin_path}")
    print(f"  APJ: {result.apj_path}")
    print(f"  manifest: {result.manifest_path}")
    print(f"  board_id: {result.board_id}")
    print(f"  git_identity: {result.git_identity}")
    print(
        f"  image_size: {result.image_size} / {MAX_IMAGE_SIZE} bytes "
        f"({free_bytes} bytes free)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
