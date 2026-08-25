from __future__ import annotations

import base64
import hashlib
import json
import subprocess
import sys
import zlib
from pathlib import Path

import pytest

from scripts import verify_ardupilot_firmware as verifier

TEST_IMAGE = b"\x00\x01FlywooF745 test firmware image\xfe\xff"


def good_feature_report() -> str:
    lines = [*sorted(verifier.REQUIRED_FEATURES)]
    lines.extend(f"!{name}" for name in sorted(verifier.FORBIDDEN_FEATURES))
    return "\n".join(lines) + "\n"


def encode_image(payload: bytes) -> str:
    return base64.b64encode(zlib.compress(payload)).decode("ascii")


def valid_apj_document(payload: bytes = TEST_IMAGE) -> dict[str, object]:
    return {
        "board_id": verifier.EXPECTED_BOARD_ID,
        "flash_total": verifier.MAX_IMAGE_SIZE,
        "git_identity": verifier.EXPECTED_GIT_IDENTITY,
        "image": encode_image(payload),
        "image_maxsize": verifier.MAX_IMAGE_SIZE,
        "image_size": len(payload),
        "magic": verifier.EXPECTED_MAGIC,
        "summary": verifier.EXPECTED_SUMMARY,
    }


def make_artifacts(
    tmp_path: Path,
    *,
    apj_payload: bytes = TEST_IMAGE,
    bin_payload: bytes = TEST_IMAGE,
    apj_overrides: dict[str, object] | None = None,
    include_bin: bool = True,
) -> tuple[Path, Path, Path]:
    root = tmp_path / "ardupilot"
    extractor = root / "Tools" / "scripts" / "extract_features.py"
    extractor.parent.mkdir(parents=True)
    extractor.write_text("# test placeholder\n", encoding="utf-8")

    build_dir = root / "custom-build"
    bin_dir = build_dir / "bin"
    bin_dir.mkdir(parents=True)
    elf_path = bin_dir / "arducopter"
    elf_path.write_bytes(b"ELF test placeholder")
    bin_path = bin_dir / "arducopter.bin"
    if include_bin:
        bin_path.write_bytes(bin_payload)

    document = valid_apj_document(apj_payload)
    if apj_overrides is not None:
        document.update(apj_overrides)
    apj_path = bin_dir / "arducopter.apj"
    apj_path.write_text(
        json.dumps(document),
        encoding="utf-8",
    )
    hw_dat_path = build_dir / "hw.dat"
    hw_dat_path.write_bytes(b"resolved FlywooF745 hw.dat")

    firmware_dir = tmp_path / "project" / "firmware"
    firmware_dir.mkdir(parents=True)
    overlay_path = firmware_dir / "FlywooF745-nogps-loiter-extra.hwdef"
    overlay_path.write_bytes(b"reviewed test overlay")
    manifest_path = firmware_dir / "FlywooF745-nogps-loiter.manifest.json"

    def digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    manifest_path.write_text(
        json.dumps(
            {
                "files": {
                    "arducopter": {"sha256": digest(elf_path)},
                    "arducopter.bin": {
                        "sha256": (
                            digest(bin_path)
                            if include_bin
                            else hashlib.sha256(bin_payload).hexdigest()
                        )
                    },
                    "arducopter.apj": {"sha256": digest(apj_path)},
                    "build/FlywooF745/hw.dat": {"sha256": digest(hw_dat_path)},
                    "firmware/FlywooF745-nogps-loiter-extra.hwdef": {
                        "sha256": digest(overlay_path)
                    },
                }
            }
        ),
        encoding="utf-8",
    )
    return root, build_dir, manifest_path


def write_apj(
    tmp_path: Path,
    *,
    payload: bytes = TEST_IMAGE,
    overrides: dict[str, object] | None = None,
) -> Path:
    document = valid_apj_document(payload)
    if overrides is not None:
        document.update(overrides)
    apj_path = tmp_path / "arducopter.apj"
    apj_path.write_text(json.dumps(document), encoding="utf-8")
    return apj_path


def stub_extractor(
    monkeypatch: pytest.MonkeyPatch,
    output: str,
    *,
    returncode: int = 0,
    stderr: str = "",
) -> list[list[str]]:
    calls: list[list[str]] = []

    def run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(command)
        return subprocess.CompletedProcess(command, returncode, output, stderr)

    monkeypatch.setattr(verifier.subprocess, "run", run)
    return calls


def test_verifies_relative_build_dir_with_official_extractor(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root, build_dir, manifest_path = make_artifacts(tmp_path)
    calls = stub_extractor(monkeypatch, good_feature_report())

    result = verifier.verify_build(
        root,
        build_dir.relative_to(root),
        manifest_path=manifest_path,
        nm="test-arm-none-eabi-nm",
    )

    assert result.board_id == verifier.EXPECTED_BOARD_ID
    assert result.git_identity == verifier.EXPECTED_GIT_IDENTITY
    assert result.image_size == len(TEST_IMAGE)
    assert result.elf_path == build_dir / "bin" / "arducopter"
    assert result.bin_path == build_dir / "bin" / "arducopter.bin"
    assert result.manifest_path == manifest_path
    assert calls == [
        [
            sys.executable,
            str(root / "Tools" / "scripts" / "extract_features.py"),
            str(build_dir / "bin" / "arducopter"),
            "--nm",
            "test-arm-none-eabi-nm",
        ]
    ]


@pytest.mark.parametrize("artifact_key", verifier.MANIFEST_ARTIFACT_KEYS)
def test_rejects_each_reviewed_manifest_hash_mismatch(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    artifact_key: str,
) -> None:
    root, build_dir, manifest_path = make_artifacts(tmp_path)
    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    document["files"][artifact_key]["sha256"] = "0" * 64
    manifest_path.write_text(json.dumps(document), encoding="utf-8")
    stub_extractor(monkeypatch, good_feature_report())

    with pytest.raises(verifier.VerificationError) as caught:
        verifier.verify_build(root, build_dir, manifest_path=manifest_path)

    assert f"SHA-256 mismatch for {artifact_key}" in str(caught.value)


def test_rejects_malformed_reviewed_manifest_hash(tmp_path: Path) -> None:
    _root, _build_dir, manifest_path = make_artifacts(tmp_path)
    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    document["files"]["arducopter"]["sha256"] = "not-a-sha256"
    manifest_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(
        verifier.VerificationError, match="must be 64 lowercase hex digits"
    ):
        verifier.load_manifest_hashes(manifest_path)


@pytest.mark.parametrize("feature", sorted(verifier.REQUIRED_FEATURES))
def test_rejects_each_required_feature_when_not_linked(feature: str) -> None:
    output = good_feature_report().replace(f"{feature}\n", f"!{feature}\n")

    errors = verifier.feature_errors(verifier.parse_feature_report(output))

    assert f"required feature is not linked: {feature}" in errors


def test_rejects_linked_optflow_srtm_feature() -> None:
    forbidden = next(iter(verifier.FORBIDDEN_FEATURES))
    output = good_feature_report().replace(f"!{forbidden}\n", f"{forbidden}\n")

    errors = verifier.feature_errors(verifier.parse_feature_report(output))

    assert errors == [f"forbidden feature is linked: {forbidden}"]


def test_fails_closed_when_feature_is_not_reported() -> None:
    required = next(iter(verifier.REQUIRED_FEATURES))
    forbidden = next(iter(verifier.FORBIDDEN_FEATURES))
    output = (
        good_feature_report()
        .replace(f"{required}\n", "")
        .replace(f"!{forbidden}\n", "")
    )

    errors = verifier.feature_errors(verifier.parse_feature_report(output))

    assert f"feature extractor did not report required feature {required}" in errors
    assert f"feature extractor did not report forbidden feature {forbidden}" in errors


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"board_id": 999}, "expected FlywooF745 id 1027"),
        ({"git_identity": "deadbeef"}, "expected '1511f271'"),
        ({"image_maxsize": 950_271}, "image_maxsize is 950271, expected 950272"),
        ({"flash_total": 950_273}, "flash_total is 950273, expected 950272"),
        ({"summary": "OtherBoard"}, "summary is 'OtherBoard', expected 'FlywooF745'"),
    ],
)
def test_rejects_wrong_board_commit_limits_or_summary(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    overrides: dict[str, object],
    message: str,
) -> None:
    root, build_dir, manifest_path = make_artifacts(tmp_path, apj_overrides=overrides)
    stub_extractor(monkeypatch, good_feature_report())

    with pytest.raises(verifier.VerificationError, match=message):
        verifier.verify_build(root, build_dir, manifest_path=manifest_path)


def test_rejects_image_larger_than_internal_flash(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    oversized = b"\0" * (verifier.MAX_IMAGE_SIZE + 1)
    root, build_dir, manifest_path = make_artifacts(
        tmp_path,
        apj_payload=oversized,
        bin_payload=oversized,
    )
    stub_extractor(monkeypatch, good_feature_report())

    with pytest.raises(verifier.VerificationError, match="limit is 950272"):
        verifier.verify_build(root, build_dir, manifest_path=manifest_path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("board_id", "1027", "board_id must be an integer"),
        ("git_identity", 1511, "git_identity must be a string"),
        ("image", 123, "image must be a base64 string"),
        ("image_maxsize", "950272", "image_maxsize must be an integer"),
        ("image_size", "32", "image_size must be an integer"),
        ("flash_total", "950272", "flash_total must be an integer"),
        ("summary", 1027, "summary must be a string"),
        ("image_size", 0, "image_size must be positive"),
    ],
)
def test_rejects_malformed_apj_metadata(
    tmp_path: Path, field: str, value: object, message: str
) -> None:
    apj_path = write_apj(tmp_path, overrides={field: value})

    with pytest.raises(verifier.VerificationError, match=message):
        verifier.load_apj_metadata(apj_path)


def test_rejects_wrong_apj_magic(tmp_path: Path) -> None:
    apj_path = write_apj(tmp_path, overrides={"magic": "not-an-apj"})

    with pytest.raises(verifier.VerificationError, match="expected 'APJFWv1'"):
        verifier.load_apj_metadata(apj_path)


@pytest.mark.parametrize(
    ("encoded_image", "message"),
    [
        ("%%%not-base64%%%", "image is not valid base64"),
        (
            base64.b64encode(b"not a zlib stream").decode("ascii"),
            "image is not valid zlib data",
        ),
    ],
)
def test_rejects_bad_apj_image_encoding(
    tmp_path: Path, encoded_image: str, message: str
) -> None:
    apj_path = write_apj(tmp_path, overrides={"image": encoded_image})

    with pytest.raises(verifier.VerificationError, match=message):
        verifier.load_apj_metadata(apj_path)


def test_rejects_decompressed_image_size_mismatch(tmp_path: Path) -> None:
    apj_path = write_apj(
        tmp_path,
        overrides={"image_size": len(TEST_IMAGE) + 1},
    )

    with pytest.raises(
        verifier.VerificationError,
        match=f"decompressed image length is {len(TEST_IMAGE)} bytes",
    ):
        verifier.load_apj_metadata(apj_path)


def test_rejects_apj_payload_different_from_bin(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root, build_dir, manifest_path = make_artifacts(
        tmp_path,
        bin_payload=b"different firmware payload",
    )
    stub_extractor(monkeypatch, good_feature_report())

    with pytest.raises(
        verifier.VerificationError,
        match=r"decompressed image does not match arducopter\.bin",
    ):
        verifier.verify_build(root, build_dir, manifest_path=manifest_path)


def test_rejects_missing_bin(tmp_path: Path) -> None:
    root, build_dir, manifest_path = make_artifacts(tmp_path, include_bin=False)

    with pytest.raises(verifier.VerificationError, match="ArduCopter BIN not found"):
        verifier.verify_build(root, build_dir, manifest_path=manifest_path)


def test_reports_extractor_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root, build_dir, manifest_path = make_artifacts(tmp_path)
    stub_extractor(
        monkeypatch,
        "",
        returncode=1,
        stderr="arm-none-eabi-nm not found",
    )

    with pytest.raises(verifier.VerificationError, match="arm-none-eabi-nm not found"):
        verifier.verify_build(root, build_dir, manifest_path=manifest_path)


def test_cli_returns_failure_for_missing_artifacts(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    exit_code = verifier.main(["--ardupilot-root", str(tmp_path)])

    assert exit_code == 1
    assert "linked ArduCopter ELF not found" in capsys.readouterr().err
