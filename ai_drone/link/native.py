"""Validate an explicitly reviewed, offline Pi native-runtime payload."""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import tomllib
import zipfile
from email.parser import BytesParser
from pathlib import Path

from ai_drone.link.deploy import (
    _iter_sync_paths,
    _validate_runtime_source,
    _validated_manifest_path,
)

MANIFEST = "native-runtime/manifest.json"
NATIVE_DISTRIBUTIONS = {
    "ai-drone-candidate-libcamera",
    "ai-drone-candidate-pykms",
    "ai-drone-debian-apriltag",
    "python-prctl",
    "lgpio",
    "pidng",
    "openexr",
    "picamera2",
    "pymavlink",
}
NATIVE_LIBRARIES = {
    "libcamera.so.0.7",
    "libcamera-base.so.0.7",
    "libkms++.so.0",
    "libkms++util.so.0",
    "libdrm.so.2",
    "libstdc++.so.6",
    "libgcc_s.so.1",
    "libc.so.6",
    "libapriltag.so.3",
    "liblgpio.so.1",
    "libcap.so.2",
    "libOpenEXR-3_1.so.30",
    "libImath-3_1.so.29",
    "libIex-3_1.so.30",
    "libIlmThread-3_1.so.30",
}
NATIVE_PACKAGES = {
    "libcamera0.7",
    "libkms++0",
    "libdrm2",
    "libapriltag3t64",
    "liblgpio1",
    "libcap2",
    "libstdc++6",
    "libgcc-s1",
    "libc6",
    "libopenexr-3-1-30",
    "libimath-3-1-29t64",
}
IDENTITY = (
    "import json, platform, sys, sysconfig; "
    "print(json.dumps(dict(version=platform.python_version(), "
    "machine=platform.machine(), soabi=sysconfig.get_config_var('SOABI'), "
    "gil_disabled=bool(sysconfig.get_config_var('Py_GIL_DISABLED')), "
    "system_site=any('/usr/lib/python3/dist-packages' in p for p in sys.path))))"
)


def _unique_object(pairs: list[tuple[str, object]]) -> dict:
    value = dict(pairs)
    if len(value) != len(pairs):
        raise ValueError("native manifest contains duplicate keys")
    return value


def _hash(path: Path) -> str:
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def _require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(f"native payload: {reason}")


def _wheel_metadata(root: Path, wheel: dict) -> str:
    path = root / _validated_manifest_path(wheel["path"])
    _require(
        path.suffix == ".whl" and bool(wheel["provenance"]), "wheel provenance missing"
    )
    with zipfile.ZipFile(path) as archive:
        metadata = BytesParser().parsebytes(
            archive.read(
                next(
                    name
                    for name in archive.namelist()
                    if name.endswith(".dist-info/METADATA")
                )
            )
        )
        tags = (
            BytesParser()
            .parsebytes(
                archive.read(
                    next(
                        name
                        for name in archive.namelist()
                        if name.endswith(".dist-info/WHEEL")
                    )
                )
            )
            .get_all("Tag", [])
        )
    _require(
        metadata["Name"] == wheel["distribution"]
        and metadata["Version"] == wheel["version"]
        and tags == wheel["tags"],
        f"wheel metadata differs: {path.name}",
    )
    return re.sub(r"[-_.]+", "-", wheel["distribution"]).lower()


def _validate_lock(root: Path, document: dict) -> None:
    metadata = tomllib.loads((root / "pyproject.toml").read_text())
    uv = metadata["tool"]["uv"]
    _require(
        set(uv["default-groups"]) >= {"native", "raspi"} and uv.get("package") is False,
        "native/raspi must be default groups and source execution must be explicit",
    )
    group = metadata["dependency-groups"]["native"]
    names = {
        re.split(r"[<>=!~\[ ;]", entry, maxsplit=1)[0].lower().replace("_", "-")
        for entry in group
    }
    _require(
        names >= NATIVE_DISTRIBUTIONS, "native default group omits required wheels"
    )
    wheel_paths = {wheel["path"] for wheel in document["wheels"]}
    _require(len(wheel_paths) == len(document["wheels"]), "duplicate wheels")
    names = {_wheel_metadata(root, wheel) for wheel in document["wheels"]}
    _require(names >= NATIVE_DISTRIBUTIONS, "required native wheels missing")
    lock = tomllib.loads((root / "uv.lock").read_text())
    for package in lock["package"]:
        source = package["source"]
        if (
            source == {"virtual": "."}
            and package["name"] == metadata["project"]["name"]
        ):
            continue
        _require(
            set(source) == {"path"} and source["path"] in wheel_paths,
            f"{package['name']} is not a payload-local wheel",
        )


def _qualification(root: Path, document: dict) -> None:
    proof = document["qualification"]
    _require(isinstance(proof, dict), "native qualification is incomplete")
    _require(
        proof["source_commit"] == document["source_commit"]
        and proof["interpreter_sha256"] == document["interpreter"]["sha256"],
        "qualification source/interpreter differs",
    )
    paths = [proof[key] for key in ("smoke", "capture", "disarmed", "transport")]
    _require(
        set(paths) <= set(document["evidence"]), "qualification evidence is not hashed"
    )
    smoke, capture, disarmed, runtime = [
        json.loads((root / name).read_text()) for name in paths
    ]
    _require(
        smoke["errors"] == []
        and smoke["synthetic_tag"]["ok"] is True
        and smoke["standard_gil"] is True
        and smoke["machine"] == "aarch64"
        and smoke["python"].startswith("3.14.7 ")
        and set(smoke["module_imports"])
        >= {
            "numpy",
            "cv2",
            "libcamera",
            "pykms",
            "lgpio",
            "prctl",
            "apriltag",
            "gpiozero",
            "pidng",
            "OpenEXR",
            "picamera2",
        }
        and all(item["ok"] is True for item in smoke["module_imports"].values()),
        "native imports/tag qualification failed",
    )
    _require(
        capture["completed"] is True
        and capture["operation"] == "inspect"
        and capture["tag_servo"] is None
        and all(
            capture["safety"][key] is False
            for key in ("allow_flight", "gpio_servo_actuation_enabled", "saw_armed")
        )
        and capture["camera"]["backend"] == "native-apriltag3"
        and capture["camera"]["encoded_frames"] > 0
        and all(
            capture["components"][key]["status"] == "ok"
            for key in ("camera", "flight_controller")
        ),
        "passive native camera qualification failed",
    )
    _require(
        disarmed["status"] == "disarmed"
        and disarmed["heartbeats"] >= 2
        and 0 <= disarmed["heartbeat_age_s"] <= 2,
        "qualification lacks final fresh disarmed state",
    )
    transport = runtime["transport"]
    _require(
        runtime["python"].startswith("3.14.7 ")
        and runtime["source"] == smoke["application_source"] == proof["source_path"]
        and runtime["actuation_allowed"] is False
        and runtime["outgoing_heartbeat_allowed"] is False
        and transport["rx_bytes"] > 0
        and all(
            transport[key] == 0
            for key in (
                "heartbeat_tx_count",
                "setpoint_tx_count",
                "rx_errors",
                "tx_errors",
                "receive_errors",
                "subscriber_overflows",
                "log_queue_overflows",
            )
        ),
        "passive runtime qualification failed",
    )


def validate_payload(root: Path, *, qualified: bool = True) -> dict:
    """Bind every deployed source, wheel and evidence file to the reviewed manifest."""
    paths = _iter_sync_paths(root)
    _validate_runtime_source(root, paths)
    document = json.loads(
        (root / MANIFEST).read_text(), object_pairs_hook=_unique_object
    )
    _require(document["format"] == "ai-drone-native-v1", "unsupported manifest format")
    _require(
        bool(re.fullmatch(r"[0-9a-f]{40}", document["source_commit"])),
        "source revision missing",
    )
    files = document["files"]
    actual = {path.relative_to(root).as_posix() for path in paths if path.is_file()}
    _require(
        set(files) == actual - {MANIFEST},
        "manifest file inventory differs from payload",
    )
    for relative, digest in files.items():
        path = root / _validated_manifest_path(relative)
        _require(_hash(path) == digest, f"SHA256 differs: {relative}")
    _require(
        bool(document["evidence"]) and set(document["evidence"]) <= files.keys(),
        "qualification evidence missing",
    )
    interpreter = document["interpreter"]
    _require(
        interpreter["version"] == "3.14.7"
        and interpreter["machine"] == "aarch64"
        and interpreter["soabi"] == "cpython-314-aarch64-linux-gnu"
        and bool(interpreter["provenance"])
        and bool(re.fullmatch(r"[0-9a-f]{64}", interpreter["archive_sha256"])),
        "unqualified interpreter identity/provenance",
    )
    libraries = document["libraries"]
    _require(
        {Path(path).name for path in libraries} >= NATIVE_LIBRARIES,
        "native library inventory incomplete",
    )
    _require(
        {name.split(":", 1)[0] for name in document["packages"]} >= NATIVE_PACKAGES,
        "native package versions missing",
    )
    _validate_lock(root, document)
    if qualified:
        _qualification(root, document)
    return document


def unpacked_bytes(root: Path, document: dict) -> int:
    total = 0
    for wheel in document["wheels"]:
        with zipfile.ZipFile(root / wheel["path"]) as archive:
            total += sum(item.file_size for item in archive.infolist())
    return total


def validate_target(
    document: dict, project: Path, uv: str, *, stage: Path | None = None
) -> None:
    """Inspect installed binaries only; do not import camera/GPIO or open devices."""
    interpreter = document["interpreter"]
    executable = Path(interpreter["path"])
    _require(
        executable.is_absolute()
        and not executable.resolve().is_relative_to(project)
        and (stage is None or not executable.resolve().is_relative_to(stage)),
        "managed interpreter must be retained outside the project and upload",
    )
    _require(
        Path(interpreter["libpython"]["path"]).resolve()
        == executable.resolve().parent.parent / "lib/libpython3.14.so.1.0",
        "libpython must belong to the retained managed interpreter",
    )
    for filename, digest in {
        str(executable): interpreter["sha256"],
        interpreter["libpython"]["path"]: interpreter["libpython"]["sha256"],
        **document["libraries"],
    }.items():
        path = Path(filename)
        _require(
            path.is_absolute() and path.is_file() and _hash(path) == digest,
            f"installed binary identity differs: {filename}",
        )
    result = subprocess.run(
        [
            uv,
            "run",
            "--no-project",
            "--no-config",
            "--offline",
            "--python",
            str(executable),
            "python",
            "-I",
            "-c",
            IDENTITY,
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    expected = {key: interpreter[key] for key in ("version", "machine", "soabi")}
    _require(
        json.loads(result.stdout)
        == {**expected, "gil_disabled": False, "system_site": False},
        "interpreter ABI/GIL differs",
    )
    packages = document["packages"]
    _require(
        all(
            re.fullmatch(r"[a-z0-9][a-z0-9+.-]*(?::[a-z0-9]+)?", name)
            for name in packages
        ),
        "invalid native package name",
    )
    result = subprocess.run(
        ["dpkg-query", "-W", "-f=${binary:Package}\t${Version}\n", *packages],
        check=True,
        capture_output=True,
        text=True,
    )
    _require(
        dict(line.split("\t", 1) for line in result.stdout.splitlines()) == packages,
        "installed native package versions differ",
    )


def install(project: Path, document: dict, uv: str) -> None:
    """Install only payload-local wheels; the caller owns rollback and disarm gates."""
    environment = {
        **os.environ,
        "UV_PROJECT_ENVIRONMENT": str(project / ".venv"),
        "UV_PYTHON_DOWNLOADS": "never",
    }
    subprocess.run(
        [uv, "venv", "--clear", "--python", document["interpreter"]["path"], ".venv"],
        cwd=project,
        env=environment,
        check=True,
    )
    subprocess.run(
        [
            uv,
            "sync",
            "--project",
            str(project),
            "--locked",
            "--offline",
            "--no-build",
            "--no-install-project",
            "--python",
            ".venv/bin/python",
            "--no-dev",
            "--group",
            "raspi",
            "--group",
            "native",
        ],
        cwd=project,
        env=environment,
        check=True,
    )
    # The deployed metadata disables project builds. Keep the familiar command
    # while the service uses the same source via python -m ai_drone.cli.main.
    launcher = project / ".venv/bin/drone"
    launcher.write_text(
        f"#!{project}/.venv/bin/python\nimport sys\nsys.path.insert(0, {str(project)!r})\n"
        "from ai_drone.cli.main import main\nraise SystemExit(main())\n"
    )
    launcher.chmod(0o755)
