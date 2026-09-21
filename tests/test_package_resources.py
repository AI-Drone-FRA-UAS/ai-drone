"""Exercise installed resources and entry points outside the source checkout."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tarfile
import zipfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _run(uv: str, arguments: list[str], directory: Path, environment: dict[str, str]):
    return subprocess.run(
        [uv, *arguments],
        cwd=directory,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
        timeout=120,
    )


@pytest.fixture(scope="module")
def installed_package(tmp_path_factory):
    uv = shutil.which("uv")
    assert uv is not None
    directory = tmp_path_factory.mktemp("installed-package")
    environment = {
        key: value for key, value in os.environ.items() if key != "VIRTUAL_ENV"
    }
    environment["UV_PROJECT_ENVIRONMENT"] = str(directory / "environment")
    distributions = directory / "dist"
    _run(uv, ["build", "--offline", "--out-dir", str(distributions)], ROOT, environment)
    _run(
        uv,
        [
            "sync",
            "--locked",
            "--offline",
            "--no-editable",
            "--no-default-groups",
            "--python",
            sys.executable,
        ],
        ROOT,
        environment,
    )
    binaries = directory / "environment" / ("Scripts" if os.name == "nt" else "bin")
    interpreter = binaries / ("python.exe" if os.name == "nt" else "python")
    command = [
        "run",
        "--no-project",
        "--no-config",
        "--offline",
        "--python",
        str(interpreter),
    ]
    return uv, directory, environment, distributions, binaries, command


def test_wheel_and_sdist_include_report_template(installed_package):
    _, _, _, distributions, _, _ = installed_package
    (wheel,) = distributions.glob("*.whl")
    with zipfile.ZipFile(wheel) as archive:
        template = archive.read("ai_drone/review/report.html")
    (source,) = distributions.glob("*.tar.gz")
    with tarfile.open(source) as archive:
        (member,) = [
            item
            for item in archive
            if item.name.endswith("/ai_drone/review/report.html")
        ]
        reader = archive.extractfile(member)
        assert reader is not None
        assert reader.read() == template
    assert b"__REPORT_DATA__" in template


def test_installed_console_script_and_report_work_without_checkout(installed_package):
    uv, directory, environment, _, binaries, command = installed_package
    drone = binaries / ("drone.exe" if os.name == "nt" else "drone")
    result = _run(uv, [*command, str(drone), "--help"], directory, environment)
    assert "power" in result.stdout and "control" in result.stdout
    recording = directory / "recording"
    recording.mkdir()
    (recording / "manifest.json").write_text(json.dumps({"completed": True}))
    (recording / "telemetry.jsonl").write_text("")
    script = """
import json, sys
from pathlib import Path
import ai_drone
from ai_drone.cli.report import build_report
from ai_drone.review.html import render_report
assert Path(ai_drone.__file__).is_relative_to(Path(sys.prefix))
recording = Path(sys.argv[1])
report = build_report(recording, video=False)
assert report.is_file()
summary = json.loads(report.with_name('summary.json').read_text())
assert all((report.parent / item['href']).is_file() for item in summary['files'])
rendered = render_report({'title': '</script><script>alert(1)</script>', 'files': [
    {'label': 'CSV', 'href': 'camera.csv'}, {'label': 'bad', 'href': 'javascript:alert(1)'}]})
assert '<script>alert(1)</script>' not in rendered
assert 'javascript:alert(1)' not in rendered
assert 'camera.csv' in rendered
print(report)
"""
    result = _run(
        uv,
        [*command, "python", "-I", "-c", script, str(recording)],
        directory,
        environment,
    )
    assert str(recording / "review/index.html") in result.stdout


def test_runtime_bundle_includes_template_and_restoration_helpers():
    from ai_drone.link.deploy import _iter_sync_paths

    names = {path.relative_to(ROOT).as_posix() for path in _iter_sync_paths(ROOT)}
    assert {
        "ai_drone/review/report.html",
        "ai_drone/cli/deploy.py",
        "scripts/setup_runtime.py",
        "scripts/power.py",
        "scripts/verify_ardupilot_firmware.py",
        "firmware/FlywooF745-nogps-loiter.manifest.json",
        "firmware/FlywooF745-nogps-loiter-extra.hwdef",
        "uv.lock",
    } <= names
    assert "drone.toml" not in names
