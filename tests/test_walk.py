from __future__ import annotations

import io
import shlex
import signal
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from ai_drone.cli import walk


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    root = tmp_path / "Pi runtime"
    root.mkdir()
    python = root / ".venv/bin/python"
    python.parent.mkdir(parents=True)
    python.symlink_to(Path(walk.sys.executable))
    monkeypatch.setattr(walk, "_runtime_root", lambda: root)
    monkeypatch.setattr(walk.sys, "executable", str(python))
    monkeypatch.setattr(walk, "_user_ids", lambda: (1000, 1000))
    monkeypatch.setenv("AI_DRONE_CONFIG", walk.os.devnull)
    return root, python


@pytest.mark.parametrize("no_video", [False, True])
def test_dry_run_is_read_only_and_preserves_venv_interpreter(
    runtime, monkeypatch, capsys, no_video
):
    root, python = runtime
    monkeypatch.setattr(
        walk.subprocess, "run", lambda *_args, **_kwargs: pytest.fail("process started")
    )
    monkeypatch.setattr(
        walk, "_validate_runtime", lambda *_args: pytest.fail("hardware checked")
    )

    assert (
        walk.main(
            ["--dry-run", "--duration", "180", *(["--no-video"] if no_video else [])]
        )
        == 0
    )

    lines = capsys.readouterr().out.splitlines()
    command = shlex.split(lines[0])
    assert command[:3] == ["sudo", "-n", "systemd-run"]
    assert str(python) in command
    assert "--duration" in command and "180.0" in command
    assert ("--no-video" in command) is no_video
    assert "--unit=ai-drone-walk.service" in command
    assert "--expand-environment=no" in command
    assert "--property=KillSignal=SIGINT" in command
    assert "--property=KillMode=mixed" in command
    assert "--property=TimeoutStopSec=180" in command
    assert "--property=Restart=no" in command
    assert "--uid=1000" in command and "--gid=1000" in command
    assert not {"--scope", "--pty", "--pipe", "--wait", "enable"}.intersection(command)
    assert not (root / "artifacts").exists()
    assert any(line.startswith("Status: ") for line in lines)
    assert any(line.startswith("Stop and finalize: ") for line in lines)


def test_output_defaults_are_unique_and_not_reserved(tmp_path):
    first = walk._output_path(None, tmp_path)
    second = walk._output_path(None, tmp_path)
    assert first != second
    assert first.parent == tmp_path / "artifacts"
    assert first.name.startswith("walk-")
    assert not first.parent.exists()


@pytest.mark.parametrize("duration", ["0", "-1", "nan", "inf", "-inf"])
def test_invalid_duration_cannot_start_a_job(runtime, monkeypatch, duration):
    monkeypatch.setattr(
        walk.subprocess, "run", lambda *_args, **_kwargs: pytest.fail("process started")
    )
    assert walk.main([f"--duration={duration}"]) == 1


@pytest.mark.parametrize("kind", ["directory", "file", "broken-symlink"])
def test_existing_output_is_rejected_before_launch(runtime, monkeypatch, kind):
    root, _python = runtime
    output = root / "existing"
    if kind == "directory":
        output.mkdir()
    elif kind == "file":
        output.write_text("keep me")
    else:
        output.symlink_to(root / "absent")
    monkeypatch.setattr(
        walk.subprocess, "run", lambda *_args, **_kwargs: pytest.fail("process started")
    )
    assert walk.main(["--output-dir", str(output)]) == 1
    assert walk.os.path.lexists(output)


def test_root_invocation_is_rejected(monkeypatch):
    monkeypatch.setattr(walk.os, "getuid", lambda: 0)
    with pytest.raises(ValueError, match="without sudo"):
        walk._user_ids()


def test_live_launch_requires_pi_and_complete_runtime(tmp_path, monkeypatch):
    monkeypatch.setattr(walk, "is_raspberry_pi", lambda: False)
    with pytest.raises(ValueError, match="Raspberry Pi"):
        walk._validate_runtime(tmp_path, Path(walk.sys.executable))
    monkeypatch.setattr(walk, "is_raspberry_pi", lambda: True)
    with pytest.raises(ValueError, match=r"record\.py"):
        walk._validate_runtime(tmp_path, Path(walk.sys.executable))


@pytest.mark.parametrize("state", ["active", "activating", "deactivating", "failed"])
def test_existing_unit_is_never_stopped_or_replaced(runtime, monkeypatch, state):
    calls = []
    monkeypatch.setattr(walk, "_validate_runtime", lambda *_args: None)

    def execute(command, **_kwargs):
        calls.append(command)
        return SimpleNamespace(
            returncode=0, stdout=f"LoadState=loaded\nActiveState={state}\n", stderr=""
        )

    monkeypatch.setattr(walk.subprocess, "run", execute)
    assert walk.main([]) == 1
    assert len(calls) == 1 and calls[0][:2] == ["systemctl", "show"]


def test_inaccessible_system_manager_fails_closed(runtime, monkeypatch):
    monkeypatch.setattr(walk, "_validate_runtime", lambda *_args: None)
    monkeypatch.setattr(
        walk.subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=1, stdout="", stderr="Failed to connect to bus"
        ),
    )
    assert walk.main([]) == 1


@pytest.mark.parametrize("raced", [False, True])
def test_launch_uses_argv_and_handles_an_atomic_unit_collision(
    runtime, monkeypatch, raced
):
    root, _python = runtime
    output = root / "walk ; $(do-not-run)"
    calls = []
    monkeypatch.setattr(walk, "_validate_runtime", lambda *_args: None)

    def execute(command, **kwargs):
        assert "shell" not in kwargs
        calls.append(command)
        if command[0] == "systemctl":
            return SimpleNamespace(
                returncode=1, stdout="LoadState=not-found\n", stderr=""
            )
        if raced:
            raise subprocess.CalledProcessError(1, command)
        assert command[-1] == str(output)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(walk.subprocess, "run", execute)
    assert walk.main(["--output-dir", str(output)]) == (1 if raced else 0)
    assert len(calls) == 2
    assert all(
        "stop" not in command and "reset-failed" not in command for command in calls
    )
    assert not output.exists()


class _Child:
    def __init__(self, output: str, result: int = 0):
        self.stdout = io.StringIO(output)
        self.result = result
        self.signals = []

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.stdout.close()

    def wait(self):
        return self.result

    def poll(self):
        return None

    def send_signal(self, signum):
        self.signals.append(signum)


@pytest.mark.parametrize(
    ("capture_code", "report_code"), [(0, 0), (1, 0), (3, 0), (0, 2)]
)
@pytest.mark.parametrize("no_video", [False, True])
def test_worker_reports_actual_capture_path_and_retains_failures(
    runtime, monkeypatch, capture_code, report_code, no_video
):
    root, python = runtime
    requested = root / "walk"
    actual = root / "walk-1"
    calls = []
    monkeypatch.setattr(walk, "_validate_runtime", lambda *_args: None)

    def spawn(command, **kwargs):
        assert kwargs["cwd"] == root and kwargs["stdin"] == subprocess.DEVNULL
        assert "shell" not in kwargs
        calls.append(command)
        if len(calls) == 1:
            assert not requested.exists()
            assert command[1:6] == [
                "run",
                "--no-sync",
                "--python",
                str(python),
                "python",
            ]
            assert command[6:8] == ["-m", "ai_drone.cli.record"]
            assert (
                command[command.index("--device") + 1]
                == "unix:/run/ai-drone/vehicle.sock"
            )
            assert command[command.index("--baud") + 1] == "115200"
            assert command[command.index("--backend") + 1] == "native"
            assert "--confirm-manual-flight-recording" not in command
            assert ("--no-video" in command) is no_video
            return _Child(
                f"Finished\nManifest: {actual / 'manifest.json'}\n", capture_code
            )
        assert command[1:] == [
            "run",
            "--no-sync",
            "--python",
            str(python),
            "python",
            "-m",
            "ai_drone.cli.report",
            str(actual),
        ]
        return _Child("Report written\n", report_code)

    monkeypatch.setattr(walk.subprocess, "Popen", spawn)
    launch = walk._launch_command(
        root,
        python,
        1000,
        1000,
        [
            "--duration",
            "300",
            "--output-dir",
            str(requested),
            *(["--no-video"] if no_video else []),
        ],
        tag_servo=False,
    )
    assert walk.main(launch[launch.index("--worker") :]) == (
        capture_code or report_code
    )
    assert len(calls) == 2


def test_worker_does_not_guess_dataset_after_a_crash(runtime, monkeypatch):
    root, python = runtime
    calls = []

    def spawn(command, **_kwargs):
        calls.append(command)
        return _Child("Capture crashed before its manifest\n", 1)

    monkeypatch.setattr(walk.subprocess, "Popen", spawn)
    assert walk._run_worker(root, python, root / "walk", []) == 1
    assert len(calls) == 1


def test_worker_forwards_stop_to_capture_and_still_reports(runtime, monkeypatch):
    root, python = runtime
    output = root / "walk"
    handlers = {}
    capture = _Child("")
    report = _Child("Report written\n")
    children = iter([capture, report])
    monkeypatch.setattr(walk.signal, "getsignal", lambda _signum: signal.SIG_DFL)
    monkeypatch.setattr(
        walk.signal,
        "signal",
        lambda signum, handler: handlers.__setitem__(signum, handler),
    )

    class Output(io.StringIO):
        def __iter__(self):
            handlers[signal.SIGINT](signal.SIGINT, None)
            return super().__iter__()

    capture.stdout = Output(f"Manifest: {output / 'manifest.json'}\n")
    monkeypatch.setattr(
        walk.subprocess, "Popen", lambda *_args, **_kwargs: next(children)
    )
    assert walk._run_worker(root, python, output, []) == 0
    assert capture.signals == [signal.SIGINT]
    assert report.signals == []
    assert handlers == {signal.SIGINT: signal.SIG_DFL, signal.SIGTERM: signal.SIG_DFL}


@pytest.mark.parametrize("name", ["elsewhere", "walk-0", "walk--1"])
def test_manifest_handoff_rejects_unrelated_paths(tmp_path, name):
    assert (
        walk._reported_dataset(
            f"Manifest: {tmp_path / name / 'manifest.json'}", tmp_path / "walk"
        )
        is None
    )


def test_worker_rechecks_output_without_touching_existing_data(runtime, monkeypatch):
    root, python = runtime
    output = root / "walk"
    output.mkdir()
    monkeypatch.setattr(
        walk.subprocess,
        "Popen",
        lambda *_args, **_kwargs: pytest.fail("capture started"),
    )
    with pytest.raises(ValueError, match="already exists"):
        walk._run_worker(root, python, output, [])


def test_detached_recording_freezes_calibration_and_storage_settings(
    runtime, tmp_path, monkeypatch, capsys
):
    root, _python = runtime
    settings = tmp_path / "drone.toml"
    settings.write_text(
        "[recording]\n"
        "storage_reserve_mib = 128\n"
        "storage_warning_mib = 512\n"
        "storage_stop_mib = 8\n"
        "storage_check_interval = 3\n"
    )
    monkeypatch.setenv("AI_DRONE_CONFIG", str(settings))
    monkeypatch.chdir(tmp_path)
    assert (
        walk.main(
            [
                "--dry-run",
                "--calibration",
                "camera calibration.json",
                "--tag-size",
                "0.224",
                "--storage-reserve-mib",
                "256",
                "--no-video",
            ]
        )
        == 0
    )
    launch = shlex.split(capsys.readouterr().out.splitlines()[0])
    assert "--setenv=AI_DRONE_CONFIG=/dev/null" in launch

    # The service starts from another cwd after the original configuration changes.
    settings.write_text("invalid configuration")
    monkeypatch.setenv("AI_DRONE_CONFIG", "/dev/null")
    monkeypatch.chdir(root)
    monkeypatch.setattr(walk, "_validate_runtime", lambda *_args: None)
    captured = []

    def run_worker(_root, _python, _output, arguments, **_kwargs):
        captured.extend(arguments)
        return 0

    monkeypatch.setattr(walk, "_run_worker", run_worker)
    assert walk.main(launch[launch.index("--worker") :]) == 0
    expected = {
        "--calibration": str(tmp_path / "camera calibration.json"),
        "--tag-size": "0.224",
        "--storage-reserve-mib": "256.0",
        "--storage-warning-mib": "512.0",
        "--storage-stop-mib": "8.0",
        "--storage-check-interval": "3.0",
    }
    for option, value in expected.items():
        assert captured[captured.index(option) + 1] == value
    assert "--no-video" in captured
    assert "--allow-flight" not in captured


SERVO_OPTIONS = [
    "--tag-servo",
    "--tag-range",
    "3:5",
    "--active-us",
    "1500",
    "--rest-us",
    "900",
    "--pulse-duration",
    "0.5",
    "--confirm-actuation",
    "SERVO_CLEAR",
    "--confirm-armed-flight",
    "ARMED_FLIGHT_TAG_SERVO_CLEAR",
]


@pytest.mark.parametrize("servo", [False, True])
def test_explicit_flight_recording_keeps_its_gates_through_detach(
    runtime, monkeypatch, capsys, servo
):
    options = SERVO_OPTIONS if servo else ["--allow-flight"]
    assert walk.main(["--dry-run", "--no-video", *options]) == 0
    launch = shlex.split(capsys.readouterr().out.splitlines()[0])
    output = Path(launch[launch.index("--output-dir") + 1])
    calls = []
    monkeypatch.setattr(walk, "_validate_runtime", lambda *_args: None)

    def spawn(command, **_kwargs):
        calls.append(command)
        if len(calls) == 1:
            return _Child(f"Manifest: {output / 'manifest.json'}\n")
        return _Child("Report written\n")

    monkeypatch.setattr(walk.subprocess, "Popen", spawn)
    assert walk.main(launch[launch.index("--worker") :]) == 0
    capture = calls[0]
    module = capture[capture.index("-m") + 1]
    assert module == (
        "ai_drone.cli.tag_servo_record" if servo else "ai_drone.cli.record"
    )
    assert "--no-video" in capture
    assert "ai_drone.cli.report" in calls[1]
    if servo:
        ids = [
            capture[index + 1]
            for index, value in enumerate(capture)
            if value == "--tag-id"
        ]
        assert ids == ["3", "4", "5"]
        assert "--tag-range" not in capture
        assert "SERVO_CLEAR" in capture and "ARMED_FLIGHT_TAG_SERVO_CLEAR" in capture
        assert capture[capture.index("--active-us") + 1] == "1500"
        assert capture[capture.index("--rest-us") + 1] == "900"
        assert capture[capture.index("--backend") + 1] == "native"
    else:
        assert "--allow-flight" in capture
        assert "--confirm-actuation" not in capture


@pytest.mark.parametrize(
    "options",
    [
        ["--tag-servo"],
        ["--confirm-manual-flight-recording", "yes"],
        [*SERVO_OPTIONS, "--backend", "opencv"],
    ],
)
def test_unsafe_recording_options_fail_before_launch(runtime, monkeypatch, options):
    monkeypatch.setattr(
        walk.subprocess, "run", lambda *_args, **_kwargs: pytest.fail("job started")
    )
    with pytest.raises(SystemExit) as error:
        walk.main(["--dry-run", *options])
    assert error.value.code == 2
