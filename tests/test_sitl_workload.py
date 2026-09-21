"""Full host recording workload beside actual isolated runtime/flight control."""

from __future__ import annotations

import json
import signal
import subprocess
import time
from contextlib import contextmanager
from pathlib import Path

import pytest

from ai_drone.settings import load_settings
from tests.sitl_workload_camera import require_codecs
from tests.test_sitl import (
    _ardupilot_root,
    _assert_running_sitl_configuration,
    _child_environment,
    _production_hover_arguments,
    _python_command,
    _running_cli,
    _running_sitl,
    _stop_process,
    _wait_for_cli,
)
from tests.test_sitl import operator_presence as operator_presence
from tests.test_sitl_profiles import _save_evidence
from tests.test_sitl_workload_camera import video_frame_count

pytestmark = pytest.mark.sitl


def _json_rows(path: Path):
    if not path.is_file():
        return []
    text = path.read_text()
    lines = text.splitlines()
    if not text.endswith("\n"):
        lines = lines[:-1]
    return [json.loads(line) for line in lines]


@contextmanager
def _camera_recorder(tmp_path: Path, endpoint: str):
    output = tmp_path / "generated-camera.log"
    command = _python_command(
        "tests.sitl_workload_camera",
        str(tmp_path / "low-space"),
        str(tmp_path / "camera-resources.json"),
        "record",
        "--device",
        endpoint,
        "--allow-flight",
        "--backend",
        "opencv",
        "--threads",
        "2",
        "--fps",
        "12",
        "--warmup",
        "0",
        "--duration",
        "90",
        "--storage-check-interval",
        "0.2",
        "--resolution",
        "1280x960",
        "--analysis-resolution",
        "640x480",
        "--output-dir",
        str(tmp_path / "camera"),
    )
    with output.open("w") as handle:
        process = subprocess.Popen(
            command,
            cwd=tmp_path,
            env=_child_environment(),
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            yield process, output
        finally:
            _stop_process(process)


def _recorder_pid(process):
    # The tests-only launcher invokes the unchanged dispatcher inside this child.
    pending = [process.pid]
    while pending:
        pid = pending.pop()
        try:
            arguments = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
            if (
                arguments
                and b"python" in Path(arguments[0].decode()).name.encode()
                and b"tests.sitl_workload_camera" in arguments
            ):
                return pid
            for task in Path(f"/proc/{pid}/task").glob("*/children"):
                pending.extend(int(value) for value in task.read_text().split())
        except FileNotFoundError:
            continue
    raise AssertionError("synthetic recorder child is not running")


def test_camera_video_tags_and_storage_pressure_do_not_starve_control(
    tmp_path, monkeypatch
):
    import os

    require_codecs()  # Missing native dependencies fail this required gate.
    root = _ardupilot_root()
    monkeypatch.chdir(tmp_path)
    settings = load_settings()
    status_path = Path(settings.runtime.status)
    endpoint = f"unix:{settings.runtime.socket}"
    dataset = tmp_path / "camera"
    snapshots = []

    def status():
        value = json.loads(status_path.read_text())
        snapshots.append(value)
        return value

    def runtime_ready():
        if not status_path.is_file():
            return False
        current = status()
        return (
            current["fresh"] and current["armed"] is False and current["operator_alive"]
        )

    with _running_sitl(root, tmp_path) as sensors:
        try:
            _assert_running_sitl_configuration(sensors)
            sensors.reset_observations()
            with _running_cli(
                tmp_path, "runtime", "runtime", "serve", "--no-network"
            ) as (runtime, runtime_log):
                _wait_for_cli(runtime_ready, runtime, runtime_log)
                with _camera_recorder(tmp_path, endpoint) as (recorder, recorder_log):
                    _wait_for_cli(
                        lambda: (
                            len(_json_rows(dataset / "camera.jsonl")) >= 12
                            and any(
                                row["tags"]
                                for row in _json_rows(dataset / "camera.jsonl")
                            )
                        ),
                        recorder,
                        recorder_log,
                        timeout=20,
                    )
                    arguments = _production_hover_arguments(10)
                    arguments[arguments.index("--device") + 1] = endpoint
                    arguments.extend(
                        ["--runtime-status", str(status_path), "--foreground"]
                    )
                    with _running_cli(tmp_path, "control", "control", *arguments) as (
                        control,
                        control_log,
                    ):
                        sensors.wait_for_mode("LOITER", timeout=60, process=control)
                        # Keep actual H.264/detection/disk load concurrent with Loiter.
                        time.sleep(2)
                        status()
                        pressure_at = time.monotonic()
                        before = len(_json_rows(dataset / "camera.jsonl"))
                        (tmp_path / "low-space").write_text(
                            "simulated free space 128 MiB\n"
                        )
                        while control.poll() is None:
                            assert recorder.poll() is None, recorder_log.read_text()
                            assert runtime.poll() is None, runtime_log.read_text()
                            status()
                            assert time.monotonic() - pressure_at < 60, (
                                control_log.read_text()
                            )
                            time.sleep(0.1)
                        assert control.returncode == 0, control_log.read_text()
                        sensors.wait_for_disarm(timeout=10)
                        flight = sensors.assert_flight_result()
                        assert flight["max_horizontal_drift_m"] <= 0.5
                        final_transport = status()["transport"]
                        assert len(_json_rows(dataset / "camera.jsonl")) > before + 12
                        os.kill(_recorder_pid(recorder), signal.SIGINT)
                        assert recorder.wait(timeout=15) == 1, recorder_log.read_text()
                manifest = json.loads((dataset / "manifest.json").read_text())
                assert manifest["components"]["camera"]["status"] == "ok"
                assert manifest["components"]["apriltags"]["detections"] > 30
                assert manifest["storage"]["video_stopped_reason"] == "storage_reserve"
                assert manifest["safety"]["gpio_servo_actuation_enabled"] is False
                assert manifest["tag_servo"] is None
                assert manifest["error"] == "interrupted by user"
                assert (
                    video_frame_count(dataset / "camera.h264")
                    == manifest["camera"]["encoded_frames"]
                )
                assert final_transport["supported"] is True
                assert (
                    final_transport["rx_bytes"] > 0 and final_transport["tx_bytes"] > 0
                )
                assert final_transport["setpoint_tx_count"] > 50
                assert final_transport["heartbeat_tx_count"] > 10
                assert final_transport["setpoint_tx_gap_max_s"] is not None
                assert final_transport["heartbeat_tx_gap_max_s"] is not None
                assert final_transport["subscriber_overflows"] == 0
                assert final_transport["log_queue_overflows"] == 0
                assert final_transport["receive_errors"] == 0
                assert final_transport["rx_errors"] == 0
                assert final_transport["tx_errors"] == 0
                delivery = manifest["telemetry"]["delivery"]
                assert delivery["known_packet_bytes"] > 0
                assert delivery["maximum_delivery_lag_s"] is not None
                resources = json.loads((tmp_path / "camera-resources.json").read_text())
                assert resources["actuation_enabled"] is False
                (tmp_path / "workload-summary.json").write_text(
                    json.dumps(
                        {
                            "scope": "isolated host generated camera, real OpenCV/CPU H264; TCP bytes are not Pi UART measurements",
                            "resources": resources,
                            "transport": final_transport,
                            "delivery": delivery,
                            "storage_pressure_monotonic": pressure_at,
                            "flight": flight,
                            "camera_frames": manifest["components"]["camera"]["frames"],
                            "encoded_frames": manifest["camera"]["encoded_frames"],
                            "tag_detections": manifest["components"]["apriltags"][
                                "detections"
                            ],
                        },
                        indent=2,
                    )
                )
        finally:
            (tmp_path / "runtime-snapshots.json").write_text(json.dumps(snapshots))
            _save_evidence(tmp_path, sensors, {"workload": "test-generated-camera"})
