from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import time
from contextlib import closing, suppress
from pathlib import Path

import pytest
from pymavlink import mavutil

from ai_drone.cli import control
from ai_drone.cli import record as inspect_cli
from ai_drone.mavlink.safety import heartbeat_is_armed

ARDUPILOT_TAG = "Copter-4.7.0"
PARAMETERS = Path(__file__).parent / "sitl" / "copter.parm"

pytestmark = pytest.mark.sitl


def _ardupilot_root() -> Path:
    value = os.environ.get("ARDUPILOT_ROOT")
    if not value:
        pytest.skip("set ARDUPILOT_ROOT to an external Copter-4.7.0 checkout")
    root = Path(value).expanduser().resolve()
    binary = root / "build" / "sitl" / "bin" / "arducopter"
    if not binary.is_file():
        pytest.fail(
            "build ArduCopter SITL in the external checkout before running this test"
        )
    head = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True
    ).strip()
    tag = subprocess.check_output(
        ["git", "-C", str(root), "rev-list", "-n", "1", ARDUPILOT_TAG],
        text=True,
    ).strip()
    if head != tag:
        pytest.fail(f"ARDUPILOT_ROOT must be checked out at {ARDUPILOT_TAG}")
    return root


def _wait_for_tcp(process: subprocess.Popen[bytes], timeout: float = 30.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            pytest.fail(f"SITL exited early with status {process.returncode}")
        with (
            suppress(OSError),
            closing(socket.create_connection(("127.0.0.1", 5760), timeout=0.25)),
        ):
            return
        time.sleep(0.25)
    pytest.fail("SITL did not listen on TCP port 5760 within 30 seconds")


def _stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def test_inspect_then_hover_and_land_in_pinned_sitl(tmp_path, monkeypatch) -> None:
    root = _ardupilot_root()
    with closing(socket.socket()) as probe:
        if probe.connect_ex(("127.0.0.1", 5760)) == 0:
            pytest.skip("TCP port 5760 is already in use")

    log_path = tmp_path / "sitl.log"
    binary = root / "build" / "sitl" / "bin" / "arducopter"
    defaults = root / "Tools" / "autotest" / "default_params" / "copter.parm"
    command = [
        str(binary),
        "-S",
        "-I0",
        "-w",
        "--home",
        "-35.363261,149.165230,584,353",
        "--model",
        "+",
        "--speedup",
        "1",
        "--defaults",
        f"{defaults},{PARAMETERS.resolve()}",
    ]
    monkeypatch.chdir(tmp_path)
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            command,
            cwd=tmp_path,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            _wait_for_tcp(process)
            inspection = tmp_path / "inspection"
            assert (
                inspect_cli.run(
                    [
                        "--device",
                        "tcp:127.0.0.1:5760",
                        "--duration",
                        "5",
                        "--output-dir",
                        str(inspection),
                    ]
                )
                == 0
            )
            manifest = json.loads((inspection / "manifest.json").read_text())
            assert manifest["components"]["camera"]["status"] == "unavailable"
            assert manifest["components"]["flight_controller"]["status"] == "ok"
            assert manifest["components"]["downward_rangefinder"]["status"] == "ok"
            assert manifest["components"]["optical_flow"]["status"] == "ok"

            assert (
                control.main(
                    [
                        "hover",
                        "--device",
                        "tcp:127.0.0.1:5760",
                        "--takeoff-alt",
                        "0.4",
                        "--max-alt",
                        "0.6",
                        "--duration",
                        "2",
                        "--min-battery",
                        "0",
                        "--confirm-flight",
                        control.FLIGHT_CONFIRMATION,
                    ]
                )
                == 0
            )
            connection = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
            try:
                heartbeat = connection.wait_heartbeat(timeout=10)
                assert heartbeat is not None
                assert not heartbeat_is_armed(heartbeat)
            finally:
                connection.close()
            flight_manifests = list(
                (tmp_path / "artifacts" / "flights").glob("*/manifest.json")
            )
            assert len(flight_manifests) == 1
            assert json.loads(flight_manifests[0].read_text())["completed"] is True
        finally:
            _stop_process(process)
