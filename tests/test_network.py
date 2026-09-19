from __future__ import annotations

import json
import shlex
import subprocess

import pytest

from ai_drone.settings import RuntimeSettings, Settings
from scripts import network


@pytest.fixture
def local(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(network, "is_raspberry_pi", lambda: True)
    monkeypatch.setattr(
        network,
        "load_settings",
        lambda: Settings(runtime=RuntimeSettings(socket="/run/custom/vehicle.sock")),
    )
    for key in ("PI_HOST", "PI_USER", "PI_DIR", "SSH_CONFIG", "AI_DRONE_CONFIG"):
        monkeypatch.delenv(key, raising=False)
    commands = []

    def run(command, **_kwargs):
        commands.append(command)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(network.subprocess, "run", run)
    return commands


@pytest.mark.parametrize(
    "action", [["hotspot", "on"], ["hotspot", "off"], ["connect", "eduroam"]]
)
def test_explicit_switch_is_queued_in_shared_runtime_without_a_serial_reader(
    local, monkeypatch, capsys, action
):
    requests = []

    def request(path, payload):
        requests.append((path, payload))
        return {"queued": payload["network"]}

    monkeypatch.setattr(network, "runtime_request", request)
    assert network.main(action) == 0
    assert requests == [("/run/custom/vehicle.sock", {"network": action})]
    assert json.loads(capsys.readouterr().out) == {"queued": action}
    assert not local


def test_local_status_reads_shared_fc_and_network_state(local, monkeypatch, capsys):
    requests = []
    state = {"armed": False, "wifi_connected": True, "wifi_profile": "uuid"}
    monkeypatch.setattr(
        network,
        "runtime_request",
        lambda path, payload: requests.append((path, payload)) or state,
    )
    assert network.main(["status"]) == 0
    assert requests == [("/run/custom/vehicle.sock", {"status": True})]
    assert json.loads(capsys.readouterr().out) == state
    assert not local


def test_list_only_reads_saved_profiles_and_never_needs_uart_or_runtime(
    local, monkeypatch
):
    monkeypatch.setattr(
        network, "runtime_request", lambda *_args: pytest.fail("runtime opened")
    )
    assert network.main(["list"]) == 0
    assert local == [
        [
            "nmcli",
            "-f",
            "NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY",
            "connection",
            "show",
        ]
    ]


@pytest.mark.parametrize(
    "error",
    [
        RuntimeError("armed"),
        FileNotFoundError("missing runtime"),
        ConnectionError("closed"),
    ],
)
@pytest.mark.parametrize(
    "action", [["status"], ["hotspot", "on"], ["connect", "eduroam"]]
)
def test_runtime_failure_does_not_fall_back_to_uart_ssh_or_network_mutation(
    local, monkeypatch, capsys, error, action
):
    def fail(*_args):
        raise error

    monkeypatch.setattr(network, "runtime_request", fail)
    assert network.main(action) == 1
    assert "Network request failed" in capsys.readouterr().err
    assert not local


def test_laptop_wrapper_preserves_unusual_profile_as_one_remote_argument(
    local, monkeypatch
):
    monkeypatch.setattr(network, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        network, "runtime_request", lambda *_args: pytest.fail("local runtime opened")
    )
    profile = "wifi; echo unwanted"
    assert network.main(["--host", "seb@192.168.4.1", "connect", profile]) == 0
    (command,) = local
    assert command[:2] == ["ssh", "seb@192.168.4.1"]
    assert shlex.split(command[-1])[-2:] == ["connect", profile]
    assert "scripts/network.py" in command[-1]


def test_explicit_remote_route_on_the_pi_uses_ssh_and_quotes_project_directory(
    local, monkeypatch
):
    directory = "/home/pilot/project with spaces; $(false)"
    monkeypatch.setenv("PI_DIR", directory)
    monkeypatch.setattr(
        network, "runtime_request", lambda *_args: pytest.fail("local runtime opened")
    )
    assert network.main(["--host", "pilot@drone", "hotspot", "off"]) == 0
    (command,) = local
    assert command[:2] == ["ssh", "pilot@drone"]
    assert shlex.split(command[-1])[:3] == ["cd", directory, "&&"]
    assert shlex.split(command[-1])[-2:] == ["hotspot", "off"]


def test_ssh_failure_is_returned_without_trying_other_routes(local, monkeypatch):
    monkeypatch.setattr(network, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(
        network.subprocess,
        "run",
        lambda command, **_kwargs: (
            local.append(command) or subprocess.CompletedProcess(command, 255)
        ),
    )
    assert network.main(["--host", "pilot@unreachable", "status"]) == 255
    assert len(local) == 1
