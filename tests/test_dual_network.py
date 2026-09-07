from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parents[1] / "scripts/setup-pi-dual-network.sh"
ROOT_GUARD = '[[ $EUID -eq 0 ]] || die "--apply must be run as root"'

FAKE_COMMAND = r"""
import json
import os
import sys
from pathlib import Path

name = Path(sys.argv[0]).name
args = sys.argv[1:]
with Path(os.environ["TEST_COMMAND_LOG"]).open("a") as stream:
    stream.write(json.dumps([name, *args]) + "\n")
if name == "uname" and args == ["-s"]:
    print("Linux")
elif name == "nmcli":
    if args[:4] == ["-g", "connection.id", "connection", "show"]:
        if args[4] == "eduroam-uplink":
            sys.exit(1)
        print(args[4])
    elif args[:4] == ["-g", "GENERAL.TYPE", "device", "show"]:
        print("wifi")
    elif args[:2] in (["connection", "clone"], ["connection", "modify"], ["connection", "up"]):
        pass
    elif args == ["-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"]:
        print("wlan0 wifi connected Hotspot")
    else:
        raise RuntimeError(f"Unexpected fake nmcli command: {args!r}")
elif name == "systemctl":
    if args == ["show", "ai-drone-network.service", "--property=LoadState", "--value"]:
        print(os.environ["TEST_SELECTOR_STATE"])
        sys.exit(int(os.environ["TEST_SELECTOR_INSPECT_RC"]))
    elif args == ["disable", "--now", "ai-drone-network.service"]:
        sys.exit(int(os.environ["TEST_SELECTOR_DISABLE_RC"]))
    else:
        raise RuntimeError(f"Unexpected fake systemctl command: {args!r}")
elif name == "ip" and args == ["-4", "route"]:
    print("default via 192.0.2.1 dev wlan1")
else:
    raise RuntimeError(f"Unexpected fake command: {name!r} {args!r}")
"""


def _run_script(
    tmp_path,
    arguments=(),
    *,
    selector_state="loaded",
    inspect_returncode=0,
    disable_returncode=0,
):
    original = SCRIPT.read_text()
    assert original.count(ROOT_GUARD) == 1
    temporary_script = tmp_path / "dual-network-test-copy.sh"
    temporary_script.write_text(
        original.replace(ROOT_GUARD, ": # Root check bypassed only in this test copy")
    )
    commands = tmp_path / "fake-bin"
    commands.mkdir()
    for name in ("nmcli", "systemctl", "ip", "uname"):
        path = commands / name
        path.write_text(f"#!{sys.executable}\n{FAKE_COMMAND}")
        path.chmod(0o700)
    grep = shutil.which("grep")
    assert grep is not None
    (commands / "grep").symlink_to(grep)
    log = tmp_path / "commands.jsonl"
    environment = os.environ.copy()
    environment.update(
        PATH=str(commands),
        TEST_COMMAND_LOG=str(log),
        TEST_SELECTOR_STATE=selector_state,
        TEST_SELECTOR_INSPECT_RC=str(inspect_returncode),
        TEST_SELECTOR_DISABLE_RC=str(disable_returncode),
    )
    # An isolated PATH exposes only fake networking tools and real text-only grep.
    completed = subprocess.run(
        ["/bin/bash", str(temporary_script), *arguments],
        env=environment,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert SCRIPT.read_text() == original
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    return completed, calls


def _network_mutations(calls):
    return [
        call
        for call in calls
        if call[:2] == ["nmcli", "connection"] and call[2] in {"clone", "modify", "up"}
    ]


@pytest.mark.parametrize(
    "arguments",
    [
        ("--source-profile", "Hotspot"),
        ("--uplink-profile", "Hotspot"),
        ("--uplink-profile", "eduroam"),
    ],
)
def test_profile_collisions_fail_before_network_or_service_changes(tmp_path, arguments):
    result, calls = _run_script(tmp_path, (*arguments, "--apply"))

    assert result.returncode != 0
    assert "different profile names" in result.stderr
    assert not _network_mutations(calls)
    assert not any(call[0] == "systemctl" for call in calls)


def test_default_preflight_never_changes_profiles_or_selector(tmp_path):
    result, calls = _run_script(tmp_path)

    assert result.returncode == 0, result.stderr
    assert "Preflight passed" in result.stdout
    assert not _network_mutations(calls)
    assert not any(call[0] == "systemctl" for call in calls)


@pytest.mark.parametrize("selector_state", ["loaded", "masked"])
def test_apply_stops_selector_before_profiles_and_retains_autoconnect(
    tmp_path, selector_state
):
    result, calls = _run_script(tmp_path, ("--apply",), selector_state=selector_state)

    assert result.returncode == 0, result.stderr
    inspected = calls.index(
        [
            "systemctl",
            "show",
            "ai-drone-network.service",
            "--property=LoadState",
            "--value",
        ]
    )
    disabled = calls.index(
        ["systemctl", "disable", "--now", "ai-drone-network.service"]
    )
    mutations = _network_mutations(calls)
    assert inspected < disabled < calls.index(mutations[0])
    assert mutations[0] == ["nmcli", "connection", "clone", "eduroam", "eduroam-uplink"]
    modified = [call for call in mutations if call[2] == "modify"]
    assert {call[3] for call in modified} == {"Hotspot", "eduroam-uplink"}
    for call in modified:
        assert call[call.index("connection.autoconnect") + 1] == "yes"
    assert [call[3:] for call in mutations if call[2] == "up"] == [
        ["eduroam-uplink", "ifname", "wlan1"],
        ["Hotspot", "ifname", "wlan0"],
    ]


def test_absent_selector_needs_no_service_mutation(tmp_path):
    result, calls = _run_script(tmp_path, ("--apply",), selector_state="not-found")

    assert result.returncode == 0, result.stderr
    assert _network_mutations(calls)
    assert not any(call[:2] == ["systemctl", "disable"] for call in calls)


@pytest.mark.parametrize("selector_state", ["loaded", "not-found", ""])
def test_failed_selector_inspection_prevents_all_mutations(tmp_path, selector_state):
    result, calls = _run_script(
        tmp_path,
        ("--apply",),
        selector_state=selector_state,
        inspect_returncode=1,
    )

    assert result.returncode != 0
    assert "cannot inspect" in result.stderr
    assert not _network_mutations(calls)
    assert not any(call[:2] == ["systemctl", "disable"] for call in calls)


def test_failed_selector_stop_prevents_profile_mutations(tmp_path):
    result, calls = _run_script(tmp_path, ("--apply",), disable_returncode=1)

    assert result.returncode != 0
    assert not _network_mutations(calls)


def test_unexpected_selector_state_prevents_all_mutations(tmp_path):
    result, calls = _run_script(tmp_path, ("--apply",), selector_state="error")

    assert result.returncode != 0
    assert "unexpected single-radio selector state" in result.stderr
    assert not _network_mutations(calls)
    assert not any(call[:2] == ["systemctl", "disable"] for call in calls)
