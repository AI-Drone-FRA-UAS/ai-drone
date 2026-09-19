from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

HOTSPOT_SCRIPT = Path("scripts/setup-pi-hotspot.sh")
pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="bash unavailable")


def _script_text() -> str:
    return HOTSPOT_SCRIPT.read_text()


def test_hotspot_script_has_valid_bash_syntax() -> None:
    subprocess.run(["bash", "-n", str(HOTSPOT_SCRIPT)], check=True)


def test_hotspot_script_rejects_password_cli_argument_without_echoing_it() -> None:
    dummy_passphrase = "offline-test-passphrase"

    completed = subprocess.run(
        ["bash", str(HOTSPOT_SCRIPT), "--password", dummy_passphrase],
        capture_output=True,
        check=False,
        text=True,
    )

    assert completed.returncode != 0
    assert "use the secure prompt or --password-file" in completed.stderr
    assert dummy_passphrase not in completed.stdout
    assert dummy_passphrase not in completed.stderr


def test_hotspot_password_file_requires_root_owner_and_private_mode() -> None:
    script = _script_text()

    assert '! -L "$PASSWORD_FILE"' in script
    assert "stat -c '%u' -- \"$PASSWORD_FILE\"" in script
    assert '[[ "$PASSWORD_FILE_UID" == "0" ]]' in script
    assert "stat -c '%a' -- \"$PASSWORD_FILE\"" in script
    assert "8#$PASSWORD_FILE_MODE & 077" in script


def test_hotspot_sets_private_umask_before_configuration_writes() -> None:
    script = _script_text()

    assert script.index("umask 077") < script.index('PASSWORD=""')
    assert script.index("umask 077") < script.index("nmcli connection add")


@pytest.fixture
def provision(tmp_path):
    model = tmp_path / "model"
    model.write_text("Raspberry Pi Zero 2 W\0")
    password = tmp_path / "password"
    password.write_text("offline-test-secret\n")
    script = tmp_path / "setup.sh"
    script.write_text(
        _script_text()
        .replace("/proc/device-tree/model", str(model))
        .replace("[[ $EUID -eq 0 ]]", "[[ 0 -eq 0 ]]")
    )
    commands = tmp_path / "commands"
    commands.mkdir()
    nmcli = commands / "nmcli"
    nmcli.write_text("""#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$CALLS"
case "$*" in
    '-g NAME connection show --active')
        [[ "$READ_FAILURE" == 0 ]] || exit 1
        printf '%s\\n' "$ACTIVE_PROFILE"
        ;;
    '-g connection.id connection show Hotspot') exit "$PROFILE_MISSING" ;;
esac
""")
    nmcli.chmod(0o755)
    stat = commands / "stat"
    stat.write_text("""#!/usr/bin/env bash
case "$2" in
    '%u') echo "$PASSWORD_UID" ;;
    '%a') echo "$PASSWORD_MODE" ;;
    *) exit 1 ;;
esac
""")
    stat.chmod(0o755)
    calls = tmp_path / "calls"

    def run(**overrides):
        environment = {
            **os.environ,
            "PATH": f"{commands}{os.pathsep}{os.environ.get('PATH', '')}",
            "CALLS": str(calls),
            "ACTIVE_PROFILE": "eduroam",
            "PROFILE_MISSING": "1",
            "READ_FAILURE": "0",
            "PASSWORD_UID": "0",
            "PASSWORD_MODE": "600",
            **overrides,
        }
        completed = subprocess.run(
            ["bash", str(script), "--password-file", str(password)],
            env=environment,
            capture_output=True,
            text=True,
            check=False,
        )
        return completed, calls.read_text().splitlines()

    return run


@pytest.mark.parametrize("missing", ["0", "1"])
def test_provisioning_saves_profile_without_activating_it(provision, missing):
    completed, calls = provision(PROFILE_MISSING=missing)
    assert completed.returncode == 0, completed.stderr
    writes = [call for call in calls if call.startswith("connection ")]
    assert len(writes) == (2 if missing == "1" else 1)
    assert all(
        call.startswith(("connection add ", "connection modify ")) for call in writes
    )
    assert all("autoconnect no" in call for call in writes)
    assert "offline-test-secret" not in completed.stdout + completed.stderr


@pytest.mark.parametrize(
    "overrides",
    [
        {"ACTIVE_PROFILE": "Hotspot"},
        {"READ_FAILURE": "1"},
        {"PASSWORD_UID": "1000"},
        {"PASSWORD_MODE": "644"},
    ],
)
def test_provisioning_refuses_unsafe_state_before_profile_writes(provision, overrides):
    completed, calls = provision(**overrides)
    assert completed.returncode != 0
    assert not any(call.startswith("connection ") for call in calls)
