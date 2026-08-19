from __future__ import annotations

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
    assert script.index("umask 077") < script.index("cat <<EOF > /etc/hostapd")
