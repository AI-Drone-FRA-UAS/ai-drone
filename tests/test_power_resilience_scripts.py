from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from ai_drone.link.deploy import RUNTIME_FILES

RESILIENCE_SCRIPT = Path("scripts/setup-pi-power-resilience.sh")
UPGRADE_SCRIPT = Path("scripts/pi-safe-upgrade.sh")
pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="bash unavailable")


@pytest.mark.parametrize("script", [RESILIENCE_SCRIPT, UPGRADE_SCRIPT])
def test_scripts_have_valid_bash_syntax(script: Path) -> None:
    subprocess.run(["bash", "-n", str(script)], check=True)


@pytest.mark.parametrize("script", [RESILIENCE_SCRIPT, UPGRADE_SCRIPT])
def test_scripts_are_deployed_to_the_pi(script: Path) -> None:
    assert script.as_posix() in RUNTIME_FILES


@pytest.mark.parametrize("script", [RESILIENCE_SCRIPT, UPGRADE_SCRIPT])
def test_scripts_refuse_to_run_off_a_raspberry_pi(script: Path) -> None:
    # Both scripts change system state, so they must fail closed when the host
    # is not the companion Pi. The development laptop is the usual mistake.
    completed = subprocess.run(
        ["bash", str(script)],
        capture_output=True,
        check=False,
        text=True,
    )

    assert completed.returncode != 0
    assert "Raspberry Pi" in completed.stderr


@pytest.mark.parametrize("script", [RESILIENCE_SCRIPT, UPGRADE_SCRIPT])
def test_scripts_reject_unknown_options(script: Path) -> None:
    completed = subprocess.run(
        ["bash", str(script), "--not-a-real-option"],
        capture_output=True,
        check=False,
        text=True,
    )

    assert completed.returncode != 0
    assert "Unknown option" in completed.stderr


def test_resilience_script_masks_the_apt_timers() -> None:
    # The 2026-08-18 corruption came from an unattended upgrade, so masking
    # both timers is the measure this script exists to apply.
    script = RESILIENCE_SCRIPT.read_text()

    assert "systemctl mask --now apt-daily.timer apt-daily-upgrade.timer" in script


def test_resilience_script_keeps_the_root_filesystem_writable() -> None:
    # drone-deploy and the sensor recorders both need a writable card, so the
    # hardening must not introduce a read-only or overlay root.
    script = RESILIENCE_SCRIPT.read_text()

    assert "overlay" not in script.lower()
    assert "remount-ro" in script  # error behaviour only, not a mount mode


def test_resilience_script_refuses_vehicle_control_autostart() -> None:
    # An unexpected reboot must never bring the vehicle-facing commands up.
    script = RESILIENCE_SCRIPT.read_text()

    assert "Refusing to finish" in script
    assert "drone-(control|motor|servo|inspect|picam)" in script


def test_resilience_script_can_revert_every_change() -> None:
    script = RESILIENCE_SCRIPT.read_text()

    assert "--revert" in script
    assert "systemctl unmask apt-daily.timer apt-daily-upgrade.timer" in script
    assert "tune2fs -e continue" in script


def test_upgrade_script_arms_recovery_before_touching_packages() -> None:
    script = UPGRADE_SCRIPT.read_text()

    arm = script.index('systemctl enable "$RECOVERY_UNIT"')
    upgrade = script.index("full-upgrade")
    assert arm < upgrade, "recovery must be armed before the upgrade starts"


def test_upgrade_script_refuses_unstable_power_and_a_busy_link() -> None:
    script = UPGRADE_SCRIPT.read_text()

    assert "vcgencmd get_throttled" in script
    assert "Power is not stable" in script
    assert "/dev/serial0 is in use" in script


def test_upgrade_script_requires_explicit_confirmation() -> None:
    script = UPGRADE_SCRIPT.read_text()

    assert 'read -r -p "Type UPGRADE to continue: " reply' in script
    assert '[[ "$reply" == "UPGRADE" ]]' in script


def test_upgrade_script_keeps_network_secrets_private() -> None:
    # The backup copies .nmconnection files, which carry Wi-Fi secrets.
    script = UPGRADE_SCRIPT.read_text()

    assert 'chmod 0700 "$backup_dir"' in script
