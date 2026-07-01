from __future__ import annotations

import tomllib
from pathlib import Path

from ai_drone import connect, deploy, pi_usb_ssh, pi_wifi
from ai_drone.pi_targets import (
    DEFAULT_PI_AP_SSID,
    DEFAULT_PI_HOSTNAME,
    DEFAULT_PI_USB_IP,
    ping_command,
    resolve_deploy_target,
    resolve_usb_target,
)


def test_deploy_target_prefers_reachable_usb_ip(tmp_path: Path) -> None:
    target = resolve_deploy_target(
        {"HOME": str(tmp_path)},
        ping=lambda host: host == DEFAULT_PI_USB_IP,
    )

    assert target.ssh_target == "seb@192.168.7.2"
    assert target.user == "seb"
    assert target.address == "192.168.7.2"
    assert target.project_dir == "/home/seb/ai-drone"


def test_deploy_target_falls_back_to_hostname(tmp_path: Path) -> None:
    target = resolve_deploy_target(
        {"HOME": str(tmp_path)},
        ping=lambda host: host == DEFAULT_PI_HOSTNAME,
    )

    assert target.ssh_target == "seb@seb-is-pm"
    assert target.address == "seb-is-pm"


def test_deploy_target_uses_explicit_host_user_and_dir(tmp_path: Path) -> None:
    target = resolve_deploy_target(
        {
            "HOME": str(tmp_path),
            "PI_HOST": "pilot@drone.local",
            "PI_DIR": "/srv/ai-drone",
        },
        ping=lambda _host: False,
    )

    assert target.ssh_target == "pilot@drone.local"
    assert target.user == "pilot"
    assert target.address == "drone.local"
    assert target.project_dir == "/srv/ai-drone"


def test_deploy_plan_uses_tar_fallback_on_windows(tmp_path: Path) -> None:
    plan = deploy.build_plan(
        ["--picam", "--dry-run", "--port", "9090"],
        environ={"HOME": str(tmp_path)},
        system="Windows",
        ping=lambda _host: False,
        rsync_path="/usr/bin/rsync",
    )

    assert plan.mode == "picam"
    assert plan.extra_args == ("--port", "9090")
    assert plan.dry_run is True
    assert plan.sync_method == "tar"


def test_deploy_plan_uses_rsync_on_unix_when_available(tmp_path: Path) -> None:
    plan = deploy.build_plan(
        [],
        environ={"HOME": str(tmp_path)},
        system="Linux",
        ping=lambda _host: False,
        rsync_path="/usr/bin/rsync",
    )

    assert plan.sync_method == "rsync"


def test_deploy_install_command_preserves_system_site_packages(tmp_path: Path) -> None:
    plan = deploy.build_plan(
        [],
        environ={
            "HOME": str(tmp_path),
            "PI_HOST": "pilot@drone",
            "PI_DIR": "/opt/drone",
        },
        system="Linux",
        ping=lambda _host: False,
        rsync_path="/usr/bin/rsync",
    )

    command = deploy.install_command(plan)

    assert command[:2] == ["ssh", "pilot@drone"]
    assert (
        "uv venv --clear --python /usr/bin/python3 --system-site-packages"
        in command[-1]
    )
    assert (
        "uv sync --locked --python .venv/bin/python --no-dev --group raspi"
        in command[-1]
    )


def test_deploy_mode_command_forwards_lidar_arguments(tmp_path: Path) -> None:
    plan = deploy.build_plan(
        ["--lidar", "--duration", "10"],
        environ={"HOME": str(tmp_path)},
        system="Linux",
        ping=lambda _host: False,
        rsync_path=None,
    )

    command = deploy.mode_command(plan)

    assert command is not None
    assert command[:2] == ["ssh", "-t"]
    assert "test_lidar.py --device /dev/serial0 --duration 10" in command[-1]


def test_tar_sync_paths_exclude_local_caches(tmp_path: Path) -> None:
    (tmp_path / "ai_drone").mkdir()
    (tmp_path / "ai_drone" / "tool.py").write_text("print('ok')\n")
    (tmp_path / ".venv").mkdir()
    (tmp_path / ".venv" / "ignored.py").write_text("ignored\n")
    (tmp_path / "tests").mkdir()
    (tmp_path / "tests" / "__pycache__").mkdir()
    (tmp_path / "tests" / "__pycache__" / "ignored.pyc").write_bytes(b"ignored")

    paths = {
        path.relative_to(tmp_path).as_posix()
        for path in deploy._iter_sync_paths(tmp_path)
    }

    assert "ai_drone/tool.py" in paths
    assert ".venv/ignored.py" not in paths
    assert "tests/__pycache__/ignored.pyc" not in paths


def test_usb_target_reads_environment() -> None:
    target = resolve_usb_target(
        {
            "PI_IP": "10.0.0.2",
            "HOST_IP": "10.0.0.1",
            "PI_USER": "pilot",
            "PI_HOSTNAME": "drone-pi",
            "USB_IFACE": "Ethernet 4",
            "TIMEOUT_SECONDS": "7",
        }
    )

    assert target.pi_ip == "10.0.0.2"
    assert target.host_ip == "10.0.0.1"
    assert target.pi_user == "pilot"
    assert target.usb_iface == "Ethernet 4"
    assert target.timeout_seconds == 7


def test_ping_command_is_platform_specific() -> None:
    assert ping_command("192.168.7.2", "Windows") == [
        "ping",
        "-n",
        "1",
        "-w",
        "1000",
        "192.168.7.2",
    ]
    assert ping_command("192.168.7.2", "Linux") == [
        "ping",
        "-c",
        "1",
        "-W",
        "1",
        "192.168.7.2",
    ]


def test_usb_linux_config_commands() -> None:
    assert pi_usb_ssh.linux_config_commands("usb0", "192.168.7.1") == [
        ["sudo", "ip", "link", "set", "usb0", "up"],
        ["sudo", "ip", "link", "set", "dev", "usb0", "mtu", "1412"],
        ["sudo", "ip", "addr", "add", "192.168.7.1/24", "dev", "usb0"],
    ]


def test_usb_windows_config_commands() -> None:
    commands = pi_usb_ssh.windows_config_commands("Ethernet 4", "192.168.7.1")

    assert commands[0][:5] == [
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-Command",
    ]
    assert "Set-NetIPInterface" in commands[0][-1]
    assert commands[1] == [
        "netsh",
        "interface",
        "ipv4",
        "set",
        "subinterface",
        "Ethernet 4",
        "mtu=1412",
        "store=active",
    ]


def test_network_ssh_targets_try_hostname_hotspot_and_usb_ip() -> None:
    assert pi_usb_ssh.network_ssh_targets(
        "seb",
        "seb-is-pm",
        "192.168.7.2",
        ["seb@custom.local"],
    ) == [
        "seb@custom.local",
        "seb@seb-is-pm.local",
        "seb@seb-is-pm",
        "seb@192.168.4.1",
        "seb@192.168.7.2",
    ]


def test_network_only_dry_run_prints_plain_ssh_targets(monkeypatch, capsys) -> None:
    monkeypatch.setattr(pi_usb_ssh.platform, "system", lambda: "Linux")

    result = pi_usb_ssh.run(
        ["--network-only", "--dry-run", "--ssh-target", "seb@drone.local"],
        environ={"TIMEOUT_SECONDS": "1"},
    )

    output = capsys.readouterr().out
    assert result == 0
    assert "Would try existing Wi-Fi/hotspot/network SSH targets first" in output
    assert "ssh -t seb@drone.local" in output
    assert "Configuring laptop side" not in output


def test_network_only_uses_reachable_target(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_run(command, **kwargs):
        calls.append(command)
        if command[:1] == ["ping"] and command[-1] == "192.168.4.1":
            return _Completed(0)
        if command[:1] == ["ping"]:
            return _Completed(1)
        return _Completed(0)

    monkeypatch.setattr(pi_usb_ssh.platform, "system", lambda: "Linux")
    monkeypatch.setattr(pi_usb_ssh.subprocess, "run", fake_run)

    result = pi_usb_ssh.run(["--network-only"], environ={"TIMEOUT_SECONDS": "1"})

    assert result == 0
    assert ["ssh", "-t", "seb@192.168.4.1"] in calls
    assert not any(command[:2] == ["sudo", "ip"] for command in calls)


def test_windows_find_script_prefers_active_usb_adapter() -> None:
    script = pi_usb_ssh.windows_find_script()

    assert "RNDIS|Remote NDIS|USB Ethernet|Ethernet Gadget|CDC" in script
    assert "$_.Status -eq 'Up'" in script
    assert "$fallback" in script


def test_usb_dry_run_uses_windows_commands(monkeypatch, capsys) -> None:
    monkeypatch.setattr(pi_usb_ssh.platform, "system", lambda: "Windows")

    result = pi_usb_ssh.run(
        ["--dry-run"],
        environ={"USB_IFACE": "Ethernet 4", "TIMEOUT_SECONDS": "1"},
    )

    output = capsys.readouterr().out
    assert result == 0
    assert "Set-NetIPInterface" in output
    assert "netsh interface ipv4 set subinterface" in output
    assert "ssh -t seb@192.168.7.2" in output


def test_connect_entrypoints_replace_drone_connect() -> None:
    scripts = tomllib.loads(Path("pyproject.toml").read_text())["project"]["scripts"]

    assert scripts["autoconnect"] == "ai_drone.connect:auto_main"
    assert scripts["manuconnect"] == "ai_drone.connect:manual_main"
    assert "drone-connect" not in scripts
    assert "drone-pi-usb-ssh" not in scripts


def test_wifi_join_command_per_platform() -> None:
    assert pi_wifi.join_command("AI-Drone-Zero", "Linux") == [
        "nmcli",
        "con",
        "up",
        "AI-Drone-Zero",
    ]
    assert pi_wifi.join_command("AI-Drone-Zero", "Darwin", "en0") == [
        "networksetup",
        "-setairportnetwork",
        "en0",
        "AI-Drone-Zero",
    ]
    assert pi_wifi.join_command("AI-Drone-Zero", "Windows") == [
        "netsh",
        "wlan",
        "connect",
        "name=AI-Drone-Zero",
        "ssid=AI-Drone-Zero",
    ]


def test_wifi_scan_detects_ssid_per_platform() -> None:
    linux_scan = "Espresso Macchiato\nAI-Drone-Zero\nXyz\n"
    assert pi_wifi.ssid_in_scan_output("AI-Drone-Zero", linux_scan, "Linux")
    assert not pi_wifi.ssid_in_scan_output("Nope", linux_scan, "Linux")

    windows_scan = "SSID 1 : Xyz\nSSID 2 : AI-Drone-Zero\n"
    assert pi_wifi.ssid_in_scan_output("AI-Drone-Zero", windows_scan, "Windows")
    assert not pi_wifi.ssid_in_scan_output("Nope", windows_scan, "Windows")


def test_auto_dry_run_lists_transports_in_priority_order(monkeypatch, capsys) -> None:
    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")

    result = connect.run_auto(["--dry-run"], environ={"TIMEOUT_SECONDS": "1"})

    output = capsys.readouterr().out
    assert result == 0
    tailscale_at = output.index("1. Tailscale")
    wifi_at = output.index("2. Pi Wi-Fi AP")
    usb_at = output.index("3. USB cable")
    assert tailscale_at < wifi_at < usb_at
    assert "ssh -t seb@seb-is-pm" in output
    assert f"nmcli con up {DEFAULT_PI_AP_SSID}" in output


def test_auto_connects_over_tailscale_first(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_run(command, **kwargs):
        calls.append(list(command))
        return _Completed(0)

    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")
    monkeypatch.setattr(connect.subprocess, "run", fake_run)

    result = connect.run_auto([], environ={"TIMEOUT_SECONDS": "1"})

    assert result == 0
    assert ["ssh", "-t", "seb@seb-is-pm"] in calls
    # Tailscale won, so no Wi-Fi join and no USB configuration happened.
    assert not any(cmd[:1] == ["nmcli"] for cmd in calls)
    assert not any(cmd[:2] == ["sudo", "ip"] for cmd in calls)


def test_auto_falls_through_to_usb_when_others_fail(monkeypatch) -> None:
    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")
    # Tailscale probe fails; AP is not broadcasting.
    monkeypatch.setattr(
        connect.subprocess, "run", lambda command, **kwargs: _Completed(1)
    )
    monkeypatch.setattr(connect.pi_wifi, "ap_available", lambda ssid, system: False)

    usb_calls: list[object] = []

    def fake_usb(*, environ=None, dry_run=False):
        usb_calls.append(environ)
        return True

    monkeypatch.setattr(connect, "connect_usb", fake_usb)

    result = connect.run_auto([], environ={"TIMEOUT_SECONDS": "1"})

    assert result == 0
    assert usb_calls  # USB was reached as the last resort


def test_manual_choice_dispatches_wifi(monkeypatch) -> None:
    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")
    monkeypatch.setattr("builtins.input", lambda _prompt="": "2")

    dispatched: list[str] = []
    monkeypatch.setattr(
        connect,
        "connect_wifi_ap",
        lambda target, system, *, dry_run: dispatched.append("wifi") or True,
    )
    monkeypatch.setattr(
        connect,
        "connect_tailscale",
        lambda ssh_target, *, dry_run: dispatched.append("tailscale") or True,
    )

    result = connect.run_manual([], environ={"TIMEOUT_SECONDS": "1"})

    assert result == 0
    assert dispatched == ["wifi"]


class _Completed:
    def __init__(self, returncode: int) -> None:
        self.returncode = returncode
        self.stdout = ""
