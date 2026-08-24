from __future__ import annotations

import os
import tomllib
from pathlib import Path

import pytest

from ai_drone.link import connect, deploy, usb_ssh, wifi
from ai_drone.link.targets import (
    DEFAULT_PI_AP_SSID,
    DEFAULT_PI_HOSTNAME,
    DEFAULT_PI_HOTSPOT_IP,
    DEFAULT_PI_USB_IP,
    ConnectionTarget,
    ping_command,
    resolve_connection_target,
    resolve_deploy_target,
)


@pytest.mark.parametrize(
    ("environment", "reachable", "expected_address", "expected_probes"),
    [
        (
            {"USB_IFACE": "usb0"},
            {DEFAULT_PI_HOSTNAME, DEFAULT_PI_HOTSPOT_IP, DEFAULT_PI_USB_IP},
            DEFAULT_PI_HOSTNAME,
            [DEFAULT_PI_HOSTNAME],
        ),
        (
            {},
            {DEFAULT_PI_HOTSPOT_IP, DEFAULT_PI_USB_IP},
            DEFAULT_PI_HOTSPOT_IP,
            [DEFAULT_PI_HOSTNAME, DEFAULT_PI_HOTSPOT_IP],
        ),
        (
            {"USB_IFACE": "usb0"},
            {DEFAULT_PI_USB_IP},
            DEFAULT_PI_USB_IP,
            [DEFAULT_PI_HOSTNAME, DEFAULT_PI_HOTSPOT_IP, DEFAULT_PI_USB_IP],
        ),
        (
            {},
            {DEFAULT_PI_USB_IP},
            DEFAULT_PI_HOSTNAME,
            [DEFAULT_PI_HOSTNAME, DEFAULT_PI_HOTSPOT_IP],
        ),
    ],
)
def test_deploy_target_uses_common_safe_connection_priority(
    tmp_path: Path,
    environment: dict[str, str],
    reachable: set[str],
    expected_address: str,
    expected_probes: list[str],
) -> None:
    probes: list[str] = []

    def ping(host: str) -> bool:
        probes.append(host)
        return host in reachable

    target = resolve_deploy_target(
        {"HOME": str(tmp_path), **environment},
        ping=ping,
    )

    assert target.ssh_target == f"seb@{expected_address}"
    assert target.user == "seb"
    assert target.address == expected_address
    assert target.project_dir == "/home/seb/ai-drone"
    assert probes == expected_probes


def test_deploy_target_uses_explicit_host_user_and_dir(tmp_path: Path) -> None:
    target = resolve_deploy_target(
        {
            "HOME": str(tmp_path),
            "PI_HOST": "drone.local",
            "PI_USER": "pilot",
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
        ["--dry-run", "--run", "inspect", "--", "--port", "9090"],
        environ={"HOME": str(tmp_path)},
        system="Windows",
        ping=lambda _host: False,
        rsync_path="/usr/bin/rsync",
    )

    assert plan.mode == "inspect"
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


@pytest.mark.parametrize(
    ("task", "module", "task_args"),
    [
        ("inspect", "ai_drone.cli.record", ("--duration", "12.5")),
        ("servo", "ai_drone.cli.servo", ("--mode", "center")),
        ("motor-test", "ai_drone.cli.motor_test", ("--motor", "1")),
        ("control", "ai_drone.cli.control", ("hover", "--duration", "2")),
    ],
)
def test_deploy_run_uses_allowlisted_module_and_forwards_arguments(
    tmp_path: Path, task: str, module: str, task_args: tuple[str, ...]
) -> None:
    plan = deploy.build_plan(
        ["--run", task, "--", *task_args],
        environ={"HOME": str(tmp_path)},
        system="Linux",
        ping=lambda _host: False,
        rsync_path=None,
    )

    command = deploy.mode_command(plan)

    assert plan.extra_args == task_args
    assert command is not None
    assert command[:2] == ["ssh", "-t"]
    assert f".venv/bin/python -m {module}" in command[-1]
    assert all(argument in command[-1] for argument in task_args)


def test_deploy_rejects_task_arguments_without_run(tmp_path: Path) -> None:
    with pytest.raises(SystemExit):
        deploy.build_plan(
            ["--", "--duration", "10"],
            environ={"HOME": str(tmp_path)},
            system="Linux",
            ping=lambda _host: False,
            rsync_path=None,
        )


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


def test_connection_target_reads_environment() -> None:
    environ = {
        "PI_IP": "10.0.0.2",
        "HOST_IP": "10.0.0.1",
        "PI_USER": "pilot",
        "PI_HOSTNAME": "drone-pi",
        "USB_IFACE": "Ethernet 4",
        "TIMEOUT_SECONDS": "7",
        "SSH_CONFIG": "/tmp/ai-drone-ssh-config",
    }
    target = resolve_connection_target(environ)

    assert isinstance(target, ConnectionTarget)
    assert target.pi_ip == "10.0.0.2"
    assert target.host_ip == "10.0.0.1"
    assert target.pi_user == "pilot"
    assert target.usb_iface == "Ethernet 4"
    assert target.timeout_seconds == 7
    assert target.ssh_config == "/tmp/ai-drone-ssh-config"


def test_connection_target_bypasses_user_ssh_config_by_default() -> None:
    target = resolve_connection_target({})

    assert target.ssh_config == os.devnull


def test_connection_target_allows_explicit_default_ssh_config_opt_in() -> None:
    target = resolve_connection_target({"SSH_CONFIG": ""})

    assert target.ssh_config is None


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
    assert usb_ssh.linux_config_commands("usb0", "192.168.7.1") == [
        ["sudo", "ip", "link", "set", "usb0", "up"],
        ["sudo", "ip", "link", "set", "dev", "usb0", "mtu", "1412"],
        ["sudo", "ip", "addr", "add", "192.168.7.1/24", "dev", "usb0"],
    ]


def test_usb_windows_config_commands() -> None:
    commands = usb_ssh.windows_config_commands("Ethernet 4", "192.168.7.1")

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


def test_windows_find_script_prefers_active_usb_adapter() -> None:
    script = usb_ssh.windows_find_script()

    assert "RNDIS|Remote NDIS|USB Ethernet|Ethernet Gadget|CDC" in script
    assert "$_.Status -eq 'Up'" in script
    assert "$fallback" in script


def test_usb_dry_run_uses_windows_commands(monkeypatch, capsys) -> None:
    monkeypatch.setattr(usb_ssh.platform, "system", lambda: "Windows")

    result = usb_ssh.run(
        ["--dry-run"],
        environ={"USB_IFACE": "Ethernet 4", "TIMEOUT_SECONDS": "1"},
    )

    output = capsys.readouterr().out
    assert result == 0
    assert "Set-NetIPInterface" in output
    assert "netsh interface ipv4 set subinterface" in output
    assert f"ssh -F {os.devnull} -t seb@192.168.7.2" in output


def test_usb_ssh_command_uses_configured_ssh_config() -> None:
    command = usb_ssh.ssh_command(
        "pilot",
        "192.168.7.2",
        "/tmp/custom-ssh-config",
    )

    assert command[:5] == [
        "ssh",
        "-F",
        "/tmp/custom-ssh-config",
        "-t",
        "pilot@192.168.7.2",
    ]


def test_usb_live_setup_refuses_guessed_interface(monkeypatch, capsys) -> None:
    monkeypatch.setattr(usb_ssh.platform, "system", lambda: "Linux")
    monkeypatch.setattr(usb_ssh, "find_usb_iface", lambda _system: "usb-dock0")
    monkeypatch.setattr(
        usb_ssh,
        "config_commands",
        lambda *_args: (_ for _ in ()).throw(
            AssertionError("guessed interfaces must never be configured")
        ),
    )

    result = usb_ssh.run([], environ={"TIMEOUT_SECONDS": "1"})

    assert result == 1
    assert "Refusing to reconfigure" in capsys.readouterr().out


@pytest.mark.parametrize(
    "environment",
    [
        {"PI_IP": "not-an-ip", "HOST_IP": "192.168.7.1"},
        {"PI_IP": "192.168.7.1", "HOST_IP": "192.168.7.1"},
        {"PI_IP": "192.168.8.2", "HOST_IP": "192.168.7.1"},
        {"PI_IP": "192.168.7.2", "HOST_IP": "192.168.7.1", "USB_IFACE": "-bad"},
    ],
)
def test_usb_rejects_unsafe_network_arguments_before_mutation(
    monkeypatch, environment
) -> None:
    monkeypatch.setattr(
        usb_ssh,
        "run_usb_transport",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("invalid arguments must be rejected before mutation")
        ),
    )

    with pytest.raises(SystemExit) as error:
        usb_ssh.run([], environ=environment)

    assert error.value.code == 2


def test_connect_entrypoints_replace_drone_connect() -> None:
    scripts = tomllib.loads(Path("pyproject.toml").read_text())["project"]["scripts"]

    assert scripts["drone-connect"] == "ai_drone.link.connect:main"
    assert "autoconnect" not in scripts
    assert "manuconnect" not in scripts
    assert "drone-pi-usb-ssh" not in scripts


def test_hardware_tool_entrypoints_live_in_package_cli() -> None:
    scripts = tomllib.loads(Path("pyproject.toml").read_text())["project"]["scripts"]

    assert scripts["drone-inspect"] == "ai_drone.cli.record:main"
    assert scripts["drone-servo"] == "ai_drone.cli.servo:main"

    # Flight functionality is consolidated behind one guarded command; the
    # former duplicate wrappers are intentionally not installed.
    assert scripts["drone-control"] == "ai_drone.cli.control:main"
    assert "drone-health" not in scripts
    assert "drone-console" not in scripts
    assert "drone-follow" not in scripts
    assert "drone-fly-and-land" not in scripts


def test_wifi_join_command_per_platform() -> None:
    assert wifi.join_command("AI-Drone-Zero", "Linux") == [
        "nmcli",
        "con",
        "up",
        "AI-Drone-Zero",
    ]
    assert wifi.join_command("AI-Drone-Zero", "Darwin", "en0") == [
        "networksetup",
        "-setairportnetwork",
        "en0",
        "AI-Drone-Zero",
    ]
    assert wifi.join_command("AI-Drone-Zero", "Windows") == [
        "netsh",
        "wlan",
        "connect",
        "name=AI-Drone-Zero",
        "ssid=AI-Drone-Zero",
    ]


def test_wifi_scan_detects_ssid_per_platform() -> None:
    linux_scan = "Espresso Macchiato\nAI-Drone-Zero\nXyz\n"
    assert wifi.ssid_in_scan_output("AI-Drone-Zero", linux_scan, "Linux")
    assert not wifi.ssid_in_scan_output("Nope", linux_scan, "Linux")

    windows_scan = "SSID 1 : Xyz\nSSID 2 : AI-Drone-Zero\n"
    assert wifi.ssid_in_scan_output("AI-Drone-Zero", windows_scan, "Windows")
    assert not wifi.ssid_in_scan_output("Nope", windows_scan, "Windows")


def test_auto_dry_run_lists_transports_in_priority_order(monkeypatch, capsys) -> None:
    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")

    result = connect.run(["--dry-run"], environ={"TIMEOUT_SECONDS": "1"})

    output = capsys.readouterr().out
    assert result == 0
    tailscale_at = output.index("1. Tailscale")
    wifi_at = output.index("2. Pi Wi-Fi AP")
    usb_at = output.index("3. USB cable")
    assert tailscale_at < wifi_at < usb_at
    assert f"ssh -F {os.devnull} -t seb@seb-is-pm" in output
    assert f"ssh -F {os.devnull} -t seb@192.168.4.1" in output
    assert f"ssh -F {os.devnull} -t seb@192.168.7.2" not in output
    assert "Pass --usb-iface" in output
    assert f"nmcli con up {DEFAULT_PI_AP_SSID}" in output


def test_auto_connects_over_tailscale_first(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_run(command, **kwargs):
        calls.append(list(command))
        return _Completed(0)

    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")
    monkeypatch.setattr(connect.subprocess, "run", fake_run)

    result = connect.run([], environ={"TIMEOUT_SECONDS": "1"})

    assert result == 0
    assert ["ssh", "-F", os.devnull, "-t", "seb@seb-is-pm"] in calls
    # Tailscale won, so no Wi-Fi join and no USB configuration happened.
    assert not any(cmd[:1] == ["nmcli"] for cmd in calls)
    assert not any(cmd[:2] == ["sudo", "ip"] for cmd in calls)


def test_auto_falls_through_to_usb_when_others_fail(monkeypatch) -> None:
    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")
    # Tailscale probe fails; AP is not broadcasting.
    monkeypatch.setattr(
        connect.subprocess, "run", lambda command, **kwargs: _Completed(1)
    )
    monkeypatch.setattr(connect.wifi, "ap_available", lambda ssid, system: False)

    usb_calls: list[object] = []

    def fake_usb(*, environ=None, dry_run=False):
        usb_calls.append(environ)
        return True

    monkeypatch.setattr(connect, "connect_usb", fake_usb)

    result = connect.run([], environ={"TIMEOUT_SECONDS": "1"})

    assert result == 0
    assert usb_calls  # USB was reached as the last resort


def test_explicit_hotspot_dispatches_only_wifi(monkeypatch) -> None:
    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")

    dispatched: list[str] = []
    monkeypatch.setattr(
        connect,
        "connect_wifi_ap",
        lambda target, system, *, dry_run: dispatched.append("wifi") or True,
    )
    monkeypatch.setattr(
        connect,
        "connect_tailscale",
        lambda ssh_target, *, ssh_config, dry_run: (
            dispatched.append("tailscale") or True
        ),
    )

    result = connect.run(["--transport", "hotspot"], environ={"TIMEOUT_SECONDS": "1"})

    assert result == 0
    assert dispatched == ["wifi"]


@pytest.mark.parametrize("transport", ["tailscale", "hotspot", "usb"])
def test_explicit_dry_run_is_successful_for_every_transport(
    monkeypatch, transport: str
) -> None:
    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")
    monkeypatch.setattr(connect, "connect_tailscale", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(connect, "connect_wifi_ap", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(connect, "connect_usb", lambda *_args, **_kwargs: True)

    assert connect.run(["--dry-run", "--transport", transport], environ={}) == 0


def test_auto_dry_run_threads_explicit_ssh_config_to_every_transport(
    monkeypatch, capsys
) -> None:
    monkeypatch.setattr(connect.platform, "system", lambda: "Linux")

    result = connect.run(
        ["--dry-run"],
        environ={
            "SSH_CONFIG": "/tmp/custom-ssh-config",
            "TIMEOUT_SECONDS": "1",
        },
    )

    output = capsys.readouterr().out
    assert result == 0
    assert output.count("ssh -F /tmp/custom-ssh-config") == 3


class _Completed:
    def __init__(self, returncode: int) -> None:
        self.returncode = returncode
        self.stdout = ""
