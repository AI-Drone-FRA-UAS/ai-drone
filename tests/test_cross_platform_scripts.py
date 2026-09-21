from __future__ import annotations

from pathlib import Path

import pytest

from ai_drone.config import sync as config_sync
from ai_drone.link import connect, deploy, usb_ssh
from ai_drone.link.targets import (
    ConnectionTarget,
    ping_command,
    resolve_connection_target,
    resolve_deploy_target,
)


@pytest.mark.parametrize(
    ("environment", "expected"),
    [
        ({}, "seb@seb-is-pm"),
        ({"USB_IFACE": "usb0"}, "seb@seb-is-pm"),
        ({"PI_HOSTNAME": "drone-pi"}, "seb@drone-pi"),
        ({"PI_HOST": "pilot@192.168.4.1"}, "pilot@192.168.4.1"),
    ],
)
def test_deploy_target_is_explicit_without_network_probes(
    monkeypatch, environment, expected
):
    monkeypatch.setattr(
        "ai_drone.link.targets.subprocess.run",
        lambda *_args, **_kwargs: pytest.fail(
            "resolving a target must not probe networks"
        ),
    )
    target = resolve_deploy_target(environment)
    assert target.ssh_target == expected


def test_deploy_target_uses_explicit_host_user_and_dir(tmp_path: Path) -> None:
    target = resolve_deploy_target(
        {
            "HOME": str(tmp_path),
            "PI_HOST": "drone.local",
            "PI_HOSTNAME": "other-pi",
            "PI_USER": "pilot",
            "PI_DIR": "/srv/ai-drone",
        },
    )

    assert target.ssh_target == "pilot@drone.local"
    assert target.user == "pilot"
    assert target.address == "drone.local"
    assert target.project_dir == "/srv/ai-drone"


def test_deploy_rejects_implicit_task_execution() -> None:
    with pytest.raises(SystemExit):
        deploy.build_plan(["--run", "inspect"], environ={})


def test_deploy_offline_uses_only_cached_dependencies() -> None:
    plan = deploy.build_plan(["--offline"], environ={"PI_HOST": "seb@drone"})
    assert plan.offline is True


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
    assert target.pi_hostname == "drone-pi"
    assert target.usb_iface == "Ethernet 4"
    assert target.timeout_seconds == 7
    assert target.ssh_config == "/tmp/ai-drone-ssh-config"


def test_connection_target_uses_normal_ssh_config_by_default() -> None:
    target = resolve_connection_target({})

    assert target.ssh_config is None


def test_connection_target_allows_explicit_default_ssh_config_opt_in() -> None:
    target = resolve_connection_target({"SSH_CONFIG": ""})

    assert target.ssh_config is None


@pytest.mark.parametrize(
    ("overrides", "expected"),
    [
        ({}, None),
        ({"SSH_CONFIG": ""}, None),
        ({"SSH_CONFIG": "/tmp/custom"}, "/tmp/custom"),
    ],
)
def test_deployment_and_connection_share_ssh_config_selection(
    tmp_path: Path, overrides: dict[str, str], expected: str | None
) -> None:
    config = tmp_path / ".ssh" / "config"
    config.parent.mkdir()
    config.write_text("Host *\n  InvalidHostConfiguration yes\n")
    environment = {"HOME": str(tmp_path), "PI_HOST": "seb@drone", **overrides}

    target = resolve_deploy_target(environment)

    assert target.ssh_config == expected
    assert target.ssh_config == resolve_connection_target(environment).ssh_config
    prefix = ["ssh", "-F", expected] if expected else ["ssh"]
    assert deploy.remote_command(target, "true") == [*prefix, "seb@drone", "true"]


def test_config_sync_export_uses_normal_ssh_config_by_default() -> None:
    plan = deploy.build_plan([], environ={"PI_HOST": "seb@drone"})

    command = config_sync.remote_export_command(
        plan, device="/dev/serial0", baud=115200, timeout=30.0
    )

    assert command[:2] == ["ssh", "seb@drone"]
    assert "ai_drone.cli.config_export" in command[-1]


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
    assert "ssh -t seb@192.168.7.2" in output


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


def test_usb_live_setup_refuses_without_usb_iface(capsys) -> None:
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


@pytest.mark.parametrize(
    ("arguments", "environment", "expected"),
    [
        ([], {}, ["ssh", "-t", "seb@seb-is-pm"]),
        (["192.168.4.1"], {}, ["ssh", "-t", "seb@192.168.4.1"]),
        (
            ["pilot@drone.local"],
            {"PI_HOST": "ignored"},
            ["ssh", "-t", "pilot@drone.local"],
        ),
        ([], {"PI_HOST": "seb@drone.local"}, ["ssh", "-t", "seb@drone.local"]),
        (
            [],
            {"SSH_CONFIG": "/tmp/custom"},
            ["ssh", "-F", "/tmp/custom", "-t", "seb@seb-is-pm"],
        ),
    ],
)
@pytest.mark.parametrize("exit_code", [0, 255])
def test_connect_only_runs_selected_ssh_route(
    monkeypatch, arguments, environment, expected, exit_code
):
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return _Completed(exit_code)

    monkeypatch.setattr(connect.subprocess, "run", run)
    assert connect.run(arguments, environ=environment) == exit_code
    assert calls == [expected]


def test_connect_dry_run_does_not_execute_commands(monkeypatch, capsys):
    monkeypatch.setattr(
        connect.subprocess, "run", lambda *_a, **_k: pytest.fail("command executed")
    )
    assert connect.run(["--dry-run"], environ={}) == 0
    assert capsys.readouterr().out.strip() == "ssh -t seb@seb-is-pm"


@pytest.mark.parametrize("transport", ["auto", "tailscale", "hotspot", "usb"])
def test_removed_transport_flags_never_execute(monkeypatch, transport):
    monkeypatch.setattr(
        connect.subprocess, "run", lambda *_a, **_k: pytest.fail("command executed")
    )
    with pytest.raises(SystemExit):
        connect.run(["--transport", transport], environ={})


class _Completed:
    def __init__(self, returncode: int) -> None:
        self.returncode = returncode
        self.stdout = ""
