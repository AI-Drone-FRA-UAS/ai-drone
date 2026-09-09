from __future__ import annotations

import json
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from ai_drone.cli import power
from ai_drone.mavlink import ownership


def idle():
    return {
        "complete": True,
        "hardware": [],
        "packages": [],
        "package_database_ok": True,
        "walk": {"ActiveState": "inactive"},
    }


def disarmed():
    return {"status": "disarmed", "heartbeat_age_s": 0.2, "heartbeats": 4}


@pytest.fixture(autouse=True)
def simulated_pi(monkeypatch):
    monkeypatch.setattr(power, "is_raspberry_pi", lambda: True)


@pytest.fixture
def orchestration(monkeypatch):
    calls = []
    monkeypatch.setattr(
        power, "_connect_pi", lambda: (["ssh", "target", "command snapshot"], idle())
    )
    monkeypatch.setattr(power, "_fresh_fc", lambda _command: disarmed())

    def remote(_command, action):
        calls.append(action)
        return {
            "quiesce": {"quiesced": True, "report": "/capture/review/index.html"},
            "idle": {"idle": True},
            "shutdown": {"shutdown_requested": True, "halt_confirmed": False},
        }[action]

    monkeypatch.setattr(power, "_remote", remote)
    return calls


@pytest.mark.parametrize("removal", power.REMOVALS)
def test_prepare_shuts_down_conservatively(orchestration, capsys, removal):
    assert power.main(["prepare", removal]) == 0
    assert orchestration == ["quiesce", "shutdown"]
    output = capsys.readouterr().out
    assert "shutdown requested" in output
    assert "halt is NOT yet confirmed" in output
    assert "SSH loss alone" in output


@pytest.mark.parametrize("removal", ["battery", "fc-usb", "pi-usb"])
def test_attestation_keeps_pi_running_for_this_invocation(orchestration, removal):
    assert power.main(["prepare", removal, "--pi-power-independent"]) == 0
    assert orchestration == ["quiesce", "idle"]
    orchestration.clear()
    assert power.main(["prepare", removal]) == 0
    assert orchestration == ["quiesce", "shutdown"]


def test_all_removal_cannot_attest_remaining_power(orchestration):
    assert power.main(["prepare", "all", "--pi-power-independent"]) == 1
    assert orchestration == []


def test_preview_calls_no_hardware_processes(monkeypatch, capsys):
    monkeypatch.setattr(
        power, "_run", lambda *_args, **_kwargs: pytest.fail("process ran")
    )
    monkeypatch.setattr(power, "_connect_pi", lambda: pytest.fail("connected"))
    assert power.main(["prepare", "battery", "--dry-run"]) == 0
    assert "Would request Pi shutdown" in capsys.readouterr().out


@pytest.mark.parametrize("condition", ["package", "dpkg", "owner", "unknown"])
def test_prepare_blocks_before_stopping_anything(orchestration, monkeypatch, condition):
    snapshot = idle()
    if condition == "package":
        snapshot["packages"] = [{"pid": 123}]
    elif condition == "dpkg":
        snapshot["package_database_ok"] = False
    elif condition == "owner":
        snapshot["hardware"] = [{"device": "/dev/serial0", "walk": False}]
    else:
        snapshot["complete"] = False
    monkeypatch.setattr(power, "_connect_pi", lambda: (["ssh"], snapshot))
    assert power.main(["prepare", "battery"]) == 1
    assert orchestration == []


@pytest.mark.parametrize("status", ["armed", "unavailable", "busy", "absent"])
def test_no_shutdown_without_selected_disarmed_fc(orchestration, monkeypatch, status):
    monkeypatch.setattr(power, "_fresh_fc", lambda _command: {"status": status})
    assert power.main(["prepare", "all"]) == 1
    assert orchestration == ["quiesce"]


def test_expired_fc_confirmation_cannot_request_shutdown(orchestration, monkeypatch):
    clock = iter([10.0, 13.0])
    monkeypatch.setattr(power.time, "monotonic", lambda: next(clock))
    assert power.main(["prepare", "pi-usb"]) == 1
    assert orchestration == ["quiesce"]


def test_status_still_inspects_usb_when_pi_unavailable(monkeypatch, capsys):
    def unavailable():
        raise RuntimeError("SSH unavailable")

    monkeypatch.setattr(power, "_connect_pi", unavailable)
    monkeypatch.setattr(power, "_probe_fc", lambda _device: disarmed())
    assert power.main(["status"]) == 1
    output = capsys.readouterr().out
    assert "Pi: unknown" in output and '"status": "disarmed"' in output
    assert "USB networking is not power proof" in output
    for choice in power.REMOVALS:
        assert f"{choice}: blocked (Pi status unavailable:" in output
        assert f"drone-power prepare {choice}" in output


def test_status_missing_both_devices_is_blocked(monkeypatch, capsys):
    def unavailable():
        raise RuntimeError("SSH unavailable")

    monkeypatch.setattr(power, "_connect_pi", unavailable)
    monkeypatch.setattr(power, "_probe_fc", lambda _device: {"status": "absent"})
    assert power.main(["status"]) == 1
    output = capsys.readouterr().out
    for choice in power.REMOVALS:
        assert f"{choice}: blocked" in output
    assert "fresh selected-FC disarmed state is not established" in output
    assert "needs Pi shutdown" not in output


@pytest.mark.parametrize("busy", ["package", "hardware", "none"])
def test_status_reports_each_removal_from_jobs_and_fc(monkeypatch, capsys, busy):
    snapshot = idle()
    if busy == "package":
        snapshot["packages"] = [{"pid": 123}]
    elif busy == "hardware":
        snapshot["hardware"] = [{"pid": 123, "walk": False}]
    monkeypatch.setattr(power, "_connect_pi", lambda: (["ssh"], snapshot))
    monkeypatch.setattr(power, "_fresh_fc", lambda _command: disarmed())
    assert power.main(["status"]) == (0 if busy == "none" else 1)
    output = capsys.readouterr().out
    for choice in power.REMOVALS:
        assert (
            f"{choice}: {'needs Pi shutdown' if busy == 'none' else 'blocked'}"
            in output
        )


def test_status_does_not_stop_known_walk(monkeypatch, capsys):
    snapshot = idle()
    snapshot["walk"]["ActiveState"] = "active"
    snapshot["hardware"] = [{"pid": 123, "walk": True}]
    monkeypatch.setattr(power, "_connect_pi", lambda: (["ssh"], snapshot))
    monkeypatch.setattr(power, "_fresh_fc", lambda _command: disarmed())
    monkeypatch.setattr(power, "_remote", lambda *_args: pytest.fail("mutated Pi"))
    assert power.main(["status"]) == 0
    assert "needs Pi shutdown" in capsys.readouterr().out


def test_busy_usb_does_not_probe_other_serial_port(monkeypatch):
    monkeypatch.setattr(power, "_probe_fc", lambda _device: {"status": "busy"})
    monkeypatch.setattr(
        power, "_remote", lambda *_args: pytest.fail("remote UART probed")
    )
    assert power._fresh_fc(["ssh"])["status"] == "busy"


def test_absent_usb_uses_guarded_pi_fallback(monkeypatch):
    calls = []
    monkeypatch.setattr(power, "_probe_fc", lambda _device: {"status": "absent"})
    monkeypatch.setattr(
        power, "_remote", lambda command, action: calls.append(action) or disarmed()
    )
    assert power._fresh_fc(["ssh"])["status"] == "disarmed"
    assert calls == ["fc"]


def test_usb_owner_inspection_failure_uses_pi_telemetry_without_opening_usb(
    tmp_path, monkeypatch, capsys
):
    device = tmp_path / "FC-USB"
    device.touch()
    monkeypatch.setattr(power, "STABLE_FLIGHT_CONTROLLER_DEVICE", device)
    monkeypatch.setattr(power, "_connect_pi", lambda: (["ssh"], idle()))

    def unavailable():
        raise RuntimeError(
            "cannot inspect all hardware owners (sudo permission required)"
        )

    monkeypatch.setattr(power, "_owners", unavailable)
    monkeypatch.setattr(power, "require_available_serial", lambda _device: None)
    monkeypatch.setattr(
        power.os, "open", lambda *_args: pytest.fail("opened unchecked USB")
    )
    calls = []

    def remote(_command, action):
        calls.append(action)
        return {**disarmed(), "device": "/dev/serial0", "battery_voltage_v": 14.2}

    monkeypatch.setattr(power, "_remote", remote)
    assert power.main(["status"]) == 0
    assert calls == ["fc"]
    output = capsys.readouterr().out
    assert 'FC USB: {"status": "unavailable"' in output
    assert '"detected": true' in output and "sudo permission required" in output
    assert '"telemetry_source": "Pi UART"' in output
    assert '"battery_voltage_v": 14.2' in output


def test_usb_inspection_failure_stays_visible_when_pi_fallback_fails(
    tmp_path, monkeypatch, capsys
):
    device = tmp_path / "FC-USB"
    device.touch()
    monkeypatch.setattr(power, "STABLE_FLIGHT_CONTROLLER_DEVICE", device)

    def owner_failure():
        raise RuntimeError("owner inspection unavailable")

    def uart_failure(*_args):
        raise RuntimeError("Pi hardware is busy")

    monkeypatch.setattr(power, "_owners", owner_failure)
    monkeypatch.setattr(power, "require_available_serial", lambda _device: None)
    monkeypatch.setattr(power, "_remote", uart_failure)
    with pytest.raises(RuntimeError, match="Pi hardware is busy"):
        power._fresh_fc(["ssh"])
    output = capsys.readouterr().out
    assert '"detected": true' in output and "owner inspection unavailable" in output


@pytest.mark.parametrize(
    "response, expected_status",
    [
        ((0, "12345", "/dev/null:"), "busy"),
        ((1, "", "inspection warning"), "unknown"),
        ((2, "", "failed"), "unknown"),
        (FileNotFoundError("fuser unavailable"), "unknown"),
    ],
)
def test_nonprivileged_usb_owner_check_blocks_busy_or_unknown_fallback(
    tmp_path, monkeypatch, response, expected_status
):
    device = tmp_path / "FC-USB"
    device.symlink_to("/dev/null")
    monkeypatch.setattr(power, "STABLE_FLIGHT_CONTROLLER_DEVICE", device)

    def all_owners_unavailable():
        raise RuntimeError("sudo permission required")

    calls = []

    def inspect(command, **kwargs):
        calls.append(command)
        assert kwargs["timeout"] == 3
        if isinstance(response, Exception):
            raise response
        return SimpleNamespace(
            returncode=response[0], stdout=response[1], stderr=response[2]
        )

    monkeypatch.setattr(power, "_owners", all_owners_unavailable)
    monkeypatch.setattr(ownership.subprocess, "run", inspect)
    monkeypatch.setattr(power.os, "open", lambda *_args: pytest.fail("opened USB"))
    monkeypatch.setattr(
        power, "_remote", lambda *_args: pytest.fail("Pi fallback used")
    )
    result = power._fresh_fc(["ssh"])
    assert result["status"] == expected_status
    assert result["detected"] is True
    assert result["reason"] == "sudo permission required"
    assert result["serial_owner_check"]
    assert calls == [["fuser", "/dev/null"]]
    with pytest.raises(RuntimeError, match="disarmed state is not established"):
        power._require_disarmed(result)


def test_nonprivileged_usb_idle_check_allows_guarded_fallback(tmp_path, monkeypatch):
    device = tmp_path / "FC-USB"
    device.symlink_to("/dev/null")
    monkeypatch.setattr(power, "STABLE_FLIGHT_CONTROLLER_DEVICE", device)

    def all_owners_unavailable():
        raise RuntimeError("sudo permission required")

    calls = []

    def inspect(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=1, stdout="", stderr="")

    monkeypatch.setattr(power, "_owners", all_owners_unavailable)
    monkeypatch.setattr(ownership.subprocess, "run", inspect)
    monkeypatch.setattr(power.os, "open", lambda *_args: pytest.fail("opened USB"))
    monkeypatch.setattr(power, "_remote", lambda _command, action: disarmed())
    result = power._fresh_fc(["ssh"])
    assert result["status"] == "disarmed"
    assert result["telemetry_source"] == "Pi UART"
    assert result["usb_connection"]["reason"] == "sudo permission required"
    assert result["usb_connection"]["serial_owner_check"] == "no visible owner"
    assert calls == [["fuser", "/dev/null"]]


def test_pi_uart_still_requires_complete_owner_inspection(monkeypatch):
    def unavailable(**_kwargs):
        raise RuntimeError("hardware-owner inspection was incomplete")

    monkeypatch.setattr(power, "_owners", unavailable)
    monkeypatch.setattr(
        power, "_probe_fc", lambda _device: pytest.fail("opened Pi UART")
    )
    with pytest.raises(RuntimeError, match="incomplete"):
        power._remote_action("fc")


def test_pi_uart_fallback_refuses_active_recorder(monkeypatch):
    snapshot = idle()
    snapshot["walk"]["ActiveState"] = "active"
    monkeypatch.setattr(power, "_pi_snapshot", lambda: snapshot)
    monkeypatch.setattr(
        power, "_probe_fc", lambda _device: pytest.fail("opened busy port")
    )
    with pytest.raises(RuntimeError, match="recorder"):
        power._remote_action("fc")


def test_busy_port_is_not_opened(tmp_path, monkeypatch):
    device = tmp_path / "serial"
    device.touch()
    monkeypatch.setattr(
        power, "_owners", lambda: {"hardware": [{"device": str(device)}]}
    )
    monkeypatch.setattr(
        power.os, "open", lambda *_args: pytest.fail("opened busy port")
    )
    assert power._probe_fc(device)["status"] == "busy"


def test_probe_closes_descriptor_when_configuration_fails(tmp_path, monkeypatch):
    import fcntl

    device = tmp_path / "serial"
    device.touch()
    closed = []
    monkeypatch.setattr(power, "_owners", lambda: {"hardware": []})
    monkeypatch.setattr(power.os, "open", lambda *_args: 987)
    monkeypatch.setattr(fcntl, "flock", lambda *_args: None)
    monkeypatch.setattr(power.os, "close", closed.append)

    def fail(_fd):
        raise OSError("configuration failed")

    monkeypatch.setattr(power, "_configure_receiver", fail)
    with pytest.raises(OSError):
        power._probe_fc(device)
    assert closed == [987]


@pytest.mark.parametrize("armed_source", [None, 2, 1])
def test_receive_only_source_filter_and_armed_latch(monkeypatch, armed_source):
    import select

    clock = [0.0]
    monkeypatch.setattr(power.time, "monotonic", lambda: clock[0])
    encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)

    def heartbeat(armed=False, source=1):
        encoder.srcSystem = source
        return mavlink.MAVLink_heartbeat_message(
            mavlink.MAV_TYPE_QUADROTOR,
            mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0,
            0,
            3,
            3,
        ).pack(encoder)

    frames = [
        heartbeat(),
        heartbeat(armed_source is not None, armed_source or 1),
        heartbeat(),
        heartbeat(),
    ]

    def ready(*_args):
        clock[0] += 0.8
        return ([987], [], []) if frames else ([], [], [])

    monkeypatch.setattr(select, "select", ready)
    monkeypatch.setattr(power.os, "read", lambda *_args: frames.pop(0))
    monkeypatch.setattr(power.os, "write", lambda *_args: pytest.fail("serial write"))
    result = power._receive_fc(987, "fake")
    assert result["status"] == ("armed" if armed_source == 1 else "disarmed")
    assert result["heartbeats"] == (3 if armed_source == 2 else 4)


@pytest.mark.parametrize("report_written", [True, False])
def test_quiesce_requires_same_invocation_report(tmp_path, monkeypatch, report_written):
    report = tmp_path / "review/index.html"
    report.parent.mkdir()
    if report_written:
        report.write_text("complete")
    before = idle()
    before["walk"] = {
        "ActiveState": "active",
        "InvocationID": "a" * 32,
        "KillSignal": "2",
        "KillMode": "mixed",
    }
    before["hardware"] = [{"walk": True}]
    snapshots = iter([before, idle()])
    monkeypatch.setattr(power, "_pi_snapshot", lambda: next(snapshots))
    commands = []

    def run(command, **_kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0, stdout=f"Report: {report}\n")

    monkeypatch.setattr(power, "_run", run)
    if report_written:
        assert power._quiesce() == {"quiesced": True, "report": str(report)}
    else:
        with pytest.raises(RuntimeError, match="confirmed report"):
            power._quiesce()
    assert commands[0] == ["sudo", "-n", "systemctl", "stop", power.WALK_UNIT]
    assert commands[1][1] == "_SYSTEMD_INVOCATION_ID=" + "a" * 32


def test_quiesce_refuses_incorrect_stop_signal(monkeypatch):
    before = idle()
    before["walk"] = {"ActiveState": "active", "KillSignal": "15", "KillMode": "mixed"}
    monkeypatch.setattr(power, "_pi_snapshot", lambda: before)
    monkeypatch.setattr(power, "_run", lambda *_args: pytest.fail("stopped unit"))
    with pytest.raises(RuntimeError, match="SIGINT"):
        power._quiesce()


def test_shutdown_rechecks_new_package_job(monkeypatch):
    snapshot = idle()
    snapshot["packages"] = [{"pid": 123}]
    monkeypatch.setattr(power, "_pi_snapshot", lambda: snapshot)
    monkeypatch.setattr(power, "_run", lambda *_args: pytest.fail("shutdown"))
    with pytest.raises(RuntimeError, match="package maintenance"):
        power._remote_action("shutdown")


def test_remote_shutdown_ack_does_not_claim_halt(monkeypatch):
    monkeypatch.setattr(power, "_pi_snapshot", lambda **_kwargs: idle())
    monkeypatch.setattr(power, "_probe_fc", lambda _device: disarmed())
    calls = []
    monkeypatch.setattr(
        power,
        "_run",
        lambda command, **_kwargs: (
            calls.append(command) or SimpleNamespace(returncode=0)
        ),
    )
    assert power._remote_action("shutdown") == {
        "shutdown_requested": True,
        "halt_confirmed": False,
    }
    assert calls == [["sudo", "-n", "systemctl", "poweroff", "--no-block"]]


@pytest.mark.parametrize("action", ["idle", "shutdown"])
@pytest.mark.parametrize("status", ["armed", "unavailable", "busy", "absent"])
def test_final_action_requires_its_own_fresh_pi_uart(monkeypatch, action, status):
    monkeypatch.setattr(power, "_pi_snapshot", lambda **_kwargs: idle())
    devices = []
    monkeypatch.setattr(
        power,
        "_probe_fc",
        lambda device: devices.append(str(device)) or {"status": status},
    )
    monkeypatch.setattr(
        power, "_run", lambda *_args, **_kwargs: pytest.fail("shutdown")
    )
    with pytest.raises(RuntimeError, match="fresh selected-FC disarmed"):
        power._remote_action(action)
    assert devices == ["/dev/serial0"]


@pytest.mark.parametrize("action", ["idle", "shutdown"])
def test_delayed_final_idle_check_expires_uart_confirmation(monkeypatch, action):
    clock = [10.0]
    monkeypatch.setattr(power.time, "monotonic", lambda: clock[0])

    def snapshot(*, deadline=None):
        if deadline is not None:
            assert deadline == pytest.approx(11.8)
            clock[0] += 3
        return idle()

    monkeypatch.setattr(power, "_pi_snapshot", snapshot)
    monkeypatch.setattr(power, "_probe_fc", lambda _device: disarmed())
    monkeypatch.setattr(
        power, "_run", lambda *_args, **_kwargs: pytest.fail("shutdown")
    )
    with pytest.raises(RuntimeError, match="expired"):
        power._remote_action(action)


def test_final_snapshot_shares_one_deadline_across_all_commands(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(power.time, "monotonic", lambda: clock[0])
    timeouts = []

    def run(command, *, timeout):
        timeouts.append(timeout)
        clock[0] += 0.5
        if "-c" in command:
            output = json.dumps(idle())
        elif command[0] == "systemctl":
            output = "LoadState=not-found\n"
        else:
            output = ""
        return SimpleNamespace(returncode=0, stdout=output)

    monkeypatch.setattr(power, "_run", run)
    assert power._pi_snapshot(deadline=11.8)["package_database_ok"] is True
    assert timeouts == pytest.approx([1.8, 1.3, 0.8])


@pytest.mark.parametrize("action", ["idle", "shutdown"])
def test_final_action_checks_uart_then_idle_within_freshness_window(
    monkeypatch, action
):
    events = []
    clock = [10.0]
    monkeypatch.setattr(power.time, "monotonic", lambda: clock[0])

    def snapshot(*, deadline=None):
        events.append("initial snapshot" if deadline is None else "final snapshot")
        if deadline is not None:
            clock[0] += 0.5
        return idle()

    def probe(device):
        events.append(str(device))
        return disarmed()

    def run(command, *, timeout):
        events.append(command[3])
        assert timeout == pytest.approx(1.3)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(power, "_pi_snapshot", snapshot)
    monkeypatch.setattr(power, "_probe_fc", probe)
    monkeypatch.setattr(power, "_run", run)
    result = power._remote_action(action)
    assert result.get("idle" if action == "idle" else "shutdown_requested") is True
    assert events == ["initial snapshot", "/dev/serial0", "final snapshot"] + (
        ["poweroff"] if action == "shutdown" else []
    )


def test_remote_actions_cannot_shutdown_the_laptop(monkeypatch):
    monkeypatch.setattr(power, "is_raspberry_pi", lambda: False)
    monkeypatch.setattr(power, "_pi_snapshot", lambda: pytest.fail("inspected laptop"))
    monkeypatch.setattr(power, "_run", lambda *_args: pytest.fail("shut down laptop"))
    with pytest.raises(RuntimeError, match="Raspberry Pi"):
        power._remote_action("shutdown")


def test_ssh_disconnect_never_retries_mutation(monkeypatch):
    calls = []
    monkeypatch.setattr(
        power,
        "_run",
        lambda command, **_kwargs: (
            calls.append(command) or SimpleNamespace(returncode=255)
        ),
    )
    with pytest.raises(RuntimeError, match="unknown"):
        power._remote(["ssh", "target", "command snapshot"], "shutdown")
    assert len(calls) == 1


@pytest.mark.parametrize("response", [None, [], "not a response", 1])
def test_non_object_pi_response_is_a_controlled_failure(monkeypatch, response):
    calls = []

    def run(command, **_kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout=json.dumps(response))

    monkeypatch.setattr(power, "_run", run)
    with pytest.raises(RuntimeError, match="invalid Pi response"):
        power._remote(["ssh", "target", "command snapshot"], "shutdown")
    assert len(calls) == 1


def test_ssh_uses_existing_target_config_and_quoted_project(monkeypatch):
    monkeypatch.setenv("PI_HOST", "seb@example.test")
    monkeypatch.setenv("PI_DIR", "/home/seb/project with $(literal)")
    monkeypatch.setenv("SSH_CONFIG", "/tmp/ssh config")
    commands = power._ssh_commands("snapshot")
    assert len(commands) == 1
    assert commands[0][:3] == ["ssh", "-F", "/tmp/ssh config"]
    assert "StrictHostKeyChecking=yes" in commands[0]
    assert "cd '/home/seb/project with $(literal)'" in commands[0][-1]
    assert "uv run --no-sync python -m ai_drone.cli.power" in commands[0][-1]


def test_owners_fails_closed_on_incomplete_scan(monkeypatch):
    monkeypatch.setattr(
        power,
        "_run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0, stdout=json.dumps({"complete": False})
        ),
    )
    with pytest.raises(RuntimeError, match="incomplete"):
        power._owners()
