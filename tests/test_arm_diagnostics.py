from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight.controller import DroneController
from ai_drone.flight.phase import Arm, ArmPending, Disarm


def status(
    text, received, *, system=1, component=1, severity=mavlink.MAV_SEVERITY_CRITICAL
):
    return SimpleNamespace(
        get_type=lambda: "STATUSTEXT",
        get_srcSystem=lambda: system,
        get_srcComponent=lambda: component,
        _received_monotonic=received,
        text=text,
        severity=severity,
    )


@pytest.fixture
def diagnostic_controller(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    drone = DroneController(device="tcp:127.0.0.1:5760")
    drone.connection = MagicMock()
    drone.connection.recv_match.return_value = None
    return drone, clock


def test_arming_timeout_preserves_fresh_fc_mag_reason_without_shortening_deadline(
    diagnostic_controller, monkeypatch
):
    drone, clock = diagnostic_controller
    for method in (
        "update_telemetry",
        "verify_firmware",
        "verify_arming_checks",
        "verify_nogps_loiter_parameters",
        "verify_onboard_logging",
        "wait_for_optical_flow",
        "wait_for_attitude",
        "wait_for_no_rc_input",
        "verify_battery_before_arming",
        "set_mode",
        "_fresh_disarmed",
    ):
        monkeypatch.setattr(drone, method, lambda *_args, **_kwargs: None)
    monkeypatch.setattr(drone, "wait_for_altitude", lambda **_kwargs: 0.05)
    monkeypatch.setattr(drone, "altitude_is_fresh", lambda: True)
    reason = "Arm: Check mag field: 1273, max 875, min 185"

    def receive(**_kwargs):
        clock[0] += 0.5
        return status(reason, clock[0])

    drone.connection.recv_match.side_effect = receive
    with pytest.raises(TimeoutError, match="Check mag field: 1273") as failure:
        drone.arm(timeout=2.0)
    assert clock[0] == 102.0
    assert isinstance(drone.phase, ArmPending)
    assert not drone.is_armed
    drone.connection.arducopter_arm.assert_called_once()
    audit = drone.command_audit()
    assert audit["arm_requested_monotonic"] == 100.0
    assert audit["arm_request_finished_monotonic"] == 102.0
    assert audit["arm_diagnostics"][0] == {
        "text": reason,
        "severity": 2,
        "received_monotonic": 100.5,
    }
    assert failure.value.__notes__ == [f"selected FC arming diagnostics: {reason}"]


@pytest.mark.parametrize(
    "message,now",
    [
        (status("Arm: stale before request", 99.9), 100.1),
        (status("Arm: request boundary", 100.0), 100.1),
        (status("Arm: queued too long", 100.1), 103.0),
        (status("Arm: future receipt", 101.0), 100.5),
        (status("Arm: invalid receipt", float("nan")), 100.5),
        (status("Arm: foreign system", 100.1, system=2), 100.1),
        (status("Arm: foreign component", 100.1, component=2), 100.1),
        (status("Arm: info only", 100.1, severity=6), 100.1),
        (status("unrelated diagnostic", 100.1), 100.1),
        (status("Arm: malformed severity", 100.1, severity=True), 100.1),
    ],
)
def test_arm_reason_cannot_use_old_spoofed_future_or_unrelated_status(
    diagnostic_controller, message, now
):
    drone, _clock = diagnostic_controller
    drone._write_command(Arm())
    drone._process_message(message, now)
    assert drone._arming_diagnostic_detail() == ""
    assert drone.command_audit()["arm_diagnostics"] == []


def test_arm_diagnostics_are_bounded_and_do_not_leak_across_requests_or_cleanup(
    diagnostic_controller,
):
    drone, clock = diagnostic_controller
    drone._write_command(Arm())
    for number in range(40):
        clock[0] += 0.01
        drone._process_message(status(f"Arm: refusal {number}", clock[0]), clock[0])
    audit = drone.command_audit()
    assert len(audit["arm_diagnostics"]) == 32
    assert audit["omitted_arm_diagnostics"] == 8
    drone._write_command(Disarm())
    clock[0] += 0.1
    drone._process_message(status("Arm: after cleanup began", clock[0]), clock[0])
    assert len(drone.command_audit()["arm_diagnostics"]) == 32
    drone._write_command(Arm())
    assert drone.command_audit()["arm_diagnostics"] == []
    assert drone.command_audit()["omitted_arm_diagnostics"] == 0
