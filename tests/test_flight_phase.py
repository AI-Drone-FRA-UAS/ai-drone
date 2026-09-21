from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight.controller import (
    DroneController,
    FlightSafetyError,
    HumanControlTaken,
)
from ai_drone.flight.phase import (
    Arm,
    Armed,
    ArmPending,
    Climb,
    CommandLong,
    Disarm,
    Flight,
    Human,
    Landed,
    Landing,
    SetMode,
    Unclaimed,
    cleanup,
    observed_disarm,
    owns_control,
    request_landing,
)
from ai_drone.flight.state import Heartbeat, Sample, VehicleState


@pytest.mark.parametrize(
    "phase,owned,obligation",
    [
        (Unclaimed(), False, None),
        (ArmPending(), True, "disarm"),
        (Armed(), True, "disarm"),
        (Flight("taking_off", 0.05), True, "land"),
        (Flight("holding_altitude", 0.05), True, "land"),
        (Flight("awaiting_loiter", 0.05), True, "land"),
        (Flight("loitering", 0.05), True, "land"),
        (Landing(ArmPending()), True, "disarm"),
        (Landing(Flight("taking_off", 0.05)), True, "land"),
        (Landed(), False, None),
        (Human(True), False, "supervise"),
    ],
)
def test_ownership_and_cleanup_are_independent_of_observed_arming(
    phase, owned, obligation
):
    assert owns_control(phase) is owned
    assert cleanup(phase) == obligation


@pytest.mark.parametrize(
    "phase",
    [
        ArmPending(),
        Flight("taking_off", 0.05),
        Landing(Flight("loitering", 0.05)),
        Human(True),
    ],
)
def test_disarmed_observation_cannot_erase_ambiguous_or_human_responsibility(phase):
    assert observed_disarm(phase) is phase


def test_landing_latch_remains_after_confirmation_and_cannot_erase_human():
    for phase in (Landed(), Human(True)):
        assert request_landing(phase) is phase


def controller_in(phase):
    drone = DroneController(device="tcp:127.0.0.1:5760")
    drone.phase = phase
    drone.connection = MagicMock()
    drone.connection.recv_match.return_value = None
    return drone


@pytest.mark.parametrize("command", [Arm(), Climb(0.3, 0.0)])
def test_ambiguous_effect_intent_is_latched_before_failed_write(command):
    drone = controller_in(Armed())
    connection = drone.connection
    if isinstance(command, Arm):
        connection.arducopter_arm.side_effect = OSError("ambiguous arm")
    else:
        connection.mav.set_attitude_target_send.side_effect = OSError("ambiguous climb")
    with pytest.raises(OSError, match="ambiguous"):
        drone._write_command(command)
    assert cleanup(drone.phase) == ("disarm" if isinstance(command, Arm) else "land")
    assert drone.command_attempts[-1].outcome == "failed"
    assert drone.command_attempts[-1].error is not None
    drone.phase = observed_disarm(drone.phase)
    land, disarm = MagicMock(), MagicMock()
    drone.land, drone.disarm = land, disarm
    drone.__exit__(OSError, OSError("original"), None)
    (disarm if isinstance(command, Arm) else land).assert_called_once()
    (land if isinstance(command, Arm) else disarm).assert_not_called()


@pytest.mark.parametrize(
    "command",
    [
        Arm(),
        Disarm(),
        SetMode("LAND"),
        Climb(0.3, 0.0),
        CommandLong(512, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ],
)
def test_every_write_boundary_yields_to_latched_human(command):
    drone = controller_in(Human(True))
    with pytest.raises(HumanControlTaken):
        drone._write_command(command)
    assert drone.connection.mock_calls == []
    assert not drone.command_attempts


def test_passive_armed_observation_cannot_authorize_climb():
    drone = controller_in(Unclaimed())
    drone.state = VehicleState(heartbeat=Sample(Heartbeat(True, "GUIDED_NOGPS"), 100.0))
    with pytest.raises(FlightSafetyError, match="arming by this controller"):
        drone._write_command(Climb(0.3, 0.0))
    assert not drone.owns_control
    drone.connection.mav.set_attitude_target_send.assert_not_called()


def test_cleanup_cannot_resume_climb_or_use_ack_helper_to_bypass_command_policy():
    drone = controller_in(Landing(Flight("taking_off", 0.05)))
    with pytest.raises(FlightSafetyError, match="landing is latched"):
        drone._write_command(Climb(0.3, 0.0))
    with pytest.raises(FlightSafetyError, match="only authorizes passive"):
        drone._send_command_long_and_wait_ack(
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (1.0,) * 7
        )
    drone.connection.mav.set_attitude_target_send.assert_not_called()
    drone.connection.mav.command_long_send.assert_not_called()


def test_written_is_not_an_arming_confirmation():
    drone = controller_in(Unclaimed())
    drone._write_command(Arm())
    assert isinstance(drone.phase, ArmPending)
    assert not drone.is_armed
    assert drone.command_attempts[-1].outcome == "written"


@pytest.mark.parametrize("command", [Arm(), SetMode("GUIDED_NOGPS"), SetMode("LOITER")])
def test_landing_latch_also_blocks_direct_arm_and_mode_writers(command):
    drone = controller_in(Landing(Flight("taking_off", 0.05)))
    before = drone.phase
    with pytest.raises(FlightSafetyError, match="landing is latched"):
        drone._write_command(command)
    assert drone.phase is before
    assert drone.connection.mock_calls == []


def test_command_audit_keeps_whole_session_totals_and_first_intents_when_recent_buffer_wraps(
    monkeypatch,
):
    import json

    clock = [100.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    drone = controller_in(Unclaimed())
    drone._write_command(Arm())
    drone.phase = Armed()
    for index in range(300):
        clock[0] += 0.4 if index == 150 else 0.05
        drone._write_command(Climb(0.0, 0.0))
    drone._write_command(SetMode("LAND"))
    audit = drone.command_audit()
    assert audit["attempted"] == audit["written"] == 302
    assert audit["failed"] == 0
    assert audit["omitted_attempt_details"] == 46
    assert len(audit["recent_attempts"]) == 256
    assert audit["first_attempts"]["Arm"]["sequence"] == 1
    assert audit["first_attempts"]["LAND"]["sequence"] == 302
    assert audit["maximum_climb_attempt_gap_s"] == pytest.approx(0.4)
    assert audit["maximum_climb_write_gap_s"] == pytest.approx(0.4)
    json.dumps(audit, allow_nan=False)


def test_audit_includes_failed_writes_and_heartbeat_recovery_gap(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    drone = controller_in(Unclaimed())
    drone.connection.arducopter_arm.side_effect = OSError("ambiguous")
    with pytest.raises(OSError):
        drone._write_command(Arm())
    drone._pump_gcs_heartbeat()
    clock[0] = 102.0
    drone.connection.mav.heartbeat_send.side_effect = OSError("lost link")
    with pytest.raises(OSError):
        drone._pump_gcs_heartbeat()
    clock[0] = 103.0
    drone.connection.mav.heartbeat_send.side_effect = None
    drone._pump_gcs_heartbeat()
    audit = drone.command_audit()
    assert audit["attempted"] == audit["failed"] == 1
    assert audit["written"] == 0
    assert audit["first_attempts"]["Arm"]["outcome"] == "failed"
    assert audit["heartbeat_failures"] == 1
    assert audit["heartbeat_writes"] == 2
    assert audit["maximum_heartbeat_gap_s"] == 3.0
