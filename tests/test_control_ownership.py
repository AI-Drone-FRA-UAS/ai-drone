from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight.controller import (
    DroneController,
    FlightSafetyError,
    HumanControlTaken,
)
from ai_drone.flight.ownership import OwnershipPolicy, human_takeover_allowed
from ai_drone.mavlink.shared import received_monotonic


def _message(kind: str, *, system: int = 1, component: int = 1, **fields):
    return SimpleNamespace(
        get_type=lambda: kind,
        get_srcSystem=lambda: system,
        get_srcComponent=lambda: component,
        **fields,
    )


def _heartbeat(*, armed: bool = True, mode: int = 2, **fields):
    return _message(
        "HEARTBEAT",
        base_mode=mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        | (mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0),
        custom_mode=mode,
        autopilot=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        type=mavlink.MAV_TYPE_QUADROTOR,
        **fields,
    )


def _rc(*, count: int = 8, **fields):
    return _message("RC_CHANNELS", chancount=count, time_boot_ms=10_000, **fields)


@pytest.fixture
def controlled(monkeypatch):
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 100.0)
    monkeypatch.setattr(
        "ai_drone.flight.controller.received_monotonic",
        lambda message, *, default: getattr(message, "received", default),
    )
    drone = DroneController(device="tcp:127.0.0.1:5760")
    drone.connection = MagicMock()
    drone.connection.recv_match.return_value = None
    drone.connection.mode_mapping.return_value = {"LAND": 9, "LOITER": 5}
    drone.connection.flightmode = "GUIDED_NOGPS"
    drone.flight_mode = "GUIDED_NOGPS"
    drone.is_armed = drone.is_flying = True
    drone._flight_started_by_controller = drone._armed_by_controller = True
    drone.last_heartbeat_time = drone.last_rc_channels_time = 100.0
    drone.rc_channel_count = 0
    return drone


def _queue(drone, *messages):
    drone.connection.recv_match.side_effect = [*messages, None]


def _take_over(drone):
    drone.human_takeover_requested = lambda: True
    _queue(drone, _heartbeat(), _rc())
    drone.update_telemetry()


def test_policy_cannot_classify_guided_flight_as_human_control():
    with pytest.raises(ValueError, match="radio-pilot modes"):
        OwnershipPolicy(pilot_modes=frozenset({"GUIDED_NOGPS"}))


@pytest.mark.parametrize(
    "change",
    [
        {"requested": False},
        {"armed": False},
        {"mode": "GUIDED_NOGPS"},
        {"mode": None},
        {"heartbeat_received": 97.0},
        {"rc_received": 98.9},
        {"rc_received": 100.1},
        {"rc_received": float("nan")},
        {"rc_channels": 0},
        {"rc_channels": None},
    ],
)
def test_handoff_requires_explicit_current_pilot_evidence(change):
    evidence = {
        "requested": True,
        "armed": True,
        "mode": "ALT_HOLD",
        "heartbeat_received": 99.5,
        "rc_received": 99.8,
        "rc_channels": 8,
        "now": 100.0,
    }
    assert human_takeover_allowed(OwnershipPolicy(), **evidence)
    assert not human_takeover_allowed(OwnershipPolicy(), **(evidence | change))


def test_operator_loss_lands_owned_flight_and_keeps_gcs_heartbeat(controlled):
    controlled.operator_alive = lambda: False
    with pytest.raises(FlightSafetyError, match="operator heartbeat lost"):
        controlled.update_telemetry()
    controlled.connection.mav.set_mode_send.assert_called_once_with(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    )
    controlled.connection.mav.heartbeat_send.assert_called_once()
    controlled.update_telemetry()  # expired operator lease cannot interrupt landing
    assert controlled.control_owner == "autonomous"


def test_operator_callback_failure_is_loss(controlled):
    def failed():
        raise OSError("heartbeat state unreadable")

    controlled.operator_alive = failed
    with pytest.raises(FlightSafetyError, match="operator heartbeat lost"):
        controlled.update_telemetry()
    controlled.connection.mav.set_mode_send.assert_called_once()


def test_unavailable_operator_blocks_arming_before_commands(controlled):
    controlled.operator_alive = lambda: False
    controlled._flight_started_by_controller = controlled._armed_by_controller = False
    controlled.is_armed = controlled.is_flying = False
    with pytest.raises(FlightSafetyError, match="operator heartbeat lost"):
        controlled.arm()
    controlled.connection.arducopter_arm.assert_not_called()
    controlled.connection.mav.set_mode_send.assert_not_called()


def test_receiver_presence_does_not_transfer_control(controlled):
    _queue(controlled, _heartbeat(), _rc())
    with pytest.raises(FlightSafetyError, match="receiver topology changed"):
        controlled.update_telemetry()
    assert controlled.control_owner == "autonomous"
    controlled.connection.mav.set_mode_send.assert_called_once()


@pytest.mark.parametrize(
    "messages",
    [
        [_heartbeat(system=2), _rc()],
        [_heartbeat(), _rc(component=2)],
        [_heartbeat(received=90.0), _rc()],
        [_heartbeat(), _rc(received=90.0)],
        [_heartbeat(mode=20), _rc()],
    ],
)
def test_invalid_takeover_evidence_cannot_override_operator_loss(controlled, messages):
    controlled.last_heartbeat_time = controlled.last_rc_channels_time = 90.0
    controlled.operator_alive = lambda: False
    controlled.human_takeover_requested = lambda: True
    _queue(controlled, *messages)
    with pytest.raises(FlightSafetyError, match="operator heartbeat lost"):
        controlled.update_telemetry()
    assert controlled.control_owner == "autonomous"
    controlled.connection.mav.set_mode_send.assert_called_once()


def test_takeover_is_resolved_before_operator_loss_or_stop(controlled):
    controlled.operator_alive = lambda: False
    controlled.stop_requested = lambda: True
    _take_over(controlled)
    assert controlled.control_owner == "human"
    assert controlled.is_flying and controlled.is_armed
    assert not controlled.owns_control
    assert controlled.flight_mode == "ALT_HOLD"  # selected packet, not global cache
    controlled.connection.mav.set_mode_send.assert_not_called()


def test_takeover_callback_failure_never_grants_authority(controlled):
    def failed():
        raise OSError("ownership state unreadable")

    controlled.human_takeover_requested = failed
    controlled.operator_alive = lambda: False
    with pytest.raises(FlightSafetyError, match="operator heartbeat lost"):
        controlled.update_telemetry()
    assert controlled.control_owner == "autonomous"


@pytest.mark.parametrize(
    "command",
    [
        lambda drone: drone.arm(),
        lambda drone: drone.takeoff(0.5),
        lambda drone: drone.set_mode("LOITER"),
        lambda drone: drone._send_level_climb(0.3),
        lambda drone: drone.enter_loiter(),
        lambda drone: drone.hold_loiter(1.0),
        lambda drone: drone.land(),
        lambda drone: drone.disarm(),
        lambda drone: drone._request_disarm(),
    ],
)
def test_no_controller_command_can_override_human_ownership(controlled, command):
    _take_over(controlled)
    with pytest.raises(HumanControlTaken):
        command(controlled)
    controlled.connection.mav.set_mode_send.assert_not_called()
    controlled.connection.mav.set_attitude_target_send.assert_not_called()
    controlled.connection.arducopter_arm.assert_not_called()
    controlled.connection.arducopter_disarm.assert_not_called()


def test_emergency_and_context_cleanup_cannot_override_human(controlled):
    _take_over(controlled)
    connection = controlled.connection
    controlled.emergency_stop()
    _queue(controlled, _heartbeat(armed=False), _rc(count=0))
    controlled.__exit__(RuntimeError, RuntimeError("recorder failed"), None)
    connection.mav.set_mode_send.assert_not_called()
    connection.arducopter_disarm.assert_not_called()
    connection.close.assert_called_once()


def test_human_latch_survives_reconnection_disarm_and_lost_rc(controlled, monkeypatch):
    _take_over(controlled)
    controlled.human_takeover_requested = lambda: False
    controlled.operator_alive = lambda: True
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 101.0)
    _queue(controlled, _heartbeat(armed=False), _rc(count=0))
    controlled.update_telemetry()
    assert controlled.control_owner == "human"
    assert not controlled.is_flying and not controlled.is_armed
    assert controlled.connection.mav.heartbeat_send.call_count == 2
    with pytest.raises(HumanControlTaken):
        controlled.arm()


def test_stale_disarmed_heartbeat_does_not_revoke_autonomous_cleanup(controlled):
    _queue(controlled, _heartbeat(armed=False, received=90.0))
    controlled.update_telemetry()
    assert controlled.is_armed and controlled.owns_control


def test_shared_receive_time_cannot_make_queued_rc_look_current(
    controlled, monkeypatch
):
    monkeypatch.setattr(
        "ai_drone.flight.controller.received_monotonic", received_monotonic
    )
    controlled.operator_alive = lambda: False
    controlled.human_takeover_requested = lambda: True
    _queue(controlled, _heartbeat(), _rc(_received_monotonic=90.0))
    with pytest.raises(FlightSafetyError, match="operator heartbeat lost"):
        controlled.update_telemetry()
    assert controlled.last_rc_channels_time == 90.0
    assert controlled.control_owner == "autonomous"


def test_takeover_during_landing_prevents_the_next_land_command(controlled):
    controlled.human_takeover_requested = lambda: True
    _queue(controlled, _heartbeat(), _rc())
    with pytest.raises(HumanControlTaken):
        controlled.land()
    assert controlled.control_owner == "human"
    controlled.connection.mav.set_mode_send.assert_not_called()
