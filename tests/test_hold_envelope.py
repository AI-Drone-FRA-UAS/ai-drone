"""A failed post-takeoff hold cannot silently complete on fresh low telemetry."""

from dataclasses import replace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight.controller import DroneController, FlightSafetyError
from ai_drone.flight.limits import HOLD_ALTITUDE_TOLERANCE_M
from ai_drone.flight.phase import Flight, Human, Landing
from ai_drone.flight.state import AltitudeAlignment, Heartbeat, Sample, VehicleState


@pytest.fixture
def drone(monkeypatch):
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 100.0)
    controller = DroneController(device="tcp:127.0.0.1:5760")
    controller.connection = MagicMock()
    controller.connection.recv_match.return_value = None
    # The pad is 0.12m above the floor, and the validated gain was 0.5m.
    controller.phase = Flight("loitering", 0.12, 0.62, 99.0)
    controller.state = VehicleState(
        heartbeat=Sample(Heartbeat(True, "LOITER"), 100.0),
        altitude=Sample(0.62, 100.0),
        local_altitude=Sample(0.5, 100.0),
        alignment=AltitudeAlignment(0.12, Sample(0.62, 100.0)),
        yaw=Sample(0.0, 100.0),
        rc_channels=Sample(0, 100.0),
        flow_quality=Sample(60, 100.0),
        ekf_flags=Sample(mavlink.EKF_POS_HORIZ_REL | mavlink.EKF_VELOCITY_HORIZ, 100.0),
    )
    return controller


@pytest.mark.parametrize("stage", ["holding_altitude", "awaiting_loiter", "loitering"])
@pytest.mark.parametrize("source", ["downward range", "aligned local altitude"])
def test_each_active_hold_phase_lands_on_either_fresh_lower_datum(drone, stage, source):
    drone.phase = replace(drone.phase, stage=stage)
    low = 0.62 - HOLD_ALTITUDE_TOLERANCE_M - 0.001
    if source == "downward range":
        drone.state = replace(drone.state, altitude=Sample(low, 100.0))
    else:
        drone.state = replace(
            drone.state,
            local_altitude=Sample(low - 0.12, 100.0),
            alignment=AltitudeAlignment(0.12, Sample(low, 100.0)),
        )
    with pytest.raises(FlightSafetyError, match=source) as failure:
        drone.update_telemetry()
    assert "hold lower bound 0.52 m" in str(failure.value)
    assert "floor target 0.62 m" in str(failure.value)
    assert isinstance(drone.phase, Landing)
    assert isinstance(drone.phase.previous, Flight)
    assert drone.phase.previous.floor_target_m == 0.62
    drone.connection.mav.set_mode_send.assert_called_once_with(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    )
    drone.connection.arducopter_disarm.assert_not_called()
    audit = drone.command_audit()
    assert audit["session_first_safety_failure"] == str(failure.value)
    assert audit["hold_altitude_tolerance_m"] == 0.10


def test_hold_lower_bound_is_inclusive_and_uses_floor_not_pad_datum(drone):
    lower = 0.62 - HOLD_ALTITUDE_TOLERANCE_M
    drone.state = replace(
        drone.state,
        altitude=Sample(lower, 100.0),
        local_altitude=Sample(lower - 0.12, 100.0),
        alignment=AltitudeAlignment(0.12, Sample(lower, 100.0)),
    )
    drone.update_telemetry()
    drone.connection.mav.set_mode_send.assert_not_called()


def test_literal_decimal_lower_boundary_does_not_fail_due_to_binary_roundoff(drone):
    drone.phase = replace(drone.phase, floor_target_m=0.52)
    drone.state = replace(drone.state, altitude=Sample(0.42, 100.0))
    drone.update_telemetry()
    drone.connection.mav.set_mode_send.assert_not_called()
    drone.state = replace(drone.state, altitude=Sample(0.419999, 100.0))
    with pytest.raises(FlightSafetyError, match="hold lower bound"):
        drone.update_telemetry()


@pytest.mark.parametrize("source", ["range", "local"])
def test_fresh_pre_target_climb_samples_do_not_prove_subsequent_hold_loss(
    drone, source
):
    drone.phase = replace(drone.phase, target_reached_at=99.8)
    if source == "range":
        drone.state = replace(drone.state, altitude=Sample(0.20, 99.7))
    else:
        drone.state = replace(
            drone.state,
            local_altitude=Sample(0.08, 99.7),
            alignment=AltitudeAlignment(0.12, Sample(0.20, 99.7)),
        )
    drone.update_telemetry()
    drone.connection.mav.set_mode_send.assert_not_called()
    if source == "range":
        drone.state = replace(drone.state, altitude=Sample(0.20, 99.8))
    else:
        drone.state = replace(
            drone.state,
            local_altitude=Sample(0.08, 99.8),
            alignment=AltitudeAlignment(0.12, Sample(0.20, 99.8)),
        )
    with pytest.raises(FlightSafetyError, match="hold lower bound"):
        drone.update_telemetry()


@pytest.mark.parametrize("received", [98.999, 100.001, float("nan")])
def test_old_or_future_local_low_values_cannot_trip_hold_lower_bound(drone, received):
    drone.state = replace(
        drone.state,
        local_altitude=Sample(-0.12, received),
        alignment=AltitudeAlignment(0.12, Sample(0.0, received)),
    )
    drone.update_telemetry()
    drone.connection.mav.set_mode_send.assert_not_called()


@pytest.mark.parametrize("raw", [None, Sample(-0.12, 98.999), Sample(-0.12, 100.001)])
def test_invalid_or_old_raw_local_revokes_otherwise_recent_alignment(drone, raw):
    drone.state = replace(
        drone.state,
        local_altitude=raw,
        alignment=AltitudeAlignment(0.12, Sample(0.0, 100.0)),
    )
    drone.update_telemetry()
    drone.connection.mav.set_mode_send.assert_not_called()


def test_fresh_raw_local_cannot_refresh_stale_alignment_receipt(drone):
    drone.state = replace(
        drone.state,
        local_altitude=Sample(-0.12, 100.0),
        alignment=AltitudeAlignment(0.12, Sample(0.0, 98.999)),
    )
    drone.update_telemetry()
    drone.connection.mav.set_mode_send.assert_not_called()


@pytest.mark.parametrize("received", [98.999, 100.001])
def test_stale_or_future_range_keeps_existing_stale_input_error(drone, received):
    drone.state = replace(drone.state, altitude=Sample(0.0, received))
    with pytest.raises(FlightSafetyError, match="altitude became stale"):
        drone.update_telemetry()
    assert drone._landing_commanded


@pytest.mark.parametrize("stage", ["taking_off", "landing", "human"])
def test_low_height_is_allowed_during_takeoff_landing_and_human_control(drone, stage):
    previous = Flight("taking_off", 0.12, 0.62)
    drone.phase = {
        "taking_off": previous,
        "landing": Landing(previous),
        "human": Human(True),
    }[stage]
    drone.state = replace(drone.state, altitude=Sample(0.02, 100.0))
    drone.update_telemetry()
    drone.connection.mav.set_mode_send.assert_not_called()


def test_missing_declared_hold_target_fails_closed(drone):
    drone.phase = replace(drone.phase, floor_target_m=None)
    with pytest.raises(FlightSafetyError, match="declared floor-referenced target"):
        drone.update_telemetry()
    assert drone._landing_commanded


def test_unexpected_fresh_disarm_does_not_complete_hold_or_erase_land_obligation(drone):
    drone.state = replace(
        drone.state, heartbeat=Sample(Heartbeat(False, "LOITER"), 100.0)
    )
    with pytest.raises(FlightSafetyError, match="disarmed unexpectedly"):
        drone.update_telemetry()
    assert isinstance(drone.phase, Landing)
    drone.connection.arducopter_disarm.assert_not_called()
    drone.connection.mav.set_mode_send.assert_called_once()


def test_failed_land_write_retains_low_height_reason_and_cleanup(drone):
    drone.state = replace(drone.state, altitude=Sample(0.02, 100.0))
    drone.connection.mav.set_mode_send.side_effect = OSError("serial write failed")
    with pytest.raises(FlightSafetyError, match="hold lower bound") as failure:
        drone.update_telemetry()
    assert isinstance(drone.phase, Landing)
    assert drone.command_attempts[-1].outcome == "failed"
    assert drone.command_audit()["session_first_safety_failure"] == str(failure.value)
    drone.connection.arducopter_disarm.assert_not_called()


def test_explicit_handoff_is_resolved_before_low_height_land(drone):
    drone.state = replace(
        drone.state, altitude=Sample(0.02, 100.0), rc_channels=Sample(8, 100.0)
    )
    drone.human_takeover_requested = lambda: True
    drone.update_telemetry()
    assert isinstance(drone.phase, Human)
    drone.connection.mav.set_mode_send.assert_not_called()
    drone.connection.arducopter_disarm.assert_not_called()


def test_loiter_transition_keeps_declared_target_in_each_stage(drone, monkeypatch):
    drone.phase = replace(drone.phase, stage="holding_altitude")
    drone.state = replace(
        drone.state, heartbeat=Sample(Heartbeat(True, "GUIDED_NOGPS"), 100.0)
    )

    def wait(**_kwargs):
        assert drone.phase == Flight("awaiting_loiter", 0.12, 0.62, 99.0)

    def mode(_mode):
        drone.state = replace(
            drone.state, heartbeat=Sample(Heartbeat(True, "LOITER"), 100.0)
        )

    monkeypatch.setattr(drone, "wait_for_relative_position", wait)
    monkeypatch.setattr(drone, "set_mode", mode)
    drone.enter_loiter()
    assert drone.phase == Flight("loitering", 0.12, 0.62, 99.0)
