"""Tests for the simulated vehicle used to rehearse flight commands.

The double is only useful if it refuses what a real ArduPilot refuses.  These
tests pin the refusals, not the flight dynamics.
"""

from __future__ import annotations

from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.sim.vehicle import (
    COPTER_MODES,
    DEFAULT_PARAMETERS,
    TOUCHDOWN_ALTITUDE_M,
    Fault,
    SimulatedVehicle,
)


def vehicle(**kwargs) -> SimulatedVehicle:
    # Never opened, so no socket is bound: these tests drive the state machine.
    return SimulatedVehicle("tcpin:127.0.0.1:0", **kwargs)


def test_arming_is_refused_outside_a_mode_that_can_be_armed():
    sim = vehicle()
    assert sim.state.mode == COPTER_MODES["STABILIZE"]
    assert sim._arm_or_disarm(True) == mavlink.MAV_RESULT_DENIED
    assert sim.state.armed is False


def test_arming_succeeds_in_guided():
    sim = vehicle()
    sim.state.mode = COPTER_MODES["GUIDED"]
    assert sim._arm_or_disarm(True) == mavlink.MAV_RESULT_ACCEPTED
    assert sim.state.armed is True


def test_the_refuse_arm_fault_denies_a_valid_arm_request():
    sim = vehicle(fault=Fault.REFUSE_ARM)
    sim.state.mode = COPTER_MODES["GUIDED"]
    assert sim._arm_or_disarm(True) == mavlink.MAV_RESULT_DENIED


def test_disarming_an_airborne_vehicle_is_refused():
    sim = vehicle()
    sim.state.mode = COPTER_MODES["GUIDED"]
    sim._arm_or_disarm(True)
    sim.state.altitude_m = 0.5
    assert sim._arm_or_disarm(False) == mavlink.MAV_RESULT_DENIED
    assert sim.state.armed is True


def test_takeoff_requires_an_armed_vehicle_in_guided():
    sim = vehicle()
    assert sim._takeoff(0.4) == mavlink.MAV_RESULT_DENIED
    sim.state.mode = COPTER_MODES["GUIDED"]
    sim._arm_or_disarm(True)
    assert sim._takeoff(0.4) == mavlink.MAV_RESULT_ACCEPTED
    assert sim.state.takeoff_target_m == 0.4


def test_takeoff_rejects_a_nonpositive_target():
    sim = vehicle()
    sim.state.mode = COPTER_MODES["GUIDED"]
    sim._arm_or_disarm(True)
    assert sim._takeoff(0.0) == mavlink.MAV_RESULT_DENIED


def test_a_climb_stops_at_the_commanded_altitude():
    sim = vehicle()
    sim.state.mode = COPTER_MODES["GUIDED"]
    sim._arm_or_disarm(True)
    sim._takeoff(0.4)
    for _ in range(200):
        sim.advance(0.05)
    assert sim.state.altitude_m == 0.4


def test_the_runaway_fault_climbs_past_the_commanded_altitude():
    sim = vehicle(fault=Fault.ALTITUDE_RUNAWAY)
    sim.state.mode = COPTER_MODES["GUIDED"]
    sim._arm_or_disarm(True)
    sim._takeoff(0.4)
    sim.state.altitude_m = 0.2
    for _ in range(100):
        sim.advance(0.05)
    assert sim.state.altitude_m > 0.4


def test_land_descends_and_then_disarms():
    sim = vehicle()
    sim.state.mode = COPTER_MODES["GUIDED"]
    sim._arm_or_disarm(True)
    sim.state.altitude_m = 0.4
    sim.state.mode = COPTER_MODES["LAND"]
    for _ in range(200):
        sim.advance(0.05)
    assert sim.state.altitude_m == 0.0
    assert sim.state.armed is False


def test_the_refuse_land_fault_stays_airborne():
    sim = vehicle(fault=Fault.REFUSE_LAND)
    sim.state.mode = COPTER_MODES["GUIDED"]
    sim._arm_or_disarm(True)
    sim.state.altitude_m = 0.4
    sim.state.mode = COPTER_MODES["LAND"]
    for _ in range(200):
        sim.advance(0.05)
    assert sim.state.altitude_m == 0.4
    assert sim.state.armed is True


def test_the_battery_sag_fault_drains_far_faster_than_a_hover():
    normal = vehicle()
    sagging = vehicle(fault=Fault.BATTERY_SAG)
    for sim in (normal, sagging):
        sim.state.mode = COPTER_MODES["GUIDED"]
        sim._arm_or_disarm(True)
        sim.state.altitude_m = 0.4
        for _ in range(20):
            sim.advance(0.1)
    assert sagging.state.battery_v < normal.state.battery_v - 1.0


def test_a_disarmed_vehicle_does_not_drain_its_battery():
    sim = vehicle()
    for _ in range(50):
        sim.advance(0.1)
    assert sim.state.battery_v == 16.0


def test_the_double_reports_a_correctly_configured_aircraft():
    # A rehearsal is worthless if the double is more permissive than the gate
    # DroneController applies before it will arm.
    assert DEFAULT_PARAMETERS["ARMING_CHECK"] == 1.0
    assert int(DEFAULT_PARAMETERS["LOG_BACKEND_TYPE"]) & 5
    assert DEFAULT_PARAMETERS["LOG_BITMASK"] > 0


def test_touchdown_altitude_is_below_the_controller_altitude_floor():
    assert 0.0 < TOUCHDOWN_ALTITUDE_M < 0.15
