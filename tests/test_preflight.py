"""Tests for the read-only pre-arm assessment.

The snapshots here are the ones that matter operationally: a healthy aircraft,
and the configuration this airframe was actually found in on 2026-08-20 --
all pre-arm checks disabled and no horizontal position estimate.
"""

from __future__ import annotations

import pytest

from ai_drone.mavlink.preflight import (
    DOWNWARD,
    EKF_ATTITUDE,
    EKF_CONST_POS_MODE,
    EKF_POS_HORIZ_REL,
    EKF_POS_VERT_ABS,
    EKF_VELOCITY_HORIZ,
    EKF_VELOCITY_VERT,
    Snapshot,
    assess,
    describe_ekf_flags,
    guided_takeoff_blockers,
)

HEALTHY_PARAMETERS = {
    "ARMING_CHECK": 1.0,
    "LOG_BACKEND_TYPE": 4.0,
    "LOG_BITMASK": 180222.0,
    "BATT_LOW_VOLT": 10.5,
    "EK3_SRC1_POSXY": 3.0,
    "FLOW_TYPE": 5.0,
    "EK3_SRC1_VELXY": 5.0,
}

# The MicoAir MTF-01P delivers roughly 10 frames a second over the FC's UART5.
HEALTHY_FLOW_SAMPLES = 150
HEALTHY_FLOW_QUALITY = 45

HEALTHY = Snapshot(
    mode="GUIDED",
    armed=False,
    parameters=HEALTHY_PARAMETERS,
    ekf_flags=EKF_ATTITUDE | EKF_POS_HORIZ_REL | EKF_POS_VERT_ABS,
    gps_fix=3,
    gps_satellites=12,
    rangefinder_cm=15,
    rangefinder_orientation=DOWNWARD,
    battery_v=15.7,
    flow_samples=HEALTHY_FLOW_SAMPLES,
    flow_quality=HEALTHY_FLOW_QUALITY,
)

# What the aircraft reported over its own telemetry link.
AS_FOUND = Snapshot(
    mode="STABILIZE",
    armed=False,
    parameters={
        "ARMING_CHECK": 0.0,
        "LOG_BACKEND_TYPE": 4.0,
        "LOG_BITMASK": 180222.0,
        "BATT_LOW_VOLT": 10.5,
        "EK3_SRC1_POSXY": 0.0,
        "EK3_SRC1_VELXY": 5.0,
        "FLOW_TYPE": 5.0,
    },
    ekf_flags=(
        EKF_ATTITUDE
        | EKF_VELOCITY_HORIZ
        | EKF_VELOCITY_VERT
        | EKF_POS_VERT_ABS
        | EKF_CONST_POS_MODE
    ),
    gps_fix=1,
    gps_satellites=0,
    rangefinder_cm=2,
    rangefinder_orientation=DOWNWARD,
    battery_v=15.69,
    flow_samples=151,
    flow_quality=45,
)


def named(checks, name):
    return next(check for check in checks if check.name == name)


def test_healthy_vehicle_has_no_guided_takeoff_blockers():
    assert guided_takeoff_blockers(assess(HEALTHY)) == []


def test_disabled_arming_checks_block_takeoff():
    check = named(assess(AS_FOUND), "arming_checks")
    assert check.passed is False
    assert "ARMING_CHECK=0" in check.detail


def test_partially_bypassed_arming_checks_block_takeoff():
    snapshot = Snapshot(parameters={**HEALTHY_PARAMETERS, "ARMING_CHECK": 22.0})
    check = named(assess(snapshot), "arming_checks")
    assert check.passed is False
    assert "arbitrary subset" in check.detail


def test_the_gps_free_check_set_is_accepted():
    from ai_drone.mavlink.arming_checks import ALL_EXCEPT_GPS

    snapshot = Snapshot(
        parameters={**HEALTHY_PARAMETERS, "ARMING_CHECK": ALL_EXCEPT_GPS}
    )
    assert named(assess(snapshot), "arming_checks").passed is True


def test_a_flow_setup_on_the_ground_is_not_reported_as_misconfigured():
    # EK3_SRC1_POSXY=0 with EK3_SRC1_VELXY=5 is the correct flow-only setup:
    # flow supplies velocity, not position.  Calling it a misconfiguration
    # would send someone off to "fix" a working aircraft.
    check = named(assess(AS_FOUND), "horizontal_position")
    assert check.passed is False
    assert "flow-based setup" in check.detail
    assert "expected on the ground" in check.detail
    assert "PreArm" in check.detail


def test_a_vehicle_with_no_horizontal_source_at_all_says_so():
    snapshot = Snapshot(
        parameters={"EK3_SRC1_POSXY": 0.0, "EK3_SRC1_VELXY": 0.0},
        ekf_flags=EKF_ATTITUDE | EKF_CONST_POS_MODE,
    )
    check = named(assess(snapshot), "horizontal_position")
    assert check.passed is False
    assert "nothing can supply one" in check.detail


def test_a_flow_setup_with_a_dead_sensor_blames_the_sensor():
    snapshot = Snapshot(
        parameters={"EK3_SRC1_POSXY": 0.0, "EK3_SRC1_VELXY": 5.0, "FLOW_TYPE": 5.0},
        ekf_flags=EKF_ATTITUDE | EKF_CONST_POS_MODE,
        flow_samples=0,
    )
    assert (
        "not delivering usable frames"
        in named(assess(snapshot), "horizontal_position").detail
    )


def test_a_silent_flow_sensor_fails_its_own_check():
    snapshot = Snapshot(parameters={"FLOW_TYPE": 5.0}, flow_samples=0)
    check = named(assess(snapshot), "optical_flow")
    assert check.passed is False
    assert "not reaching the flight controller" in check.detail


def test_low_quality_flow_frames_are_rejected():
    snapshot = Snapshot(parameters={"FLOW_TYPE": 5.0}, flow_samples=120, flow_quality=4)
    check = named(assess(snapshot), "optical_flow")
    assert check.passed is False
    assert "discard" in check.detail


def test_healthy_flow_passes_its_own_check():
    assert named(assess(HEALTHY), "optical_flow").passed is True


def test_a_vehicle_without_a_flow_sensor_is_unknown_not_failed():
    snapshot = Snapshot(parameters={"FLOW_TYPE": 0.0})
    assert named(assess(snapshot), "optical_flow").passed is None


def test_missing_horizontal_position_falls_back_to_the_gps_reason():
    snapshot = Snapshot(
        parameters={
            **HEALTHY_PARAMETERS,
            "EK3_SRC1_POSXY": 3.0,
            "EK3_SRC1_VELXY": 3.0,
        },
        ekf_flags=EKF_ATTITUDE,
        gps_fix=1,
        gps_satellites=0,
    )
    check = named(assess(snapshot), "horizontal_position")
    assert check.passed is False
    assert "GPS fix type 1" in check.detail


def test_as_found_configuration_blocks_a_guided_takeoff():
    blockers = {check.name for check in guided_takeoff_blockers(assess(AS_FOUND))}
    assert blockers == {"arming_checks", "horizontal_position"}


def test_an_armed_vehicle_blocks_every_test():
    snapshot = Snapshot(
        armed=True,
        parameters=HEALTHY_PARAMETERS,
        flow_samples=HEALTHY_FLOW_SAMPLES,
        flow_quality=HEALTHY_FLOW_QUALITY,
    )
    check = named(assess(snapshot), "disarmed")
    assert check.passed is False
    assert "do not approach" in check.detail


def test_a_sideways_rangefinder_is_not_an_altitude_source():
    snapshot = Snapshot(
        parameters=HEALTHY_PARAMETERS,
        rangefinder_cm=120,
        rangefinder_orientation=0,
        flow_samples=HEALTHY_FLOW_SAMPLES,
        flow_quality=HEALTHY_FLOW_QUALITY,
    )
    assert named(assess(snapshot), "downward_rangefinder").passed is False


def test_a_missing_rangefinder_fails_rather_than_being_unknown():
    assert named(assess(Snapshot()), "downward_rangefinder").passed is False


def test_disabled_onboard_logging_blocks_takeoff():
    snapshot = Snapshot(parameters={**HEALTHY_PARAMETERS, "LOG_BACKEND_TYPE": 0.0})
    assert named(assess(snapshot), "onboard_logging").passed is False


def test_unanswered_parameters_are_unknown_not_passing():
    checks = assess(Snapshot())
    assert named(checks, "arming_checks").passed is None
    assert named(checks, "onboard_logging").passed is None


def test_a_flat_battery_blocks_takeoff():
    snapshot = Snapshot(
        parameters=HEALTHY_PARAMETERS,
        battery_v=10.4,
        flow_samples=HEALTHY_FLOW_SAMPLES,
        flow_quality=HEALTHY_FLOW_QUALITY,
    )
    assert named(assess(snapshot), "battery").passed is False


@pytest.mark.parametrize(
    ("flags", "expected"),
    [
        (0, "none"),
        (EKF_ATTITUDE, "attitude"),
        (EKF_ATTITUDE | EKF_CONST_POS_MODE, "attitude const_pos_mode"),
    ],
)
def test_ekf_flags_describe_themselves(flags, expected):
    assert describe_ekf_flags(flags) == expected
