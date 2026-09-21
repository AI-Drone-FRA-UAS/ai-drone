from unittest.mock import MagicMock

import pytest

from ai_drone.flight.controller import DroneController, FlightSafetyError
from ai_drone.flight.params import (
    FLOW_COMPASS,
    FLOW_INERTIAL_EXPERIMENTAL,
    REQUIRED_NOGPS_LOITER_PARAMETERS,
    select_navigation_profile,
)
from ai_drone.flight.phase import Flight


@pytest.mark.parametrize(
    "endpoint",
    [
        "/dev/serial0",
        "udp:127.0.0.1:5760",
        "tcp:localhost:5760",
        "tcp:192.168.1.2:5760",
        "tcp:127.0.0.1:0",
        "tcp:127.0.0.1:65536",
        "tcp:127.0.0.1:5760:extra",
    ],
)
def test_experimental_profile_rejects_nonliteral_or_nonlocal_endpoint(endpoint):
    with pytest.raises(ValueError, match="isolated local SITL"):
        select_navigation_profile(
            "flow-inertial-experimental", endpoint, isolated_sitl=True
        )


def test_experimental_profile_requires_explicit_isolated_simulator_opt_in():
    with pytest.raises(ValueError, match="isolated local SITL"):
        select_navigation_profile(
            "flow-inertial-experimental", "tcp:127.0.0.1:5760", isolated_sitl=False
        )
    assert (
        select_navigation_profile(
            "flow-inertial-experimental", "tcp:127.0.0.1:5760", isolated_sitl=True
        )
        is FLOW_INERTIAL_EXPERIMENTAL
    )


def test_baseline_exact_invariants_are_unchanged_and_experimental_settings_are_immutable():
    assert FLOW_COMPASS.parameters == REQUIRED_NOGPS_LOITER_PARAMETERS
    assert FLOW_COMPASS.parameters["EK3_SRC1_YAW"] == 1.0
    changes = {
        key: value
        for key, value in FLOW_INERTIAL_EXPERIMENTAL.parameters.items()
        if value != FLOW_COMPASS.parameters.get(key)
    }
    assert changes == {
        "EK3_SRC1_YAW": 0.0,
        "COMPASS_USE": 0.0,
        "COMPASS_USE2": 0.0,
        "COMPASS_USE3": 0.0,
    }
    assert FLOW_INERTIAL_EXPERIMENTAL.parameters["GUID_OPTIONS"] == 0.0
    assert FLOW_INERTIAL_EXPERIMENTAL.parameters["FS_EKF_ACTION"] == 1.0
    with pytest.raises(TypeError):
        # Deliberately attempt a forbidden write to verify runtime immutability too.
        FLOW_INERTIAL_EXPERIMENTAL.parameters["GUID_OPTIONS"] = 8.0  # ty: ignore[invalid-assignment]


def test_candidate_uses_normal_arming_checks_and_exact_candidate_parameter_contract(
    monkeypatch,
):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")
    monkeypatch.setattr("socket.if_nameindex", lambda: [(1, "lo")])
    drone = DroneController(
        device="tcp:127.0.0.1:5760", navigation_profile="flow-inertial-experimental"
    )
    drone.connection = MagicMock()
    parameters = {
        **FLOW_INERTIAL_EXPERIMENTAL.parameters,
        "RNGFND2_TYPE": 0.0,
        "ARMING_SKIPCHK": 0.0,
    }
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter",
        lambda _connection, key: parameters[key],
    )
    drone.verify_nogps_loiter_parameters()
    drone.verify_arming_checks()
    parameters["ARMING_SKIPCHK"] = 1.0
    with pytest.raises(FlightSafetyError, match="no checks skipped"):
        drone.verify_arming_checks()
    parameters["EK3_SRC1_YAW"] = 1.0
    with pytest.raises(FlightSafetyError, match="exact value 0"):
        drone.verify_nogps_loiter_parameters()
    drone.connection.mav.param_set_send.assert_not_called()


def test_whole_sequence_experiment_budget_includes_prearm_and_reserves_landing(
    monkeypatch,
):
    clock = [100.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")
    monkeypatch.setattr("socket.if_nameindex", lambda: [(1, "lo")])
    drone = DroneController(
        device="tcp:127.0.0.1:5760", navigation_profile="flow-inertial-experimental"
    )
    drone.connection = MagicMock()
    clock[0] = 160.0
    drone._require_profile_time(30.0)
    clock[0] = 160.001
    with pytest.raises(FlightSafetyError, match="reserved for LAND"):
        drone._require_profile_time(30.0)
    drone.connection.mav.set_mode_send.assert_not_called()
    drone.phase = Flight("holding_altitude", 0.05)
    with pytest.raises(FlightSafetyError, match="reserved for LAND"):
        drone._require_profile_time(30.0)
    drone.connection.mav.set_mode_send.assert_called_once()


def test_an_earlier_explicit_simulator_reference_is_not_restarted_at_connection(
    monkeypatch,
):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")
    monkeypatch.setattr("socket.if_nameindex", lambda: [(1, "lo")])
    monkeypatch.setenv("AI_DRONE_PROFILE_REFERENCE_MONOTONIC", "50")
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 100.0)
    drone = DroneController(
        device="tcp:127.0.0.1:5760", navigation_profile="flow-inertial-experimental"
    )
    assert drone.profile_initialized_at == 50.0
    with pytest.raises(FlightSafetyError, match="duration budget"):
        drone._require_profile_time(45.0)
