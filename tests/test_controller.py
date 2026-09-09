from __future__ import annotations

import argparse
import math
import time
from contextlib import contextmanager
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone import DroneController, FlightSafetyError
from ai_drone.cli import control
from ai_drone.flight.controller import (
    ATTITUDE_TARGET_MASK,
    EXPECTED_FIRMWARE_COMMIT,
    GUIDED_TAKEOFF_CLIMB_FRACTION,
    MAX_PHYSICAL_ALTITUDE_M,
    REQUIRED_NOGPS_LOITER_PARAMETERS,
)


def _message(message_type: str, **fields):
    return SimpleNamespace(
        get_type=lambda: message_type,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
        **fields,
    )


def _battery_message(voltage: float, **overrides):
    fields = dict.fromkeys(
        (
            "onboard_control_sensors_present",
            "onboard_control_sensors_enabled",
            "onboard_control_sensors_health",
        ),
        mavlink.MAV_SYS_STATUS_SENSOR_BATTERY,
    )
    fields.update(overrides)
    return _message("SYS_STATUS", voltage_battery=voltage, **fields)


def _stub_arm_preconditions(monkeypatch, controller) -> None:
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
        monkeypatch.setattr(controller, method, lambda *_args, **_kwargs: None)
    monkeypatch.setattr(controller, "wait_for_altitude", lambda **_kwargs: 0.05)
    monkeypatch.setattr(controller, "altitude_is_fresh", lambda: True)


def test_passive_context_never_controls_an_already_armed_vehicle() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    controller.is_armed = True

    controller.__exit__(None, None, None)

    assert controller.connection is None
    connection.mav.set_mode_send.assert_not_called()
    connection.arducopter_disarm.assert_not_called()
    connection.close.assert_called_once()


@pytest.mark.parametrize(("system", "component"), [(1, 1), (42, 7)])
def test_connect_binds_to_requested_ardupilot_identity(
    monkeypatch, system, component
) -> None:
    controller = DroneController(
        device="udp:127.0.0.1:14550", target_system=system, target_component=component
    )
    connection = MagicMock()
    connection.flightmode = "STABILIZE"
    gcs = _message(
        "HEARTBEAT",
        base_mode=0,
        autopilot=mavlink.MAV_AUTOPILOT_INVALID,
        type=mavlink.MAV_TYPE_GCS,
    )
    gcs.get_srcSystem = lambda: 255
    expected = _message(
        "HEARTBEAT",
        base_mode=0,
        autopilot=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        type=mavlink.MAV_TYPE_QUADROTOR,
    )
    expected.get_srcSystem = lambda: system
    expected.get_srcComponent = lambda: component
    connection.recv_match.side_effect = [gcs, expected]
    monkeypatch.setattr(
        "ai_drone.flight.controller.open_ardupilot_connection",
        lambda *_args, **_kwargs: connection,
    )

    controller.connect()

    assert (controller.target_system, controller.target_component) == (
        system,
        component,
    )
    assert (connection.target_system, connection.target_component) == (
        system,
        component,
    )
    assert controller.heartbeat_is_fresh()
    assert not controller.is_armed
    connection.arducopter_arm.assert_not_called()
    connection.mav.set_mode_send.assert_not_called()
    controller.close()


def test_connect_closes_without_commands_when_intended_vehicle_is_absent(
    monkeypatch,
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550", target_system=42)
    connection = MagicMock()
    connection.recv_match.return_value = None
    monkeypatch.setattr(
        "ai_drone.flight.controller.open_ardupilot_connection",
        lambda *_args, **_kwargs: connection,
    )

    with pytest.raises(TimeoutError, match="42/1"):
        controller.connect()

    assert controller.connection is None
    connection.close.assert_called_once_with()
    connection.mav.heartbeat_send.assert_not_called()
    connection.mav.command_long_send.assert_not_called()
    connection.arducopter_arm.assert_not_called()


def test_disarmed_heartbeat_during_pending_arm_preserves_cleanup(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    _stub_arm_preconditions(monkeypatch, controller)
    connection.recv_match.side_effect = [
        _message("HEARTBEAT", base_mode=0),
        OSError("link failed before arm confirmation"),
        None,  # no queued heartbeats when cleanup starts
        _message("HEARTBEAT", base_mode=0),
    ]

    with pytest.raises(OSError, match="before arm confirmation"):
        controller.arm()

    assert controller._arm_command_sent
    controller.__exit__(OSError, OSError("link failed"), None)

    connection.arducopter_arm.assert_called_once_with()
    connection.arducopter_disarm.assert_called_once_with()
    connection.mav.set_mode_send.assert_not_called()
    connection.close.assert_called_once_with()
    assert not controller._arm_command_sent


def test_confirmed_arm_transfers_pending_ownership_until_disarmed(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    _stub_arm_preconditions(monkeypatch, controller)
    connection.recv_match.side_effect = [
        _message("HEARTBEAT", base_mode=0),
        _message("HEARTBEAT", base_mode=mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
    ]

    controller.arm()

    assert controller._armed_by_controller
    assert not controller._arm_command_sent
    controller._process_message(_message("HEARTBEAT", base_mode=0), time.monotonic())
    controller.__exit__(None, None, None)

    connection.arducopter_disarm.assert_not_called()
    connection.close.assert_called_once_with()


@pytest.mark.parametrize("queued_disarmed", [False, True])
def test_disarm_requires_new_heartbeat_not_cached_or_queued_state(
    monkeypatch, queued_disarmed
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    controller._arm_command_sent = True
    clock = [100.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    queue = [_message("HEARTBEAT", base_mode=0)] if queued_disarmed else []

    def receive(*, type, blocking, timeout: float | None = None):
        if not blocking and queue:
            return queue.pop()
        if blocking:
            assert timeout is not None
            clock[0] += timeout
        return None

    connection.recv_match.side_effect = receive

    with pytest.raises(TimeoutError, match="did not confirm disarming"):
        controller.disarm(timeout=0.5)

    assert controller._arm_command_sent
    connection.arducopter_disarm.assert_called_once_with()


def test_disarm_waits_for_matching_disarmed_heartbeat() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    controller._arm_command_sent = True
    foreign = _message("HEARTBEAT", base_mode=0)
    foreign.get_srcSystem = lambda: 2
    connection.recv_match.side_effect = [
        _message("HEARTBEAT", base_mode=0),  # queued before confirmation
        None,
        foreign,
        _message("HEARTBEAT", base_mode=mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
        _message("HEARTBEAT", base_mode=0),
    ]

    controller.disarm()

    assert connection.recv_match.call_count == 5
    assert not controller.is_armed
    assert not controller._arm_command_sent


def test_downward_live_packet_updates_altitude_and_forward_sensor_is_ignored() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    connection.recv_match.side_effect = [
        _message(
            "DISTANCE_SENSOR",
            orientation=mavlink.MAV_SENSOR_ROTATION_PITCH_270,
            time_boot_ms=1_000,
            current_distance=42,
            min_distance=2,
            max_distance=1200,
            signal_quality=0,
        ),
        _message(
            "DISTANCE_SENSOR",
            orientation=mavlink.MAV_SENSOR_ROTATION_NONE,
            time_boot_ms=1_050,
            current_distance=300,
            min_distance=20,
            max_distance=800,
            signal_quality=80,
        ),
        _message(
            "DISTANCE_SENSOR",
            orientation=mavlink.MAV_SENSOR_ROTATION_PITCH_270,
            time_boot_ms=1_100,
            current_distance=47,
            min_distance=2,
            max_distance=1200,
            signal_quality=0,
        ),
        None,
    ]

    controller.update_telemetry()

    assert controller.current_altitude == 0.47


def test_stale_and_future_downward_samples_are_rejected() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    base = dict(
        orientation=mavlink.MAV_SENSOR_ROTATION_PITCH_270,
        min_distance=2,
        max_distance=1200,
        signal_quality=0,
    )
    controller._process_message(
        _message("DISTANCE_SENSOR", time_boot_ms=10_000, current_distance=40, **base),
        100.0,
    )
    controller._process_message(
        _message("DISTANCE_SENSOR", time_boot_ms=8_000, current_distance=90, **base),
        100.1,
    )
    controller._process_message(
        _message("DISTANCE_SENSOR", time_boot_ms=15_000, current_distance=95, **base),
        100.2,
    )

    assert controller.current_altitude == 0.4


def test_arming_checks_must_be_exactly_all(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    requested = []
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter",
        lambda _connection, name: requested.append(name) or 4.0,
    )

    with pytest.raises(FlightSafetyError, match="ARMING_SKIPCHK=4"):
        controller.verify_arming_checks()
    assert requested == ["ARMING_SKIPCHK"]


def test_nogps_loiter_parameters_must_match_reviewed_47_values(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    requested = []
    parameters = {**REQUIRED_NOGPS_LOITER_PARAMETERS, "RNGFND2_TYPE": 0.0}
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter",
        lambda _connection, name: requested.append(name) or parameters[name],
    )

    controller.verify_nogps_loiter_parameters()

    assert requested == ["RNGFND2_TYPE", *REQUIRED_NOGPS_LOITER_PARAMETERS]
    assert REQUIRED_NOGPS_LOITER_PARAMETERS["EK3_SRC1_POSXY"] == 0.0
    assert REQUIRED_NOGPS_LOITER_PARAMETERS["EK3_SRC1_POSZ"] == 1.0
    assert REQUIRED_NOGPS_LOITER_PARAMETERS["FS_OPTIONS"] == 8.0
    assert REQUIRED_NOGPS_LOITER_PARAMETERS["FS_THR_ENABLE"] == 0.0
    assert REQUIRED_NOGPS_LOITER_PARAMETERS["FS_DR_ENABLE"] == 1.0
    assert REQUIRED_NOGPS_LOITER_PARAMETERS["LAND_SPD_MS"] == 0.15
    assert REQUIRED_NOGPS_LOITER_PARAMETERS["RNGFND1_MAX"] == 1.0
    assert REQUIRED_NOGPS_LOITER_PARAMETERS["MAV_GCS_SYSID"] == 255.0


@pytest.mark.parametrize("forward_type", [0.0, 10.0])
def test_wrong_nogps_parameter_blocks_flight(monkeypatch, forward_type) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    parameters = {
        **REQUIRED_NOGPS_LOITER_PARAMETERS,
        "RNGFND2_TYPE": forward_type,
        "RNGFND2_ORIENT": 0.0,
        "RNGFND2_MIN": 0.1,
        "RNGFND2_MAX": 15.0,
        "FS_OPTIONS": 16.0,
    }
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter",
        lambda _connection, name: parameters[name],
    )

    with pytest.raises(FlightSafetyError, match=r"FS_OPTIONS=16.*exact value 8"):
        controller.verify_nogps_loiter_parameters()


@pytest.mark.parametrize("minimum", [0.1, 0.10000000149011612])
def test_reviewed_forward_mt15_configuration_is_accepted(monkeypatch, minimum) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    parameters = {
        **REQUIRED_NOGPS_LOITER_PARAMETERS,
        "RNGFND2_TYPE": 10.0,
        "RNGFND2_ORIENT": 0.0,
        "RNGFND2_MIN": minimum,
        "RNGFND2_MAX": 15.0,
    }
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter",
        lambda _connection, name: parameters[name],
    )

    controller.verify_nogps_loiter_parameters()

    controller.connection.arducopter_arm.assert_not_called()
    controller.connection.mav.param_set_send.assert_not_called()


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("RNGFND2_TYPE", 1.0),
        ("RNGFND2_TYPE", 9.0),
        ("RNGFND2_TYPE", 10.1),
        ("RNGFND2_ORIENT", 25.0),
        ("RNGFND2_ORIENT", 2.0),
        ("RNGFND2_MIN", 0.0),
        ("RNGFND2_MIN", 0.02),
        ("RNGFND2_MIN", 0.2),
        ("RNGFND2_MAX", 0.0),
        ("RNGFND2_MAX", 10.0),
        ("RNGFND2_MAX", 16.0),
        ("RNGFND2_MAX", float("inf")),
        ("RNGFND2_MIN", float("nan")),
        ("RNGFND1_TYPE", 0.0),
        ("RNGFND1_ORIENT", 0.0),
        ("RNGFND1_MAX", 15.0),
    ],
)
def test_forward_mt15_cannot_relax_reviewed_rangefinder_configuration(
    monkeypatch, name, value
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    parameters = {
        **REQUIRED_NOGPS_LOITER_PARAMETERS,
        "RNGFND2_TYPE": 10.0,
        "RNGFND2_ORIENT": 0.0,
        "RNGFND2_MIN": 0.1,
        "RNGFND2_MAX": 15.0,
        name: value,
    }
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter",
        lambda _connection, requested: parameters[requested],
    )

    with pytest.raises(FlightSafetyError, match=name):
        controller.verify_nogps_loiter_parameters()

    controller.connection.arducopter_arm.assert_not_called()
    controller.connection.mav.param_set_send.assert_not_called()


def test_firmware_gate_requests_and_accepts_exact_copter_470(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    sent = []
    monkeypatch.setattr(controller, "_drain_messages", lambda: None)

    def send(command, parameters, *, timeout):
        sent.append((command, parameters, timeout))
        controller.flight_sw_version = (4 << 24) | (7 << 16)
        controller.flight_custom_version = EXPECTED_FIRMWARE_COMMIT

    monkeypatch.setattr(controller, "_send_command_long_and_wait_ack", send)

    controller.verify_firmware(timeout=2.0)

    assert sent == [
        (
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            (
                float(mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
            2.0,
        )
    ]


@pytest.mark.parametrize(
    ("packed", "commit", "expected"),
    [
        ((4 << 24) | (6 << 16) | (3 << 8), EXPECTED_FIRMWARE_COMMIT, "4.6.3"),
        ((4 << 24) | (7 << 16), b"deadbeef", "deadbeef"),
    ],
)
def test_firmware_gate_rejects_wrong_version_or_commit(
    monkeypatch, packed, commit, expected
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    monkeypatch.setattr(controller, "_drain_messages", lambda: None)

    def send(*_args, **_kwargs):
        controller.flight_sw_version = packed
        controller.flight_custom_version = commit

    monkeypatch.setattr(controller, "_send_command_long_and_wait_ack", send)

    with pytest.raises(FlightSafetyError, match=expected):
        controller.verify_firmware()


@pytest.mark.parametrize(
    ("parameters", "message"),
    [
        ({"LOG_BACKEND_TYPE": 2.0, "LOG_BITMASK": 1.0}, "onboard"),
        ({"LOG_BACKEND_TYPE": 1.0, "LOG_BITMASK": 0.0}, "LOG_BITMASK"),
    ],
)
def test_flight_requires_onboard_logging(monkeypatch, parameters, message) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    monkeypatch.setattr(
        "ai_drone.flight.controller.request_parameter",
        lambda _connection, name: parameters[name],
    )

    with pytest.raises(FlightSafetyError, match=message):
        controller.verify_onboard_logging()


def test_land_timeout_never_force_disarms(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    controller.connection = connection
    controller.is_armed = True
    monkeypatch.setattr(controller, "update_telemetry", lambda: None)
    clock = [0.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(
        "ai_drone.flight.controller.time.sleep",
        lambda seconds: clock.__setitem__(0, clock[0] + seconds),
    )

    with pytest.raises(TimeoutError, match="LAND remains commanded"):
        controller.land(timeout=1.0)

    connection.arducopter_disarm.assert_not_called()
    assert connection.mav.set_mode_send.call_count >= 1


def test_land_retries_until_disarmed_heartbeat_is_observed(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    controller.connection = connection
    controller.is_armed = True
    controller._flight_started_by_controller = True
    clock = [0.0]
    updates = [0]

    def update() -> None:
        updates[0] += 1
        if updates[0] >= 12:
            controller.is_armed = False
            controller._flight_started_by_controller = False

    monkeypatch.setattr(controller, "update_telemetry", update)
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(
        "ai_drone.flight.controller.time.sleep",
        lambda seconds: clock.__setitem__(0, clock[0] + seconds),
    )

    controller.land(timeout=5.0)

    assert connection.mav.set_mode_send.call_count >= 2
    connection.arducopter_disarm.assert_not_called()


def test_cleanup_after_takeoff_always_monitors_land_and_never_disarms(
    monkeypatch,
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    controller.is_armed = True
    controller._armed_by_controller = True
    controller._flight_started_by_controller = True
    controller._landing_commanded = True
    land = MagicMock(side_effect=TimeoutError("still armed"))
    monkeypatch.setattr(controller, "land", land)

    controller.__exit__(RuntimeError, RuntimeError("failed"), None)

    land.assert_called_once_with()
    connection.arducopter_disarm.assert_not_called()
    connection.close.assert_called_once_with()


def test_gcs_heartbeat_is_sent_at_one_hz(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    clock = [10.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])

    controller._pump_gcs_heartbeat()
    clock[0] = 10.5
    controller._pump_gcs_heartbeat()
    clock[0] = 11.0
    controller._pump_gcs_heartbeat()

    assert connection.mav.heartbeat_send.call_count == 2
    connection.mav.heartbeat_send.assert_called_with(
        mavlink.MAV_TYPE_GCS,
        mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavlink.MAV_STATE_ACTIVE,
    )


def test_flow_and_ekf_messages_gate_relative_navigation(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 100.0)
    flags = mavlink.EKF_VELOCITY_HORIZ | mavlink.EKF_POS_HORIZ_REL

    controller._process_message(_message("OPTICAL_FLOW", quality=67), 99.5)
    controller._process_message(_message("EKF_STATUS_REPORT", flags=flags), 99.5)
    assert controller.navigation_is_healthy()

    controller._process_message(
        _message("EKF_STATUS_REPORT", flags=flags | mavlink.EKF_CONST_POS_MODE),
        99.75,
    )
    assert not controller.navigation_is_healthy()


def test_fresh_zero_rc_channel_count_confirms_receiver_is_absent(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    clock = [100.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])

    controller._process_message(
        _message("RC_CHANNELS", time_boot_ms=10_000, chancount=0), 99.5
    )

    assert controller.no_rc_input_is_confirmed()
    clock[0] = 101.0
    assert not controller.no_rc_input_is_confirmed()


def test_battery_guard_requires_fresh_voltage_before_arming(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550", min_battery_voltage=14.4)
    controller.connection = MagicMock()

    def update() -> None:
        controller.battery_voltage = 14.09
        controller.last_battery_time = time.monotonic()

    monkeypatch.setattr(controller, "update_telemetry", update)

    with pytest.raises(FlightSafetyError, match=r"14.09 V.*14.40 V"):
        controller.verify_battery_before_arming(timeout=0.1)

    controller.connection.mav.set_mode_send.assert_not_called()
    controller.connection.arducopter_arm.assert_not_called()


@pytest.mark.parametrize("voltage", [0, -1, 65_535, 65_536, math.nan, math.inf])
def test_invalid_battery_report_immediately_invalidates_previous_reading(
    monkeypatch, voltage
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 100.0)
    controller._process_message(_battery_message(16_000), 99.8)
    assert controller.battery_voltage == 16.0
    assert controller.battery_is_fresh()

    controller._process_message(_battery_message(voltage), 99.9)

    assert controller.battery_voltage is None
    assert not controller.battery_is_fresh()


@pytest.mark.parametrize(
    "field",
    [
        "onboard_control_sensors_present",
        "onboard_control_sensors_enabled",
        "onboard_control_sensors_health",
    ],
)
def test_battery_status_must_confirm_present_enabled_and_healthy(
    monkeypatch, field
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 100.0)
    controller._process_message(_battery_message(16_000), 99.7)

    controller._process_message(_battery_message(16_000, **{field: 0}), 99.8)

    assert controller.battery_voltage is None
    assert not controller.battery_is_fresh()
    controller._process_message(_battery_message(15_900), 99.9)
    assert controller.battery_voltage == 15.9
    assert controller.battery_is_fresh()


def test_unknown_battery_stream_cannot_satisfy_prearm_guard(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550", min_battery_voltage=14.4)
    connection = MagicMock()
    controller.connection = connection
    clock = [100.0]
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(
        "ai_drone.flight.controller.time.sleep",
        lambda seconds: clock.__setitem__(0, clock[0] + seconds),
    )
    monkeypatch.setattr(
        controller,
        "update_telemetry",
        lambda: controller._process_message(_battery_message(65_535), clock[0]),
    )

    with pytest.raises(FlightSafetyError, match="no fresh battery voltage"):
        controller.verify_battery_before_arming(timeout=0.1)

    connection.arducopter_arm.assert_not_called()


def test_unknown_battery_during_flight_commands_land() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550", min_battery_voltage=14.4)
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    controller.connection = connection
    controller._flight_started_by_controller = True
    controller.is_armed = True
    controller.rc_channel_count = 0
    controller.last_rc_channels_time = time.monotonic()
    controller._process_message(_battery_message(16_000), time.monotonic())
    connection.recv_match.side_effect = [_battery_message(65_535), None]

    with pytest.raises(FlightSafetyError, match="battery telemetry became stale"):
        controller.update_telemetry()

    assert controller._landing_commanded
    connection.mav.set_mode_send.assert_called_once_with(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    )
    connection.arducopter_disarm.assert_not_called()


def test_low_battery_during_controller_flight_commands_land() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550", min_battery_voltage=14.4)
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    connection.recv_match.return_value = None
    controller.connection = connection
    controller._flight_started_by_controller = True
    controller.is_armed = True
    controller.rc_channel_count = 0
    controller.last_rc_channels_time = time.monotonic()
    controller.battery_voltage = 14.0
    controller.last_battery_time = time.monotonic()

    with pytest.raises(FlightSafetyError, match=r"14.00 V.*14.40 V"):
        controller.update_telemetry()

    connection.mav.set_mode_send.assert_called_once_with(
        1,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        connection.mode_mapping.return_value["LAND"],
    )


@pytest.mark.parametrize("channel_count", [4, 8, 16])
def test_active_rc_receiver_is_not_the_no_receiver_topology(
    monkeypatch, channel_count
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 100.0)

    controller._process_message(
        _message(
            "RC_CHANNELS",
            time_boot_ms=10_000,
            chancount=channel_count,
            chan3_raw=1000,
        ),
        99.5,
    )

    assert not controller.no_rc_input_is_confirmed()


def test_wait_for_no_rc_input_accepts_fresh_zero_channel_report(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()

    def update() -> None:
        controller.rc_channel_count = 0
        controller.last_rc_channels_time = time.monotonic()

    monkeypatch.setattr(controller, "update_telemetry", update)

    controller.wait_for_no_rc_input(timeout=0.1)


def test_wait_for_no_rc_input_rejects_active_receiver(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()

    def update() -> None:
        controller.rc_channel_count = 8
        controller.last_rc_channels_time = time.monotonic()

    monkeypatch.setattr(controller, "update_telemetry", update)

    with pytest.raises(FlightSafetyError, match="active RC receiver"):
        controller.wait_for_no_rc_input(timeout=0.1)


def test_local_position_ceiling_is_aligned_to_rangefinder_and_lands(
    monkeypatch,
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550", max_altitude=0.8)
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    connection.recv_match.return_value = None
    controller.connection = connection
    controller._flight_started_by_controller = True
    controller.is_armed = True
    base = dict(
        orientation=mavlink.MAV_SENSOR_ROTATION_PITCH_270,
        min_distance=2,
        max_distance=100,
        signal_quality=0,
    )
    controller._process_message(
        _message("DISTANCE_SENSOR", time_boot_ms=1_000, current_distance=5, **base),
        100.0,
    )
    controller._process_message(
        _message("LOCAL_POSITION_NED", time_boot_ms=1_010, z=-12.0), 100.0
    )
    controller._process_message(
        _message("LOCAL_POSITION_NED", time_boot_ms=1_510, z=-12.8), 100.5
    )
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 100.5)

    with pytest.raises(FlightSafetyError, match=r"altitude 0.85 m exceeds 0.80 m"):
        controller.update_telemetry()

    assert controller.local_position_altitude_aligned == pytest.approx(0.85)
    connection.mav.set_mode_send.assert_called_once()
    connection.arducopter_disarm.assert_not_called()


def test_takeoff_uses_guided_nogps_flag_and_rangefinder_delta(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550", max_altitude=0.8)
    controller.connection = MagicMock()
    controller.is_armed = True
    controller._armed_by_controller = True
    controller.flight_mode = "GUIDED_NOGPS"
    controller.current_altitude = 0.05
    now = time.monotonic()
    controller.yaw_rad = 0.0
    controller.last_attitude_time = now
    controller.flow_quality = 67
    controller.last_flow_time = now
    controller.last_heartbeat_time = now
    controller.last_telemetry_time = now
    sent: list[float] = []
    monkeypatch.setattr(controller, "wait_for_altitude", lambda **_kwargs: 0.05)
    monkeypatch.setattr(
        controller,
        "_send_level_climb",
        sent.append,
    )

    updates = 0

    def update() -> None:
        nonlocal updates
        updates += 1
        timestamp = time.monotonic()
        controller.current_altitude = 0.05 if updates == 1 else 0.53
        controller.last_telemetry_time = timestamp
        controller.last_heartbeat_time = timestamp

    monkeypatch.setattr(controller, "update_telemetry", update)

    controller.takeoff(0.5)

    assert sent == [GUIDED_TAKEOFF_CLIMB_FRACTION, 0.0]
    assert controller._ground_reference == 0.05
    assert controller._flight_started_by_controller


def test_guided_nogps_climb_is_level_and_uses_climb_rate_field(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    controller.is_armed = True
    controller.flight_mode = "GUIDED_NOGPS"
    controller.yaw_rad = math.pi / 2
    controller.last_attitude_time = 10.0
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic", lambda: 10.0)
    monkeypatch.setattr("ai_drone.flight.controller.time.monotonic_ns", lambda: 10**10)

    controller._send_level_climb(0.4)

    call = connection.mav.set_attitude_target_send.call_args.args
    assert call[:4] == (10_000, 1, 1, ATTITUDE_TARGET_MASK)
    assert call[4] == pytest.approx([math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)])
    assert call[5:8] == (0.0, 0.0, 0.0)
    assert call[8] == pytest.approx(0.7)


def test_command_ack_is_required_and_rejection_includes_vehicle_text() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    connection.recv_match.side_effect = [
        _message("STATUSTEXT", text="takeoff refused"),
        _message(
            "COMMAND_ACK",
            command=mavlink.MAV_CMD_NAV_TAKEOFF,
            result=mavlink.MAV_RESULT_FAILED,
        ),
    ]

    with pytest.raises(FlightSafetyError, match="takeoff refused"):
        controller._send_command_long_and_wait_ack(
            mavlink.MAV_CMD_NAV_TAKEOFF,
            (0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5),
        )


def test_command_ack_acceptance_sends_exact_parameters() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    controller.connection = connection
    connection.recv_match.return_value = _message(
        "COMMAND_ACK",
        command=mavlink.MAV_CMD_NAV_TAKEOFF,
        result=mavlink.MAV_RESULT_ACCEPTED,
    )
    parameters = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5)

    controller._send_command_long_and_wait_ack(mavlink.MAV_CMD_NAV_TAKEOFF, parameters)

    connection.mav.command_long_send.assert_called_once_with(
        1,
        1,
        mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        *parameters,
    )


def test_enter_loiter_waits_for_navigation_then_confirms_mode(monkeypatch) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    controller.connection = MagicMock()
    controller.is_armed = True
    controller._flight_started_by_controller = True
    wait = MagicMock()
    monkeypatch.setattr(controller, "wait_for_relative_position", wait)
    monkeypatch.setattr(controller, "navigation_is_healthy", lambda: True)
    monkeypatch.setattr(controller, "no_rc_input_is_confirmed", lambda: True)

    def set_mode(mode: str) -> None:
        controller.flight_mode = mode

    monkeypatch.setattr(controller, "set_mode", set_mode)

    controller.enter_loiter(timeout=8.0, stable_for=1.5)

    wait.assert_called_once_with(timeout=8.0, stable_for=1.5)
    assert controller.flight_mode == "LOITER"


def test_enter_loiter_lands_instead_of_accepting_valid_low_throttle_rc(
    monkeypatch,
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9, "LOITER": 5}
    controller.connection = connection
    controller.is_armed = True
    controller._flight_started_by_controller = True
    controller.rc_channel_count = 8
    controller.last_rc_channels_time = time.monotonic()
    monkeypatch.setattr(
        controller, "wait_for_relative_position", lambda **_kwargs: None
    )

    with pytest.raises(FlightSafetyError, match="zero receiver channels"):
        controller.enter_loiter()

    connection.mav.set_mode_send.assert_called_once_with(
        1,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        connection.mode_mapping.return_value["LAND"],
    )


def test_active_receiver_during_controller_flight_commands_land_even_after_mode_change(
    monkeypatch,
) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    connection.recv_match.return_value = None
    controller.connection = connection
    controller.is_armed = True
    controller._flight_started_by_controller = True
    # An RC mode channel can move the vehicle away from Loiter before the next
    # telemetry cycle.  The topology guard must remain active in every mode.
    controller.flight_mode = "ALT_HOLD"
    controller.rc_channel_count = 8
    controller.last_rc_channels_time = time.monotonic()
    monkeypatch.setattr(controller, "navigation_is_healthy", lambda: True)

    with pytest.raises(FlightSafetyError, match="receiver topology changed"):
        controller.update_telemetry()

    connection.mav.set_mode_send.assert_called_once_with(
        1,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        connection.mode_mapping.return_value["LAND"],
    )


def test_controller_hard_caps_physical_altitude_below_one_metre() -> None:
    with pytest.raises(ValueError, match="max_altitude"):
        DroneController(
            device="udp:127.0.0.1:14550",
            max_altitude=MAX_PHYSICAL_ALTITUDE_M + 0.01,
        )


def test_controller_rejects_an_invalid_battery_guard() -> None:
    with pytest.raises(ValueError, match="min_battery_voltage"):
        DroneController(
            device="udp:127.0.0.1:14550",
            min_battery_voltage=60.01,
        )


def test_stop_callback_requests_land_after_takeoff() -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    connection.recv_match.return_value = None
    controller.connection = connection
    controller._flight_started_by_controller = True
    controller.stop_requested = lambda: True

    with pytest.raises(FlightSafetyError, match="stop requested"):
        controller.update_telemetry()

    assert controller._landing_commanded
    connection.mav.set_mode_send.assert_called_once()
    connection.arducopter_disarm.assert_not_called()


def test_flight_confirmation_is_checked_before_device_access(monkeypatch) -> None:
    monkeypatch.setattr(
        control,
        "_controller",
        lambda _args: pytest.fail("must not access MAVLink before confirmation"),
    )
    assert control.main(["hover", "--confirm-flight", "yes"]) == 1


def test_cli_forwards_battery_threshold_into_controller(monkeypatch) -> None:
    created = {}

    def fake_controller(**kwargs):
        created.update(kwargs)
        return object()

    monkeypatch.setattr(control, "DroneController", fake_controller)
    args = argparse.Namespace(
        device="tcp:127.0.0.1:5760",
        baud=115200,
        max_alt=0.8,
        min_battery=14.4,
    )

    control._controller(args)

    assert created["min_battery_voltage"] == 14.4


def test_loiter_monitor_lands_if_mode_changes() -> None:
    drone = MagicMock()
    drone.flight_mode = "ALT_HOLD"

    with pytest.raises(FlightSafetyError, match="left LOITER mode for ALT_HOLD"):
        control._monitor(drone, duration=0.1, min_battery_v=14.4)

    drone.emergency_stop.assert_called_once_with()


@pytest.mark.parametrize("mode", ["ALT_HOLD", None])
def test_public_loiter_hold_lands_if_mode_changes(monkeypatch, mode) -> None:
    controller = DroneController(device="udp:127.0.0.1:14550")
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LAND": 9}
    controller.connection = connection
    controller.flight_mode = "LOITER"
    monkeypatch.setattr(
        controller, "update_telemetry", lambda: setattr(controller, "flight_mode", mode)
    )
    monkeypatch.setattr(controller, "altitude_is_fresh", lambda: True)
    monkeypatch.setattr(controller, "heartbeat_is_fresh", lambda: True)

    with pytest.raises(FlightSafetyError, match="left LOITER mode"):
        controller.hold_loiter(duration=0.1)

    connection.mav.set_mode_send.assert_called_once_with(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    )
    connection.arducopter_disarm.assert_not_called()


def test_hover_cli_runs_guided_nogps_takeoff_loiter_hold_and_land(
    monkeypatch,
) -> None:
    calls: list[object] = []
    stop = SimpleNamespace(is_set=lambda: False)
    drone = SimpleNamespace(ekf_flags=11, stop_requested=None)
    drone.takeoff = lambda altitude: calls.append(("takeoff", altitude))
    drone.enter_loiter = lambda **kwargs: calls.append(("loiter", kwargs))
    drone.land = lambda: calls.append("land")
    record = SimpleNamespace(
        event=lambda name, **fields: calls.append(("event", name, fields))
    )

    @contextmanager
    def flight_session(_args):
        yield drone, record

    @contextmanager
    def termination_event():
        yield stop

    monkeypatch.setattr(control, "_flight_session", flight_session)
    monkeypatch.setattr(control, "_termination_event", termination_event)
    monkeypatch.setattr(
        control,
        "_monitor",
        lambda controlled, duration, voltage: calls.append(
            ("hold", controlled, duration, voltage)
        ),
    )

    assert (
        control.main(
            [
                "hover",
                "--duration",
                "2",
                "--navigation-timeout",
                "12",
                "--confirm-flight",
                control.FLIGHT_CONFIRMATION,
            ]
        )
        == 0
    )

    assert drone.stop_requested == stop.is_set
    assert ("takeoff", 0.5) in calls
    assert ("loiter", {"timeout": 12.0}) in calls
    assert ("hold", drone, 2.0, 14.4) in calls
    assert calls.index(("takeoff", 0.5)) < calls.index(("loiter", {"timeout": 12.0}))
    assert calls.index(("loiter", {"timeout": 12.0})) < calls.index("land")


def test_hover_cli_rejects_a_ceiling_at_or_above_one_metre(monkeypatch) -> None:
    monkeypatch.setattr(
        control,
        "_controller",
        lambda _args: pytest.fail("must reject the ceiling before device access"),
    )
    assert (
        control.main(
            [
                "hover",
                "--max-alt",
                "1.0",
                "--confirm-flight",
                control.FLIGHT_CONFIRMATION,
            ]
        )
        == 1
    )
