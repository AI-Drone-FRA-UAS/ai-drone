from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone import DroneController, FlightSafetyError
from ai_drone.cli import control


def _message(message_type: str, **fields):
    return SimpleNamespace(
        get_type=lambda: message_type,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
        **fields,
    )


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
    controller.connection = connection
    controller.is_armed = True
    monkeypatch.setattr(controller, "set_mode", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(controller, "update_telemetry", lambda: None)
    times = iter([0.0, 0.0, 2.0])
    monkeypatch.setattr(
        "ai_drone.flight.controller.time.monotonic", lambda: next(times)
    )

    with pytest.raises(TimeoutError, match="LAND remains commanded"):
        controller.land(timeout=1.0)

    connection.arducopter_disarm.assert_not_called()


def test_flight_confirmation_is_checked_before_device_access(monkeypatch) -> None:
    monkeypatch.setattr(
        control,
        "_controller",
        lambda _args: pytest.fail("must not access MAVLink before confirmation"),
    )
    assert control.main(["hover", "--confirm-flight", "yes"]) == 1
