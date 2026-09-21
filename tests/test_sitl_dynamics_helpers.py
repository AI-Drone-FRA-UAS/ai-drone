"""Fault-injection scheduling and selected-FC readback without any transport."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from tests.test_sitl_dynamics import ALL_MOTORS, _dynamics_sensors


def _state():
    return SimpleNamespace(
        alt=584.5, roll=0, pitch=0, yaw=0, vn=0, ve=0, vd=0, xgyro=0, ygyro=0
    )


def test_flow_fault_starts_only_after_observed_arm_and_preserves_range(monkeypatch):
    monkeypatch.setattr("tests.test_sitl_dynamics.time.monotonic", lambda: 10.0)
    sensors = _dynamics_sensors("missing-relative-aiding")()
    sender = MagicMock()
    sensors._send_sensors(sender, _state(), 584)
    assert sender.optical_flow_send.call_count == 1
    assert not sensors.injections
    sensors._current_armed = True
    sensors._send_sensors(sender, _state(), 584)
    sensors._send_sensors(sender, _state(), 584)
    assert sender.optical_flow_send.call_count == 1
    assert sender.distance_sensor_send.call_count == 3
    assert len(sensors.injections) == 1
    sender.param_set_send.assert_not_called()


@pytest.mark.parametrize(
    "fault,parameter,value",
    [
        ("unexpected-descent", "SIM_ENGINE_FAIL", ALL_MOTORS),
        ("unexpected-low-throttle-rc", "SIM_RC_FAIL", 0.0),
    ],
)
def test_physics_fault_waits_for_loiter_then_retries_until_readback(
    monkeypatch, fault, parameter, value
):
    clock = [10.0]
    monkeypatch.setattr("tests.test_sitl_dynamics.time.monotonic", lambda: clock[0])
    sensors = _dynamics_sensors(fault)()
    sender = MagicMock()
    sensors._current_armed = True
    sensors._current_mode = "LOITER"
    sensors._send_sensors(sender, _state(), 584)
    clock[0] = 11.99
    sensors._send_sensors(sender, _state(), 584)
    sender.param_set_send.assert_not_called()
    clock[0] = 12.0
    sensors._send_sensors(sender, _state(), 584)
    assert sender.param_set_send.call_args.args[2:4] == (parameter.encode(), value)
    clock[0] = 12.49
    sensors._send_sensors(sender, _state(), 584)
    assert sender.param_set_send.call_count == 1
    clock[0] = 12.5
    sensors._send_sensors(sender, _state(), 584)
    assert sender.param_set_send.call_count == 2
    sensors.parameter_readback[parameter] = value
    clock[0] = 13.0
    sensors._send_sensors(sender, _state(), 584)
    assert sender.param_set_send.call_count == 2
    assert len(sensors.injections) == 1
    assert sensors.injections[0]["height"] == 0.5


def test_unrequested_mode_change_is_sent_once_only_after_loiter_delay(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr("tests.test_sitl_dynamics.time.monotonic", lambda: clock[0])
    sensors = _dynamics_sensors("unexpected-mode")()
    sender = MagicMock()
    sensors._current_armed = True
    sensors._current_mode = "LOITER"
    sensors._send_sensors(sender, _state(), 584)
    sender.set_mode_send.assert_not_called()
    clock[0] = 12.0
    sensors._send_sensors(sender, _state(), 584)
    sensors._send_sensors(sender, _state(), 584)
    sender.set_mode_send.assert_called_once()
    assert sender.set_mode_send.call_args.args[-1] == 2  # actual FC ALT_HOLD
    sender.param_set_send.assert_not_called()


@pytest.mark.parametrize("source", [(2, 1), (1, 2), (1, 1)])
def test_fault_readback_requires_selected_fc(source):
    sensors = _dynamics_sensors("unexpected-descent")()
    message = MagicMock()
    message.get_type.return_value = "PARAM_VALUE"
    message.get_srcSystem.return_value, message.get_srcComponent.return_value = source
    message.param_id = b"SIM_ENGINE_FAIL\x00"
    message.param_value = ALL_MOTORS
    sensors._observe_vehicle_message(message)
    expected = {"SIM_ENGINE_FAIL": ALL_MOTORS} if source == (1, 1) else {}
    assert sensors.parameter_readback == expected
