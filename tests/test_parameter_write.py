"""Tests for the project's only general parameter write.

The important tests are the refusals.  A write path that can be talked into
disabling the pre-arm checks would undo every other guarantee in this
repository.
"""

from __future__ import annotations

import argparse
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from ai_drone.cli import parameters as cli
from ai_drone.mavlink.arming_checks import ALL_EXCEPT_GPS
from ai_drone.mavlink.parameter_write import (
    PROTECTED_PARAMETERS,
    ParameterWriteError,
    set_parameter,
)


def _heartbeat(armed: bool = False):
    return SimpleNamespace(
        get_type=lambda: "HEARTBEAT",
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
        base_mode=128 if armed else 0,
    )


def _connection(name: str, values):
    connection = MagicMock()
    connection.target_system = 1
    connection.target_component = 1
    remaining = list(values)

    def recv_match(*args, **kwargs):
        if kwargs.get("type") == "HEARTBEAT":
            return None if kwargs.get("blocking") is False else _heartbeat()
        if not remaining:
            return None
        value = remaining.pop(0) if len(remaining) > 1 else remaining[0]
        return SimpleNamespace(
            get_type=lambda: "PARAM_VALUE",
            get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1,
            param_id=name,
            param_value=value,
        )

    connection.recv_match.side_effect = recv_match
    return connection


def test_disabling_the_arming_checks_is_refused():
    connection = _connection("ARMING_CHECK", [1.0])
    with pytest.raises(ParameterWriteError, match="safety gate"):
        set_parameter(connection, "ARMING_CHECK", 0.0)
    connection.mav.param_set_send.assert_not_called()


@pytest.mark.parametrize("value", [0.0, 22.0, 8.0, 1043957.0, 1043959.0])
def test_any_other_arming_check_mask_is_refused(value):
    # Only the two documented configurations are reachable. An arbitrary mask
    # would let a caller silently drop the IMU or battery check.
    connection = _connection("ARMING_CHECK", [1.0])
    with pytest.raises(ParameterWriteError):
        set_parameter(connection, "ARMING_CHECK", value)
    connection.mav.param_set_send.assert_not_called()


def test_the_gps_free_configuration_is_allowed():
    connection = _connection("ARMING_CHECK", [1.0, ALL_EXCEPT_GPS])
    result = set_parameter(connection, "ARMING_CHECK", ALL_EXCEPT_GPS)
    assert result.current == ALL_EXCEPT_GPS


def test_setting_the_arming_checks_to_the_full_set_is_allowed():
    connection = _connection("ARMING_CHECK", [0.0, 1.0])
    result = set_parameter(connection, "ARMING_CHECK", 1.0)
    assert result.current == 1.0


def test_the_protected_list_covers_the_arming_checks():
    permitted = PROTECTED_PARAMETERS["ARMING_CHECK"]
    assert permitted(1.0) is True
    assert permitted(ALL_EXCEPT_GPS) is True
    assert permitted(0.0) is False


def test_an_ordinary_parameter_is_written_and_verified():
    connection = _connection("RNGFND2_TYPE", [10.0, 0.0])
    result = set_parameter(connection, "RNGFND2_TYPE", 0.0)

    connection.mav.param_set_send.assert_called_once()
    assert connection.mav.param_set_send.call_args.args[2] == b"RNGFND2_TYPE"
    assert result.previous == 10.0
    assert result.current == 0.0
    assert result.changed is True


def test_a_value_the_vehicle_never_confirms_raises():
    connection = _connection("RNGFND2_TYPE", [10.0])
    with pytest.raises(ParameterWriteError, match="did not confirm"):
        set_parameter(connection, "RNGFND2_TYPE", 0.0, timeout=1.0)


def test_an_already_correct_parameter_is_not_rewritten():
    connection = _connection("RNGFND2_TYPE", [0.0])
    result = set_parameter(connection, "RNGFND2_TYPE", 0.0)
    connection.mav.param_set_send.assert_not_called()
    assert result.changed is False


def test_an_armed_vehicle_is_never_written_to():
    connection = MagicMock()
    connection.target_system = 1
    connection.target_component = 1
    connection.recv_match.return_value = _heartbeat(armed=True)
    with pytest.raises(RuntimeError):
        set_parameter(connection, "RNGFND2_TYPE", 0.0, timeout=1.0)
    connection.mav.param_set_send.assert_not_called()


@pytest.mark.parametrize("name", ["", "lowercase", "WAY_TOO_LONG_PARAM_NAME", "A B"])
def test_an_invalid_parameter_name_is_rejected(name):
    with pytest.raises(ValueError, match="invalid ArduPilot parameter name"):
        set_parameter(MagicMock(), name, 1.0)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True])
def test_a_nonfinite_value_is_rejected(value):
    with pytest.raises(ValueError, match="finite"):
        set_parameter(MagicMock(), "RNGFND2_TYPE", value)


def test_writing_requires_the_exact_confirmation_phrase():
    args = argparse.Namespace(
        confirm="yes", name="RNGFND2_TYPE", value=0.0, device=None, baud=115200
    )
    with pytest.raises(ValueError, match=cli.CONFIRMATION):
        cli.cmd_set(args)
