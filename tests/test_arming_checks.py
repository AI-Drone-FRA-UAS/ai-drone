"""Tests for the one parameter write in the project.

The property that matters is negative: this tool must be unable to leave the
aircraft with anything other than the full pre-arm check set.
"""

from __future__ import annotations

import argparse
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from ai_drone.cli import arming
from ai_drone.cli.arming import restore_arming_checks
from ai_drone.mavlink.arming_checks import (
    ALL_CHECKS,
    ALL_EXCEPT_GPS,
    GPS_CHECKS,
    INS,
    PARAMETER,
    RANGEFINDER,
    ArmingCheckError,
    describe,
    is_acceptable,
)


def _heartbeat(armed: bool = False):
    return SimpleNamespace(
        get_type=lambda: "HEARTBEAT",
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
        base_mode=128 if armed else 0,
    )


def _param(value: float):
    return SimpleNamespace(
        get_type=lambda: "PARAM_VALUE",
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
        param_id=PARAMETER,
        param_value=value,
    )


def _connection(values):
    """A connection whose parameter reads walk ``values``, then hold the last.

    ``require_fresh_disarmed_heartbeat`` drains queued heartbeats before it
    waits for a new one, so the drain pass has to come back empty.
    """

    connection = MagicMock()
    connection.target_system = 1
    connection.target_component = 1
    remaining = list(values)

    def recv_match(*args, **kwargs):
        if kwargs.get("type") == "HEARTBEAT":
            return None if kwargs.get("blocking") is False else _heartbeat()
        if not remaining:
            return None
        return _param(remaining.pop(0) if len(remaining) > 1 else remaining[0])

    connection.recv_match.side_effect = recv_match
    return connection


def test_a_vehicle_already_running_every_check_is_not_written_to():
    connection = _connection([ALL_CHECKS])
    result = restore_arming_checks(connection)

    connection.mav.param_set_send.assert_not_called()
    assert result.changed is False
    assert "already" in result.describe()


def test_disabled_checks_are_restored_and_verified():
    connection = _connection([0.0, ALL_CHECKS])
    result = restore_arming_checks(connection)

    connection.mav.param_set_send.assert_called_once()
    name, value = connection.mav.param_set_send.call_args.args[2:4]
    assert name == PARAMETER.encode("ascii")
    assert value == ALL_CHECKS
    assert result.previous == 0.0
    assert result.current == ALL_CHECKS
    assert result.changed is True


def test_an_arbitrary_subset_is_replaced_with_the_full_set():
    connection = _connection([22.0, ALL_CHECKS])
    restore_arming_checks(connection)
    assert connection.mav.param_set_send.call_args.args[3] == ALL_CHECKS


def test_the_gps_free_configuration_keeps_every_other_check():
    # The point of the relaxation is that it is surgical.
    assert not int(ALL_EXCEPT_GPS) & GPS_CHECKS
    assert int(ALL_EXCEPT_GPS) & INS
    assert int(ALL_EXCEPT_GPS) & RANGEFINDER


def test_only_the_two_documented_values_are_acceptable():
    assert is_acceptable(ALL_CHECKS)
    assert is_acceptable(ALL_EXCEPT_GPS)
    for rejected in (0.0, 22.0, 8.0, ALL_EXCEPT_GPS + 1):
        assert not is_acceptable(rejected)


def test_without_gps_writes_the_gps_free_configuration():
    connection = _connection([ALL_CHECKS, ALL_EXCEPT_GPS])
    result = restore_arming_checks(connection, without_gps=True)
    assert connection.mav.param_set_send.call_args.args[3] == ALL_EXCEPT_GPS
    assert result.current == ALL_EXCEPT_GPS


def test_a_disabled_check_set_is_described_as_unable_to_report():
    assert "cannot tell anyone what is wrong" in describe(0.0)


def test_a_bitmask_is_described_as_an_integer_not_in_scientific_notation():
    # ARMING_CHECK=1.04396e+06 is not a value anyone can type into a GCS.
    assert "1043958" in describe(ALL_EXCEPT_GPS)
    assert "e+" not in describe(ALL_EXCEPT_GPS)


def test_a_vehicle_that_does_not_confirm_raises_rather_than_reporting_success():
    connection = _connection([0.0])
    with pytest.raises(ArmingCheckError, match="Do not arm"):
        restore_arming_checks(connection, timeout=1.0)


def test_an_armed_vehicle_is_never_written_to():
    connection = MagicMock()
    connection.target_system = 1
    connection.target_component = 1
    connection.recv_match.return_value = _heartbeat(armed=True)

    with pytest.raises(RuntimeError):
        restore_arming_checks(connection, timeout=1.0)
    connection.mav.param_set_send.assert_not_called()


def test_restoring_requires_the_exact_confirmation_phrase():
    parser_args = argparse.Namespace(
        confirm="yes", without_gps=False, device=None, baud=115200
    )
    with pytest.raises(ValueError, match=arming.CONFIRMATION):
        arming.cmd_restore(parser_args)


def test_the_cli_exposes_no_way_to_choose_a_value():
    # A --value or --set option would defeat the entire point of the command.
    help_text = arming._parser().format_help()
    assert "--value" not in help_text
    assert "--set" not in help_text
    assert "--disable" not in help_text


def test_every_command_line_parser_builds():
    """A parser that raises on construction makes its whole command unusable.

    This caught a real break: the guarded-parameter table changed shape and the
    help text was still formatting its values as numbers.
    """

    from ai_drone.cli import arming, control, parameters, rehearse

    for module in (arming, control, parameters, rehearse):
        assert module._parser().format_help()


def test_the_guard_does_not_depend_on_an_import_side_effect():
    # Importing only the write path must still protect ARMING_CHECK.
    from ai_drone.mavlink.parameter_write import PROTECTED_PARAMETERS

    assert PARAMETER in PROTECTED_PARAMETERS
    assert PROTECTED_PARAMETERS[PARAMETER](0.0) is False
