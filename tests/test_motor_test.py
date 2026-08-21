from __future__ import annotations

from types import SimpleNamespace

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.cli import motor_test


def test_motor_test_command_is_bounded_mavlink_motor_test_only() -> None:
    calls = []
    connection = SimpleNamespace(
        target_system=1,
        target_component=1,
        mav=SimpleNamespace(command_long_send=lambda *args: calls.append(args)),
    )

    motor_test._send_motor_test(
        connection,
        first_motor=1,
        throttle_percent=7.0,
        duration=0.5,
        motor_count=4,
    )

    assert len(calls) == 1
    assert calls[0][2] == mavlink.MAV_CMD_DO_MOTOR_TEST
    assert calls[0][2] != mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert calls[0][5] == mavlink.MOTOR_TEST_THROTTLE_PERCENT
    assert calls[0][6:10] == (
        7.0,
        0.5,
        4,
        mavlink.MOTOR_TEST_ORDER_SEQUENCE,
    )


def test_motor_count_rejects_duplicate_output_assignments(monkeypatch) -> None:
    functions = iter([33.0, 33.0, 34.0, 35.0, 0.0, 0.0, 0.0, 0.0])
    monkeypatch.setattr(
        motor_test,
        "_request_parameter",
        lambda *_args, **_kwargs: next(functions),
    )

    with pytest.raises(RuntimeError, match=r"Motor1.*SERVO1.*SERVO2"):
        motor_test._configured_motor_count(SimpleNamespace())


@pytest.mark.parametrize("function", [33.5, float("nan"), float("inf")])
def test_motor_count_rejects_invalid_function_values(
    monkeypatch, function: float
) -> None:
    monkeypatch.setattr(
        motor_test,
        "_request_parameter",
        lambda *_args, **_kwargs: function,
    )

    with pytest.raises(RuntimeError, match="not a finite integer"):
        motor_test._configured_motor_count(SimpleNamespace())


@pytest.mark.parametrize("throttle", [0, 10.1, 50])
def test_motor_test_rejects_unbounded_throttle_before_connecting(throttle) -> None:
    with pytest.raises(SystemExit):
        motor_test.main(
            [
                "--motor",
                "1",
                "--throttle-percent",
                str(throttle),
                "--confirm-props-removed",
                "PROPS_REMOVED",
                "--confirm-vehicle-secured",
                "VEHICLE_SECURED",
            ]
        )


def test_motor_test_requires_exact_physical_confirmations() -> None:
    with pytest.raises(SystemExit):
        motor_test.main(
            [
                "--motor",
                "1",
                "--confirm-props-removed",
                "yes",
                "--confirm-vehicle-secured",
                "VEHICLE_SECURED",
            ]
        )


@pytest.mark.parametrize("arming_check", [0.0, 1.4, 2.0])
def test_motor_test_refuses_an_unpermitted_check_set(
    monkeypatch, arming_check: float
) -> None:
    heartbeat = SimpleNamespace(
        base_mode=0,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
    )
    connection = SimpleNamespace(
        target_system=1,
        target_component=1,
        wait_heartbeat=lambda *, timeout: heartbeat,
        close=lambda: None,
    )
    monkeypatch.setattr(
        motor_test, "resolve_mavlink_endpoint", lambda *_args, **_kwargs: "fake"
    )
    monkeypatch.setattr(
        motor_test.mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )
    monkeypatch.setattr(
        motor_test,
        "_request_parameter",
        lambda *_args: arming_check,
    )

    with pytest.raises(SystemExit, match=r"ARMING_CHECK=.*This project flies with"):
        motor_test.main(
            [
                "--motor",
                "1",
                "--confirm-props-removed",
                "PROPS_REMOVED",
                "--confirm-vehicle-secured",
                "VEHICLE_SECURED",
            ]
        )


def test_motor_cleanup_closes_connection_when_stop_command_fails(
    monkeypatch, capsys
) -> None:
    connection = SimpleNamespace(closed=False)
    connection.close = lambda: setattr(connection, "closed", True)

    def fail_stop(_connection, _motor) -> None:
        raise OSError("synthetic serial failure")

    monkeypatch.setattr(motor_test, "_stop_motor_test", fail_stop)
    monkeypatch.setattr(
        motor_test,
        "_wait_until_disarmed",
        lambda _connection: pytest.fail("must not wait after the stop command failed"),
    )

    assert not motor_test._cleanup_motor_test(
        connection,
        started=True,
        first_motor=1,
    )
    assert connection.closed
    assert "could not send the motor-test stop command" in capsys.readouterr().err


def test_motor_cleanup_reports_disarmed_only_after_matching_heartbeat(
    monkeypatch,
) -> None:
    calls = []
    connection = SimpleNamespace(close=lambda: calls.append("close"))
    monkeypatch.setattr(
        motor_test,
        "_stop_motor_test",
        lambda _connection, motor: calls.append(("stop", motor)),
    )
    monkeypatch.setattr(
        motor_test,
        "_wait_until_disarmed",
        lambda _connection: True,
    )

    assert motor_test._cleanup_motor_test(
        connection,
        started=True,
        first_motor=3,
    )
    assert calls == [("stop", 3), "close"]


def test_post_stop_disarm_check_requires_a_fresh_heartbeat(monkeypatch) -> None:
    outcomes = iter([RuntimeError("still armed"), object()])
    calls = []
    connection = SimpleNamespace(target_system=1, target_component=1)

    def fresh_check(*_args, **kwargs):
        calls.append(kwargs)
        outcome = next(outcomes)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome

    monkeypatch.setattr(
        motor_test,
        "require_fresh_disarmed_heartbeat",
        fresh_check,
    )

    assert motor_test._wait_until_disarmed(connection, timeout=1.0)
    assert len(calls) == 2
    assert all(call["system_id"] == 1 for call in calls)


def test_post_stop_disarm_check_reports_fresh_heartbeat_timeout(monkeypatch) -> None:
    connection = SimpleNamespace(target_system=1, target_component=1)
    monkeypatch.setattr(
        motor_test,
        "require_fresh_disarmed_heartbeat",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(TimeoutError()),
    )

    assert not motor_test._wait_until_disarmed(connection, timeout=1.0)


def test_partial_motor_command_write_still_triggers_stop_cleanup(monkeypatch) -> None:
    heartbeat = SimpleNamespace(
        base_mode=0,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
    )
    connection = SimpleNamespace(
        target_system=1,
        target_component=1,
        wait_heartbeat=lambda *, timeout: heartbeat,
        close=lambda: setattr(connection, "closed", True),
        closed=False,
        mav=SimpleNamespace(command_long_send=lambda *_args: None),
        recv_match=lambda **_kwargs: None,
    )
    sends = []

    def fail_first_send(*_args, **_kwargs) -> None:
        sends.append("send")
        if len(sends) == 1:
            raise OSError("partial serial write")

    monkeypatch.setattr(
        motor_test, "resolve_mavlink_endpoint", lambda *_args, **_kwargs: "fake"
    )
    monkeypatch.setattr(
        motor_test.mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )
    monkeypatch.setattr(motor_test, "_request_parameter", lambda *_args, **_kwargs: 1.0)
    monkeypatch.setattr(motor_test, "_configured_motor_count", lambda _connection: 4)
    monkeypatch.setattr(motor_test, "_send_motor_test", fail_first_send)
    monkeypatch.setattr(
        motor_test,
        "require_fresh_disarmed_heartbeat",
        lambda *_args, **_kwargs: heartbeat,
    )
    monkeypatch.setattr(motor_test, "_wait_until_disarmed", lambda _connection: True)
    monkeypatch.setattr(motor_test.time, "sleep", lambda _seconds: None)

    with pytest.raises(OSError, match="partial serial write"):
        motor_test.main(
            [
                "--motor",
                "1",
                "--countdown",
                "3",
                "--confirm-props-removed",
                "PROPS_REMOVED",
                "--confirm-vehicle-secured",
                "VEHICLE_SECURED",
            ]
        )

    assert sends == ["send", "send"]
    assert connection.closed
