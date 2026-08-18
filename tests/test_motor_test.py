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
