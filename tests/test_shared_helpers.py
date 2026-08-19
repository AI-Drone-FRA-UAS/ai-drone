from __future__ import annotations

import argparse

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.cli_parsing import parse_even_resolution
from ai_drone.mavlink.safety import (
    heartbeat_is_armed,
    is_armed_vehicle_heartbeat,
    is_vehicle_message,
    require_fresh_disarmed_heartbeat,
)
from ai_drone.platform import is_raspberry_pi


class _Message:
    def __init__(
        self,
        *,
        message_type: str = "HEARTBEAT",
        system: int = 1,
        component: int = 1,
        armed: bool = False,
    ) -> None:
        self._message_type = message_type
        self._system = system
        self._component = component
        self.base_mode = mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0

    def get_type(self) -> str:
        return self._message_type

    def get_srcSystem(self) -> int:
        return self._system

    def get_srcComponent(self) -> int:
        return self._component


class _Connection:
    def __init__(
        self,
        *,
        queued: list[_Message] | None = None,
        incoming: list[_Message] | None = None,
    ) -> None:
        self.queued = list(queued or [])
        self.incoming = list(incoming or [])

    def recv_match(
        self,
        *,
        type: str,
        blocking: bool,
        timeout: float | None = None,
    ) -> _Message | None:
        assert type == "HEARTBEAT"
        if blocking:
            assert timeout is not None and timeout > 0
            return self.incoming.pop(0) if self.incoming else None
        assert timeout is None
        return self.queued.pop(0) if self.queued else None


def test_platform_detection_uses_supplied_device_tree_path(tmp_path) -> None:
    model = tmp_path / "model"
    model.write_text("Raspberry Pi Zero 2 W Rev 1.0\0")

    assert is_raspberry_pi(model)
    assert not is_raspberry_pi(tmp_path / "missing-model")


@pytest.mark.parametrize("value", ["1280", "1280x", "axb"])
def test_resolution_parser_rejects_malformed_values(value: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError, match="1280x960"):
        parse_even_resolution(value)


def test_mavlink_safety_filters_vehicle_and_component() -> None:
    armed = _Message(armed=True)

    assert heartbeat_is_armed(armed)
    assert is_vehicle_message(armed, system_id=1, component_id=1)
    assert not is_vehicle_message(armed, system_id=2)
    assert not is_vehicle_message(armed, system_id=1, component_id=2)
    assert is_armed_vehicle_heartbeat(armed, system_id=1)
    assert not is_armed_vehicle_heartbeat(armed, system_id=2)
    assert not is_armed_vehicle_heartbeat(
        _Message(message_type="STATUSTEXT", armed=True),
        system_id=1,
    )


def test_fresh_disarmed_check_drains_stale_heartbeat_and_filters_source() -> None:
    stale = _Message(system=1)
    other_vehicle = _Message(system=42)
    fresh = _Message(system=1)
    connection = _Connection(
        queued=[stale],
        incoming=[other_vehicle, fresh],
    )

    result = require_fresh_disarmed_heartbeat(
        connection,
        system_id=1,
        component_id=1,
        timeout=1.0,
    )

    assert result is fresh
    assert connection.queued == []
    assert connection.incoming == []


@pytest.mark.parametrize("queued", [True, False])
def test_fresh_disarmed_check_rejects_matching_armed_heartbeat(queued: bool) -> None:
    armed = _Message(system=1, armed=True)
    connection = _Connection(
        queued=[armed] if queued else [],
        incoming=[] if queued else [armed],
    )

    with pytest.raises(RuntimeError, match="ARMED"):
        require_fresh_disarmed_heartbeat(
            connection,
            system_id=1,
            component_id=1,
            timeout=1.0,
        )


@pytest.mark.parametrize("timeout", [0.0, -1.0, float("nan"), float("inf")])
def test_fresh_disarmed_check_rejects_invalid_timeout(timeout: float) -> None:
    with pytest.raises(ValueError, match="heartbeat timeout"):
        require_fresh_disarmed_heartbeat(
            _Connection(),
            system_id=1,
            component_id=1,
            timeout=timeout,
        )
