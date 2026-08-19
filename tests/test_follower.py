from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from ai_drone import AutonomousFollower, PersonTarget, get_person_target


class Detection:
    def __init__(self, box, confidence=0.85, class_id=0) -> None:
        self.box = box
        self.confidence = confidence
        self.class_id = class_id


def test_person_target_retains_tuple_compatibility() -> None:
    target = PersonTarget(2.0, 10.0, 0.0, 0.9, 100.0)
    distance, *_ = target
    assert distance == 2.0


def test_get_person_target_selects_largest_valid_person() -> None:
    target = get_person_target(
        [
            Detection([10, 10, 30, 30], confidence=0.9),
            Detection([100, 50, 300, 350], confidence=0.8),
            Detection([0, 0, 500, 400], class_id=2),
        ],
        640,
        480,
    )
    assert target is not None
    assert target.box_area == 60_000
    assert target.distance_m > 0


def test_follow_controller_commands_forward_and_yaw() -> None:
    follower = AutonomousFollower(
        MagicMock(), target_dist_m=2.0, kp_dist=0.5, kp_yaw=0.5
    )
    command = follower.compute_velocity_command(
        PersonTarget(3.0, 100.0, 0.0, 0.8, 1_000.0)
    )
    assert command[0] > 0.0
    assert command[1:3] == (0.0, 0.0)
    assert command[3] > 0.0


def test_battery_guard_commands_land() -> None:
    drone = MagicMock()
    drone.battery_voltage = 13.8
    drone.current_altitude = 0.5
    drone.max_altitude = 0.8
    drone.is_flying = True
    drone.altitude_is_fresh.return_value = True
    drone.heartbeat_is_fresh.return_value = True
    follower = AutonomousFollower(drone, min_battery_v=14.4)

    with pytest.raises(RuntimeError, match="battery"):
        follower.check_safety_guardrails()

    drone.emergency_stop.assert_called_once()
