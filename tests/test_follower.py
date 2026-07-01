"""Unit-Tests für das autonome Person-Tracking-Modul (ai_drone/follower.py)."""

from __future__ import annotations

import pytest
from unittest.mock import MagicMock

from ai_drone.follower import AutonomousFollower, PersonTarget, get_person_target


class MockDetection:
    """Hilfsklasse zum Simulieren einer Kamera-Detektion von modlib oder numpy."""

    def __init__(
        self, box: list[float], confidence: float = 0.85, class_id: int = 0
    ) -> None:
        self.box = box
        self.confidence = confidence
        self.class_id = class_id


def test_get_person_target_calculation():
    """Testet die Berechnung von Pixel-Offset und Distanzschätzung aus Bounding Boxes."""
    # Bounding Box von x1=200 bis x2=440 (Breite 240 px), y1=100 bis y2=340 (Höhe 240 px)
    # Bildauflösung 640x480 -> Mitte ist (320, 240) -> Box-Zentrum xc=320, yc=220 -> offset_x = 0
    det = MockDetection(box=[200.0, 100.0, 440.0, 340.0], confidence=0.9, class_id=0)
    target = get_person_target([det], frame_width=640, frame_height=480)

    assert target is not None
    assert target.offset_x_px == pytest.approx(0.0, abs=1e-2)
    assert target.offset_y_px == pytest.approx(-20.0, abs=1e-2)
    assert target.confidence == 0.9
    assert target.distance_m > 0.0  # Distanz sollte plausibel geschätzt werden


def test_get_person_target_filters_classes_and_confidence():
    """Testet, dass Nicht-Personen und Detektionen mit geringer Konfidenz ignoriert werden."""
    det_low_conf = MockDetection(
        box=[10.0, 10.0, 50.0, 50.0], confidence=0.2, class_id=0
    )
    det_car = MockDetection(
        box=[100.0, 100.0, 300.0, 300.0], confidence=0.9, class_id=2
    )  # z.B. Auto

    target = get_person_target(
        [det_low_conf, det_car], frame_width=640, frame_height=480, min_confidence=0.4
    )
    assert target is None


def test_compute_velocity_command_forward_and_yaw():
    """Testet die P-Regelung für Vorwärts-/Rückwärtsbewegung und Gier-Rate."""
    mock_drone = MagicMock()
    follower = AutonomousFollower(
        mock_drone, target_dist_m=2.0, kp_dist=0.5, kp_yaw=0.5
    )

    # Person ist 3.0 m entfernt (> 2.0 m Target) und rechts im Bild (offset_x = +100 px)
    target = PersonTarget(
        distance_m=3.0,
        offset_x_px=100.0,
        offset_y_px=0.0,
        confidence=0.8,
        box_area=1000.0,
    )

    vx, vy, vz, yaw_rate = follower.compute_velocity_command(target)

    # Sollte vorwärts fliegen (+vx) und nach rechts drehen (+yaw_rate)
    assert vx > 0.0
    assert vy == 0.0
    assert vz == 0.0
    assert yaw_rate > 0.0


def test_lost_target_hover():
    """Testet, dass bei Verlust des Targets auf Schwebeflug (0 m/s) geschaltet wird."""
    mock_drone = MagicMock()
    follower = AutonomousFollower(mock_drone)

    vx, vy, vz, yaw_rate = follower.compute_velocity_command(None)
    assert (vx, vy, vz, yaw_rate) == (0.0, 0.0, 0.0, 0.0)


def test_safety_battery_guardrail_triggers_landing():
    """Testet den automatischen Sicherheitsabbruch bei kritischem Batteriestand."""
    mock_drone = MagicMock()
    mock_drone.battery_voltage = 13.8  # Kritisch unter Standard von 14.4 V

    follower = AutonomousFollower(mock_drone, min_battery_v=14.4)

    with pytest.raises(RuntimeError, match="Batterie-Wächter"):
        follower.check_safety_guardrails()

    mock_drone.emergency_stop.assert_called_once()


def test_simulated_tracking_runs_cleanly():
    """Testet die Ausführung der simulierten Autonomie-Schleife."""
    mock_drone = MagicMock()
    mock_drone.is_flying = True
    mock_drone.is_armed = True
    mock_drone.battery_voltage = 15.5
    mock_drone.current_altitude = 0.5
    mock_drone.max_altitude = 1.0

    follower = AutonomousFollower(mock_drone)
    # Kurz laufen lassen
    follower.run_simulated_tracking(duration_s=0.25)

    # send_velocity_body sollte mehrfach aufgerufen worden sein
    assert mock_drone.send_velocity_body.call_count >= 1
