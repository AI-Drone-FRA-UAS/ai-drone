"""Unit-Tests für das GameMode Drohnensteuerungs-Modul (gameMode)."""

import time
from typing import Any
from unittest.mock import MagicMock

import pytest

from gameMode.actor import (
    DEFAULT_FAILSAFE_CONFIG,
    DroneGameActor,
    FailsafeConfig,
    FailsafeException,
    PWM_NEUTRAL,
    PWM_THROTTLE_DISARM,
    PWM_THROTTLE_HOVER,
    Vector3,
)


def test_vector3_operations() -> None:
    """Testet die 3D-Vektor-Arithmetik und Richtungs-Presets."""
    v1 = Vector3(1.0, 2.0, 3.0)
    v2 = Vector3(0.5, -1.0, 2.0)

    # Addition & Subtraktion
    add = v1 + v2
    assert pytest.approx(add.x) == 1.5
    assert pytest.approx(add.y) == 1.0
    assert pytest.approx(add.z) == 5.0

    sub = v1 - v2
    assert pytest.approx(sub.x) == 0.5
    assert pytest.approx(sub.y) == 3.0
    assert pytest.approx(sub.z) == 1.0

    # Skalierung & Multiplikation
    scaled = v1 * 2.0
    assert scaled.x == 2.0 and scaled.y == 4.0 and scaled.z == 6.0
    rscaled = 0.5 * v1
    assert rscaled.x == 0.5 and rscaled.y == 1.0 and rscaled.z == 1.5

    # Länge & Normalisierung
    v_norm = Vector3(3.0, 4.0, 0.0)
    assert pytest.approx(v_norm.length()) == 5.0
    normalized = v_norm.normalized()
    assert pytest.approx(normalized.length()) == 1.0
    assert pytest.approx(normalized.x) == 0.6
    assert pytest.approx(normalized.y) == 0.8

    # Nullvektor Normalisierung
    assert Vector3.zero().normalized() == Vector3.zero()

    # Presets
    assert Vector3.forward() == Vector3(1.0, 0.0, 0.0)
    assert Vector3.back() == Vector3(-1.0, 0.0, 0.0)
    assert Vector3.left() == Vector3(0.0, -1.0, 0.0)
    assert Vector3.right() == Vector3(0.0, 1.0, 0.0)
    assert Vector3.up() == Vector3(0.0, 0.0, 1.0)
    assert Vector3.down() == Vector3(0.0, 0.0, -1.0)


def create_test_actor(
    failsafe_cfg: FailsafeConfig | None = None,
) -> tuple[DroneGameActor, MagicMock]:
    """Hilfsfunktion zur Erstellung einer gemockten Actor-Instanz ohne echte MAVLink-Hardware."""
    cfg = failsafe_cfg or DEFAULT_FAILSAFE_CONFIG
    actor = DroneGameActor(
        device="udp:127.0.0.1:14550",
        failsafe_config=cfg,
        auto_connect=False,
    )
    mock_conn = MagicMock()
    mock_conn.mode_mapping.return_value = {"ALT_HOLD": 2, "FLOWHOLD": 22, "LAND": 9}
    mock_conn.target_system = 1
    mock_conn.target_component = 1
    mock_conn.flightmode = "ALT_HOLD"
    mock_conn.recv_match.return_value = None

    actor.connection = mock_conn
    actor.flight_mode = "ALT_HOLD"
    actor.is_armed = True
    actor.last_telemetry_time = time.monotonic()
    return actor, mock_conn


def test_axis_input_to_rc_mapping() -> None:
    """Testet die Umrechnung von normalisierten Game-Inputs (-1.0..1.0) in RC-Kanalwerte."""
    actor, _ = create_test_actor()

    # Neutral
    actor.set_axis_input(forward=0.0, strafe=0.0, vertical=0.0, yaw=0.0)
    assert actor._target_pitch == PWM_NEUTRAL
    assert actor._target_roll == PWM_NEUTRAL
    assert actor._target_throttle == PWM_THROTTLE_HOVER
    assert actor._target_yaw == PWM_NEUTRAL

    # Vorwärts (Pitch kleiner 1500)
    actor.set_axis_input(forward=1.0)
    assert actor._target_pitch == 1500 - 120  # 1380 PWM
    assert actor._target_roll == PWM_NEUTRAL

    # Rückwärts (Pitch größer 1500)
    actor.set_axis_input(forward=-1.0)
    assert actor._target_pitch == 1500 + 120  # 1620 PWM

    # Strafe Rechts (Roll größer 1500)
    actor.set_axis_input(strafe=1.0)
    assert actor._target_roll == 1500 + 120  # 1620 PWM

    # Steigen (Throttle größer 1500)
    actor.set_axis_input(vertical=1.0)
    assert actor._target_throttle == 1500 + 120  # 1620 PWM

    # Drehen Rechts (Yaw größer 1500)
    actor.set_axis_input(yaw=1.0)
    assert actor._target_yaw == 1500 + 120  # 1620 PWM


def test_altitude_failsafe_trigger() -> None:
    """Testet das sofortige Auslösen des Failsafes bei Überschreiten des Höhenlimits."""
    cfg = FailsafeConfig(max_altitude=0.80)
    actor, mock_conn = create_test_actor(cfg)
    actor.is_flying = True

    # 1. Normale Höhe -> Kein Failsafe
    actor.current_altitude = 0.50
    actor._check_failsafes()
    assert actor._failsafe_triggered is False

    # 2. Höhe über Limit -> Sofortiger Failsafe
    actor.current_altitude = 0.85
    actor._check_failsafes()
    assert actor._failsafe_triggered is True
    assert "Höhenlimit" in actor._failsafe_reason
    assert actor.is_flying is False

    # Verifiziere Kill-Befehle
    assert actor._target_throttle == PWM_THROTTLE_DISARM
    mock_conn.arducopter_disarm.assert_called()


def test_vertical_speed_failsafe_trigger() -> None:
    """Testet das Auslösen des Failsafes bei zu hoher Steig-/Sinkgeschwindigkeit."""
    cfg = FailsafeConfig(max_vertical_speed=0.80)
    actor, _ = create_test_actor(cfg)
    actor.is_flying = True

    # Moderate Geschwindigkeit -> Kein Failsafe
    actor.filtered_vz = 0.30
    actor._check_failsafes()
    assert actor._failsafe_triggered is False

    # Zu schnelles Steigen -> Failsafe
    actor.filtered_vz = 0.95
    actor._check_failsafes()
    assert actor._failsafe_triggered is True
    assert "Vertikalgeschwindigkeit" in actor._failsafe_reason


def test_tilt_angle_failsafe_trigger() -> None:
    """Testet das Auslösen des Failsafes bei kritischer Schräglage (Roll/Pitch)."""
    cfg = FailsafeConfig(max_tilt_angle_deg=25.0)
    actor, _ = create_test_actor(cfg)
    actor.is_flying = True

    # Moderate Neigung -> Kein Failsafe
    actor.roll_deg = 10.0
    actor.pitch_deg = -5.0
    actor._check_failsafes()
    assert actor._failsafe_triggered is False

    # Kritische Schräglage (z. B. 30° Roll) -> Failsafe
    actor.roll_deg = 32.0
    actor._check_failsafes()
    assert actor._failsafe_triggered is True
    assert "Schräglage" in actor._failsafe_reason


def test_telemetry_timeout_failsafe_trigger() -> None:
    """Testet das Auslösen des Failsafes bei Signalverlust / Sensor-Timeout."""
    cfg = FailsafeConfig(telemetry_timeout_s=0.20)
    actor, _ = create_test_actor(cfg)
    actor.is_flying = True

    # Kürzlich empfangen -> OK
    actor.last_telemetry_time = time.monotonic()
    actor._check_failsafes()
    assert actor._failsafe_triggered is False

    # Signalverlust (vor 0.5s) -> Failsafe
    actor.last_telemetry_time = time.monotonic() - 0.50
    actor._check_failsafes()
    assert actor._failsafe_triggered is True
    assert "Sensor-Timeout" in actor._failsafe_reason


def test_movement_methods_execution() -> None:
    """Testet die diskreten Bewegungsmethoden wie move_forward, move_left, etc."""
    actor, _ = create_test_actor()
    actor.is_flying = True

    # move_forward
    actor.move_forward(duration_s=0.05, speed=0.4)
    # Nach Abschluss muss die Drohne wieder im Schwebeflug (1500) sein
    assert actor._target_pitch == PWM_NEUTRAL
    assert actor._target_roll == PWM_NEUTRAL
    assert actor._target_throttle == PWM_THROTTLE_HOVER

    # move mit Vektor
    actor.move(Vector3.left(), duration_s=0.05, speed=0.4)
    assert actor._target_pitch == PWM_NEUTRAL
    assert actor._target_roll == PWM_NEUTRAL


def test_takeoff_procedure_and_climb() -> None:
    """Testet den prozeduralen Start (Takeoff)."""
    actor, mock_conn = create_test_actor()
    actor.is_armed = False
    actor.current_altitude = 0.02

    # Nach arducopter_arm wird is_armed True
    def mock_arm(*_args: Any, **_kwargs: Any) -> None:
        actor.is_armed = True

    mock_conn.arducopter_arm.side_effect = mock_arm

    # Simuliere schnellen Höhengewinn
    def trigger_height(*_args: Any, **_kwargs: Any) -> None:
        actor.current_altitude = 0.52

    mock_conn.mav.rc_channels_override_send.side_effect = trigger_height

    actor.takeoff(height_m=0.50, timeout=1.0)

    assert actor.is_flying is True
    assert actor.is_armed is True
    assert actor._target_throttle == PWM_THROTTLE_HOVER


def test_land_procedure_and_touchdown() -> None:
    """Testet den Landeablauf mit Touchdown-Erkennung."""
    actor, mock_conn = create_test_actor()
    actor.is_flying = True
    actor.is_armed = True
    actor.current_altitude = 0.40

    # Simuliere schnellen Sinkflug auf Touchdown-Höhe
    def trigger_touchdown(*_args: Any, **_kwargs: Any) -> None:
        actor.current_altitude = 0.03

    mock_conn.mav.rc_channels_override_send.side_effect = trigger_touchdown

    actor.land(timeout=1.0)

    assert actor.is_flying is False
    assert actor._target_throttle == PWM_THROTTLE_DISARM
    mock_conn.arducopter_disarm.assert_called()


def test_context_manager_safety_trap() -> None:
    """Testet, dass der Kontext-Manager beim Verlassen im Flugzustand sofort Not-Disarm ausführt."""
    actor, mock_conn = create_test_actor()
    actor.is_flying = True
    actor.is_armed = True

    with actor:
        pass  # Verlasse Block während is_flying = True

    assert actor.is_flying is False
    assert actor._target_throttle == PWM_THROTTLE_DISARM
    mock_conn.arducopter_disarm.assert_called()


def test_failsafe_blocks_new_commands() -> None:
    """Testet, dass nach einem Failsafe keine neuen Steuerungsbefehle mehr angenommen werden."""
    actor, _ = create_test_actor()
    actor.trigger_failsafe("Test-Absturz")

    with pytest.raises(FailsafeException):
        actor.set_axis_input(forward=1.0)

    with pytest.raises(FailsafeException):
        actor.takeoff(height_m=0.50)

    with pytest.raises(FailsafeException):
        actor.hover(duration_s=0.5)
