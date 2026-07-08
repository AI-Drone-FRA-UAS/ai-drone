"""Unit-Tests für das modulare MAVLink-Steuerungsmodul (DroneController)."""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.controller import DroneController


def test_find_device_paths(tmp_path: Any) -> None:
    """Testet die automatische und manuelle Schnittstellenerkennung."""
    # 1. Netzwerk-Strings (udp, tcp) werden direkt akzeptiert
    assert DroneController.find_device("udp:127.0.0.1:14550") == "udp:127.0.0.1:14550"
    assert DroneController.find_device("tcp:192.168.1.2:5760") == "tcp:192.168.1.2:5760"

    # 2. Existierende Datei/Device wird akzeptiert
    device_file = tmp_path / "ttyACM_test"
    device_file.touch()
    assert DroneController.find_device(str(device_file)) == str(device_file)

    # 3. Nicht-existierende Datei wirft FileNotFoundError
    with pytest.raises(FileNotFoundError):
        DroneController.find_device(str(tmp_path / "non_existent"))

    # 4. Automatisches Finden, wenn requested=None
    with (
        patch("ai_drone.mavlink_devices.Path.glob") as mock_glob,
        patch("ai_drone.mavlink_devices.Path.exists") as mock_exists,
    ):
        # Simuliere, dass /dev/serial0 existiert
        mock_glob.return_value = []
        mock_exists.side_effect = lambda: True
        found = DroneController.find_device(None)
        assert "serial0" in str(found) or "tty" in str(found)


def test_telemetry_state_tracking() -> None:
    """Testet das nicht-blockierende Auslesen und Updaten von Sensordaten."""
    controller = DroneController(device="udp:127.0.0.1:14550")
    mock_conn = MagicMock()
    controller.connection = mock_conn

    # Simuliere MAVLink-Nachrichten in der Warteschlange
    msg_heartbeat = SimpleNamespace(
        get_type=lambda: "HEARTBEAT",
        base_mode=mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
    )
    mock_conn.flightmode = "GUIDED"

    msg_pos = SimpleNamespace(
        get_type=lambda: "LOCAL_POSITION_NED",
        z=-0.45,  # -z ist 0.45 m Höhe
    )
    msg_sys = SimpleNamespace(
        get_type=lambda: "SYS_STATUS",
        voltage_battery=15800,  # 15.8 V in Millivolt
    )

    # recv_match liefert nacheinander die Nachrichten, dann None
    mock_conn.recv_match.side_effect = [msg_heartbeat, msg_pos, msg_sys, None]

    controller.update_telemetry()

    assert controller.is_armed is True
    assert controller.flight_mode == "GUIDED"
    assert controller.current_altitude == 0.45
    assert controller.battery_voltage == 15.8


def test_set_mode_verification_and_timeout() -> None:
    """Testet die Verifizierung des Moduswechsels und die Timeout-Behandlung."""
    controller = DroneController(device="udp:127.0.0.1:14550")
    mock_conn = MagicMock()
    controller.connection = mock_conn
    mock_conn.mode_mapping.return_value = {"GUIDED": 4, "LAND": 9}

    # 1. Erfolgreicher Moduswechsel
    controller.flight_mode = "STABILIZE"

    # Nach dem Aufruf von set_mode_send ändert sich die simulierte Eigenschaft
    def change_mode(*_args: Any, **_kwargs: Any) -> None:
        controller.flight_mode = "GUIDED"

    mock_conn.mav.set_mode_send.side_effect = change_mode
    mock_conn.recv_match.return_value = None

    controller.set_mode("GUIDED", timeout=1.0)
    mock_conn.mav.set_mode_send.assert_called_once()

    # 2. Timeout bei fehlerhaftem/ausbleibendem Moduswechsel
    mock_conn.mav.set_mode_send.reset_mock()
    controller.flight_mode = "STABILIZE"
    mock_conn.mav.set_mode_send.side_effect = None  # Bleibt STABILIZE

    with pytest.raises(RuntimeError, match="Timeout"):
        controller.set_mode("GUIDED", timeout=0.2)


def test_send_velocity_body_formatting() -> None:
    """Testet die exakte MAVLink-Formatierung bei Body-Frame-Geschwindigkeitsbefehlen."""
    controller = DroneController(device="udp:127.0.0.1:14550")
    mock_conn = MagicMock()
    controller.connection = mock_conn
    controller.is_armed = True
    controller.is_flying = True
    controller.target_system = 1
    controller.target_component = 1

    controller.send_velocity_body(vx=0.5, vy=0.0, vz=-0.1, yaw_rate_deg=15.0)

    # Prüfe, ob set_position_target_local_ned_send mit korrekten Bits gesendet wurde
    mock_conn.mav.set_position_target_local_ned_send.assert_called_once()
    args = mock_conn.mav.set_position_target_local_ned_send.call_args[0]

    # args: (time_boot_ms, target_sys, target_comp, frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate)
    assert args[3] == mavlink.MAV_FRAME_BODY_NED  # Frame 8
    assert args[4] == 0x05C7  # Bitmask 1479 (nur vx, vy, vz und yaw_rate aktiv)
    assert args[8] == 0.5  # vx
    assert args[9] == 0.0  # vy
    assert args[10] == -0.1  # vz
    assert math.isclose(args[15], math.radians(15.0), rel_tol=1e-5)  # yaw_rate in rad/s


def test_safety_trap_on_exception() -> None:
    """Testet die automatische Notlandung, wenn im Context Manager eine Exception auftritt."""
    with patch("ai_drone.controller.mavutil.mavlink_connection") as mock_mavconn:
        mock_conn = MagicMock()
        mock_conn.target_system = 1
        mock_conn.target_component = 1
        mock_mavconn.return_value = mock_conn
        mock_conn.mode_mapping.return_value = {"LAND": 9}

        with pytest.raises(ValueError, match="Simulierter Crash"):
            with DroneController(device="udp:127.0.0.1:14550") as drone:
                drone.is_flying = True
                drone.is_armed = True
                raise ValueError("Simulierter Crash im Flug")

        # Prüfe, ob beim Exit automatisch Emergency Stop (Wechsel in LAND) ausgeführt wurde
        mock_conn.mav.set_mode_send.assert_called_with(
            1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
        )


def test_altitude_guard_triggers_emergency_stop() -> None:
    """Testet, ob bei Überschreitung des Höhenlimits sofort eine Notlandung ausgelöst wird."""
    controller = DroneController(device="udp:127.0.0.1:14550", max_altitude=0.8)
    mock_conn = MagicMock()
    controller.connection = mock_conn
    mock_conn.mode_mapping.return_value = {"LAND": 9}
    controller.is_flying = True

    # Simuliere Höhe von 0.95 m (> 0.8 m Max)
    msg_pos = SimpleNamespace(
        get_type=lambda: "LOCAL_POSITION_NED",
        z=-0.95,
    )
    mock_conn.recv_match.side_effect = [msg_pos, None]

    with patch.object(
        controller, "emergency_stop", wraps=controller.emergency_stop
    ) as mock_stop:
        controller.update_telemetry()
        mock_stop.assert_called_once()
        assert controller.is_flying is False
