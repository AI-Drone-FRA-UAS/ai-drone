"""The application verifies isolation independently of the SITL harness."""

from argparse import Namespace

import pytest

from ai_drone.cli import control
from ai_drone.flight.controller import DroneController, isolated_sitl_environment


@pytest.mark.parametrize(
    "interfaces",
    [["lo", "enp1s0"], ["lo", "tailscale0"], ["lo", "veth0"], ["eth0"], []],
)
def test_marker_cannot_enable_experiments_with_nonisolated_interfaces(
    monkeypatch, interfaces
):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")
    monkeypatch.setattr("socket.if_nameindex", lambda: list(enumerate(interfaces)))
    monkeypatch.setattr(
        "ai_drone.flight.controller.open_ardupilot_connection",
        lambda *args, **kwargs: pytest.fail("connection opened"),
    )
    assert not isolated_sitl_environment()
    baseline = DroneController(device="tcp:127.0.0.1:5760")
    assert baseline.altitude_hold_limit_s == 10.0
    with pytest.raises(ValueError, match="isolated local SITL"):
        DroneController(
            device="tcp:127.0.0.1:5760",
            navigation_profile="flow-inertial-experimental",
        )


def test_interface_inspection_failure_does_not_enable_experiments(monkeypatch):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")

    def unavailable():
        raise OSError("interface enumeration unavailable")

    monkeypatch.setattr("socket.if_nameindex", unavailable)
    assert not isolated_sitl_environment()
    assert DroneController(device="tcp:127.0.0.1:5760").altitude_hold_limit_s == 10.0
    with pytest.raises(ValueError, match="isolated local SITL"):
        DroneController(
            device="tcp:127.0.0.1:5760",
            navigation_profile="flow-inertial-experimental",
        )


@pytest.mark.parametrize("marker", [None, "", "0", "true"])
def test_loopback_namespace_also_requires_explicit_opt_in(monkeypatch, marker):
    monkeypatch.delenv("AI_DRONE_ISOLATED_SITL", raising=False)
    if marker is not None:
        monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", marker)
    monkeypatch.setattr("socket.if_nameindex", lambda: [(1, "lo")])
    assert not isolated_sitl_environment()
    assert DroneController(device="tcp:127.0.0.1:5760").altitude_hold_limit_s == 10.0


def test_marked_loopback_namespace_enables_only_literal_local_tcp(monkeypatch):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")
    monkeypatch.setattr("socket.if_nameindex", lambda: [(1, "lo")])
    assert isolated_sitl_environment()
    experimental = DroneController(
        device="tcp:127.0.0.1:5760",
        navigation_profile="flow-inertial-experimental",
    )
    assert experimental.navigation_profile.experimental
    assert experimental.altitude_hold_limit_s == 30.0
    assert DroneController(device="tcp:192.168.1.10:5760").altitude_hold_limit_s == 10.0


def _extended_hold_arguments():
    return Namespace(
        command="altitude-hold",
        device="tcp:127.0.0.1:5760",
        baud=115200,
        max_alt=0.8,
        takeoff_alt=0.5,
        duration=30.0,
        min_battery=0.0,
        navigation_timeout=20.0,
    )


def test_cli_duration_validation_checks_actual_interfaces(monkeypatch):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")
    monkeypatch.setattr("socket.if_nameindex", lambda: [(1, "lo"), (2, "wlan0")])
    with pytest.raises(ValueError, match="--duration"):
        control._validate_common(_extended_hold_arguments())
    monkeypatch.setattr("socket.if_nameindex", lambda: [(1, "lo")])
    control._validate_common(_extended_hold_arguments())


def test_cli_rejects_spoofed_marker_before_opening_flight_session(monkeypatch):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")
    monkeypatch.setattr("socket.if_nameindex", lambda: [(1, "lo"), (2, "wlan0")])
    monkeypatch.setattr(
        control, "_flight_session", lambda args: pytest.fail("flight session opened")
    )
    assert (
        control.main(
            [
                "altitude-hold",
                "--device",
                "tcp:127.0.0.1:5760",
                "--duration",
                "30",
                "--confirm-flight",
                control.FLIGHT_CONFIRMATION,
            ]
        )
        == 1
    )
