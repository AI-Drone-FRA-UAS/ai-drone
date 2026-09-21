import pytest

from tests.test_sitl import _require_loopback_namespace


@pytest.mark.parametrize(
    "interfaces,marker",
    [(["lo", "eth0"], "1"), (["lo", "tailscale0"], "1"), (["lo"], "0")],
)
def test_simulation_refuses_possible_aircraft_routes(monkeypatch, interfaces, marker):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", marker)
    monkeypatch.setattr("socket.if_nameindex", lambda: list(enumerate(interfaces)))
    with pytest.raises(RuntimeError, match="loopback-only"):
        _require_loopback_namespace()


def test_simulation_accepts_only_marked_loopback_namespace(monkeypatch):
    monkeypatch.setenv("AI_DRONE_ISOLATED_SITL", "1")
    monkeypatch.setattr("socket.if_nameindex", lambda: [(1, "lo")])
    _require_loopback_namespace()
