from __future__ import annotations

import pytest

from ai_drone.cli.power import _guard_idle, _require_disarmed
from ai_drone.power_state import FcAbsent, FcBusy, FcUnavailable, parse_fc_probe


@pytest.mark.parametrize(
    "status,kind",
    [
        ("absent", FcAbsent),
        ("busy", FcBusy),
        ("unknown", FcUnavailable),
        ("unavailable", FcUnavailable),
    ],
)
def test_unavailable_probe_preserves_reason_and_cannot_authorize_power(status, kind):
    document = {
        "status": status,
        "reason": "owner inspection failed",
        "heartbeat_age_s": 0,
    }
    parsed = parse_fc_probe(document)
    assert isinstance(parsed, kind)
    assert parsed.reason == "owner inspection failed"
    with pytest.raises(RuntimeError, match="fresh selected-FC"):
        _require_disarmed(document)


@pytest.mark.parametrize(
    "age", [True, "0", None, -1, 2.001, float("nan"), float("inf"), 10**500]
)
def test_invalid_heartbeat_representation_never_authorizes_power(age):
    with pytest.raises(RuntimeError, match="fresh selected-FC"):
        _require_disarmed({"status": "disarmed", "heartbeat_age_s": age})


@pytest.mark.parametrize(
    "field,value",
    [
        ("hardware", None),
        ("packages", {}),
        ("runtime", []),
        ("walk", {"ActiveState": "unknown"}),
    ],
)
def test_malformed_snapshot_fails_closed_before_power_action(field, value):
    snapshot = {
        "complete": True,
        "package_database_ok": True,
        "packages": [],
        "hardware": [],
        "walk": {"ActiveState": "inactive"},
        "runtime": None,
        field: value,
    }
    with pytest.raises(RuntimeError, match=r"unknown|malformed"):
        _guard_idle(snapshot)


def test_truthy_owner_role_is_not_a_runtime_or_recorder_identity():
    snapshot = {
        "complete": True,
        "package_database_ok": True,
        "packages": [],
        "hardware": [{"walk": "yes"}],
        "walk": {"ActiveState": "active"},
        "runtime": None,
    }
    with pytest.raises(RuntimeError, match="hardware is busy"):
        _guard_idle(snapshot, allow_walk=True)
