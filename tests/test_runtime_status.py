"""One representation with deliberately different health/maintenance policies."""

import pytest

from ai_drone.runtime_status import RuntimeStatus


def status(**changes):
    return RuntimeStatus.parse(
        {
            "status": "disarmed",
            "armed": False,
            "fresh": True,
            "source_known": True,
            "system_id": 1,
            "component_id": 1,
            "heartbeat_age_s": 0.5,
            "updated_monotonic": 10.0,
            "closed": False,
            **changes,
        }
    )


def test_restart_health_does_not_authorize_armed_maintenance():
    observed = status(armed=True, status="armed")
    assert observed.healthy_after_restart(10.5)
    assert not observed.disarmed(10.5)


def test_effective_age_preserves_inclusive_boundary_and_original_publication():
    observed = status()
    assert observed.disarmed(11.5)
    assert not observed.disarmed(11.50001)
    assert not observed.disarmed(9.9)
    assert observed.heartbeat_age_s == 0.5


@pytest.mark.parametrize(
    "value", [True, "0.1", float("nan"), float("inf"), -1, 10**400]
)
def test_invalid_age_never_authorizes(value):
    observed = status(heartbeat_age_s=value)
    assert not observed.disarmed(10)
    assert not observed.healthy_after_restart(10)


@pytest.mark.parametrize("changes", [{"system_id": True}, {"armed": 0}, {"fresh": 1}])
def test_boolean_and_integer_evidence_are_not_interchangeable(changes):
    assert not status(**changes).disarmed(10)


def test_installer_receipt_policy_remains_distinct_from_published_snapshot_age():
    observed = status(updated_monotonic=None)
    assert observed.disarmed_at_receipt()
    assert not observed.disarmed(10)


@pytest.mark.parametrize(
    "changes", [{"closed": True}, {"error": "link"}, {"network_error": "nmcli"}]
)
def test_restart_health_requires_healthy_resources(changes):
    assert not status(**changes).healthy_after_restart(10)
