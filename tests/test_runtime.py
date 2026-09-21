from __future__ import annotations

import json
import sys
import threading
import time
from dataclasses import replace
from types import SimpleNamespace
from typing import Any

import pytest

from ai_drone import network, runtime
from ai_drone.network import Activation, NetworkAttempt, WifiProfile
from ai_drone.settings import OperatorSettings, RuntimeSettings, Settings

EDUROAM = "11111111-1111-4111-8111-111111111111"
PHONE = "22222222-2222-4222-8222-222222222222"
HOTSPOT = "33333333-3333-4333-8333-333333333333"
PROFILES = (
    WifiProfile(EDUROAM, "eduroam", priority=100),
    WifiProfile(PHONE, "phone", priority=10),
    WifiProfile(HOTSPOT, "Hotspot", autoconnect=False, mode="ap"),
)


class Hub:
    def __init__(self):
        self.armed = False
        self.age = 0.1
        self.healthy = True

    def status(self):
        return {
            "armed": self.armed,
            "heartbeat_age_s": self.age,
            "fresh": self.healthy and self.age is not None and 0 <= self.age <= 2,
        }


class Monitor:
    def __init__(self, settings):
        self.configured = bool(settings.endpoints)
        self.last_seen = None
        self.is_alive = False
        self.started = False
        self.closed = False

    def alive(self):
        return self.is_alive

    def start(self):
        self.started = True

    def close(self):
        self.closed = True


class Server:
    def __init__(self, *_args):
        self.control_owner = None
        self.is_network_busy = False
        self.is_maintenance = False
        self.closed = False

    def status(self):
        return {"clients": 0}

    def set_network_busy(self, busy):
        if busy and self.control_owner:
            raise RuntimeError("control owner holds the lease")
        self.is_network_busy = busy

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.closed = True


@pytest.fixture
def access(monkeypatch, tmp_path):
    manifest = tmp_path / "profiles.json"
    manifest.write_text(
        json.dumps(
            {
                "schema": 1,
                "profiles": {
                    profile.uuid: {
                        "autoconnect": profile.autoconnect,
                        "mode": profile.mode,
                    }
                    for profile in PROFILES
                },
            }
        )
    )
    settings = Settings(
        runtime=RuntimeSettings(
            socket=str(tmp_path / "vehicle.sock"),
            status=str(tmp_path / "status.json"),
            network_profiles=str(manifest),
        )
    )
    monkeypatch.setattr(runtime, "OperatorMonitor", Monitor)
    hub: Any = Hub()
    value: Any = runtime.VehicleAccess(hub, settings, manage_network=True)
    value.server = Server()
    value.profiles = PROFILES
    value.link = (EDUROAM, True, False)
    value.link_observed = time.monotonic()
    mutations = []

    def activate(profile, snapshot, **kwargs):
        assert value.server.is_network_busy
        assert runtime.fresh_disarmed(snapshot())
        mutations.append((profile.uuid, kwargs))
        return Activation(profile.uuid, time.monotonic())

    monkeypatch.setattr(runtime, "begin_activation", activate)
    monkeypatch.setattr(
        runtime, "run_nmcli", lambda arguments: mutations.append(arguments)
    )
    monkeypatch.setattr(runtime, "read_link", lambda: value.link)
    monkeypatch.setattr(runtime, "read_profiles", lambda **kwargs: PROFILES)
    monkeypatch.setattr(runtime, "inhibit_autoconnect", lambda: None)
    monkeypatch.setattr(
        runtime,
        "poll_activation",
        lambda activation, snapshot, **kwargs: network.poll_activation(
            activation, snapshot, run=lambda _command: "", **kwargs
        ),
    )
    value.mutations = mutations
    return value


def test_network_profiles_rejects_missing_or_invalid_schema(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"profiles": {}}))
    settings = Settings(
        runtime=RuntimeSettings(
            socket=str(tmp_path / "vehicle.sock"),
            status=str(tmp_path / "status.json"),
            network_profiles=str(bad),
        )
    )
    hub: Any = Hub()
    with pytest.raises(ValueError, match="schema must be 1"):
        runtime.VehicleAccess(hub, settings, manage_network=True)


@pytest.mark.parametrize(
    "document",
    [
        {"schema": True, "profiles": {}},
        {"schema": 1.0, "profiles": {}},
        {"schema": 1, "profiles": []},
        {"schema": 1, "profiles": {EDUROAM: {"autoconnect": True, "mode": []}}},
        {"schema": 1, "profiles": {EDUROAM: {"autoconnect": 1, "mode": ""}}},
        {"schema": 1, "profiles": {EDUROAM: {"mode": "infrastructure"}}},
        {"schema": 1, "profiles": {"invalid": {"autoconnect": True, "mode": ""}}},
    ],
)
def test_saved_network_policy_rejects_every_malformed_record(document):
    with pytest.raises(ValueError):
        network.parse_saved_profiles(document)


@pytest.mark.parametrize(
    "armed,age", [(True, 0.1), (None, None), (False, 3), (False, float("nan"))]
)
def test_network_request_requires_fresh_disarmed_fc_before_reserving_the_lease(
    access, armed, age
):
    access.hub.armed, access.hub.age = armed, age
    with pytest.raises(RuntimeError, match="fresh disarmed"):
        access.request({"network": ["connect", "phone"]})
    assert not access.server.is_network_busy
    assert access.requests.empty()
    assert not access.mutations


def test_active_control_owner_blocks_network_admission_even_when_grounded(access):
    access.server.control_owner = "controller"
    with pytest.raises(RuntimeError, match="control owner"):
        access.request({"network": ["connect", "phone"]})
    assert access.requests.empty()
    assert not access.mutations


def test_failed_fc_reader_blocks_network_even_with_a_recent_disarmed_heartbeat(access):
    access.hub.healthy = False
    with pytest.raises(RuntimeError, match="fresh disarmed"):
        access.request({"network": ["connect", "phone"]})
    assert not access.server.is_network_busy
    assert not access.mutations


@pytest.mark.parametrize(
    "armed,age,owner",
    [(False, 0.1, "controller"), (True, 3, "controller"), (True, 0.1, None)],
)
def test_human_handoff_requires_a_fresh_airborne_control_owner(
    access, armed, age, owner
):
    access.hub.armed, access.hub.age = armed, age
    access.server.control_owner = owner
    with pytest.raises(RuntimeError, match="active airborne controller"):
        access.request({"human": True})
    assert not access.human_requested


def test_human_handoff_is_an_explicit_request_without_network_or_fc_commands(access):
    access.hub.armed = True
    access.server.control_owner = "controller"
    assert access.request({"human": True}) == {"handoff_requested": True}
    assert access.human_requested
    assert not access.mutations


def test_manual_request_is_queued_with_lease_then_runs_after_requester_returns(access):
    assert access.request({"network": ["connect", "phone"]}) == {
        "queued": ["connect", "phone"]
    }
    assert access.server.is_network_busy
    assert not access.mutations
    access.network_tick(time.monotonic())
    assert access.mutations == [(PHONE, {"explicit_hotspot": False})]
    assert access.activation is not None
    assert access.server.is_network_busy


def test_second_queued_request_does_not_remove_the_first_request_lease(access):
    access.request({"network": ["connect", "phone"]})
    with pytest.raises(RuntimeError, match="already queued"):
        access.request({"network": ["connect", "eduroam"]})
    assert access.server.is_network_busy
    assert access.requests.get_nowait() == ["connect", "phone"]


def test_arming_after_queueing_blocks_activation_and_releases_network_lease(access):
    access.request({"network": ["connect", "phone"]})
    access.hub.armed = True
    with pytest.raises(RuntimeError, match="became armed or unknown"):
        access.network_tick(time.monotonic())
    assert not access.mutations
    assert not access.server.is_network_busy


def test_failed_activation_retains_lease_until_next_terminal_nm_observation(
    access, monkeypatch
):
    def reject(*_args, **_kwargs):
        raise RuntimeError("FC became stale")

    monkeypatch.setattr(runtime, "begin_activation", reject)
    access.request({"network": ["connect", "phone"]})
    with pytest.raises(RuntimeError, match="FC became stale"):
        access.network_tick(time.monotonic())
    assert access.server.is_network_busy
    assert access.activation is not None
    access.network_tick(time.monotonic())
    assert not access.server.is_network_busy and access.activation is None


def test_immediate_preferred_profile_failure_does_not_starve_fallback(
    access, monkeypatch
):
    attempted = []
    clock = time.monotonic()
    monkeypatch.setattr(runtime.time, "monotonic", lambda: clock)
    access.link = (None, False, False)

    def activate(profile, _snapshot, **_kwargs):
        attempted.append(profile.uuid)
        if profile.uuid == EDUROAM:
            raise RuntimeError("profile activation rejected")
        return Activation(profile.uuid, clock)

    monkeypatch.setattr(runtime, "begin_activation", activate)
    with pytest.raises(RuntimeError, match="activation rejected"):
        access.network_tick(clock)
    clock += 16
    access.network_tick(clock)
    clock += 1
    access.network_tick(clock)
    assert attempted == [EDUROAM, PHONE]


def test_hotspot_requires_explicit_command_and_cannot_be_started_by_connect(access):
    access.request({"network": ["connect", "Hotspot"]})
    with pytest.raises(ValueError, match="hotspot on"):
        access.network_tick(time.monotonic())
    assert not access.server.is_network_busy
    assert not access.mutations
    access.request({"network": ["hotspot", "on"]})
    access.network_tick(time.monotonic())
    assert access.mutations == [(HOTSPOT, {"explicit_hotspot": True})]


def test_hotspot_off_retains_lease_until_asynchronous_disconnect_finishes(access):
    access.link = (HOTSPOT, True, False)
    access.attempts.append(NetworkAttempt(EDUROAM, 1))
    access.request({"network": ["hotspot", "off"]})
    access.network_tick(time.monotonic())
    assert access.mutations == [["--wait", "0", "connection", "down", "uuid", HOTSPOT]]
    assert access.server.is_network_busy and access.deactivation is not None
    assert not access.attempts
    access.link = (HOTSPOT, False, True)
    access.network_tick(time.monotonic())
    assert access.server.is_network_busy
    access.link = (None, False, False)
    access.network_tick(time.monotonic())
    assert not access.server.is_network_busy and access.deactivation is None


@pytest.mark.parametrize("failure", ["up_timeout", "post_submit_inhibit"])
def test_activation_with_uncertain_outcome_keeps_control_blocked_until_settled(
    access, monkeypatch, failure
):
    commands = []

    def run(arguments):
        commands.append(arguments)
        if failure == "up_timeout" and "up" in arguments:
            raise RuntimeError("activation submit timed out")
        if failure == "post_submit_inhibit" and len(commands) == 3:
            raise RuntimeError("post-submit inhibition failed")
        return ""

    monkeypatch.setattr(
        runtime,
        "begin_activation",
        lambda profile, snapshot, **kwargs: network.begin_activation(
            profile, snapshot, run=run, **kwargs
        ),
    )
    access.request({"network": ["connect", "phone"]})
    with pytest.raises(RuntimeError):
        access.network_tick(time.monotonic())
    assert any("up" in command for command in commands)
    assert access.server.is_network_busy and access.activation is not None
    access.link = (PHONE, False, True)
    access.network_tick(time.monotonic())
    assert access.server.is_network_busy
    access.link = (PHONE, True, False)
    access.network_tick(time.monotonic())
    assert not access.server.is_network_busy and access.activation is None


def test_failed_hotspot_down_keeps_lease_and_unknown_nm_state_cannot_release_it(
    access, monkeypatch
):
    access.link = (HOTSPOT, True, False)
    access.request({"network": ["hotspot", "off"]})

    def failure(*_args):
        raise RuntimeError("unknown NetworkManager outcome")

    monkeypatch.setattr(runtime, "run_nmcli", failure)
    with pytest.raises(RuntimeError):
        access.network_tick(time.monotonic())
    assert access.deactivation is not None and access.server.is_network_busy
    monkeypatch.setattr(runtime, "read_link", failure)
    with pytest.raises(RuntimeError):
        access.network_tick(time.monotonic() + 100)
    access._release_network_if_idle()
    assert access.server.is_network_busy


@pytest.mark.parametrize("transitioning", [True, False])
def test_hotspot_off_timeout_releases_only_after_a_known_terminal_state(
    access, transitioning
):
    now = time.monotonic()
    access.deactivation = Activation(HOTSPOT, now - 50)
    access.server.is_network_busy = True
    access.link = (HOTSPOT, not transitioning, transitioning)
    access.network_tick(now)
    assert "timed out" in access.network_error
    assert access.server.is_network_busy is transitioning


def test_hotspot_off_does_not_disconnect_a_working_client(access):
    access.request({"network": ["hotspot", "off"]})
    access.network_tick(time.monotonic())
    assert not access.mutations
    assert not access.server.is_network_busy


def test_hotspot_off_rechecks_control_lease_before_execution(access):
    access.link = (HOTSPOT, True, False)
    access.request({"network": ["hotspot", "off"]})
    access.server.is_network_busy = False
    access.server.control_owner = "controller"
    with pytest.raises(RuntimeError, match="control owner"):
        access.network_tick(time.monotonic())
    assert not access.mutations


def test_terminal_activation_preserves_lease_for_an_already_queued_request(
    access, monkeypatch
):
    access.activation = Activation(PHONE, time.monotonic())
    access.server.is_network_busy = True
    access.request({"network": ["hotspot", "on"]})
    monkeypatch.setattr(
        runtime, "poll_activation", lambda *_args, **_kwargs: "connected"
    )
    access.network_tick(time.monotonic())
    assert access.activation is None
    assert access.server.is_network_busy
    assert not access.requests.empty()


def test_grounded_recording_without_a_control_lease_allows_failover(access):
    # Recording subscribers do not hold VehicleServer's command lease.
    access.hub.recording_subscribers = 1
    access.link = (None, False, False)
    access.network_tick(time.monotonic())
    assert access.mutations == [(EDUROAM, {"explicit_hotspot": False})]
    assert access.hub.recording_subscribers == 1


def test_working_route_and_explicit_ap_never_trigger_automatic_switching(access):
    access.network_tick(time.monotonic())
    access.link = (HOTSPOT, True, False)
    access.operator.configured = True
    access.operator.last_seen = 1
    access.operator.is_alive = False
    access.network_tick(time.monotonic())
    assert not access.mutations


def test_operator_warmup_preserves_boot_connection_until_timeout(access, monkeypatch):
    now = access.started + 4.9
    monkeypatch.setattr(runtime.time, "monotonic", lambda: now)
    access.operator.configured = True
    access.network_tick(now)
    assert not access.mutations
    now += 0.2
    access.network_tick(now)
    assert access.mutations == [(PHONE, {"explicit_hotspot": False})]


def test_previously_seen_operator_loss_does_not_get_a_new_boot_grace(access):
    access.operator.configured = True
    access.operator.last_seen = access.started
    access.operator.is_alive = False
    access.network_tick(time.monotonic())
    assert access.mutations == [(PHONE, {"explicit_hotspot": False})]


@pytest.mark.parametrize("outcome", ["pending", "connected", "failed", "cancelled"])
def test_pending_activation_keeps_lease_until_a_terminal_result(
    access, monkeypatch, outcome
):
    access.activation = Activation(PHONE, time.monotonic())
    access.server.is_network_busy = True
    monkeypatch.setattr(runtime, "poll_activation", lambda *_args, **_kwargs: outcome)
    access.network_tick(time.monotonic())
    assert access.server.is_network_busy is (outcome == "pending")
    assert (access.activation is not None) is (outcome == "pending")
    if outcome in {"failed", "cancelled"}:
        assert outcome in access.network_error


def test_status_distinguishes_authenticated_operator_from_a_stale_wifi_observation(
    access,
):
    access.operator.configured = True
    access.operator.is_alive = True
    assert runtime.operator_has_control_link(access.status())
    access.link_observed = time.monotonic() - 5
    status = access.status()
    assert status["operator_alive"]
    assert not status["wifi_connected"]
    assert not runtime.operator_has_control_link(status)
    status["wifi_required"] = False
    assert runtime.operator_has_control_link(status)


@pytest.mark.parametrize(
    "contents",
    ["not JSON", "[]", "{}", '{"updated_monotonic": -1}', '{"updated_monotonic": NaN}'],
)
def test_runtime_status_rejects_missing_malformed_or_stale_data(tmp_path, contents):
    path = tmp_path / "status.json"
    path.write_text(contents)
    with pytest.raises(RuntimeError, match="fresh vehicle runtime status"):
        runtime.read_runtime_status(path)


def test_runtime_status_accepts_only_current_local_monotonic_timestamp(tmp_path):
    path = tmp_path / "status.json"
    document = {"updated_monotonic": time.monotonic(), "armed": False}
    path.write_text(json.dumps(document))
    assert runtime.read_runtime_status(path) == document
    document["updated_monotonic"] = time.monotonic() + 10
    path.write_text(json.dumps(document))
    with pytest.raises(RuntimeError):
        runtime.read_runtime_status(path)


def test_status_publishing_continues_while_a_network_query_is_blocked(
    access, monkeypatch
):
    blocked = threading.Event()
    release = threading.Event()
    published = []
    publication = threading.Event()
    stop = threading.Event()
    server = Server()
    monkeypatch.setitem(
        sys.modules,
        "ai_drone.mavlink.server",
        SimpleNamespace(VehicleServer=lambda *_args, **_kwargs: server),
    )

    def link():
        blocked.set()
        release.wait(2)
        return access.link

    def write(_path, content):
        published.append(json.loads(content))
        publication.set()

    monkeypatch.setattr(runtime, "read_link", link)
    monkeypatch.setattr(runtime, "atomic_write_text", write)
    thread = threading.Thread(target=access.run, args=(stop,), daemon=True)
    try:
        thread.start()
        assert blocked.wait(1)
        assert publication.wait(1)
        before = len(published)
        publication.clear()
        assert publication.wait(1)
        assert len(published) > before
        assert published[-1]["updated_monotonic"] > published[0]["updated_monotonic"]
    finally:
        stop.set()
        release.set()
        thread.join(timeout=3)
    assert not thread.is_alive()
    assert server.closed
    assert access.operator.started and access.operator.closed


def test_status_only_service_does_not_touch_networkmanager(access, monkeypatch):
    settings = replace(access.settings, operator=OperatorSettings())
    access = runtime.VehicleAccess(access.hub, settings, manage_network=False)
    access.server = Server()
    with pytest.raises(RuntimeError, match="not enabled"):
        access.request({"network": ["connect", "eduroam"]})
    assert not access.status()["wifi_required"]


def test_network_failure_is_published_without_stopping_shared_vehicle_access(
    access, monkeypatch
):
    stop = threading.Event()
    observed = threading.Event()
    server = Server()
    monkeypatch.setitem(
        sys.modules,
        "ai_drone.mavlink.server",
        SimpleNamespace(VehicleServer=lambda *_args, **_kwargs: server),
    )

    def fail_read():
        raise RuntimeError("NetworkManager request failed or timed out")

    def publish(_path, content):
        if "timed out" in (json.loads(content)["network_error"] or ""):
            observed.set()

    monkeypatch.setattr(runtime, "read_link", fail_read)
    monkeypatch.setattr(runtime, "atomic_write_text", publish)
    thread = threading.Thread(target=access.run, args=(stop,), daemon=True)
    try:
        thread.start()
        assert observed.wait(1)
        assert thread.is_alive()
        assert not stop.is_set()
        assert not server.closed
    finally:
        stop.set()
        thread.join(timeout=3)
    assert not thread.is_alive()
    assert server.closed and access.operator.closed


def test_network_startup_failure_does_not_prevent_fc_access_from_starting(
    access, monkeypatch
):
    stop = threading.Event()

    class StopServer(Server):
        def __enter__(self):
            stop.set()
            return self

    server = StopServer()
    monkeypatch.setitem(
        sys.modules,
        "ai_drone.mavlink.server",
        SimpleNamespace(VehicleServer=lambda *_args, **_kwargs: server),
    )

    def unavailable():
        raise RuntimeError("NetworkManager not ready")

    monkeypatch.setattr(runtime, "inhibit_autoconnect", unavailable)
    access.run(stop)
    assert "not ready" in access.network_error
    assert server.closed and access.operator.closed


def test_publisher_disk_failure_propagates_to_main_thread_for_service_restart(
    access, monkeypatch
):
    stop = threading.Event()
    server = Server()
    monkeypatch.setitem(
        sys.modules,
        "ai_drone.mavlink.server",
        SimpleNamespace(VehicleServer=lambda *_args, **_kwargs: server),
    )

    def failed_write(*_args):
        raise OSError("disk full")

    monkeypatch.setattr(runtime, "atomic_write_text", failed_write)
    with pytest.raises(RuntimeError, match="status publication failed") as error:
        access.run(stop)
    assert isinstance(error.value.__cause__, OSError)
    assert stop.is_set() and server.closed and access.operator.closed


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"network": "connect"},
        {"network": ["bad", "x"]},
        {"network": ["hotspot", "automatic"]},
        {"network": ["connect", "phone"], "extra": True},
    ],
)
def test_unknown_requests_have_no_effect(access, payload):
    with pytest.raises(ValueError):
        access.request(payload)
    assert not access.mutations
    assert not access.server.is_network_busy
