from __future__ import annotations

import subprocess
from dataclasses import replace

import pytest

from ai_drone import network

EDUROAM = "11111111-1111-4111-8111-111111111111"
PHONE = "22222222-2222-4222-8222-222222222222"
HOTSPOT = "33333333-3333-4333-8333-333333333333"
GROUND = network.NetworkSnapshot(armed=False, heartbeat_age_s=0.1)
PROFILES = (
    network.WifiProfile(EDUROAM, "eduroam", priority=100),
    network.WifiProfile(PHONE, "phone", priority=10),
    network.WifiProfile(HOTSPOT, "Hotspot", priority=1000, mode="ap"),
)


@pytest.mark.parametrize(
    "state",
    [
        network.NetworkSnapshot(),
        replace(GROUND, armed=True),
        replace(GROUND, armed=0),
        replace(GROUND, heartbeat_age_s=None),
        replace(GROUND, heartbeat_age_s=True),
        replace(GROUND, heartbeat_age_s=-0.1),
        replace(GROUND, heartbeat_age_s=2.01),
        replace(GROUND, heartbeat_age_s=float("nan")),
        replace(GROUND, heartbeat_age_s=float("inf")),
    ],
)
def test_no_automatic_switch_with_unsafe_or_unknown_fc(state):
    assert network.choose_network(state, PROFILES, now=100).action == "blocked"


def test_working_client_does_not_chase_a_preferred_network():
    state = replace(
        GROUND, active_uuid=PHONE, active_mode="infrastructure", connected=True
    )
    assert network.choose_network(state, PROFILES, now=100).action == "keep"
    state = replace(state, operator_configured=True, operator_alive=True)
    assert network.choose_network(state, PROFILES, now=100).action == "keep"


def test_operator_loss_moves_to_another_profile_only_when_explicitly_monitored():
    state = replace(GROUND, active_uuid=EDUROAM, connected=True)
    assert network.choose_network(state, PROFILES, now=100).action == "keep"
    state = replace(state, operator_configured=True, operator_alive=False)
    decision = network.choose_network(state, PROFILES, now=100)
    assert decision.profile_uuid == PHONE
    assert decision.reason == "operator heartbeat lost"


def test_priority_orders_the_first_attempt_and_failed_candidates_do_not_starve_fallback():
    first = network.choose_network(GROUND, PROFILES, now=100)
    assert first.profile_uuid == EDUROAM
    attempts = [network.NetworkAttempt(EDUROAM, 100)]
    assert (
        network.choose_network(GROUND, PROFILES, now=114, attempts=attempts).action
        == "keep"
    )
    assert (
        network.choose_network(
            GROUND, PROFILES, now=115, attempts=attempts
        ).profile_uuid
        == PHONE
    )
    attempts.append(network.NetworkAttempt(PHONE, 115))
    assert (
        network.choose_network(
            GROUND, PROFILES, now=130, attempts=attempts
        ).profile_uuid
        == EDUROAM
    )


@pytest.mark.parametrize("armed", [None, False, True])
def test_explicit_hotspot_survives_operator_loss_until_an_explicit_stop(armed):
    state = replace(
        GROUND,
        armed=armed,
        active_uuid=HOTSPOT,
        active_mode="ap",
        connected=True,
        operator_configured=True,
    )
    assert network.choose_network(state, PROFILES, now=100).action == "keep"


def test_only_reachable_enabled_infrastructure_profiles_are_candidates():
    profiles = (
        replace(PROFILES[0], reachable=False),
        replace(PROFILES[1], autoconnect=False),
        PROFILES[2],
        network.WifiProfile("unused", "mesh", mode="mesh"),
    )
    assert network.choose_network(GROUND, profiles, now=100).action == "keep"


def test_pending_activation_is_never_replaced():
    state = replace(GROUND, activating=True)
    assert network.choose_network(state, PROFILES, now=100).action == "keep"


def test_current_profile_is_not_restarted_for_operator_loss_without_an_alternative():
    state = replace(
        GROUND, connected=True, active_uuid=EDUROAM, operator_configured=True
    )
    assert network.choose_network(state, PROFILES[:1], now=100).action == "keep"


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf")])
def test_policy_rejects_invalid_timeouts(value):
    with pytest.raises(ValueError):
        network.choose_network(GROUND, PROFILES, now=100, cooldown_s=value)


def test_policy_rejects_foreign_or_invalid_attempt_clock():
    for timestamp in (float("nan"), 101):
        with pytest.raises(ValueError):
            network.choose_network(
                GROUND,
                PROFILES,
                now=100,
                attempts=[network.NetworkAttempt(EDUROAM, timestamp)],
            )


def profile_runner(calls, *, autoconnect="yes"):
    def run(arguments):
        calls.append(arguments)
        if arguments == ["-g", "UUID", "connection", "show"]:
            return f"{EDUROAM}\n{PHONE}\n{HOTSPOT}"
        if "CONNECTIONS.AVAILABLE-CONNECTIONS" in arguments:
            return f"{EDUROAM} | eduroam\n{HOTSPOT} | Hotspot"
        identifier = arguments[-1]
        name = "wifi: with\\slash; $(false)" if identifier == PHONE else "eduroam"
        return "\n".join(
            [
                f"connection.id:{name}",
                "connection.type:802-11-wireless",
                f"connection.autoconnect:{autoconnect}",
                "connection.autoconnect-priority:100",
                f"802-11-wireless.mode:{'ap' if identifier == HOTSPOT else 'infrastructure'}",
            ]
        )

    return run


def test_profile_adapter_uses_saved_uuids_and_preserves_unusual_names():
    calls = []
    profiles = network.read_profiles(run=profile_runner(calls))
    assert [profile.uuid for profile in profiles] == [EDUROAM, PHONE, HOTSPOT]
    assert profiles[1].name == "wifi: with\\slash; $(false)"
    assert [profile.reachable for profile in profiles] == [True, False, True]
    assert all(call[-2] == "uuid" for call in calls[2:])
    assert all("$(false)" not in argument for call in calls for argument in call)


def test_manifest_eligibility_replaces_disabled_client_autoconnect_but_not_ap_state():
    profiles = network.read_profiles(
        eligible_uuids={EDUROAM, HOTSPOT}, run=profile_runner([], autoconnect="no")
    )
    assert [profile.autoconnect for profile in profiles] == [True, False, False]
    assert network.choose_network(GROUND, profiles, now=100).profile_uuid == EDUROAM


@pytest.mark.parametrize(
    "identifier", ["--ask", "bad; reboot", "11111111111141118111111111111111"]
)
def test_noncanonical_profile_identifiers_never_reach_activation(identifier):
    calls = []
    with pytest.raises(ValueError):
        network.begin_activation(
            replace(PROFILES[0], uuid=identifier),
            lambda: GROUND,
            run=lambda command: calls.append(command) or "",
        )
    assert not calls


@pytest.mark.parametrize(
    "state,connected,activating",
    [
        (30, False, False),
        (50, False, True),
        (100, True, False),
        (110, False, True),
        (120, False, False),
    ],
)
def test_link_adapter_uses_numeric_state_and_uuid(state, connected, activating):
    result = network.read_link(
        run=lambda _arguments: (
            f"GENERAL.STATE:{state} (state text)\nGENERAL.CON-UUID:{EDUROAM}"
        )
    )
    assert result == (EDUROAM, connected, activating)


def test_disconnected_link_without_profile_has_no_identifier():
    assert network.read_link(
        run=lambda _arguments: "GENERAL.STATE:30 (disconnected)\nGENERAL.CON-UUID:--"
    ) == (None, False, False)


@pytest.mark.parametrize("state", [0, 101, 130])
def test_unknown_nm_state_is_not_treated_as_a_finished_transition(state):
    with pytest.raises(RuntimeError, match="state is unknown"):
        network.read_link(
            run=lambda _arguments: f"GENERAL.STATE:{state}\nGENERAL.CON-UUID:--"
        )


def test_activation_checks_fresh_state_after_inhibiting_and_reasserts_after_submit():
    calls = []

    def snapshot():
        calls.append("FC checked")
        return GROUND

    activation = network.begin_activation(
        PROFILES[0],
        snapshot,
        run=lambda command: calls.append(command) or "",
        monotonic=lambda: 100,
    )
    assert activation == network.Activation(EDUROAM, 100)
    assert calls == [
        ["device", "set", "wlan0", "autoconnect", "no"],
        "FC checked",
        ["--wait", "0", "connection", "up", "uuid", EDUROAM, "ifname", "wlan0"],
        ["device", "set", "wlan0", "autoconnect", "no"],
    ]


def test_failed_activation_keeps_autoconnect_inhibited():
    calls = []

    def run(command):
        calls.append(command)
        if "up" in command:
            raise RuntimeError("activation failed")
        return ""

    with pytest.raises(RuntimeError, match="activation failed"):
        network.begin_activation(PROFILES[0], lambda: GROUND, run=run)
    assert calls[-1] == ["device", "set", "wlan0", "autoconnect", "no"]


@pytest.mark.parametrize(
    "state",
    [
        replace(GROUND, armed=True),
        network.NetworkSnapshot(),
        replace(GROUND, activating=True),
    ],
)
def test_activation_never_starts_with_an_unsafe_fc_or_busy_interface(state):
    calls = []
    with pytest.raises(RuntimeError):
        network.begin_activation(
            PROFILES[0], lambda: state, run=lambda command: calls.append(command) or ""
        )
    assert not any("up" in command for command in calls)


@pytest.mark.parametrize(
    "explicit,autoconnect", [(False, False), (False, True), (True, True)]
)
def test_hotspot_requires_explicit_request_and_disabled_autoconnect(
    explicit, autoconnect
):
    with pytest.raises(ValueError):
        network.begin_activation(
            replace(PROFILES[2], autoconnect=autoconnect),
            lambda: GROUND,
            explicit_hotspot=explicit,
            run=lambda _: pytest.fail("network command executed"),
        )


def test_explicit_safe_hotspot_uses_the_same_fresh_fc_gate():
    calls = []
    activation = network.begin_activation(
        replace(PROFILES[2], autoconnect=False),
        lambda: GROUND,
        explicit_hotspot=True,
        run=lambda command: calls.append(command) or "",
    )
    assert activation.profile_uuid == HOTSPOT
    assert calls[1][-3:] == [HOTSPOT, "ifname", "wlan0"]


@pytest.mark.parametrize(
    "unsafe", [replace(GROUND, armed=True), network.NetworkSnapshot()]
)
def test_poll_cancels_only_our_unfinished_activation_if_fc_becomes_unsafe(unsafe):
    calls = []
    state = replace(unsafe, active_uuid=EDUROAM, activating=True)
    result = network.poll_activation(
        network.Activation(EDUROAM, 100),
        state,
        now=101,
        run=lambda command: calls.append(command) or "",
    )
    assert result == "pending"
    assert calls[-1] == ["--wait", "0", "connection", "down", "uuid", EDUROAM]


def test_poll_never_disconnects_an_established_link_even_after_arming():
    calls = []
    state = replace(GROUND, armed=True, active_uuid=EDUROAM, connected=True)
    assert (
        network.poll_activation(
            network.Activation(EDUROAM, 100),
            state,
            now=101,
            run=lambda command: calls.append(command) or "",
        )
        == "connected"
    )
    assert not any("down" in command for command in calls)


def test_poll_cancels_timed_out_activation_without_restoring_another_profile():
    calls = []
    state = replace(GROUND, active_uuid=EDUROAM, activating=True)
    assert (
        network.poll_activation(
            network.Activation(EDUROAM, 100, timeout_s=10),
            state,
            now=110,
            run=lambda command: calls.append(command) or "",
        )
        == "pending"
    )
    assert not any("up" in command for command in calls)


def test_poll_reports_pending_and_failed_without_disrupting_another_connection():
    activation = network.Activation(EDUROAM, 100)
    calls = []
    run = lambda command: calls.append(command) or ""  # noqa: E731
    state = replace(GROUND, active_uuid=EDUROAM, activating=True)
    assert network.poll_activation(activation, state, now=101, run=run) == "pending"
    assert network.poll_activation(activation, GROUND, now=102, run=run) == "failed"
    other = replace(GROUND, armed=True, active_uuid=PHONE, activating=True)
    assert network.poll_activation(activation, other, now=103, run=run) == "pending"
    assert not any("down" in command for command in calls)
    terminal = replace(other, active_uuid=None, activating=False)
    assert (
        network.poll_activation(activation, terminal, now=104, run=run) == "cancelled"
    )


def test_nmcli_adapter_elevates_mutations_only_and_bounds_commands(monkeypatch):
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, "ok\n", "")

    monkeypatch.setattr(network.subprocess, "run", run)
    monkeypatch.setattr(network.os, "geteuid", lambda: 1000)
    assert network.run_nmcli(["-g", "UUID", "connection", "show"]) == "ok"
    network.inhibit_autoconnect()
    assert calls[0][0][0] == "nmcli"
    assert calls[1][0][:3] == ["sudo", "-n", "nmcli"]
    assert all(options["timeout"] == 5 for _, options in calls)
    assert all(options["env"]["LC_ALL"] == "C" for _, options in calls)


def test_nmcli_errors_do_not_expose_profile_output(monkeypatch):
    monkeypatch.setattr(
        network.subprocess,
        "run",
        lambda command, **_: subprocess.CompletedProcess(
            command, 10, "sensitive", "private"
        ),
    )
    with pytest.raises(RuntimeError, match="exit 10") as error:
        network.run_nmcli(["-g", "UUID", "connection", "show"])
    assert "sensitive" not in str(error.value)
    assert "private" not in str(error.value)


@pytest.mark.parametrize(
    "failure",
    [
        subprocess.TimeoutExpired("nmcli", 5),
        subprocess.SubprocessError("private detail"),
    ],
)
def test_nmcli_subprocess_failures_are_runtime_errors_that_do_not_stop_fc_access(
    monkeypatch, failure
):
    def fail(*_args, **_kwargs):
        raise failure

    monkeypatch.setattr(network.subprocess, "run", fail)
    with pytest.raises(RuntimeError, match="failed or timed out") as error:
        network.run_nmcli(["-g", "UUID", "connection", "show"])
    assert "private detail" not in str(error.value)
