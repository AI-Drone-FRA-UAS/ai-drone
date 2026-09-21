"""Grounded Wi-Fi selection and bounded NetworkManager operations.

The caller supplies selected-flight-controller and operator status. This module
never opens a serial port, changes laptop networking, or starts an AP implicitly.
"""

from __future__ import annotations

import math
import os
import subprocess
import time
from collections.abc import Callable, Collection, Sequence
from dataclasses import dataclass
from typing import Literal, NamedTuple
from uuid import UUID


@dataclass(frozen=True)
class WifiProfile:
    uuid: str
    name: str
    priority: int = 0
    autoconnect: bool = True
    mode: str = "infrastructure"
    reachable: bool = True


@dataclass(frozen=True)
class NetworkSnapshot:
    armed: bool | None = None
    heartbeat_age_s: float | None = None
    active_uuid: str | None = None
    active_mode: str | None = None
    connected: bool = False
    operator_configured: bool = False
    operator_alive: bool = False
    activating: bool = False


@dataclass(frozen=True)
class NetworkAttempt:
    profile_uuid: str
    started_at: float


@dataclass(frozen=True)
class NetworkDecision:
    action: Literal["keep", "blocked", "activate"]
    reason: str
    profile_uuid: str | None = None


@dataclass(frozen=True)
class Activation:
    profile_uuid: str
    started_at: float
    timeout_s: float = 45


class LinkState(NamedTuple):
    profile_uuid: str | None = None
    connected: bool = False
    transitioning: bool = False


@dataclass(frozen=True)
class Idle:
    pass


@dataclass(frozen=True)
class Activating:
    operation: Activation


@dataclass(frozen=True)
class Deactivating:
    operation: Activation


NetworkJob = Idle | Activating | Deactivating


@dataclass(frozen=True)
class NetworkWork:
    """Outstanding effects, queued intent and maintenance are independent facts.

    An uncertain submit remains an outstanding job until NetworkManager supplies
    a terminal observation. Maintenance blocks new work but cannot erase a job.
    The server separately serializes maintenance and control acquisition with
    its lease; this value never replaces that atomic owner.
    """

    job: NetworkJob
    request_pending: bool
    maintenance: bool

    @property
    def effects_pending(self) -> bool:
        return not isinstance(self.job, Idle) or self.request_pending

    @property
    def blocked(self) -> bool:
        return self.maintenance


@dataclass(frozen=True)
class SavedProfile:
    uuid: str
    autoconnect: bool
    mode: str


def parse_saved_profiles(document: object) -> tuple[SavedProfile, ...]:
    """Parse the persisted eligibility policy before permitting network effects."""
    if (
        not isinstance(document, dict)
        or type(document.get("schema")) is not int
        or document["schema"] != 1
        or not isinstance(document.get("profiles"), dict)
    ):
        raise ValueError("network profiles schema must be 1 with a profiles object")
    profiles = []
    for identifier, value in document["profiles"].items():
        if not isinstance(identifier, str):
            raise ValueError("network profile UUID must be a string")
        identifier = _uuid(identifier)
        if (
            not isinstance(value, dict)
            or type(value.get("autoconnect")) is not bool
            or not isinstance(value.get("mode"), str)
            or value["mode"] not in {"", "infrastructure", "ap", "adhoc", "mesh"}
        ):
            raise ValueError(f"invalid saved network profile {identifier}")
        profiles.append(SavedProfile(identifier, value["autoconnect"], value["mode"]))
    return tuple(profiles)


Runner = Callable[[list[str]], str]


def fresh_disarmed(snapshot: NetworkSnapshot, timeout_s: float = 2) -> bool:
    age = snapshot.heartbeat_age_s
    return (
        snapshot.armed is False
        and age is not None
        and type(age) in (int, float)
        and math.isfinite(age)
        and 0 <= age <= timeout_s
    )


def choose_network(
    snapshot: NetworkSnapshot,
    profiles: Sequence[WifiProfile],
    *,
    now: float,
    attempts: Sequence[NetworkAttempt] = (),
    cooldown_s: float = 15,
    heartbeat_timeout_s: float = 2,
) -> NetworkDecision:
    """Preserve a working route; rotate eligible clients after actual loss.

    Keep attempts for the current outage and clear them when the operator route
    recovers. Least recently attempted profiles win before configured priority,
    so an unavailable preferred network cannot starve the saved fallbacks.
    """
    if not all(
        math.isfinite(value) for value in (now, cooldown_s, heartbeat_timeout_s)
    ):
        raise ValueError("network timing must be finite")
    if cooldown_s <= 0 or heartbeat_timeout_s <= 0:
        raise ValueError("network timeouts must be positive")
    if snapshot.active_mode == "ap":
        return NetworkDecision("keep", "explicit hotspot remains active")
    if not fresh_disarmed(snapshot, heartbeat_timeout_s):
        return NetworkDecision("blocked", "fresh disarmed FC state is required")
    if snapshot.activating:
        return NetworkDecision("keep", "network activation is already pending")
    if snapshot.connected and (
        not snapshot.operator_configured or snapshot.operator_alive
    ):
        return NetworkDecision("keep", "current operator route is working")
    if any(
        not math.isfinite(attempt.started_at) or attempt.started_at > now
        for attempt in attempts
    ):
        raise ValueError(
            "network attempt timestamps must be finite and not in the future"
        )
    if attempts and now - max(attempt.started_at for attempt in attempts) < cooldown_s:
        return NetworkDecision("keep", "waiting before the next network attempt")
    candidates = [
        profile
        for profile in profiles
        if profile.autoconnect
        and profile.reachable
        and profile.mode in {"", "infrastructure"}
        and (not snapshot.connected or profile.uuid != snapshot.active_uuid)
    ]
    if not candidates:
        return NetworkDecision("keep", "no reachable saved client alternative")
    last_attempt = {
        profile.uuid: max(
            (
                attempt.started_at
                for attempt in attempts
                if attempt.profile_uuid == profile.uuid
            ),
            default=-math.inf,
        )
        for profile in candidates
    }
    selected = min(
        candidates,
        key=lambda profile: (
            last_attempt[profile.uuid],
            -profile.priority,
            profile.name.casefold() != "eduroam",
            profile.uuid,
        ),
    )
    return NetworkDecision(
        "activate",
        "operator heartbeat lost" if snapshot.connected else "Wi-Fi link lost",
        selected.uuid,
    )


def run_nmcli(arguments: list[str]) -> str:
    """Use a bounded subprocess without a shell or secret-bearing output."""
    command = ["nmcli", *arguments]
    mutation = arguments[:2] in (["device", "set"], ["--wait", "0"])
    if mutation and os.geteuid() != 0:
        command = ["sudo", "-n", *command]
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
            env={**os.environ, "LC_ALL": "C"},
        )
    except subprocess.SubprocessError as error:
        raise RuntimeError("NetworkManager request failed or timed out") from error
    if result.returncode:
        raise RuntimeError(f"NetworkManager request failed (exit {result.returncode})")
    return result.stdout.strip()


def _uuid(value: str) -> str:
    try:
        parsed = str(UUID(value))
    except ValueError:
        raise ValueError("a saved NetworkManager profile UUID is required") from None
    if parsed != value.lower():
        raise ValueError("a canonical NetworkManager profile UUID is required")
    return parsed


def _interface(value: str) -> str:
    if (
        not value
        or len(value) > 15
        or any(
            character
            not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-"
            for character in value
        )
    ):
        raise ValueError("invalid Wi-Fi interface")
    return value


def _properties(output: str) -> dict[str, str]:
    """nmcli multiline keys split once, retaining colons/backslashes in names."""
    return dict(line.split(":", 1) for line in output.splitlines() if ":" in line)


def read_profiles(
    interface: str = "wlan0",
    *,
    eligible_uuids: Collection[str] | None = None,
    run: Runner = run_nmcli,
) -> tuple[WifiProfile, ...]:
    interface = _interface(interface)
    eligible = (
        {_uuid(identifier) for identifier in eligible_uuids}
        if eligible_uuids is not None
        else None
    )
    available = run(
        [
            "--escape",
            "no",
            "-g",
            "CONNECTIONS.AVAILABLE-CONNECTIONS",
            "device",
            "show",
            interface,
        ]
    )
    reachable = {
        line.partition(" | ")[0].strip()
        for line in available.splitlines()
        if " | " in line
    }
    profiles = []
    identifiers = run(["-g", "UUID", "connection", "show"]).splitlines()
    fields = (
        "connection.id,connection.type,connection.autoconnect,"
        "connection.autoconnect-priority,802-11-wireless.mode"
    )
    for identifier in identifiers:
        identifier = _uuid(identifier.strip())
        values = _properties(
            run(
                [
                    "--escape",
                    "no",
                    "-t",
                    "-f",
                    fields,
                    "connection",
                    "show",
                    "uuid",
                    identifier,
                ]
            )
        )
        if values.get("connection.type") != "802-11-wireless":
            continue
        mode = values.get("802-11-wireless.mode", "")
        autoconnect = values["connection.autoconnect"] == "yes"
        if eligible is not None and mode in {"", "infrastructure"}:
            autoconnect = identifier in eligible
        profiles.append(
            WifiProfile(
                identifier,
                values["connection.id"],
                int(values["connection.autoconnect-priority"]),
                autoconnect,
                mode,
                identifier in reachable,
            )
        )
    return tuple(profiles)


def read_link(interface: str = "wlan0", *, run: Runner = run_nmcli) -> LinkState:
    """Return profile UUID, connected, and activation-in-progress."""
    values = _properties(
        run(
            [
                "--escape",
                "no",
                "-t",
                "-f",
                "GENERAL.STATE,GENERAL.CON-UUID",
                "device",
                "show",
                _interface(interface),
            ]
        )
    )
    state = int(values["GENERAL.STATE"].split(" ", 1)[0])
    if state not in range(10, 121, 10):
        raise RuntimeError("NetworkManager Wi-Fi state is unknown")
    identifier = values.get("GENERAL.CON-UUID", "")
    return LinkState(
        _uuid(identifier) if identifier not in {"", "--"} else None,
        state == 100,
        40 <= state < 100 or state == 110,
    )


def inhibit_autoconnect(interface: str = "wlan0", *, run: Runner = run_nmcli) -> None:
    """Preserve the active link while preventing independent profile activation."""
    run(["device", "set", _interface(interface), "autoconnect", "no"])


def begin_activation(
    profile: WifiProfile,
    snapshot: Callable[[], NetworkSnapshot],
    *,
    interface: str = "wlan0",
    explicit_hotspot: bool = False,
    run: Runner = run_nmcli,
    monotonic: Callable[[], float] = time.monotonic,
    timeout_s: float = 45,
) -> Activation:
    """Submit one explicit activation; the caller polls without blocking FC reads."""
    identifier, interface = _uuid(profile.uuid), _interface(interface)
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("network activation timeout must be positive and finite")
    if profile.mode == "ap":
        if not explicit_hotspot or profile.autoconnect:
            raise ValueError(
                "hotspot requires an explicit command and autoconnect disabled"
            )
    elif profile.mode not in {"", "infrastructure"}:
        raise ValueError("only saved infrastructure Wi-Fi profiles are supported")
    elif explicit_hotspot:
        raise ValueError("explicit hotspot command requires a saved AP profile")
    inhibit_autoconnect(interface, run=run)
    state = snapshot()
    if not fresh_disarmed(state) or state.activating:
        raise RuntimeError(
            "network switching requires fresh disarmed FC state and an idle interface"
        )
    started = monotonic()
    try:
        run(
            ["--wait", "0", "connection", "up", "uuid", identifier, "ifname", interface]
        )
    finally:
        # Explicit activation can unblock device autoconnect in NetworkManager.
        inhibit_autoconnect(interface, run=run)
    return Activation(identifier, started, timeout_s)


def poll_activation(
    activation: Activation,
    snapshot: NetworkSnapshot,
    *,
    now: float,
    interface: str = "wlan0",
    run: Runner = run_nmcli,
) -> Literal["pending", "connected", "failed", "cancelled"]:
    """Cancel only our unfinished activation on unsafe FC state or timeout.

    Never tear down an already established link. Cancellation cannot undo a
    completed radio transition or make independent RC arming atomic with it.
    """
    identifier = _uuid(activation.profile_uuid)
    inhibit_autoconnect(interface, run=run)
    if snapshot.active_uuid == identifier and snapshot.connected:
        return "connected"
    expired = now - activation.started_at >= activation.timeout_s
    if not fresh_disarmed(snapshot) or expired:
        if snapshot.activating:
            if snapshot.active_uuid == identifier:
                run(["--wait", "0", "connection", "down", "uuid", identifier])
            return "pending"
        return "cancelled"
    if not snapshot.activating and snapshot.active_uuid != identifier:
        return "failed"
    return "pending"
