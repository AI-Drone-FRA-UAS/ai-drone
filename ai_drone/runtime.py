"""Pi access service: one FC reader, operator presence and grounded networking."""

from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path
from typing import Any

from ai_drone.durability import atomic_write_text
from ai_drone.mavlink.shared import SharedMavlink
from ai_drone.network import (
    Activation,
    NetworkAttempt,
    NetworkSnapshot,
    begin_activation,
    choose_network,
    fresh_disarmed,
    inhibit_autoconnect,
    parse_saved_profiles,
    poll_activation,
    read_link,
    read_profiles,
    run_nmcli,
)
from ai_drone.operator import OperatorMonitor
from ai_drone.settings import Settings


def read_runtime_status(path: Path, *, max_age: float = 2.5) -> dict:
    try:
        value = json.loads(path.read_text())
        age = time.monotonic() - value["updated_monotonic"]
        if not 0 <= age <= max_age:
            raise ValueError("vehicle runtime status is stale")
        return value
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise RuntimeError("fresh vehicle runtime status is unavailable") from error


def operator_has_control_link(status: dict) -> bool:
    return bool(
        status.get("operator_configured")
        and status.get("operator_alive")
        and (not status.get("wifi_required", True) or status.get("wifi_connected"))
    )


class VehicleAccess:
    """Resource owner; networking decisions remain in the pure policy module."""

    def __init__(self, hub: SharedMavlink, settings: Settings, *, manage_network: bool):
        self.hub = hub
        self.settings = settings
        self.manage_network = manage_network
        self.operator = OperatorMonitor(settings.operator)
        self.server: Any = None
        self.human_requested = False
        self.link = (None, False, False)
        self.link_observed = 0.0
        self.started = time.monotonic()
        self.profiles = ()
        self.attempts: list[NetworkAttempt] = []
        self.activation: Activation | None = None
        self.deactivation: Activation | None = None
        self.requests: queue.Queue[list[str]] = queue.Queue(maxsize=1)
        self.network_error: str | None = None
        self._publish_error: Exception | None = None
        self._eligible: tuple[str, ...] | None = None
        if manage_network:
            try:
                document = json.loads(
                    Path(settings.runtime.network_profiles).read_text()
                )
            except (OSError, json.JSONDecodeError) as error:
                raise ValueError(f"failed to read network profiles: {error}") from error
            self._eligible = tuple(
                profile.uuid
                for profile in parse_saved_profiles(document)
                if profile.autoconnect and profile.mode in {"", "infrastructure"}
            )
            if not self._eligible:
                raise ValueError(
                    "network policy requires at least one saved client profile"
                )

    def network_snapshot(self) -> NetworkSnapshot:
        status = self.hub.status()
        active = next(
            (profile for profile in self.profiles if profile.uuid == self.link[0]), None
        )
        return NetworkSnapshot(
            armed=status["armed"] if status["fresh"] else None,
            heartbeat_age_s=status["heartbeat_age_s"],
            active_uuid=self.link[0],
            active_mode=active.mode if active else None,
            connected=self.link[1],
            activating=self.link[2],
            operator_configured=self.operator.configured
            and (
                self.operator.last_seen is not None
                or time.monotonic() - self.started >= self.settings.operator.timeout
            ),
            operator_alive=self.operator.alive(),
        )

    def status(self) -> dict:
        return {
            **self.hub.status(),
            "updated_monotonic": time.monotonic(),
            "operator_configured": self.operator.configured,
            "operator_alive": self.operator.alive(),
            "operator_last_seen_monotonic": self.operator.last_seen,
            "operator_timeout_s": self.settings.operator.timeout,
            "human_requested": self.human_requested,
            "wifi_required": self.manage_network,
            "wifi_connected": self.link[1]
            and time.monotonic() - self.link_observed <= 4,
            "wifi_profile": self.link[0],
            "network_busy": bool(self.server and self.server.is_network_busy),
            "network_error": self.network_error,
            "control_client": self.server.control_owner if self.server else None,
            "clients": self.server.status()["clients"] if self.server else 0,
            "maintenance": bool(self.server and self.server.is_maintenance),
        }

    def request(self, request: dict) -> dict:
        if request == {"status": True}:
            return self.status()
        if request == {"human": True}:
            vehicle = self.hub.status()
            if (
                not self.server.control_owner
                or vehicle["armed"] is not True
                or not vehicle["fresh"]
            ):
                raise RuntimeError(
                    "pilot handoff requires an active airborne controller"
                )
            self.human_requested = True
            return {"handoff_requested": True}
        action = request.get("network")
        if (
            set(request) != {"network"}
            or not isinstance(action, list)
            or len(action) != 2
        ):
            raise ValueError("unknown vehicle runtime request")
        if action[0] not in {"connect", "hotspot"} or not isinstance(action[1], str):
            raise ValueError("invalid network command")
        if action[0] == "hotspot" and action[1] not in {"on", "off"}:
            raise ValueError("invalid hotspot state")
        if not self.manage_network:
            raise RuntimeError("network policy is not enabled")
        if not fresh_disarmed(self.network_snapshot()):
            raise RuntimeError("network switching requires fresh disarmed FC telemetry")
        self.server.set_network_busy(True)
        try:
            self.requests.put_nowait(action)
        except queue.Full as error:
            raise RuntimeError("another network switch is already queued") from error
        return {"queued": action}

    def _activate(self, profile, *, explicit_hotspot=False):
        self.server.set_network_busy(True)
        started = time.monotonic()
        self.attempts.append(NetworkAttempt(profile.uuid, started))
        self.attempts = self.attempts[-max(8, len(self.profiles) * 2) :]
        # A timed-out submit or a failed post-submit check can still leave NM
        # switching radios. Retain the lease until a fresh poll settles it.
        self.activation = Activation(profile.uuid, started)
        self.activation = begin_activation(
            profile, self.network_snapshot, explicit_hotspot=explicit_hotspot
        )

    def _release_network_if_idle(self) -> None:
        if (
            self.activation is None
            and self.deactivation is None
            and self.requests.empty()
        ):
            self.server.set_network_busy(False)

    def _manual(self, action: list[str]) -> None:
        self.server.set_network_busy(True)
        try:
            if not fresh_disarmed(self.network_snapshot()):
                raise RuntimeError(
                    "vehicle became armed or unknown before network switch"
                )
            if action == ["hotspot", "off"]:
                active = next(
                    (
                        profile
                        for profile in self.profiles
                        if profile.uuid == self.link[0]
                    ),
                    None,
                )
                if active and active.mode == "ap":
                    self.deactivation = Activation(active.uuid, time.monotonic())
                    run_nmcli(
                        ["--wait", "0", "connection", "down", "uuid", active.uuid]
                    )
                self.attempts.clear()
                return
            name = "Hotspot" if action == ["hotspot", "on"] else action[1]
            matches = [
                profile
                for profile in self.profiles
                if name in (profile.uuid, profile.name)
            ]
            if len(matches) != 1:
                raise ValueError("network profile must match one saved UUID or name")
            profile = matches[0]
            if action[0] == "connect" and profile.mode == "ap":
                raise ValueError("use hotspot on to activate the access point")
            self._activate(profile, explicit_hotspot=action[0] == "hotspot")
        finally:
            self._release_network_if_idle()

    def _poll_network_change(self, now: float) -> bool:
        """Finish an outstanding activation/disconnection before admitting new work."""
        if self.deactivation is not None:
            pending = self.deactivation
            if not self.link[2] and (
                not self.link[1] or self.link[0] != pending.profile_uuid
            ):
                self.deactivation = None
                self.network_error = None
                self._release_network_if_idle()
            elif now - pending.started_at >= pending.timeout_s:
                self.network_error = "hotspot disconnection timed out"
                if not self.link[2]:
                    self.deactivation = None
                    self._release_network_if_idle()
            return True
        if self.activation is not None:
            outcome = poll_activation(self.activation, self.network_snapshot(), now=now)
            if outcome != "pending":
                self.activation = None
                self._release_network_if_idle()
                self.network_error = (
                    None if outcome == "connected" else f"network activation {outcome}"
                )
            return True
        return False

    def network_tick(self, now: float) -> None:
        if self.server.is_maintenance:
            return
        self.link = read_link()
        self.link_observed = time.monotonic()
        self.profiles = read_profiles(eligible_uuids=self._eligible)
        if self._poll_network_change(now):
            return
        try:
            action = self.requests.get_nowait()
        except queue.Empty:
            pass
        else:
            self._manual(action)
            return
        snapshot = self.network_snapshot()
        decision = choose_network(
            snapshot, self.profiles, now=now, attempts=self.attempts
        )
        if decision.action == "activate":
            profile = next(
                profile
                for profile in self.profiles
                if profile.uuid == decision.profile_uuid
            )
            self._activate(profile)
        elif snapshot.connected and (
            not snapshot.operator_configured or snapshot.operator_alive
        ):
            self.attempts.clear()

    def _publish(self, stop: threading.Event) -> None:
        # NetworkManager queries may block; FC and operator freshness must not.
        try:
            while not stop.is_set():
                atomic_write_text(
                    Path(self.settings.runtime.status),
                    json.dumps(self.status(), allow_nan=False) + "\n",
                )
                stop.wait(0.2)
        except Exception as error:
            self._publish_error = error
            stop.set()

    def run(self, stop: threading.Event) -> None:
        from ai_drone.mavlink.server import VehicleServer

        runtime = self.settings.runtime
        self.operator.start()
        try:
            if self.manage_network:
                try:
                    inhibit_autoconnect()
                except (OSError, ValueError, RuntimeError) as error:
                    self.network_error = str(error)
            with VehicleServer(
                Path(runtime.socket),
                self.hub,
                self.request,
                network_busy=lambda: (
                    self.activation is not None
                    or self.deactivation is not None
                    or not self.requests.empty()
                ),
            ) as server:
                self.server = server
                publisher = threading.Thread(
                    target=self._publish,
                    args=(stop,),
                    name="vehicle-status",
                    daemon=True,
                )
                publisher.start()
                next_network = 0.0
                try:
                    while not stop.is_set():
                        now = time.monotonic()
                        if self.hub.status().get("error"):
                            raise RuntimeError(
                                "FC reader failed; restarting access service"
                            )
                        if self.manage_network and now >= next_network:
                            try:
                                self.network_tick(now)
                            except (OSError, ValueError, RuntimeError) as error:
                                self.network_error = str(error)
                                self._release_network_if_idle()
                            next_network = time.monotonic() + 1
                        if (
                            not server.control_owner
                            and self.hub.status()["armed"] is False
                        ):
                            self.human_requested = False
                        stop.wait(0.2)
                finally:
                    stop.set()
                    publisher.join(timeout=2)
                if self._publish_error is not None:
                    raise RuntimeError(
                        "runtime status publication failed"
                    ) from self._publish_error
        finally:
            self.operator.close()
