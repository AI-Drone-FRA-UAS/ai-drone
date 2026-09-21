"""Parsed runtime observations and explicit health/maintenance policies.

A status record is evidence, never a lasting authorization. Consumers provide
their current clock and retain their own operation deadline.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    try:
        number = float(value)
    except OverflowError:
        return None
    return number if math.isfinite(number) else None


@dataclass(frozen=True)
class RuntimeStatus:
    status: str | None
    armed: bool | None
    fresh: bool
    source_known: bool
    system_id: int | None
    component_id: int | None
    heartbeat_age_s: float | None
    updated_monotonic: float | None
    closed: bool | None
    error: object
    network_error: object

    @classmethod
    def parse(cls, value: object) -> RuntimeStatus:
        if not isinstance(value, dict):
            raise ValueError("runtime status must be an object")

        # Invalid or absent scalar evidence becomes unknown, never false/disarmed.
        # Extra fields remain the responsibility of their own policy boundary.
        def integer(name: str) -> int | None:
            item = value.get(name)
            return item if type(item) is int else None

        def boolean(name: str) -> bool | None:
            item = value.get(name)
            return item if type(item) is bool else None

        status = value.get("status")
        return cls(
            status if isinstance(status, str) else None,
            boolean("armed"),
            boolean("fresh") is True,
            boolean("source_known") is True,
            integer("system_id"),
            integer("component_id"),
            _number(value.get("heartbeat_age_s")),
            _number(value.get("updated_monotonic")),
            boolean("closed"),
            value.get("error"),
            value.get("network_error"),
        )

    @property
    def selected_source(self) -> bool:
        return (
            self.fresh
            and self.source_known
            and (
                self.system_id,
                self.component_id,
            )
            == (1, 1)
        )

    def publication_age(self, now: float, *, max_age: float = 2) -> float | None:
        updated = self.updated_monotonic
        if updated is None or not 0 <= now - updated <= max_age:
            return None
        return now - updated

    def effective_heartbeat_age(self, now: float) -> float | None:
        elapsed = self.publication_age(now)
        age = self.heartbeat_age_s
        if elapsed is None or age is None or age < 0:
            return None
        return age + elapsed

    def healthy_after_restart(self, now: float) -> bool:
        """Read-only health permits an armed FC or newly connected clients."""
        age = self.effective_heartbeat_age(now)
        return (
            self.selected_source
            and self.closed is False
            and self.error is None
            and self.network_error is None
            and age is not None
            and age <= 2
        )

    def disarmed_at_receipt(self) -> bool:
        """Installer RPC policy: age at the observation's immediate receipt."""
        age = self.heartbeat_age_s
        return (
            self.selected_source
            and self.status == "disarmed"
            and self.armed is False
            and age is not None
            and 0 <= age <= 2
        )

    def disarmed(self, now: float) -> bool:
        age = self.effective_heartbeat_age(now)
        return self.disarmed_at_receipt() and age is not None and age <= 2
