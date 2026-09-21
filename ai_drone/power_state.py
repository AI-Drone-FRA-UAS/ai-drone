"""Typed observations at the power-inspection JSON boundary.

No unavailable probe authorizes a power action. Receipt-time freshness remains
the caller's policy and never extends the lifetime of a heartbeat.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True)
class FcObserved:
    status: Literal["armed", "disarmed"]
    heartbeat_age_s: float | None

    @property
    def fresh_disarmed(self) -> bool:
        return (
            self.status == "disarmed"
            and self.heartbeat_age_s is not None
            and 0 <= self.heartbeat_age_s <= 2
        )


@dataclass(frozen=True)
class FcAbsent:
    reason: str


@dataclass(frozen=True)
class FcBusy:
    reason: str


@dataclass(frozen=True)
class FcUnavailable:
    reason: str


FcProbe = FcObserved | FcAbsent | FcBusy | FcUnavailable


def parse_fc_probe(value: object) -> FcProbe:
    if not isinstance(value, dict):
        return FcUnavailable("FC response is not an object")
    status = value.get("status")
    reason = str(value.get("reason", value.get("serial_owner_check", status)))
    if status == "absent":
        return FcAbsent(reason)
    if status == "busy":
        return FcBusy(reason)
    if status not in ("armed", "disarmed"):
        return FcUnavailable(reason)
    age = value.get("heartbeat_age_s")
    # Bound before conversion: arbitrarily large JSON integers may overflow float.
    if isinstance(age, int | float) and not isinstance(age, bool) and 0 <= age <= 2:
        return FcObserved(status, float(age))
    return FcObserved(status, None)


@dataclass(frozen=True)
class HardwareOwner:
    walk: bool
    runtime: bool

    @classmethod
    def parse(cls, value: object) -> HardwareOwner:
        if not isinstance(value, dict):
            raise RuntimeError("hardware-owner record is malformed")
        return cls(value.get("walk") is True, value.get("runtime") is True)


@dataclass(frozen=True)
class PiSnapshot:
    complete: bool
    package_database_ok: bool
    packages_active: bool
    hardware: tuple[HardwareOwner, ...]
    walk_active: bool
    runtime: dict | None

    @classmethod
    def parse(cls, value: object) -> PiSnapshot:
        if not isinstance(value, dict):
            raise RuntimeError("Pi snapshot is not an object")
        hardware = value.get("hardware")
        packages = value.get("packages")
        walk = value.get("walk")
        runtime = value.get("runtime")
        if (
            not isinstance(hardware, list)
            or not isinstance(packages, list)
            or not isinstance(walk, dict)
            or (runtime is not None and not isinstance(runtime, dict))
        ):
            raise RuntimeError("hardware-owner status is unknown or malformed")
        active = walk.get("ActiveState")
        if active not in (
            "active",
            "activating",
            "deactivating",
            "reloading",
            "inactive",
            "failed",
        ):
            raise RuntimeError("recorder service state is unknown or changing")
        return cls(
            value.get("complete") is True,
            value.get("package_database_ok") is True,
            bool(packages),
            tuple(HardwareOwner.parse(owner) for owner in hardware),
            active in ("active", "activating", "deactivating", "reloading"),
            runtime,
        )
