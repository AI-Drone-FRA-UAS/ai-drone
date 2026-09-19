"""Pure evidence checks for an explicit, one-way handoff to the radio pilot."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ai_drone.validation import finite_in_range

PILOT_MODES = frozenset({"STABILIZE", "ALT_HOLD", "LOITER", "POSHOLD"})


@dataclass(frozen=True)
class OwnershipPolicy:
    heartbeat_max_age: float = 2.5
    rc_max_age: float = 1.0
    pilot_modes: frozenset[str] = PILOT_MODES

    def __post_init__(self) -> None:
        for name in ("heartbeat_max_age", "rc_max_age"):
            finite_in_range(getattr(self, name), name, minimum=0.05, maximum=10.0)
        modes = frozenset(self.pilot_modes)
        if not modes or not modes <= PILOT_MODES:
            raise ValueError("pilot_modes must select supported radio-pilot modes")
        object.__setattr__(self, "pilot_modes", modes)


def human_takeover_allowed(
    policy: OwnershipPolicy,
    *,
    requested: bool,
    armed: bool,
    mode: str | None,
    heartbeat_received: float,
    rc_received: float,
    rc_channels: int | None,
    now: float,
) -> bool:
    """Require intent, a pilot mode, and current evidence from the selected FC."""

    return (
        requested
        and armed
        and mode in policy.pilot_modes
        and rc_channels is not None
        and 0 < rc_channels <= 18
        and all(
            math.isfinite(received) and received > 0 and 0 <= now - received <= limit
            for received, limit in (
                (heartbeat_received, policy.heartbeat_max_age),
                (rc_received, policy.rc_max_age),
            )
        )
    )
