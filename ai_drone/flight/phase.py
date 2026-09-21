"""Pure mission authority and cleanup state, independent of observed arming."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, assert_never


@dataclass(frozen=True)
class Unclaimed:
    pass


@dataclass(frozen=True)
class ArmPending:
    """An arm write was attempted; disarmed telemetry cannot erase its ambiguity."""


@dataclass(frozen=True)
class Armed:
    pass


Stage = Literal["taking_off", "holding_altitude", "awaiting_loiter", "loitering"]


@dataclass(frozen=True)
class Flight:
    stage: Stage
    ground_reference: float | None
    floor_target_m: float | None = None
    target_reached_at: float | None = None


@dataclass(frozen=True)
class Landing:
    # Keep the original obligation when LAND is requested again or its send fails.
    previous: Unclaimed | ArmPending | Armed | Flight


@dataclass(frozen=True)
class Landed:
    landing_commanded: bool = True


@dataclass(frozen=True)
class Human:
    flight_started: bool


Phase = Unclaimed | ArmPending | Armed | Flight | Landing | Landed | Human


def owns_control(phase: Phase) -> bool:
    match phase:
        case ArmPending() | Armed() | Flight():
            return True
        case Landing(previous=previous):
            return owns_control(previous)
        case Unclaimed() | Landed() | Human():
            return False
        case _ as impossible:
            assert_never(impossible)


def cleanup(phase: Phase) -> Literal["land", "disarm", "supervise"] | None:
    match phase:
        case Flight():
            return "land"
        case ArmPending() | Armed():
            return "disarm"
        case Landing(previous=previous):
            return cleanup(previous)
        case Human():
            return "supervise"
        case Unclaimed() | Landed():
            return None
        case _ as impossible:
            assert_never(impossible)


def observed_disarm(phase: Phase) -> Phase:
    """Telemetry may release confirmed ground arming, never ambiguous cleanup."""
    return Landed(landing_commanded=False) if isinstance(phase, Armed) else phase


def request_landing(phase: Phase) -> Phase:
    match phase:
        case Landed(landing_commanded=False):
            return Landed()
        case Human() | Landing() | Landed():
            return phase
        case Unclaimed() | ArmPending() | Armed() | Flight():
            return Landing(phase)
        case _ as impossible:
            assert_never(impossible)


def ground_reference(phase: Phase) -> float | None:
    match phase:
        case Flight(ground_reference=ground):
            return ground
        case Landing(previous=previous):
            return ground_reference(previous)
        case Unclaimed() | ArmPending() | Armed() | Landed() | Human():
            return None
        case _ as impossible:
            assert_never(impossible)


@dataclass(frozen=True)
class SetMode:
    name: str


@dataclass(frozen=True)
class Arm:
    pass


@dataclass(frozen=True)
class Disarm:
    pass


@dataclass(frozen=True)
class Climb:
    fraction: float
    yaw: float


@dataclass(frozen=True)
class CommandLong:
    command: int
    parameters: tuple[float, float, float, float, float, float, float]


Command = SetMode | Arm | Disarm | Climb | CommandLong


@dataclass(frozen=True)
class CommandAttempt:
    sequence: int
    command: Command
    attempted_at: float
    outcome: Literal["attempting", "written", "queued", "failed"]
    error: str | None = None


def command_fields(command: Command) -> dict[str, str | float | int | list[float]]:
    match command:
        case Arm():
            return {"kind": "arm"}
        case Disarm():
            return {"kind": "disarm"}
        case SetMode(name=name):
            return {"kind": "set_mode", "mode": name}
        case Climb(fraction=fraction, yaw=yaw):
            return {"kind": "climb", "fraction": fraction, "yaw_rad": yaw}
        case CommandLong(command=identifier, parameters=parameters):
            return {
                "kind": "command_long",
                "command": identifier,
                "parameters": list(parameters),
            }
        case _ as impossible:
            assert_never(impossible)


def attempt_fields(attempt: CommandAttempt) -> dict[str, object]:
    return {
        "sequence": attempt.sequence,
        "attempted_monotonic": attempt.attempted_at,
        "outcome": attempt.outcome,
        "error": attempt.error,
        **command_fields(attempt.command),
    }
