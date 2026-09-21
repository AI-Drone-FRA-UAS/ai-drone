"""Explicit capture operations and their shared parser defaults."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar, Literal


@dataclass(frozen=True)
class InspectOperation:
    allow_flight: bool = False
    name: ClassVar[Literal["inspect"]] = "inspect"
    actuation_enabled: ClassVar[bool] = False
    stop_after_disarm: ClassVar[bool] = False


@dataclass(frozen=True)
class TagServoOperation:
    name: ClassVar[Literal["tag-servo"]] = "tag-servo"
    allow_flight: ClassVar[bool] = True
    actuation_enabled: ClassVar[bool] = True
    stop_after_disarm: ClassVar[bool] = True


@dataclass(frozen=True)
class TagMountOperation:
    name: ClassVar[Literal["tag-mount"]] = "tag-mount"
    allow_flight: ClassVar[bool] = True
    actuation_enabled: ClassVar[bool] = True
    stop_after_disarm: ClassVar[bool] = False


RecordingOperation = InspectOperation | TagServoOperation | TagMountOperation


@dataclass(frozen=True)
class ParserSpec:
    default_duration: float | None
    backends: tuple[str, ...]
    default_backend: str
    actuation_enabled: bool


def parse_operation(name: str, *, allow_flight: bool = False) -> RecordingOperation:
    match name:
        case "inspect":
            return InspectOperation(allow_flight=allow_flight)
        case "tag-servo":
            return TagServoOperation()
        case "tag-mount":
            return TagMountOperation()
        case _:
            raise ValueError(f"unknown recording operation {name!r}")


def parser_spec(name: str) -> ParserSpec:
    operation = parse_operation(name)
    if operation.actuation_enabled:
        return ParserSpec(None, ("native",), "native", True)
    return ParserSpec(10.0, ("auto", "native", "opencv"), "auto", False)
