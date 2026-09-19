"""Complete, read-only ArduPilot parameter snapshot helpers."""

from __future__ import annotations

import hashlib
import math
import time
from dataclasses import asdict, dataclass
from typing import Any

from ai_drone.mavlink.parameters import (
    PARAMETER_NAME_PATTERN,
    decode_parameter_name,
)
from ai_drone.mavlink.safety import heartbeat_is_armed, is_vehicle_message

MIN_MAV_PARAM_TYPE = 1
MAX_MAV_PARAM_TYPE = 10
MAX_PARAMETER_COUNT = 65_535


@dataclass(frozen=True)
class ParameterRecord:
    """One parameter returned by the MAVLink parameter protocol."""

    name: str
    value: float
    param_type: int
    index: int
    count: int


def format_parameter_value(value: float) -> str:
    """Render an ArduPilot parameter value without needless decimal noise."""

    number = float(value)
    if not math.isfinite(number):
        raise ValueError("ArduPilot parameter values must be finite")
    if number.is_integer() and abs(number) < 2**53:
        return str(int(number))
    return format(number, ".9g")


def render_parameter_file(records: list[ParameterRecord]) -> str:
    """Return a deterministic ArduPilot-compatible comma-separated file."""

    ordered = sorted(records, key=lambda record: record.name)
    for record in ordered:
        if PARAMETER_NAME_PATTERN.fullmatch(record.name) is None:
            raise ValueError(f"Invalid ArduPilot parameter name {record.name!r}")
    return "".join(
        f"{record.name},{format_parameter_value(record.value)}\n" for record in ordered
    )


def parameter_sha256(records: list[ParameterRecord]) -> str:
    """Hash the deterministic parameter representation."""

    return hashlib.sha256(render_parameter_file(records).encode()).hexdigest()


def records_to_json(
    records: list[ParameterRecord],
) -> list[dict[str, int | float | str]]:
    """Convert records into a JSON-safe list."""

    return [asdict(record) for record in sorted(records, key=lambda item: item.index)]


def _json_integer(item: dict[str, Any], field: str, position: int) -> int:
    value = item.get(field)
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"Snapshot parameter {position} {field} must be an integer")
    return value


def _record_from_json(item: dict[str, Any], position: int) -> ParameterRecord:
    name = item.get("name")
    if not isinstance(name, str) or PARAMETER_NAME_PATTERN.fullmatch(name) is None:
        raise ValueError(f"Snapshot parameter {position} has an invalid ArduPilot name")
    raw_value = item.get("value")
    if isinstance(raw_value, bool) or not isinstance(raw_value, int | float):
        raise ValueError(f"Snapshot parameter {position} value must be a JSON number")
    value = float(raw_value)
    if not math.isfinite(value):
        raise ValueError("Snapshot contains a non-finite parameter value")
    param_type = _json_integer(item, "param_type", position)
    index = _json_integer(item, "index", position)
    count = _json_integer(item, "count", position)
    if not MIN_MAV_PARAM_TYPE <= param_type <= MAX_MAV_PARAM_TYPE:
        raise ValueError(
            f"Snapshot parameter {position} param_type must be between "
            f"{MIN_MAV_PARAM_TYPE} and {MAX_MAV_PARAM_TYPE}"
        )
    if index < 0:
        raise ValueError(f"Snapshot parameter {position} index must not be negative")
    if not 1 <= count <= MAX_PARAMETER_COUNT:
        raise ValueError(
            f"Snapshot parameter {position} count must be between 1 and "
            f"{MAX_PARAMETER_COUNT}"
        )
    return ParameterRecord(
        name=name, value=value, param_type=param_type, index=index, count=count
    )


def records_from_json(items: list[dict[str, Any]]) -> list[ParameterRecord]:
    """Validate and reconstruct records from a remote export bundle."""
    records = [_record_from_json(item, position) for position, item in enumerate(items)]

    expected = len(records)
    if not records:
        raise ValueError("Snapshot contains no parameters")
    if len({record.name for record in records}) != expected:
        raise ValueError("Snapshot contains duplicate parameter names")
    if len({record.index for record in records}) != expected:
        raise ValueError("Snapshot contains duplicate parameter indexes")
    indexes = {record.index for record in records}
    expected_indexes = set(range(expected))
    if indexes != expected_indexes:
        raise ValueError(
            "Snapshot parameter indexes must be contiguous from 0 to "
            f"{expected - 1}; received {sorted(indexes)}"
        )
    announced = {record.count for record in records}
    if announced != {expected}:
        raise ValueError(
            f"Snapshot is incomplete: received {expected}, announced {sorted(announced)}"
        )
    return records


def _completed_parameters(
    by_index: dict[int, ParameterRecord],
) -> list[ParameterRecord]:
    records = list(by_index.values())
    if len({record.name for record in records}) != len(records):
        raise RuntimeError("Flight controller returned duplicate parameter names")
    return sorted(records, key=lambda record: record.name)


def _request_missing_parameters(
    connection: Any,
    expected: int,
    by_index: dict[int, ParameterRecord],
    system: int,
    component: int,
    batch: int,
) -> None:
    missing = [index for index in range(expected) if index not in by_index]
    for index in missing[:batch]:
        connection.mav.param_request_read_send(system, component, b"", index)


def download_all_parameters(
    connection: Any,
    *,
    timeout: float = 180.0,
    retry_after: float = 2.0,
    retry_batch: int = 48,
) -> list[ParameterRecord]:
    """Download a complete indexed PARAM_VALUE set, retrying missing indexes.

    The caller must wait for the initial heartbeat and verify that the vehicle is
    disarmed. This function also aborts if an armed heartbeat appears while the
    download is in progress.
    """

    if timeout <= 0 or retry_after <= 0 or retry_batch <= 0:
        raise ValueError("timeout, retry_after, and retry_batch must be positive")

    target_system = int(connection.target_system)
    target_component = int(connection.target_component)
    connection.mav.param_request_list_send(target_system, target_component)

    deadline = time.monotonic() + timeout
    last_parameter = time.monotonic()
    expected: int | None = None
    by_index: dict[int, ParameterRecord] = {}

    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["PARAM_VALUE", "HEARTBEAT"], blocking=True, timeout=0.5
        )
        now = time.monotonic()

        if message is not None and is_vehicle_message(
            message,
            system_id=target_system,
            component_id=target_component,
        ):
            message_type = message.get_type()
            if message_type == "HEARTBEAT" and heartbeat_is_armed(message):
                raise RuntimeError("Vehicle became ARMED; parameter download aborted.")
            elif message_type == "PARAM_VALUE":
                count = int(message.param_count)
                index = int(message.param_index)
                if count <= 0 or index < 0 or index >= count:
                    continue
                expected = count if expected is None else max(expected, count)
                by_index[index] = ParameterRecord(
                    name=decode_parameter_name(message.param_id),
                    value=float(message.param_value),
                    param_type=int(message.param_type),
                    index=index,
                    count=count,
                )
                last_parameter = now

        if expected is not None and len(by_index) == expected:
            return _completed_parameters(by_index)

        if expected is not None and now - last_parameter >= retry_after:
            _request_missing_parameters(
                connection,
                expected,
                by_index,
                target_system,
                target_component,
                retry_batch,
            )
            last_parameter = now

    if expected is None:
        raise TimeoutError(
            "No PARAM_VALUE messages received from the flight controller"
        )
    missing = [index for index in range(expected) if index not in by_index]
    raise TimeoutError(
        f"Incomplete parameter download: {len(by_index)}/{expected}; "
        f"missing indexes {missing[:20]}"
    )
