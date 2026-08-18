"""Complete, read-only ArduPilot parameter snapshot helpers."""

from __future__ import annotations

import hashlib
import math
import time
from dataclasses import asdict, dataclass
from typing import Any

from pymavlink.dialects.v10 import ardupilotmega as mavlink


@dataclass(frozen=True)
class ParameterRecord:
    """One parameter returned by the MAVLink parameter protocol."""

    name: str
    value: float
    param_type: int
    index: int
    count: int


def heartbeat_is_armed(message: Any) -> bool:
    """Return whether a HEARTBEAT has the safety-armed flag set."""

    return bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def decode_parameter_name(value: str | bytes) -> str:
    """Decode the fixed-width MAVLink PARAM_VALUE identifier."""

    if isinstance(value, bytes):
        return value.split(b"\0", 1)[0].decode("ascii", errors="strict")
    return value.split("\0", 1)[0]


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


def records_from_json(items: list[dict[str, Any]]) -> list[ParameterRecord]:
    """Validate and reconstruct records from a remote export bundle."""

    records = [
        ParameterRecord(
            name=str(item["name"]),
            value=float(item["value"]),
            param_type=int(item["param_type"]),
            index=int(item["index"]),
            count=int(item["count"]),
        )
        for item in items
    ]
    expected = len(records)
    if not records:
        raise ValueError("Snapshot contains no parameters")
    if len({record.name for record in records}) != expected:
        raise ValueError("Snapshot contains duplicate parameter names")
    if len({record.index for record in records}) != expected:
        raise ValueError("Snapshot contains duplicate parameter indexes")
    announced = {record.count for record in records}
    if announced != {expected}:
        raise ValueError(
            f"Snapshot is incomplete: received {expected}, announced {sorted(announced)}"
        )
    return records


def _from_target(message: Any, connection: Any) -> bool:
    source_system = getattr(message, "get_srcSystem", lambda: 0)()
    source_component = getattr(message, "get_srcComponent", lambda: 0)()
    target_system = int(connection.target_system)
    target_component = int(connection.target_component)
    component_matches = target_component == 0 or source_component in (
        0,
        target_component,
    )
    return source_system in (0, target_system) and component_matches


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

        if message is not None and _from_target(message, connection):
            message_type = message.get_type()
            if message_type == "HEARTBEAT":
                if heartbeat_is_armed(message):
                    raise RuntimeError(
                        "Vehicle became ARMED; parameter download aborted."
                    )
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
            records = list(by_index.values())
            if len({record.name for record in records}) != expected:
                raise RuntimeError(
                    "Flight controller returned duplicate parameter names"
                )
            return sorted(records, key=lambda record: record.name)

        if expected is not None and now - last_parameter >= retry_after:
            missing = [index for index in range(expected) if index not in by_index]
            for index in missing[:retry_batch]:
                connection.mav.param_request_read_send(
                    target_system,
                    target_component,
                    b"",
                    index,
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
