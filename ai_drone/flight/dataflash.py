from __future__ import annotations

from time import monotonic
from typing import Any, TypedDict


class DataFlashLog(TypedDict):
    number: int
    size_bytes: int
    time_utc_s: int


def latest_dataflash_log(
    connection: Any, timeout_s: float = 5.0
) -> DataFlashLog | None:
    system = int(connection.target_system)
    component = int(connection.target_component)
    connection.mav.log_request_list_send(system, component, 0, 0xFFFF)
    deadline = monotonic() + timeout_s
    requested: int | None = None

    while (remaining := deadline - monotonic()) > 0:
        message = connection.recv_match(
            type="LOG_ENTRY", blocking=True, timeout=remaining
        )
        if message is None or int(message.num_logs) == 0:
            return None
        latest = int(message.last_log_num)
        if int(message.id) == latest:
            return {
                "number": latest,
                "size_bytes": int(message.size),
                "time_utc_s": int(message.time_utc),
            }
        if requested != latest:
            connection.mav.log_request_list_send(system, component, latest, latest)
            requested = latest
    return None
