from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from ai_drone.flight.dataflash import latest_dataflash_log
from ai_drone.flight.provenance import flight_code_sha256


class FakeConnection:
    target_system = 1
    target_component = 1

    def __init__(self, messages: list[Any]) -> None:
        self.messages = iter(messages)
        self.sent: list[tuple[int, ...]] = []
        self.mav = SimpleNamespace(log_request_list_send=self._request)

    def _request(self, *values: int) -> None:
        self.sent.append(values)

    def recv_match(self, **_: Any) -> Any:
        return next(self.messages, None)


def test_flight_code_hash_tracks_package_content(tmp_path: Path) -> None:
    package = tmp_path / "ai_drone"
    package.mkdir()
    source = package / "module.py"
    source.write_text("VALUE = 1\n")
    before = flight_code_sha256(tmp_path)
    source.write_text("VALUE = 2\n")
    assert flight_code_sha256(tmp_path) != before


def test_latest_dataflash_log_requests_authoritative_latest_entry() -> None:
    source = {"get_srcSystem": lambda: 1, "get_srcComponent": lambda: 1}
    older = SimpleNamespace(
        id=8, num_logs=2, last_log_num=9, size=10, time_utc=1, **source
    )
    latest = SimpleNamespace(
        id=9, num_logs=2, last_log_num=9, size=1234, time_utc=2, **source
    )
    connection = FakeConnection([older, latest])

    assert latest_dataflash_log(connection) == {
        "number": 9,
        "size_bytes": 1234,
        "time_utc_s": 2,
    }
    assert connection.sent == [(1, 1, 0, 0xFFFF), (1, 1, 9, 9)]


@pytest.mark.parametrize(("system", "component"), [(2, 1), (1, 42)])
@pytest.mark.parametrize("num_logs", [0, 2])
def test_foreign_log_entries_cannot_change_selection_or_report_no_logs(
    system: int, component: int, num_logs: int
) -> None:
    foreign = SimpleNamespace(
        id=80,
        num_logs=num_logs,
        last_log_num=81,
        size=9999,
        time_utc=1000,
        get_srcSystem=lambda: system,
        get_srcComponent=lambda: component,
    )
    matching = SimpleNamespace(
        id=9,
        num_logs=2,
        last_log_num=9,
        size=1234,
        time_utc=2,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
    )
    connection = FakeConnection([foreign, matching])

    assert latest_dataflash_log(connection) == {
        "number": 9,
        "size_bytes": 1234,
        "time_utc_s": 2,
    }
    assert connection.sent == [(1, 1, 0, 0xFFFF)]
