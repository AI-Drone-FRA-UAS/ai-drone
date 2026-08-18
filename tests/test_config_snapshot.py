from __future__ import annotations

import hashlib
from types import SimpleNamespace

import pytest

from ai_drone import config_sync
from ai_drone.config_snapshot import (
    ParameterRecord,
    download_all_parameters,
    format_parameter_value,
    records_from_json,
    records_to_json,
    render_parameter_file,
)


class FakeMav:
    def __init__(self) -> None:
        self.list_requests: list[tuple[int, int]] = []
        self.index_requests: list[int] = []

    def param_request_list_send(self, system: int, component: int) -> None:
        self.list_requests.append((system, component))

    def param_request_read_send(
        self, system: int, component: int, name: bytes, index: int
    ) -> None:
        assert (system, component, name) == (1, 1, b"")
        self.index_requests.append(index)


class FakeConnection:
    target_system = 1
    target_component = 1

    def __init__(self, messages: list[SimpleNamespace]) -> None:
        self.messages = iter(messages)
        self.mav = FakeMav()

    def recv_match(self, **_kwargs):
        return next(self.messages, None)


def parameter_message(index: int, name: str, value: float) -> SimpleNamespace:
    return SimpleNamespace(
        get_type=lambda: "PARAM_VALUE",
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
        param_count=2,
        param_index=index,
        param_id=name,
        param_value=value,
        param_type=9,
    )


def test_parameter_file_is_sorted_and_stable() -> None:
    records = [
        ParameterRecord("BETA", 1.25, 9, 1, 2),
        ParameterRecord("ALPHA", 2.0, 9, 0, 2),
    ]

    assert render_parameter_file(records) == "ALPHA,2\nBETA,1.25\n"
    assert format_parameter_value(0.0599999987) == "0.0599999987"


def test_parameter_json_round_trip_requires_complete_unique_indexes() -> None:
    records = [
        ParameterRecord("ALPHA", 2.0, 9, 0, 2),
        ParameterRecord("BETA", 1.25, 9, 1, 2),
    ]

    assert records_from_json(records_to_json(records)) == records

    duplicate = records_to_json(records)
    duplicate[1]["index"] = 0
    with pytest.raises(ValueError, match="duplicate parameter indexes"):
        records_from_json(duplicate)


def test_download_requests_full_list_and_returns_all_parameters() -> None:
    connection = FakeConnection(
        [parameter_message(1, "BETA", 1.25), parameter_message(0, "ALPHA", 2.0)]
    )

    records = download_all_parameters(connection, timeout=1.0)

    assert connection.mav.list_requests == [(1, 1)]
    assert [record.name for record in records] == ["ALPHA", "BETA"]


def test_write_snapshot_verifies_hash_and_uses_date_paths(tmp_path) -> None:
    records = [ParameterRecord("ALPHA", 2.0, 9, 0, 1)]
    parameter_text = render_parameter_file(records)
    bundle = {
        "captured_at": "2026-08-18T16:00:00+00:00",
        "parameter_count": 1,
        "parameter_sha256": hashlib.sha256(parameter_text.encode()).hexdigest(),
        "parameters": records_to_json(records),
        "source": {"endpoint": "/dev/serial0", "baud": 115200},
    }

    parameter_path, metadata_path = config_sync.write_snapshot(bundle, tmp_path)

    assert parameter_path == tmp_path / "params/flywoo-f745-live-2026-08-18.param"
    contents = parameter_path.read_text()
    assert "# Vehicle was DISARMED\n" in contents
    assert contents.endswith(parameter_text)
    assert metadata_path == tmp_path / "state/2026-08-18/drone-config.json"
    assert '"parameters"' not in metadata_path.read_text()
