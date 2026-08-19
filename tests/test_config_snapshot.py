from __future__ import annotations

import hashlib
from types import SimpleNamespace

import pytest

from ai_drone.cli import config_export
from ai_drone.config import sync as config_sync
from ai_drone.config.snapshot import (
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

    shifted = records_to_json(records)
    shifted[0]["index"] = 2
    shifted[1]["index"] = 3
    with pytest.raises(ValueError, match="indexes must be contiguous"):
        records_from_json(shifted)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_parameter_json_rejects_non_finite_values(value: float) -> None:
    items = records_to_json([ParameterRecord("ALPHA", 2.0, 9, 0, 1)])
    items[0]["value"] = value

    with pytest.raises(ValueError, match="non-finite parameter value"):
        records_from_json(items)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("name", "", "invalid ArduPilot name"),
        ("name", "BAD\nINJECT", "invalid ArduPilot name"),
        ("name", "BAD,INJECT", "invalid ArduPilot name"),
        ("name", "lowercase", "invalid ArduPilot name"),
        ("name", "A" * 17, "invalid ArduPilot name"),
        ("value", "2.0", "JSON number"),
        ("value", True, "JSON number"),
        ("param_type", 9.5, "must be an integer"),
        ("param_type", 11, "between 1 and 10"),
        ("index", 0.5, "must be an integer"),
        ("count", "1", "must be an integer"),
        ("count", 65_536, "between 1 and 65535"),
    ],
)
def test_parameter_json_rejects_lossy_or_unsafe_fields(
    field: str, value: object, message: str
) -> None:
    items = records_to_json([ParameterRecord("ALPHA", 2.0, 9, 0, 1)])
    items[0][field] = value  # ty: ignore[invalid-assignment]

    with pytest.raises(ValueError, match=message):
        records_from_json(items)


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
        "schema_version": 1,
        "captured_at": "2026-08-18T16:00:00+00:00",
        "parameter_count": 1,
        "parameter_sha256": hashlib.sha256(parameter_text.encode()).hexdigest(),
        "parameters": records_to_json(records),
        "source": {"endpoint": "/dev/serial0", "baud": 115200},
        "vehicle": {"armed": False},
    }

    parameter_path, metadata_path = config_sync.write_snapshot(bundle, tmp_path)

    assert parameter_path == tmp_path / "params/flywoo-f745-live-2026-08-18.param"
    contents = parameter_path.read_text()
    assert "# Vehicle was DISARMED\n" in contents
    assert contents.endswith(parameter_text)
    assert metadata_path == tmp_path / "state/2026-08-18/drone-config.json"
    assert '"parameters"' not in metadata_path.read_text()


def test_config_export_certifies_a_fresh_final_disarmed_heartbeat(
    monkeypatch, tmp_path
) -> None:
    initial = SimpleNamespace(
        type=2,
        autopilot=3,
        base_mode=0,
        custom_mode=0,
        system_status=3,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
    )
    final = SimpleNamespace(
        type=2,
        autopilot=3,
        base_mode=0,
        custom_mode=4,
        system_status=4,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
    )
    connection = SimpleNamespace(
        target_system=1,
        target_component=1,
        wait_heartbeat=lambda *, timeout: initial,
        close=lambda: setattr(connection, "closed", True),
        closed=False,
    )
    records = [ParameterRecord("ALPHA", 2.0, 9, 0, 1)]
    bundles = []
    monkeypatch.setattr(
        config_export,
        "resolve_mavlink_endpoint",
        lambda *_args, **_kwargs: tmp_path / "serial",
    )
    monkeypatch.setattr(
        config_export.mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )
    monkeypatch.setattr(
        config_export,
        "download_all_parameters",
        lambda *_args, **_kwargs: records,
    )
    monkeypatch.setattr(
        config_export,
        "require_fresh_disarmed_heartbeat",
        lambda *_args, **_kwargs: final,
    )
    monkeypatch.setattr(
        config_export,
        "_write_bundle",
        lambda bundle, _output: bundles.append(bundle),
    )

    result = config_export.main(["--device", str(tmp_path / "serial")])

    assert result == 0
    assert bundles[0]["vehicle"]["custom_mode"] == 4
    assert bundles[0]["vehicle"]["system_status"] == 4
    assert connection.closed


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda bundle: bundle.update(schema_version=2), "schema_version"),
        (lambda bundle: bundle["vehicle"].update(armed=True), "disarmed"),
        (lambda bundle: bundle.update(parameter_count=2), "parameter_count"),
        (lambda bundle: bundle.update(captured_at="2026-08-18"), "timezone"),
        (
            lambda bundle: bundle["source"].update(endpoint="/dev/serial0\n# injected"),
            "control characters",
        ),
    ],
)
def test_write_snapshot_rejects_invalid_bundle_before_writing(
    tmp_path, mutation, message
) -> None:
    records = [ParameterRecord("ALPHA", 2.0, 9, 0, 1)]
    bundle = {
        "schema_version": 1,
        "captured_at": "2026-08-18T16:00:00+00:00",
        "parameter_count": 1,
        "parameter_sha256": hashlib.sha256(
            render_parameter_file(records).encode()
        ).hexdigest(),
        "parameters": records_to_json(records),
        "source": {"endpoint": "/dev/serial0", "baud": 115200},
        "vehicle": {"armed": False},
    }
    mutation(bundle)

    with pytest.raises(ValueError, match=message):
        config_sync.write_snapshot(bundle, tmp_path)

    assert not (tmp_path / "params").exists()
    assert not (tmp_path / "state").exists()
