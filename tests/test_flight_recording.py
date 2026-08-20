from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import MagicMock

from ai_drone.flight.recording import FlightRecorder


class Connection:
    def __init__(self) -> None:
        self.mav = MagicMock()
        self.target_system = self.target_component = 1
        self.logfile = None

    def setup_logfile(self, path: str) -> None:
        self.logfile = Path(path).open("wb")  # noqa: SIM115 - connection owns it


def test_flight_recorder_writes_complete_bundle(tmp_path) -> None:
    connection = Connection()
    record = FlightRecorder(connection, {"command": "hover"}, tmp_path / "flight")
    assert connection.logfile is not None
    connection.logfile.write(b"telemetry")
    record.event("hover_started", target_alt_m=0.4)
    record.finish()
    record.close()

    manifest = json.loads((record.root / "manifest.json").read_text())
    events = [
        json.loads(line)
        for line in (record.root / "events.jsonl").read_text().splitlines()
    ]
    assert manifest["completed"] is True
    assert manifest["metadata"] == {"command": "hover"}
    assert "OPTICAL_FLOW" in manifest["requested_messages"]
    assert [event["event"] for event in events] == [
        "recording_started",
        "hover_started",
        "completed",
    ]
    assert (record.root / "telemetry.tlog").read_bytes() == b"telemetry"
    assert connection.logfile is None
