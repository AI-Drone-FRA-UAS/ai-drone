from __future__ import annotations

import copy
import json
import math
from html.parser import HTMLParser
from typing import Any

import pytest

from ai_drone.review.html import render_report


class ReportDocument(HTMLParser):
    def __init__(self, source: str) -> None:
        super().__init__()
        self.elements: list[tuple[str, dict[str, str | None]]] = []
        self.scripts: list[tuple[dict[str, str | None], str]] = []
        self._script: tuple[dict[str, str | None], list[str]] | None = None
        self.text: list[str] = []
        self.feed(source)

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attributes = dict(attrs)
        self.elements.append((tag, attributes))
        if tag == "script":
            self._script = attributes, []

    def handle_data(self, data: str) -> None:
        if self._script is not None:
            self._script[1].append(data)
        else:
            self.text.append(data)

    def handle_endtag(self, tag: str) -> None:
        if tag == "script" and self._script is not None:
            attributes, chunks = self._script
            self.scripts.append((attributes, "".join(chunks)))
            self._script = None

    def payload(self) -> dict[str, Any]:
        embedded = [
            body
            for attrs, body in self.scripts
            if attrs.get("type") == "application/json"
        ]
        assert len(embedded) == 1
        return json.loads(embedded[0])


@pytest.fixture
def payload() -> dict[str, Any]:
    return {
        "title": "Bench capture",
        "started_utc": "2026-09-09T14:00:00Z",
        "duration_s": 2.0,
        "completed": True,
        "error": None,
        "warnings": [],
        "vehicle": {"system": 1, "component": 1},
        "summary": [
            {
                "label": "Forward range",
                "samples": 3,
                "valid_samples": 2,
                "min": 1.5,
                "max": 1.6,
                "unit": "m",
            }
        ],
        "series": [
            {
                "id": "forward_range",
                "label": "Forward range",
                "unit": "m",
                "panel": "Range",
                "points": [[0.0, 1.5, True], [0.1, 0.0, False], [2.0, 1.6, True]],
                "gap_s": 0.5,
            }
        ],
        "messages": {"DISTANCE_SENSOR": 3, "HEARTBEAT": 1},
        "files": [{"label": "Telemetry CSV", "href": "telemetry.csv"}],
        "camera": {
            "frames": 0,
            "video": None,
            "first_frame": None,
            "last_frame": None,
            "video_note": "No video was recorded.",
        },
    }


def test_renderer_preserves_recorded_evidence_without_mutating_payload(payload) -> None:
    before = copy.deepcopy(payload)
    result = render_report(payload)
    document = ReportDocument(result)

    assert document.payload() == before
    assert payload == before
    assert result == render_report(payload)
    assert result.startswith("<!doctype html>")
    assert len(document.scripts) == 2
    assert "Bench capture" in document.text


def test_navigation_panels_precede_other_sensors_without_reordering_signals(payload):
    panel_names = [
        "Battery",
        "Attitude",
        "Optical flow angular rate",
        "Optical flow velocity",
        "Distance",
        "Optical flow quality",
        "Distance",
        "Pressure",
    ]
    payload["series"] = [
        {**payload["series"][0], "panel": name, "id": str(index)}
        for index, name in enumerate(panel_names)
    ]
    before = copy.deepcopy(payload)

    document = ReportDocument(render_report(payload))

    assert [series["id"] for series in document.payload()["series"]] == [
        "4",
        "6",
        "5",
        "3",
        "2",
        "1",
        "0",
        "7",
    ]
    assert payload == before
    assert any(
        tag == "a" and attributes.get("href") == "#camera-section"
        for tag, attributes in document.elements
    )


def test_report_has_accessible_controls_and_independent_video_description(
    payload,
) -> None:
    document = ReportDocument(render_report(payload))
    elements = {attrs.get("id"): (tag, attrs) for tag, attrs in document.elements}

    assert elements["cursor"][1]["type"] == "range"
    assert elements["play"][1]["type"] == "button"
    assert elements["play"][1]["aria-pressed"] == "false"
    assert elements["window"][0] == "select"
    assert elements["speed"][0] == "select"
    assert any(attrs.get("for") == "cursor" for _, attrs in document.elements)
    assert (
        "Video playback is independent. Video and telemetry are not calibrated "
        "to a shared time reference."
    ) in document.text


def test_untrusted_text_cannot_close_json_script_or_create_html(payload) -> None:
    attack = '</script><img src=x onerror="alert(1)"> & " <script>bad()</script>'
    payload["title"] = attack
    payload["warnings"] = [attack]
    payload["error"] = attack
    payload["series"][0].update(label=attack, panel=attack, unit=attack, id=attack)
    payload["messages"] = {attack: 7}
    payload["files"][0]["label"] = attack
    payload["camera"]["video_note"] = attack
    document = ReportDocument(render_report(payload))

    assert len(document.scripts) == 2
    assert not any(tag == "img" for tag, _ in document.elements)
    assert not any(
        name.lower().startswith("on")
        for _, attrs in document.elements
        for name in attrs
    )
    assert document.payload()["warnings"] == [attack]
    assert document.payload()["series"][0]["label"] == attack
    assert attack in document.text
    embedded = document.scripts[0][1]
    assert "<" not in embedded
    assert ">" not in embedded
    assert "&" not in embedded


def test_template_like_text_is_never_substituted_inside_user_data(payload) -> None:
    payload["title"] = '__REPORT_DATA__ "__REPORT_TITLE__"'
    payload["warnings"] = ["__REPORT_TITLE__", "__REPORT_DATA__"]
    document = ReportDocument(render_report(payload))

    assert document.payload()["title"] == payload["title"]
    assert document.payload()["warnings"] == payload["warnings"]
    assert payload["title"] in document.text


@pytest.mark.parametrize(
    "href",
    [
        "javascript:alert(1)",
        " JaVaScRiPt:alert(1)",
        "data:text/html,unsafe",
        "https://example.com/telemetry.csv",
        "http://example.com/video.mp4",
        "file:///etc/passwd",
        "blob:example",
        "//example.com/image.jpg",
        "/absolute/path.csv",
        "\\\\example.com\\video.mp4",
        "java\nscript:alert(1)",
        "safe\x00.csv",
        "?download=1",
        "#fragment",
        "",
        None,
    ],
)
def test_asset_and_download_urls_reject_nonlocal_or_executable_references(
    payload, href
) -> None:
    payload["files"][0]["href"] = href
    for name in ("video", "first_frame", "last_frame"):
        payload["camera"][name] = href
    result = ReportDocument(render_report(payload)).payload()

    assert result["files"][0]["href"] is None
    assert result["camera"]["video"] is None
    assert result["camera"]["first_frame"] is None
    assert result["camera"]["last_frame"] is None


@pytest.mark.parametrize(
    "href",
    ["telemetry.csv", "./raw/events.jsonl", "../camera/video.mp4", "first%20frame.jpg"],
)
def test_relative_assets_remain_available(payload, href) -> None:
    payload["files"][0]["href"] = href
    payload["camera"]["video"] = href
    result = ReportDocument(render_report(payload)).payload()

    assert result["files"][0]["href"] == href
    assert result["camera"]["video"] == href


def test_nonfinite_numbers_become_null_without_losing_invalid_or_gap_markers(
    payload,
) -> None:
    payload["series"][0]["points"] = [
        [0.0, 0.0, True],
        [0.0, 0.0, False],
        [1.0, math.nan, False],
        [2.0, math.inf, False],
        [3.0, None, False],
    ]
    payload["summary"][0].update(min=-math.inf, max=math.nan)
    serialized_before = json.dumps(payload)
    result = ReportDocument(render_report(payload)).payload()

    assert result["series"][0]["points"] == [
        [0.0, 0.0, True],
        [0.0, 0.0, False],
        [1.0, None, False],
        [2.0, None, False],
        [3.0, None, False],
    ]
    assert result["summary"][0]["min"] is None
    assert result["summary"][0]["max"] is None
    assert result["series"][0]["gap_s"] == 0.5
    assert json.dumps(payload) == serialized_before


def test_empty_partial_capture_renders_and_preserves_error(payload) -> None:
    payload.update(duration_s=0, completed=False, error="UART disconnected", series=[])
    payload["summary"] = []
    payload["messages"] = {}
    result = ReportDocument(render_report(payload)).payload()

    assert result["duration_s"] == 0
    assert result["completed"] is False
    assert result["error"] == "UART disconnected"
    assert result["series"] == []
    assert ReportDocument(render_report({})).payload()["camera"]["video"] is None


def test_report_requires_no_external_scripts_styles_fonts_or_network(payload) -> None:
    document = ReportDocument(render_report(payload))
    assert not any(
        (tag == "script" and attrs.get("src")) or tag in ("link", "iframe", "object")
        for tag, attrs in document.elements
    )
    policy = next(
        attrs["content"]
        for tag, attrs in document.elements
        if tag == "meta" and attrs.get("http-equiv") == "Content-Security-Policy"
    )
    assert policy is not None and "connect-src 'none'" in policy
    javascript = document.scripts[1][1]
    assert ".innerHTML" not in javascript
    assert "eval(" not in javascript
    assert "fetch(" not in javascript
