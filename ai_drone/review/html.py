"""Render a self-contained browser report without external assets or services."""

from __future__ import annotations

import html
import json
import math
from importlib.resources import files
from typing import Any
from urllib.parse import urlsplit


def _relative_href(value: Any) -> str | None:
    """Allow local relative assets, never executable or remote URL schemes."""
    if not isinstance(value, str):
        return None
    value = value.strip()
    if not value or value.startswith(("/", "\\")):
        return None
    if "\\" in value or any(ord(char) < 32 or ord(char) == 127 for char in value):
        return None
    try:
        parts = urlsplit(value)
    except ValueError:
        return None
    if parts.scheme or parts.netloc or not parts.path:
        return None
    return value


def _finite_json(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_json(item) for item in value]
    return value


def _panel_priority(series: dict[str, Any]) -> int:
    """Put navigation sensors first without reordering related signals."""
    panel = str(series.get("panel", "")).casefold()
    if panel in ("distance", "range"):
        return 0
    if panel.startswith("optical flow"):
        if "quality" in panel:
            return 1
        if "velocity" in panel:
            return 2
        return 3
    if panel == "attitude":
        return 4
    return 5


def render_report(payload: dict[str, Any]) -> str:
    """Return an offline report; leave the caller's payload untouched."""
    data = _finite_json(payload)
    if "series" in data:
        data["series"].sort(key=_panel_priority)
    for item in data.get("files", []):
        item["href"] = _relative_href(item.get("href"))
    camera = data.get("camera") or {}
    for name in ("video", "first_frame", "last_frame"):
        camera[name] = _relative_href(camera.get(name))
    data["camera"] = camera
    serialized = json.dumps(
        data, ensure_ascii=True, allow_nan=False, separators=(",", ":")
    )
    for character, escaped in (("<", "\\u003c"), (">", "\\u003e"), ("&", "\\u0026")):
        serialized = serialized.replace(character, escaped)
    title = html.escape(str(data.get("title") or "Drone capture review"))
    template = (
        files("ai_drone.review").joinpath("report.html").read_text(encoding="utf-8")
    )
    before, marker, after = template.partition("__REPORT_DATA__")
    assert marker
    # Substitute only in the template, never inside user-supplied JSON.
    return (
        before.replace("__REPORT_TITLE__", title)
        + serialized
        + after.replace("__REPORT_TITLE__", title)
    )
