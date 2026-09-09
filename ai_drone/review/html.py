"""Render a self-contained browser report without external assets or services."""

from __future__ import annotations

import html
import json
import math
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
    before, marker, after = _PAGE.partition("__REPORT_DATA__")
    assert marker
    # Substitute only in the template, never inside user-supplied JSON.
    return (
        before.replace("__REPORT_TITLE__", title)
        + serialized
        + after.replace("__REPORT_TITLE__", title)
    )


_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src 'self' file:; media-src 'self' file:; connect-src 'none'; object-src 'none'; base-uri 'none'; form-action 'none'">
<title>__REPORT_TITLE__ · Drone capture review</title>
<style>
:root{color-scheme:light;--ink:#172d3a;--muted:#596b77;--line:#dbe3e8;--paper:#fff;--accent:#087f80;--bad:#ba3329;--soft:#f2f6f8}
*{box-sizing:border-box}
body{margin:0;background:var(--soft);color:var(--ink);font:15px/1.55 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
main{max-width:1280px;margin:auto;padding:34px 28px 44px}
h1,h2,h3,p{margin-top:0}
h1{font-size:clamp(25px,3vw,39px);line-height:1.2;letter-spacing:-.035em;margin-bottom:10px;overflow-wrap:anywhere}
h2{font-size:21px;letter-spacing:-.025em;margin-bottom:5px}
h3{font-size:17px;line-height:1.3;margin-bottom:0}
a{color:#076c91;text-underline-offset:3px;overflow-wrap:anywhere}
button,input,select{font:inherit}
button,select,input[type=number]{border:1px solid #becbd3;border-radius:7px;background:#fff;color:var(--ink);padding:7px 11px}
button{cursor:pointer}
button:hover{background:#eef5f6}
button:disabled,input:disabled,select:disabled{opacity:.5;cursor:default}
button:focus-visible,a:focus-visible,input:focus-visible,select:focus-visible,canvas:focus-visible{outline:3px solid #087f8070;outline-offset:3px}
.eyebrow{text-transform:uppercase;letter-spacing:.14em;font-size:11px;font-weight:750;color:var(--accent);margin-bottom:9px}
.hero{display:flex;align-items:flex-start;justify-content:space-between;gap:20px;padding-bottom:24px;border-bottom:1px solid var(--line)}
.muted{color:var(--muted)}
.hero p{margin-bottom:0}
.badge{display:inline-block;white-space:nowrap;border-radius:20px;padding:6px 12px;font-size:12px;font-weight:700;background:#e6f4ef;color:#1b6850}
.badge.partial{background:#fff0d9;color:#85510b}
.facts{display:grid;grid-template-columns:2fr 1fr 1fr 1fr;gap:14px;margin:22px 0}
.fact,.card{background:var(--paper);border:1px solid var(--line);border-radius:12px}
.fact{padding:15px 18px}
.fact dt{color:var(--muted);font-size:12px;margin-bottom:5px}
.fact dd{margin:0;font-size:16px;font-weight:650;overflow-wrap:anywhere}
.notice{margin:12px 0;padding:13px 17px;border:1px solid #ead3a5;border-left:4px solid #c38a31;background:#fff9ed;border-radius:8px}
.notice.error{border-color:#ecc3bc;border-left-color:var(--bad);background:#fff3f0}
.notice p{margin:0}
.notice ul{margin:5px 0;padding-left:20px}
.section{margin-top:30px}
.section-intro{color:var(--muted);font-size:13px;margin-bottom:16px;max-width:920px}
.transport{position:sticky;top:0;z-index:2;background:#f2f6f8f5;border:1px solid var(--line);border-radius:11px;padding:13px 16px;margin:16px 0;box-shadow:0 4px 12px #172d3a06}
.transport-row{display:flex;align-items:center;gap:14px;flex-wrap:wrap}
.play{background:var(--accent);color:white;border-color:var(--accent);min-width:76px;font-weight:650}
.play:hover{background:#07686b}
.cursor-time{font-variant-numeric:tabular-nums;font-weight:650;min-width:115px}
.slider-wrap{flex:1;min-width:170px}
input[type=range]{width:100%;accent-color:var(--accent);cursor:pointer;vertical-align:middle}
.small-control{font-size:12px;color:var(--muted);display:flex;align-items:center;gap:7px}
.small-control select{font-size:12px;padding:6px 8px}
.timeline-note{display:flex;justify-content:space-between;gap:12px;font-size:11px;color:var(--muted);margin-top:7px}
.charts{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px}
.chart-card{padding:18px 18px 10px;min-width:0}
.chart-card.wide{grid-column:1/-1}
.chart-heading{display:flex;justify-content:space-between;gap:14px;align-items:baseline}
.unit{font-size:12px;color:var(--muted)}
.legend{display:flex;gap:8px 12px;flex-wrap:wrap;margin:13px 0 6px}
.legend-item{display:flex;align-items:center;gap:8px;padding:6px 9px;border:1px solid var(--line);font-size:12px;border-radius:7px}
.legend-item[aria-pressed=false]{opacity:.45}
.swatch{width:9px;height:9px;border-radius:50%;flex-shrink:0}
.legend-value{font-variant-numeric:tabular-nums;font-weight:650;min-width:64px;text-align:right}
.legend-value.invalid{color:var(--bad)}
.plot{height:245px;position:relative}
.wide .plot{height:290px}
canvas{display:block;width:100%;height:100%;touch-action:pan-y}
.chart-foot{display:flex;justify-content:space-between;color:var(--muted);font-size:11px;gap:10px;padding:0 3px 4px}
.invalid-key{color:var(--bad)}
.empty{padding:35px 22px;text-align:center;color:var(--muted)}
.support-grid{display:grid;grid-template-columns:minmax(0,1.2fr) minmax(0,1fr);gap:20px}
.support-grid>.card{padding:20px;min-width:0}
.table-wrap{overflow:auto}
table{width:100%;border-collapse:collapse;font-size:13px;text-align:left}
th{color:var(--muted);font-size:11px;font-weight:650;text-transform:uppercase;letter-spacing:.04em;white-space:nowrap}
th,td{padding:10px 9px;border-bottom:1px solid #e8edf0;vertical-align:top}
th:first-child,td:first-child{padding-left:0}
td.number,th.number{text-align:right;font-variant-numeric:tabular-nums;white-space:nowrap}
tbody tr:last-child td{border-bottom:0}
.camera{padding:20px}
.camera video{width:100%;max-height:560px;background:#17212a;border-radius:8px;display:block;margin:15px 0 12px}
.previews{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:15px;margin-top:16px}
figure{margin:0}
figure img{width:100%;height:260px;object-fit:contain;background:#17212a;border-radius:8px;display:block}
figcaption{font-size:12px;color:var(--muted);padding-top:6px}
.download-list{display:flex;flex-wrap:wrap;gap:10px;list-style:none;padding:0;margin:14px 0 0}
.download-list a{display:block;border:1px solid var(--line);border-radius:7px;padding:8px 12px;background:white;font-size:13px}
details summary{cursor:pointer;font-weight:650;font-size:14px;margin-bottom:10px}
.count-total{color:var(--muted);font-size:12px;font-weight:400;margin-left:8px}
.messages{max-height:320px;overflow:auto}
footer{margin-top:28px;padding-top:16px;border-top:1px solid var(--line);color:var(--muted);font-size:12px}
.sr-only{position:absolute;width:1px;height:1px;padding:0;overflow:hidden;clip:rect(0,0,0,0);white-space:nowrap;border:0}
@media(max-width:760px){main{padding:22px 15px 30px}.hero{display:block}.hero .badge{margin-top:12px}.facts{grid-template-columns:1fr 1fr}.charts,.support-grid{grid-template-columns:1fr}.chart-card.wide{grid-column:auto}.transport{padding:11px}.transport-row{gap:9px}.slider-wrap{flex-basis:100%;order:3}.timeline-note{margin-top:10px}.plot,.wide .plot{height:240px}.chart-card{padding:15px 12px 8px}.previews{grid-template-columns:1fr}figure img{height:auto;max-height:400px}.fact{padding:12px}.fact dd{font-size:14px}}
@media print{.transport{position:static}.play,select,input{display:none}body{background:white}main{max-width:none}.chart-card,.camera,.support-grid>.card{break-inside:avoid}.charts{display:block}.chart-card{margin-bottom:18px}a{color:inherit}}
</style>
</head>
<body>
<main>
<header class="hero">
<div><div class="eyebrow">Drone · capture review</div><h1>__REPORT_TITLE__</h1>
<p class="muted">Explore the recorded sensor data. Missing samples remain visible as gaps. <a href="#camera-section">Jump to camera</a></p></div>
<span id="capture-status" class="badge">Recorded capture</span>
</header>
<noscript><div class="notice">Enable JavaScript to explore the charts and view the recorded values. The report does not need an internet connection.</div></noscript>
<dl id="facts" class="facts"></dl>
<div id="notices" aria-label="Capture notes"></div>
<section class="section" aria-labelledby="telemetry-heading">
<h2 id="telemetry-heading">Telemetry over time</h2>
<p class="section-intro">Move the cursor or click a chart to inspect a moment. Values use the latest earlier sample within each sensor's freshness limit. Lines stop at invalid samples and recording gaps.</p>
<div class="transport">
<div class="transport-row">
<button id="play" type="button" class="play" aria-pressed="false">Play</button>
<output id="cursor-time" class="cursor-time" for="cursor">0.00 s</output>
<div class="slider-wrap"><label for="cursor" class="sr-only">Telemetry time in seconds</label><input id="cursor" type="range" min="0" max="0" value="0" step="0.01"></div>
<label class="small-control" for="speed">Speed <select id="speed"><option value="0.5">0.5&times;</option><option value="1" selected>1&times;</option><option value="2">2&times;</option><option value="5">5&times;</option></select></label>
<label class="small-control" for="window">View <select id="window"><option value="0">Full recording</option></select></label>
</div>
<div class="timeline-note"><span id="window-label"></span><span>Elapsed seconds · chart playback only</span></div>
</div>
<div id="charts" class="charts"></div>
</section>
<section id="camera-section" class="section card camera" aria-labelledby="camera-heading">
<h2 id="camera-heading">Camera</h2>
<p class="section-intro">Video playback is independent. Video and telemetry are not calibrated to a shared time reference.</p>
<div id="camera-content"></div>
</section>
<div class="support-grid section">
<section class="card" aria-labelledby="summary-heading"><h2 id="summary-heading">Recorded values</h2>
<p class="section-intro">Ranges summarize valid samples across the full recording.</p>
<div class="table-wrap"><table id="summary-table"><thead><tr><th>Signal</th><th class="number">Valid / total</th><th class="number">Minimum</th><th class="number">Maximum</th></tr></thead><tbody></tbody></table></div></section>
<section class="card" aria-labelledby="files-heading"><h2 id="files-heading">Files &amp; message counts</h2>
<p class="section-intro">Download the exported tables and original recording files.</p>
<ul id="files" class="download-list"></ul>
<details class="section" open><summary>Recorded messages <span id="message-total" class="count-total"></span></summary><div class="messages table-wrap"><table id="message-table"><thead><tr><th>Message</th><th class="number">Count</th></tr></thead><tbody></tbody></table></div></details>
</section>
</div>
<footer>Recorded data · Use the exported tables to inspect individual samples.</footer>
</main>
<script id="report-data" type="application/json">__REPORT_DATA__</script>
<script>
'use strict';
(function () {
  const data = JSON.parse(document.getElementById('report-data').textContent);
  const byId = id => document.getElementById(id);
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const text = value => value == null ? '' : String(value);
  const make = (tag, className, content) => {
    const element = document.createElement(tag);
    if (className) element.className = className;
    if (content !== undefined) element.textContent = text(content);
    return element;
  };
  const number = value => finite(value) ? value.toLocaleString(undefined, {maximumFractionDigits: 3}) : '—';
  const seconds = value => value.toFixed(2) + ' s';
  const clamp = (value, low, high) => Math.min(high, Math.max(low, value));
  const palette = ['#3d65b1', '#8062ad', '#588135', '#bd5d18', '#067a79', '#aa4d76'];
  const groups = new Map();
  let duration = finite(data.duration_s) ? Math.max(0, data.duration_s) : 0;
  let seriesNumber = 0;
  for (const raw of data.series || []) {
    const label = text(raw.label || raw.id || 'Signal');
    const panel = text(raw.panel || 'Telemetry');
    const unit = text(raw.unit);
    const points = (raw.points || []).filter(point =>
      Array.isArray(point) && finite(point[0]) && point[0] >= 0
    ).map(point => [point[0], finite(point[1]) ? point[1] : null, point[2] === true])
      .sort((a, b) => a[0] - b[0]);
    if (points.length) duration = Math.max(duration, points[points.length - 1][0]);
    const identity = text(raw.id).toLowerCase() + ' ' + label.toLowerCase();
    const color = identity.includes('down') ? '#067a79' :
      identity.includes('forward') ? '#bd5d18' : palette[seriesNumber % palette.length];
    const series = {label, unit, points, color, visible: true,
      gap: finite(raw.gap_s) ? Math.max(0, raw.gap_s) : 0};
    seriesNumber++;
    // Different physical units never share a numerical y-axis.
    const key = JSON.stringify([panel, unit]);
    if (!groups.has(key)) groups.set(key, {panel, unit, series: []});
    groups.get(key).series.push(series);
  }

  const complete = data.completed === true && !data.error;
  byId('capture-status').textContent = complete ? 'Capture complete' : 'Partial capture';
  byId('capture-status').classList.toggle('partial', !complete);
  const camera = data.camera || {};
  const facts = [
    ['Started (UTC)', data.started_utc || 'Start time unavailable'],
    ['Recorded duration', seconds(duration)],
    ['Flight controller', text((data.vehicle || {}).system) + ' / ' + text((data.vehicle || {}).component)],
    ['Camera frames', number(camera.frames || 0)]
  ];
  for (const [label, value] of facts) {
    const item = make('div', 'fact');
    item.append(make('dt', '', label), make('dd', '', value));
    byId('facts').append(item);
  }
  if (!complete) {
    const notice = make('div', 'notice error');
    notice.append(make('p', '', data.error ?
      'Capture ended with an error: ' + text(data.error) :
      'This capture is incomplete. The report shows the samples that were saved.'));
    byId('notices').append(notice);
  }
  if ((data.warnings || []).length) {
    const notice = make('div', 'notice');
    notice.append(make('p', '', 'Notes about this recording'));
    const list = make('ul');
    for (const warning of data.warnings) list.append(make('li', '', warning));
    notice.append(list);
    byId('notices').append(notice);
  }

  let cursor = 0;
  let playing = false;
  let frame = null;
  let playbackFrame = null;
  let lastPlayback = null;
  const slider = byId('cursor');
  slider.max = String(duration);
  slider.disabled = duration === 0;
  byId('play').disabled = duration === 0;
  for (const span of [1, 5, 15, 30, 60, 120]) {
    if (span < duration) {
      const option = make('option', '', span + ' seconds');
      option.value = String(span);
      byId('window').append(option);
    }
  }
  byId('window').disabled = byId('window').options.length === 1;
  const bounds = () => {
    const selected = Number(byId('window').value);
    const span = selected > 0 ? Math.min(selected, duration) : duration;
    const start = clamp(cursor - span / 2, 0, Math.max(0, duration - span));
    return [start, start + span];
  };
  const prior = (series, at) => {
    let low = 0, high = series.points.length;
    while (low < high) {
      const middle = (low + high) >>> 1;
      if (series.points[middle][0] <= at) low = middle + 1;
      else high = middle;
    }
    if (low === 0) return null;
    const point = series.points[low - 1];
    return at - point[0] <= series.gap ? point : null;
  };
  const valid = point => point[2] && finite(point[1]);
  const axisNumber = value => {
    if (!finite(value)) return '—';
    const magnitude = Math.abs(value);
    if (magnitude >= 100000 || (magnitude > 0 && magnitude < 0.001)) return value.toExponential(1);
    return value.toLocaleString(undefined, {maximumFractionDigits: magnitude < 1 ? 3 : 2});
  };

  function draw(group) {
    const canvas = group.canvas;
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(1, rect.width), height = Math.max(1, rect.height);
    const ratio = Math.min(window.devicePixelRatio || 1, 3);
    if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
      canvas.width = Math.round(width * ratio);
      canvas.height = Math.round(height * ratio);
    }
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, height);
    const [start, end] = bounds();
    const pad = {left: Math.min(70, width * .23), right: 17, top: 18, bottom: 35};
    const plotWidth = Math.max(1, width - pad.left - pad.right);
    const plotHeight = Math.max(1, height - pad.top - pad.bottom);
    const x = value => pad.left + (end > start ? (value - start) / (end - start) : .5) * plotWidth;
    let low = Infinity, high = -Infinity, samples = 0;
    for (const series of group.series) {
      if (!series.visible) continue;
      for (const point of series.points) {
        if (point[0] < start || point[0] > end || !finite(point[1])) continue;
        low = Math.min(low, point[1]);
        high = Math.max(high, point[1]);
        samples++;
      }
    }
    if (!samples) { low = 0; high = 1; }
    else {
      const spread = Math.max(Math.abs(low), Math.abs(high), .01);
      const margin = low === high ? Math.max(spread * .1, .01) : spread * .08;
      low = Math.max(-Number.MAX_VALUE, low - margin);
      high = Math.min(Number.MAX_VALUE, high + margin);
    }
    const scale = Math.max(Math.abs(low), Math.abs(high), 1);
    const y = value => pad.top + (1 - (value / scale - low / scale) / (high / scale - low / scale)) * plotHeight;
    ctx.font = '11px system-ui, sans-serif';
    ctx.lineWidth = 1;
    for (let index = 0; index <= 4; index++) {
      const fraction = index / 4;
      const yy = pad.top + plotHeight * fraction;
      ctx.strokeStyle = '#e7edf0';
      ctx.beginPath(); ctx.moveTo(pad.left, yy); ctx.lineTo(pad.left + plotWidth, yy); ctx.stroke();
      ctx.fillStyle = '#596b77'; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
      ctx.fillText(axisNumber(high * (1 - fraction) + low * fraction), pad.left - 9, yy);
    }
    const tickCount = end > start ? (width < 450 ? 3 : 5) : 0;
    for (let index = 0; index <= tickCount; index++) {
      const at = tickCount ? start + (end - start) * index / tickCount : 0;
      ctx.textAlign = index === 0 && tickCount ? 'left' : index === tickCount && tickCount ? 'right' : 'center';
      ctx.textBaseline = 'top'; ctx.fillStyle = '#596b77';
      ctx.fillText(axisNumber(at) + ' s', x(at), pad.top + plotHeight + 11);
    }
    ctx.save();
    ctx.beginPath(); ctx.rect(pad.left - 3, pad.top - 3, plotWidth + 6, plotHeight + 6); ctx.clip();
    for (const series of group.series) {
      const current = prior(series, cursor);
      series.value.classList.remove('invalid');
      series.value.title = '';
      if (!series.visible) series.value.textContent = 'Hidden';
      else if (!current) series.value.textContent = 'No recent sample';
      else if (!valid(current)) {
        series.value.textContent = finite(current[1]) ? 'Invalid: ' + number(current[1]) : 'Invalid sample';
        series.value.classList.add('invalid');
      } else series.value.textContent = number(current[1]) + (series.unit ? ' ' + series.unit : '');
      if (current && series.visible) series.value.title = 'Sample at ' + seconds(current[0]);
      if (!series.visible) continue;
      let previous = null;
      ctx.strokeStyle = series.color; ctx.fillStyle = series.color; ctx.lineWidth = 1.8;
      for (const point of series.points) {
        if (point[0] < start) { previous = valid(point) ? point : null; continue; }
        if (valid(point)) {
          if (previous && point[0] - previous[0] <= series.gap) {
            ctx.beginPath(); ctx.moveTo(x(previous[0]), y(previous[1])); ctx.lineTo(x(point[0]), y(point[1])); ctx.stroke();
          } else if (point[0] <= end) {
            ctx.beginPath(); ctx.arc(x(point[0]), y(point[1]), 2.2, 0, Math.PI * 2); ctx.fill();
          }
          previous = point;
        } else {
          previous = null;
          if (finite(point[1]) && point[0] <= end) {
            const xx = x(point[0]), yy = y(point[1]);
            ctx.save(); ctx.strokeStyle = '#ba3329'; ctx.lineWidth = 1.8;
            ctx.beginPath(); ctx.moveTo(xx - 4, yy - 4); ctx.lineTo(xx + 4, yy + 4);
            ctx.moveTo(xx - 4, yy + 4); ctx.lineTo(xx + 4, yy - 4); ctx.stroke(); ctx.restore();
          }
        }
        if (point[0] > end) break;
      }
      if (current && valid(current)) {
        ctx.beginPath(); ctx.arc(x(cursor), y(current[1]), 3.5, 0, Math.PI * 2);
        ctx.fillStyle = series.color; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
      }
    }
    ctx.strokeStyle = '#172d3a70'; ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(x(cursor), pad.top); ctx.lineTo(x(cursor), pad.top + plotHeight); ctx.stroke();
    ctx.restore();
    if (!samples) {
      ctx.fillStyle = '#596b77'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('No samples in this time window', pad.left + plotWidth / 2, pad.top + plotHeight / 2);
    }
    group.plotBounds = {left: pad.left, width: plotWidth, start, end};
  }

  for (const group of groups.values()) {
    const card = make('article', 'card chart-card' + (group.panel.toLowerCase().includes('range') ? ' wide' : ''));
    const heading = make('div', 'chart-heading');
    heading.append(make('h3', '', group.panel), make('span', 'unit', group.unit || 'unitless'));
    const legend = make('div', 'legend');
    for (const series of group.series) {
      const button = make('button', 'legend-item');
      button.type = 'button'; button.setAttribute('aria-pressed', 'true');
      button.setAttribute('aria-label', 'Show or hide ' + series.label);
      const swatch = make('span', 'swatch'); swatch.style.backgroundColor = series.color;
      series.value = make('span', 'legend-value', '—');
      button.append(swatch, make('span', '', series.label), series.value);
      button.addEventListener('click', () => {
        series.visible = !series.visible;
        button.setAttribute('aria-pressed', String(series.visible));
        schedule();
      });
      legend.append(button);
    }
    const plot = make('div', 'plot');
    group.canvas = make('canvas');
    group.canvas.tabIndex = 0;
    group.canvas.setAttribute('role', 'img');
    group.canvas.setAttribute('aria-label', group.panel + (group.unit ? ', ' + group.unit : '') +
      ' over elapsed seconds. Click to move the cursor, or use the left and right arrow keys.');
    group.canvas.textContent = 'Time-series chart. Recorded values are available in the summary and downloaded tables.';
    group.canvas.addEventListener('pointerdown', event => {
      if (!group.plotBounds) return;
      const rect = group.canvas.getBoundingClientRect(), box = group.plotBounds;
      setCursor(box.start + clamp((event.clientX - rect.left - box.left) / box.width, 0, 1) * (box.end - box.start));
    });
    group.canvas.addEventListener('keydown', event => {
      if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') {
        event.preventDefault();
        setCursor(cursor + (event.key === 'ArrowLeft' ? -1 : 1) * Math.max(duration / 100, .01));
      }
    });
    plot.append(group.canvas);
    const foot = make('div', 'chart-foot');
    foot.append(make('span', '', 'Click chart to inspect · click a legend to hide a signal'), make('span', 'invalid-key', '\u00d7 invalid sample'));
    card.append(heading, legend, plot, foot);
    byId('charts').append(card);
  }
  if (!groups.size) byId('charts').append(make('div', 'card empty', 'No plottable telemetry was saved in this capture.'));

  function render() {
    frame = null;
    slider.value = String(cursor);
    byId('cursor-time').textContent = seconds(cursor) + ' / ' + seconds(duration);
    slider.setAttribute('aria-valuetext', seconds(cursor) + ' elapsed');
    const [start, end] = bounds();
    byId('window-label').textContent = 'Showing ' + seconds(start) + ' to ' + seconds(end);
    for (const group of groups.values()) draw(group);
  }
  function schedule() { if (frame === null) frame = requestAnimationFrame(render); }
  function setCursor(value) { cursor = clamp(finite(value) ? value : 0, 0, duration); schedule(); }
  function stop() {
    playing = false; lastPlayback = null;
    if (playbackFrame !== null) cancelAnimationFrame(playbackFrame);
    playbackFrame = null;
    byId('play').textContent = 'Play'; byId('play').setAttribute('aria-pressed', 'false');
  }
  function advance(now) {
    playbackFrame = null;
    if (!playing) return;
    if (lastPlayback !== null) setCursor(cursor + (now - lastPlayback) / 1000 * Number(byId('speed').value));
    lastPlayback = now;
    if (cursor >= duration) { stop(); return; }
    playbackFrame = requestAnimationFrame(advance);
  }
  byId('play').addEventListener('click', () => {
    if (playing) { stop(); return; }
    if (cursor >= duration) setCursor(0);
    playing = true; lastPlayback = null;
    byId('play').textContent = 'Pause'; byId('play').setAttribute('aria-pressed', 'true');
    playbackFrame = requestAnimationFrame(advance);
  });
  slider.addEventListener('input', () => setCursor(Number(slider.value)));
  byId('window').addEventListener('change', schedule);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
  window.addEventListener('resize', schedule);
  if (typeof ResizeObserver !== 'undefined') {
    const observer = new ResizeObserver(schedule);
    for (const group of groups.values()) observer.observe(group.canvas.parentElement);
  }

  const summaryBody = byId('summary-table').querySelector('tbody');
  for (const row of data.summary || []) {
    const tr = make('tr');
    const unit = row.unit ? ' ' + text(row.unit) : '';
    tr.append(make('td', '', row.label),
      make('td', 'number', number(row.valid_samples) + ' / ' + number(row.samples)),
      make('td', 'number', number(row.min) + (finite(row.min) ? unit : '')),
      make('td', 'number', number(row.max) + (finite(row.max) ? unit : '')));
    summaryBody.append(tr);
  }
  if (!summaryBody.children.length) {
    const row = make('tr'), cell = make('td', 'muted', 'No signal summaries are available.');
    cell.colSpan = 4; row.append(cell); summaryBody.append(row);
  }
  const download = (href, label) => {
    const link = make('a', '', label); link.href = href; link.download = ''; return link;
  };
  for (const file of data.files || []) {
    if (!file.href) continue;
    const item = make('li'); item.append(download(file.href, file.label || file.href)); byId('files').append(item);
  }
  if (!byId('files').children.length) byId('files').append(make('li', 'muted', 'No downloadable files were included.'));
  const entries = Object.entries(data.messages || {}).filter(([, count]) => finite(count) && count >= 0);
  entries.sort((a, b) => b[1] - a[1] || a[0].localeCompare(b[0]));
  let totalMessages = 0;
  for (const [name, count] of entries) {
    totalMessages += count;
    const row = make('tr'); row.append(make('td', '', name), make('td', 'number', number(count)));
    byId('message-table').querySelector('tbody').append(row);
  }
  byId('message-total').textContent = number(totalMessages) + ' total';
  if (!entries.length) {
    const row = make('tr'), cell = make('td', 'muted', 'No messages were recorded.');
    cell.colSpan = 2; row.append(cell); byId('message-table').querySelector('tbody').append(row);
  }
  const media = byId('camera-content');
  if (camera.video) {
    const video = make('video'); video.controls = true; video.preload = 'metadata';
    video.playsInline = true; video.src = camera.video;
    video.setAttribute('aria-label', 'Recorded camera video, independent of telemetry playback');
    video.addEventListener('error', () => {
      if (!media.querySelector('.video-error')) media.append(make('p', 'muted video-error',
        'This browser could not open the video. Download it to view with a compatible player.'));
    });
    media.append(video, download(camera.video, 'Download video'));
  } else {
    const previews = make('div', 'previews');
    for (const [field, label] of [['first_frame', 'First saved frame'], ['last_frame', 'Last saved frame']]) {
      if (!camera[field]) continue;
      const figure = make('figure'), img = make('img'); img.src = camera[field]; img.alt = label; img.loading = 'lazy';
      figure.append(img, make('figcaption', '', label)); previews.append(figure);
    }
    if (previews.children.length) media.append(previews);
    else media.append(make('p', 'muted', 'No camera preview or browser-playable video was included.'));
  }
  if (camera.video_note) media.append(make('p', 'muted', camera.video_note));
  render();
})();
</script>
</body>
</html>
"""
