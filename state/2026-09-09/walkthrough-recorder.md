# Timed disarmed walkthrough recording verification

The Pi now has `drone-walk` and `drone-report`. The existing `drone-inspect`
already records orientation-labelled MT-15 forward distances alongside the
MTF-01P downward range and optical flow. The new launcher keeps a timed
recording alive independently of SSH and generates an offline browser report
with full CSV exports afterward. See the
[operating instructions](../../docs/SENSOR_RECORDING.md#timed-room-walkthrough).

## Live bench verification

A 10-second recording ran on `seb-is-pm` from **11:07:56 to 11:08:06 UTC**
(13:07:56–13:08:06 CEST). The launching SSH session returned while the temporary
system-manager service continued. The Pi used `/dev/serial0` at 115200 baud;
the laptop reached the Pi over Tailscale, not USB.

| Observation | Recorded result |
| --- | --- |
| Actual capture duration | 10.000974 seconds; completed without error |
| Forward MT-15 | 199 readings, 1.51–1.58 m |
| Downward MTF-01P | 199 readings, all reporting 0.02 m |
| Optical flow | 200 messages, quality 49–73 |
| Camera | 299 analyzed frames, 301 encoded frames; MP4 report copy generated |
| FC telemetry | 1,599 messages, including 10 disarmed heartbeats |
| Additional data | Attitude, raw IMU/magnetometer, barometer, battery, local EKF position, status and output telemetry |
| Battery pack | 13.819–13.824 V during capture; percentage/cell calibration not established |
| Actuation | No arm, mode, RC, motor, throttle, mission or servo commands |

The distances above are the reported values in the stationary bench setup,
within each packet's advertised limits. This does not establish physical
accuracy or valid flight clearance at 2 cm. A user room walk has not yet been
performed. The temporary unit finished normally and released the serial port;
no walkthrough service is enabled at boot.

## Reports and timing

The Pi dataset is `/home/seb/ai-drone/artifacts/walk-bench-20260909/`.
The local copy and generated browser report are under
`artifacts/walkthrough-20260909/walk-bench-20260909/review/`.
Raw recordings and local verification artifacts are intentionally not committed.

The report contains 398 range rows, 200 flow rows, 100 IMU rows, 300 motion
rows, 100 environment rows, 20 battery rows, 110 event rows and 299 camera rows.
Raw JSONL/tlog files retain messages outside these derived exports.

Charts use elapsed capture time and preserve gaps and invalid readings.
Full CSVs retain selected samples while charts use a bounded envelope.
Range orientation, rather than sensor ID alone, identifies forward and down.
The downward FC limit remains 1 m; higher reported values are retained but
flagged as outside its configured range. No FC parameters or firmware changed.

Camera video has independent playback controls. Its MP4 uses an approximate
constant frame rate estimated from the encoder timestamps. It is not precisely
synchronized to telemetry receipt; original H.264, PTS and camera metadata
remain available. The optional Pi ffmpeg package was installed to produce this
browser copy without re-encoding or adding Python dependencies.

## Software verification

- 658 offline tests passed; 2 optional tests skipped and 3 SITL tests deselected.
  This change does not alter flight-control logic; no live armed test was run.
- Ruff formatting/lint, type checking, import contracts, dependency checking,
  offline lock verification, whitespace checks and the 18-page docs build passed.
- Eight isolated Chromium interaction checks passed on a real earlier recording:
  charts/cursor, time window, signal visibility, playback, camera metadata and
  narrow-screen layout. No JavaScript exceptions or external page requests.
- All 56 deployed runtime files were hash-verified; previous Pi runtime files
  were backed up before deployment. Current provenance is in the Pi's
  `artifacts/deployment/current.json`.

Recorder status also now expires stale flow and heartbeat observations after
two seconds. Last values remain in raw logs and manifests, but stale live
numbers are hidden and an expired heartbeat is shown as unknown vehicle state.
The ordinary recorder's armed-start refusal and armed-transition abort remain
in effect. The walkthrough launcher never supplies the manual-flight exception.
