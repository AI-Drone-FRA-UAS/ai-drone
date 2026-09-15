# Startup melody refinement with acoustic recordings

This records the first acoustic pass. The subsequent
[middle-note articulation update](startup-tone-articulation.md) describes the
current installed voices and identifies the confirmation sound after the melody.

The initial WAV 34 conversion was measurably sharp and fast. On 9 September
2026, the user supplied a recording, requested further startup-tone tests and
laptop-microphone analysis, and confirmed the props-off, battery-connected setup.
The revised melody is installed and independently read back on all four ESCs.
Subjective agreement with the soundtrack remains unconfirmed.

## Current configuration

Passthrough channels 0, 1 and 3 play the corrected bright melody. Channel 2
plays the same rhythm two octaves lower, adding fundamentals present in the
reference. Channel numbers are not a newly established physical motor order.
All voices retain the 16-slot phrase from the [initial installation](startup-tone.md),
with approximately 18 ms articulation gaps after sounded notes.

Beep volume remains 5. Full 1,024-byte EEPROM comparisons prove that only
offsets within the 128-byte melody field changed. The complete bootloaders and
firmware filenames match fresh before-test captures. Motor configuration,
firmware and bootloaders were not updated. One successful melody write was
performed per ESC; no rollback was required.

| Voice | Channels, zero-based | Melody SHA256 |
| --- | --- | --- |
| Corrected bright | 0, 1, 3 | `5e059cfd2a205ed7b41ab043bdec94c6e892750937b04d703169528a79b6cde1` |
| Lower fundamental | 2 | `077152c1f134d450ad2cd5be4b2511a15054df51928736a9b0ec492536de9d5e` |

Complete EEPROM SHA256 is
`6edf09d29eb445259144f8f6a5b092df44ff0720d84153319172719c3615cd0f`
for the bright voices, and
`ada9baafb9faf62179bb4280daa72f1a25531caa8cf15f0b2b6082d64a5b81d5`
for the lower voice.

## Acoustic evidence and limits

The root MP4 was decoded from 0:17 to its end into
`drone-music-currently-from-00m17s.wav`: 4.506271 seconds, 48 kHz, stereo PCM16.
No normalization, pitch shifting, filtering or time stretching was applied to
that extraction. The MP4 and all supplied references remain intact.

References 31–35 were inspected; reference 34 matches the original source hash.
Its spectrum includes fundamentals around 266, 355, 423, 400 and 532 Hz as well
as the stronger upper partials used by the first conversion. The previous
timer model underestimated measured pitch and overestimated duration. The new
conversion uses empirical factors 1.104 for pitch and 0.925 for duration;
the underlying clock/prescaler cause has not been proven.

| Measurement | Original recording | Revised ensemble | Reference target |
| --- | --- | --- | --- |
| Repeated bright C partial | 1,187.99 Hz | 1,064.94 Hz | 1,065.22 Hz |
| Tempo from repeated F onsets | 151.23 BPM | 138.79 BPM | 140 BPM |
| Median absolute pitch error, six sampled note positions | 154.82 cents | 16.30 cents | 0 cents |

These are FFT peak and onset measurements, not a subjective similarity score.
The comparison uses interior windows at six positions, rather than every
instant of every note. Three usable later recordings measured 138.79–139.32 BPM;
the very weak first repeat did not give a reliable onset/pitch estimate and is
explicitly marked inconclusive in `repeatability.json`.

Maintenance resets can trigger neighboring ESC melodies: these recordings
must be treated as ensembles, not isolated motor measurements. The final
configuration has coordinated voices, but a physical battery power cycle has
not been independently recorded. Motor harmonics, resonances and the room
remain audible; this cannot establish an exact reproduction of the original
synthesizer's timbre, stereo image or effects.

## Verification and cleanup

The existing probe/writer tests passed (25 tests). The same 11 writer tests
passed separately with each new exact tune, including baseline rejection,
false/missing acknowledgments, complete readback and verified rollback paths.
An independent melody decoder checked both 128-byte candidates. The writer's
only behavioral change is its exact accepted melody hash; raw-state checks,
the one-page write boundary and recovery behavior are retained.

ESC 4 rejected a bootloader read during the multi-channel session, before any
write to that ESC. Its standalone retry performed fresh identity and baseline
checks, completed one write, and verified the complete result. The final
independent capture again checked all four EEPROMs, bootloaders and filenames.

`SERVO_BLH_AUTO` was restored to 0 and the disarmed FC rebooted. Post-reboot
readback confirmed `SERVO_BLH_AUTO/MASK/TEST=0`, DShot600, the unchanged motor
mapping and `ARMING_SKIPCHK=0`. A fresh disarmed STABILIZE heartbeat and 15.136 V
battery telemetry were received. Microphone recording stopped and USB was
closed. No arm, throttle, motor-test, servo or flight-mode commands were sent.

## Local evidence

The ignored directory `artifacts/tone-refinement-20260909/` contains:

- `listen.html`, `before-recorded.wav`, `after-recorded.wav`, and
  `reference34-matched.wav`: actual recordings and a volume-matched comparison.
- `comparison.json`, `reference-pitches.json`, and `repeatability.json`:
  measurements with their stated limits.
- `baseline-esc/`, `final-esc/`, and `final-verification.json`: complete fresh
  before/after captures, identities and independent preservation checks.
- `corrected-high/` and `corrected-fundamental/`: exact candidates, plans,
  sine previews and bounded writers. Sine previews do not simulate motor timbre.
- `*-write/` and `*-recording/`: write/readback logs and microphone captures;
  `fc-*.json`: setup, cleanup and post-reboot state.
- Preparation, capture, comparison and independent verification scripts.

The `baseline-esc/` images preserve the previously installed melody for recovery.
Restoring them requires the same reviewed single-sector write procedure and
fresh live-state checks; it was not necessary during this run.
