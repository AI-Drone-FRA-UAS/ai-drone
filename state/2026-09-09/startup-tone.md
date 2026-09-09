# ESC startup melody — 9 September 2026

All four ESCs now contain the same approximately 3.429-second melody derived
from the user's `34_fuller_less_ambience.wav`. Each complete 1,024-byte EEPROM
was read back and compared with its original: precisely 59 bytes changed,
all inside the startup-melody field. Beep volume, motor settings, firmware and
bootloader were preserved. All four ESC resets were acknowledged, and both
write sessions ended with a fresh disarmed flight-controller heartbeat.

The user confirmed that the propellers were physically removed and identified
the unwanted sound as the ESC startup sound. The FC remained disarmed; the
melody uses motor windings as sound transducers. No arm, throttle, motor-test
or servo command was sent during this operation. User confirmation of the new
sound's musical fidelity is still pending; EEPROM verification establishes the
saved notes, not their subjective sound or an independently recorded power cycle.

## Sound and conversion

The unchanged original is
`/home/abaris/Music/drone-tone/round5/34_fuller_less_ambience.wav`: 44,100 Hz,
stereo PCM16, 151,200 frames, or 3.428571 seconds. Its SHA256 is
`0307a38564b91475a5e3f9325462909c5d2de3d86ea9ee565bc332e932431d7d`.

The conversion approximates the dominant pitched components as sixteen eighth
notes/rests at 140 BPM:

```text
F6 C6 rest C6 G#6 C6 rest C6 G6 C6 C6 rest F6 C6 C7 rest
```

An ESC plays a monophonic electrical melody; it cannot reproduce the WAV's
stereo image, ambience or synthesizer texture. The selected pitches follow
measured dominant components, which need not be the original fundamentals.
The final 128-byte melody uses 62 bytes and models a 3,429 ms duration.
Pitch selection compensates for the installed AM32 2.16 / AT32F421 timer
implementation, using the matching
[AM32 source](https://github.com/am32-firmware/AM32/blob/d621bf2a2be1ed4b4ac9597df089f0727edbc67c/Src/sounds.c).
Predicted pitch quantization is approximately 8–37 cents; this is a source-based
estimate, not an acoustic measurement. The earlier generic RTTTL candidate is
retained as analysis only and is not the final installed byte sequence.

The installed melody SHA256 is
`27d0685ef29b172bc9c6e625b03f6a936a85f6ba7feee8cbc3bd344d329d0d85`.
Beep volume remains **5**. The FC buzzer configuration and warning sounds were
not changed. No speaker or audio service was added to the Raspberry Pi.

## Installed ESC identity and write boundary

| Field | Verified on each ESC |
| --- | --- |
| Firmware target | `F4A_4IN1_F421` |
| MCU family | AT32F421 |
| AM32 firmware | 2.16 |
| EEPROM layout | 2 |
| EEPROM bootloader revision byte | 1; retained unchanged |
| Four-way identity | `06 1f 02 04`; `0x1f06` is a compatibility signature |
| Actual bootloader | Exact binary match to F421 PA2 V4 |
| Settings region | Wire address `0x7c00`, 1,024 bytes |
| Melody field | Offsets 48–175 inclusive within that region |
| Firmware filename | `0x7be0`, 32 bytes, NUL-padded `F4A_4IN1_F421` |

The full 4,096-byte bootloader capture from every ESC matches both the
[official F421 PA2 V4 HEX](https://github.com/AlkaMotors/AM32_Bootloader_F051/blob/d2a455bbaab6eff7597c2a53317a080778cc1c6e/Bootloaders/AM32_F421_PA2_BOOTLOADER_V4.hex)
and the
[historical source build](https://github.com/AlkaMotors/AT32F421_AM32_Bootloader/blob/922493dd0e54bae1c92cecdd9fd5472ce099dd21/Objects/Am32.hex),
expanded at `0x08000000` and padded with `FF`. Its SHA256 is
`4ff5d1fcd232c268923e150cbed3ac11e4965d26fd96ab85ff920b1d4414a820`.
This binary comparison establishes the bootloader version; the EEPROM revision
byte and compatibility signature alone do not.

Independent source, binary and Artery device-pack inspection confirmed a
1,024-byte erase sector and a maximum 256-byte programming buffer for this
exact bootloader/device. The original bytes 256–1023 were all `FF`. Therefore
one `DeviceWrite` (`0x3b`) of the original first 256 bytes with only the melody
replaced, at `0x7c00`, reconstructs the whole settings sector correctly after
its implicit erase. No standalone erase, firmware write or bootloader update
was performed. This procedure is specific to the verified images and must not
be generalized to another MCU, bootloader or settings layout.

Immediately before each write, the writer required that ESC's entire original
EEPROM, bootloader and filename to match its immutable backup. After the single
write it verified the entire EEPROM, bootloader and filename again. Readback
was mandatory even with an ACK, because this bootloader/bridge can acknowledge
without proving successful flash programming. No rollback was needed.

| Result | ESC 1 | ESC 2 | ESC 3 | ESC 4 |
| --- | --- | --- | --- | --- |
| Zero-based passthrough channel | 0 | 1 | 2 | 3 |
| Candidate writes | 1 | 1 | 1 | 1 |
| Complete EEPROM verified | Yes | Yes | Yes | Yes |
| Non-melody bytes unchanged | Yes | Yes | Yes | Yes |
| Bootloader and filename unchanged | Yes | Yes | Yes | Yes |
| Device reset acknowledged | Yes | Yes | Yes | Yes |

Channel numbers identify passthrough channels, not a newly inferred physical
motor order. Existing output functions `34, 35, 36, 33` were retained.
All original EEPROM images have SHA256
`dbedbd588f663975545adf39e6136ff506a2f02645709ddfa96da36fde608846`;
all final images have SHA256
`69c1188017f80f7f27df5d7159fcfe6fca27ed7d85ba70b39c3b0ef35ce5f350`.
Exactly 59 offsets changed, between 48 and 108 inclusive.

## Temporary passthrough and recovery

`SERVO_BLH_AUTO` was temporarily set from 0 to 1 after a parameter backup and
disarmed FC reboot. `SERVO_BLH_MASK=0`, `SERVO_BLH_TEST=0`, the motor mapping and
DShot600 remained unchanged. After all four tunes were verified,
`SERVO_BLH_AUTO=0` was restored and read back successfully. The subsequent
[FC firmware update and reboot](firmware-and-power.md) also removed the temporary
passthrough handler from the running configuration.

The first write attempt and a subsequent read-only retry timed out at the MSP
API query, before entering ESC access or sending any ESC write. Normal FC
telemetry continued. A disarmed FC-only reboot restored the handshake, after
which the first-channel pilot and remaining three writes succeeded.

Source inspection gives a plausible mechanism: while temporary passthrough is
enabled, an idle link feeds bytes to both the MAVLink and alternative parsers.
An incoming MAVLink packet ending with `/` can leave the alternative parser in
a partial four-way frame; recognizing the valid MAVLink packet does not clear
that partial state. The first `drone-check` version-request packet reproduces
that terminal byte offline, and subsequent MSP bytes can then be consumed as
an incomplete frame. This matches the observed stall, but the preceding live
request was not captured, so it is not a proven diagnosis of that occurrence.
No maintained runtime workaround was added for this temporary maintenance mode.

A separate protocol requirement was verified during cleanup: after a successful
`InterfaceExit`, waiting 4.1 seconds and sending one fixed, non-actuating GCS
heartbeat resumes MAVLink handling. Passive receipt alone does not clear the
alternative-active state. Both successful write sessions performed that step
and verified a fresh FC system 1/component 1 disarmed heartbeat.

## Evidence and recovery material

The following are **local, ignored maintenance artifacts**, retained under
`artifacts/startup-tone-20260909/`; they are not shipped with the tracked source:

- `esc-second-read/`: complete original settings, repeated first-block reads,
  identities and filenames for all four ESCs.
- `esc-bootloader-read/`: complete ESC 2–4 bootloader captures;
  `esc1-bootloader-read/`: the separate complete ESC 1 capture.
- `candidate-v216/`: final tune bytes, per-ESC candidate images, a quiet sine
  preview and `plan.json`. Its “offline candidate” status records preparation;
  the live completion records are the separate write summaries below.
- `write-esc1-after-fc-reboot/` and `write-esc2-4/`: successful write summaries,
  complete immediate-before/after images and raw serial/event logs.
- `write-esc1/` and `esc1-after-msp-timeout/`: unsuccessful handshake evidence;
  neither attempt wrote an ESC.
- `analysis/BOOTLOADER-WRITE-AUDIT.md` and
  `analysis/legacy-f421-write-audit.json`: independent identity, erase-boundary,
  raw-preservation, rollback-model and writer review.
- `prepare_v216_melody.py`, `esc_readonly_probe.py` and
  `esc_melody_writer.py`: the exact offline conversion and bounded maintenance
  tools. Probe tests: 14 passed; writer tests: 11 passed, also independently
  rerun. The writer SHA256 is
  `98cc9def81af24c2a81efb11fbabd343e160c3b36a7e58a1fbaeab24ac0a36c0`.

Passthrough restoration is recorded separately in
`artifacts/tone-20260909/passthrough-restore.json`.

The original per-ESC first 256 bytes can reconstruct its original full sector
with the same verified bootloader and erased tail, followed by complete
EEPROM/identity readback. That rollback was modeled and tested offline, but
was not needed live. Retain the original full backups before any later change;
the underlying erase/program operation is not atomic across power loss.
