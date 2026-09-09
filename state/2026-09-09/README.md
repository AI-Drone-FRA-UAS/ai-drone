# 9 September 2026

The [forward MT-15 integration](mt15-integration.md) is verified after the
user swapped two data wires and UART3's receive/transmit pin functions were
swapped in software. The FC and Pi now report forward distances separately
from the MTF-01P's downward range and optical flow.

The complete [parameter snapshot](../../params/flywoo-f745-live-2026-09-09.param)
contains 1,187 parameters; [metadata](drone-config.json) records its time,
source and checksum. The drone remained disarmed throughout live maintenance.

The [walkthrough recorder](walkthrough-recorder.md) captures timed sensor and
camera datasets. The [software update](software-update.md) records Pi
package and uv upgrades and the original firmware candidate preparation.

The subsequent [firmware and power update](firmware-and-power.md) records the
installed custom ArduCopter 4.7.1 image, matched simulation, live acceptance
and new connection/shutdown commands. The [startup melody](startup-tone.md)
was adapted from WAV 34, saved and replayed on all four ESCs with propellers
removed. The compass pre-arm warning still requires physical investigation.
