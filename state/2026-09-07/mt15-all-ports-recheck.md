**Forward MT-15: diagram review and all-UART check — 7 September 2026, 14:06–14:18 CEST**

The direct FlywooF745 USB connection worked throughout the checks. Target 1/1 reported disarmed; USB parameter request/reply and fresh disarmed heartbeat were verified again after passthrough release. The MT-15 was **not identified or configured**. The user reports it is powered, and that removed FPV camera/VTX wiring was reused. The electrical path of its actual TX/RX wires remains unverified.

The complete pre-check parameter download contains all 1,172 entries: [fc-before-all-ports.json](../../artifacts/mt15-configuration/20260907/fc-before-all-ports.json), SHA-256 `26bbb213d61785b6944fd092c700149e3e034f42a8c6f6a6f3b487fee61b78dd`. This capture belongs to the project drone. No different-drone reference data was used.

**What the wiring diagram establishes**

The user's [drone diagram](../../../drone-sensor-diagram.png) shows the front analog connector as `CAM / 5V / GND / T3 / VTX / VBAT / GND`. It has TX3 but **no RX3**. `CAM` and `VTX` are analog video signals. The old VTX control wire goes to T3; its video wire goes to VTX. Reusing those as a sensor TX/RX pair would leave no UART receive path to the FC. This is a likely explanation, not proof of the actual rewiring.

The opposite board face's six-pin DJI connector provides `VBAT / GND / TX3 / RX3 / GND / RX1`. This matches the [official ArduPilot V3 pinout](https://ardupilot.org/copter/_images/flywooF745AIOv3Connections.jpg). The diagram's old UART1/DJI label is not a reliable guide to this board's serial mapping. The photographs obscure the FC connectors and cannot establish the current MT-15 wire routing.

With battery and USB disconnected, the data connections to inspect/correct are **MT-15 TX → FC RX3** on the DJI connector and **MT-15 RX → FC TX3**, with a common ground. Use the printed signal names rather than presumed wire colours. Verify regulated 5 V at the sensor supply; do not confuse the FC's VBAT pin with its 5 V output. The old diagram includes a separate 5 V regulator for the VTX, so it does not prove the sensor is presently connected to raw battery voltage.

**Live port results**

All seven hardware UARTs were read through MAVLink SERIAL_CONTROL; USB remained MAVLink throughout. The first receive windows lasted 1.5 seconds each. Queued data can predate a window, so byte counts are not reliable estimates of stream rates.

| Port | Live configuration | Observed result |
| --- | --- | --- |
| USB / SERIAL0 | MAVLink2, 115200 setting | Working FC heartbeat, battery, parameter and MAVFTP responses |
| SERIAL1 | MAVLink1, 115200 | Zero received bytes; FC UART diagnostics also show RX=0 |
| SERIAL2 | MAVLink2, 460800 | 2,584 bytes; 136 valid RADIO_STATUS packets from system255/component68, consistent with the known telemetry radio |
| SERIAL3 | Tramp protocol44, baud115200, VTX_ENABLE=0 | Zero received bytes at all eight tested rates; no response to the manufacturer's sensor heartbeat at115200 |
| SERIAL4 | Pi MAVLink2, 115200 | Zero received bytes during this short window. FC diagnostics show earlier RX activity; the Pi-to-FC connection was verified earlier today, not retested actively here |
| SERIAL5 | Downward sensor MAVLink1, 115200 | 8,446 bytes; 151 OPTICAL_FLOW and150 DISTANCE_SENSOR packets from system1/component88. Sample distances2 cm, orientation25/down, flow quality58–61. These are the downward sensor, not the forward MT-15 |
| SERIAL6 | GPS protocol5, configured57600; GPS1_TYPE=GPS2_TYPE=0 | Signal initially undecodable. At230400 baud, 14 checksum-valid UBX GPS messages:6 NAV-PVT,6 NAV-DOP,2 NAV-TIMEGPS. All PVT samples report no fix and0 satellites used. GPS is connected and transmitting, but disabled in FC configuration |
| SERIAL7 | MAVLink1, 115200 | Zero bytes in this scan. FC statistics show one historical received byte and one framing error; no identifiable device |

SERIAL3 receive checks covered9600,19200,38400,57600,115200,230400,460800 and921600 baud. SERIAL6 receive checks covered those rates plus420000. SERIAL1/2/4/5/7 were read at their existing rates; no baud sweep or sensor query was sent to the known Pi, radio, or downward sensor. Silence does not exclude an I2C-configured or query-only sensor, disconnected receive wire, or another electrical fault.

The [FC's own UART diagnostics](../../artifacts/mt15-configuration/20260907/fc-uarts.txt), read at14:17:45, independently show SERIAL3 TX=80 and RX=0, with zero framing/noise/overrun errors. The80 transmitted bytes are the four vendor identification heartbeats sent in this check. SERIAL6 has many framing/noise errors, consistent with the configured57600 baud differing from its observed230400 GPS stream. Diagnostic TXBD/RXBD fields are measured traffic rates, not configured baud values.

The6 NAV-PVT samples advance by200 ms, consistent with5 Hz GPS output. Their date/time validity flags are clear, so the encoded2021 date is not valid clock evidence. No valid MT-15 identity or MicoLink range frame was found. MAVLink UNKNOWN_* detections in wrong-baud SERIAL6 bytes are discarded as false protocol recognition.

**Changes and recovery**

No persistent FC parameter writes, sensor SET commands, reboot, arm/mode/RC/motor/servo commands, or raw SERIAL_PASS changes were sent. The manufacturer's [MicoAssistant](https://micoair.cn/assistant/) heartbeat was sent only on SERIAL3 in this recheck, at115200. Its settings-GET branch requires a valid device15/product16 MT-15 heartbeat and was never reached.

SERIAL_CONTROL temporarily locked MAVLink consumers while reading each port. Every lock was released. Temporary baud changes were restored to SERIAL3=115200 and SERIAL6=57600. VTX_ENABLE=0 and the pinned SerialManager initialization support115200 as the UART3 baseline; disabled Tramp does not override it to9600. GPS remains disabled and its original configuration was preserved despite discovering its actual230400 stream. Final USB parameter responses reconfirmed SERIAL3_PROTOCOL=44 and SERIAL6_PROTOCOL=5; fresh disarmed USB heartbeats followed the tests.

The first scan's final UART6 logging step encountered an unsupported bytearray in a false UNKNOWN_* MAVLink decode. Its artifact preserves all14 preceding completed windows, including all seven ports and all eight UART3 rates. UART6 was subsequently recaptured with corrected serialization; both follow-up scripts completed and verified release/USB communication. This logging issue is not a sensor failure.

**Battery and remaining work**

At14:18:31 CEST, the analog battery monitor reported **11.914 V,0.94 A**,1041 mAh consumed and68% estimated remaining. At14:15:57 it had reported12.012 V. The percentage is a consumed-charge estimate, not independent state-of-charge evidence; individual cell voltages and calibration were not measured. The earlier commentary's four-cell description was an assumption, not a cell-count measurement. The voltage is below the application's14.4 V flight-test guard. Powered testing ended, and the user was asked to disconnect the battery and recharge before further powered checks.

The MT-15 requires a verified UART receive connection and a valid device identity before configuration can be completed. After correcting/confirming RX3/TX3 and sensor power, repeat identification, preserve the returned sensor settings, configure ArduPilot output and forward orientation, and validate a separate forward distance stream before enabling a second FC rangefinder. A powered LED and a successful FC USB connection do not establish MT-15 communication.

Raw captures and the exact diagnostic scripts are in [the MT-15 artifact directory](../../artifacts/mt15-configuration/20260907/): `all-uart-passive.json`, `uart3-identity-probe.json`, `uart6-passive-followup.json`, `uart6-baud-scan.json`, `uart6-offline-decoding.json`, and `fc-uarts.txt`. This report supersedes the narrower earlier [ports1/7 passthrough attempt](mt15-passthrough-attempt.md).
