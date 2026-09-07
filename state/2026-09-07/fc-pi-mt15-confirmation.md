**FC, Raspberry Pi and MT-15 reconnection check — 7 September 2026, 15:41–15:46 CEST**

The **FC USB connection and Pi-to-FC UART connection both work**. The FC remained disarmed. The forward MT-15's connection to the FC is **not confirmed**: no incoming bytes were observed on UART3. Its successful [direct USB configuration](mt15-direct-usb-configuration.md) remains valid evidence about the sensor itself, but does not establish the subsequent FC wiring or integration.

**Connections and battery**

- Direct USB: FlywooF745 `1209:5741`, serial `200023000451333436353531`, `/dev/ttyACM0`; stable by-id path ends `-if00`. Fresh FC target1/1 heartbeats, parameter replies, battery and other requested messages prove bidirectional communication. Firmware reports ArduCopter4.7.0 custom `1511f271`.
- Raspberry Pi: SSH to `seb@192.168.4.1` succeeded through AI-Drone-Zero. Its `/dev/serial0` maps to `ttyAMA0`. Fresh parameter replies and requested telemetry over this port confirm the Pi↔FC UART4 connection in both directions. USB Ethernet at192.168.7.2 and Tailscale SSH did not respond; both attempts ran outside the restricted sandbox.
- The laptop's Wi-Fi was restored to eduroam after the Pi check. SSH and serial descriptors were closed; the report confirms tested connectivity rather than an ongoing background session.
- Battery at15:46:36: **16.132 V,0.86 A**,92 mAh consumed, estimated97%. Earlier USB/Pi samples agreed at16.132–16.133 V. This is total pack voltage from the analog monitor; no individual cell measurements or independent state-of-charge verification were performed.
- Pi temperature53.7°C; `get_throttled=0x0`. Its clock remains incorrect, reporting September1; the host-side metadata establishes the actual capture time. The auxiliary `pinctrl get 14 15` invocation returned a syntax error, explaining the health command's nonzero exit; SSH and the subsequent MAVLink probe succeeded.

**Current MT-15 and sensor results**

The full [pre-check FC snapshot](../../artifacts/mt15-configuration/20260907-fc-integration/fc-before.json) contains all1,172 parameters, SHA-256 `036c89758d15f56b71a1fff4f59af44b0045a1388401a177b5a4362a84cf4be6`. At the start of this check, `SERIAL3_PROTOCOL=1`, `SERIAL3_BAUD=115`, `SERIAL3_OPTIONS=0` were already set. `RNGFND2_TYPE=0` still disables the forward rangefinder; a named parameter request returned `RNGFND2_ORIENT=0`. These settings were read, not written in this turn.

| Port | Receive-only result |
| --- | --- |
| UART1 | Zero incoming bytes |
| UART2 | 2,603 bytes;137 valid RADIO_STATUS frames from system255/component68 |
| UART3 | Zero incoming bytes in a2-second direct probe and a separate1.5-second all-port scan at its configured115200 baud |
| UART4 | Pi↔FC requests/replies verified over SSH; quiet during the later passive scan after the Pi probe had closed |
| UART5 | 5,567 bytes containing347 checksum-valid MSPv2 packets:174 optical-flow and173 range packets |
| UART6 | 998 undecoded bytes at its configured57600. GPS was positively identified at230400 earlier today; this check did not repeat that baud sweep |
| UART7 | One zero byte; no identifiable stream |

No MT-15 forward MAVLink stream appeared in these captures. [FC UART statistics](../../artifacts/mt15-configuration/20260907-fc-integration/fc-uarts.txt), read at15:46:43, independently show UART3 TX=6860 but RX=0, with zero framing/noise/overrun errors. This remains consistent with a missing receive connection. The known-native115200 MT-15 output should reach **FC RX3 from MT-15 TX**. The old analog VTX connector has TX3 and video but no RX3; RX3 is on the separate DJI connector, as established in the [diagram review](mt15-all-ports-recheck.md). The actual rewired pins have not been physically traced.

The downward sensor's valid MSP range payloads report20–34 mm; raw optical-flow quality43–65/255. One CRC-invalid candidate was excluded. These packets have no model/orientation/timestamp fields and do not identify the forward sensor. No native MAVLink range/flow packets were found on UART5 in this capture.

Both USB and Pi received live gyro, accelerometer, compass and pressure samples, with those health bits set. Optical-flow and laser-position health bits were clear, and neither connection received an FC DISTANCE_SENSOR or OPTICAL_FLOW response during repeated requests. Pre-arm health was also clear; RC reports zero channels. The existing `ARMING_SKIPCHK=4` compass-check bypass remains set. These observations do not establish flight readiness.

**Why the downward sensor is not integrated at this boot**

The confirmed firmware revision's [GCS header](../../../ardupilot/libraries/GCS_MAVLink/GCS_MAVLink.h) selects five MAVLink channels for this1 MB target. Startup allocates them in serial order. Current settings assign those five to USB and UART1/2/3/4, leaving UART5 without a MAVLink backend. UART5 remains initialized as a hardware UART and can be read through SERIAL_CONTROL. Its TX=0 in the FC statistics, while UART3 transmits, corroborates this allocation.

The live mapping is MAV1→USB, MAV2→UART1, MAV3→UART2, MAV4→UART3, MAV5→UART4. MAV2/4/5_OPTIONS=2 make those channels private for MAVLink forwarding; this does not prevent addressed Pi requests to the FC or explain the zero raw UART3 RX count.

UART5 currently emits MSPv2, while its configured FC protocol is MAVLink1. The [manufacturer documentation](https://micoair.cn/en/docs/software-tutorial/micoassistant-guide) describes an Auto protocol option. A fallback to MSP when FC heartbeats disappear is a plausible explanation, but the sensor's saved setting and selection logic were not read or tested here. Do not infer that someone permanently reconfigured the downward sensor. The matching local generated Flywoo build enables base MSP but disables `HAL_MSP_RANGEFINDER_ENABLED` and `HAL_MSP_OPTICALFLOW_ENABLED`, so changing UART5 to MSP alone would not restore range/flow in that build. These compile flags were inspected locally, not independently queried from the running FC.

**Remaining integration work**

First verify the MT-15 TX→RX3 data path and power-cycle after any configuration-session use, as required by the direct-USB verification. Sensor settings and ID0 quirk were not changed or re-probed here.

After valid forward packets reach the FC, integration needs a free MAVLink slot and an enabled second rangefinder. A concrete candidate is disabling the proven-unused UART1 (`SERIAL1_PROTOCOL=-1`), keeping UART3 at MAVLink115200, and enabling `RNGFND2_TYPE=10`, orientation0, minimum0.02 m and maximum15 m from the sensor's advertised bounds. Remapping requires reviewing MAV2/3/4/5_OPTIONS for their new UART2/3/4/5 roles; radio/Pi public and sensor ports private would be0/2/0/2. Startup-only allocation requires a disarmed reboot. This is a proposed correction, not an applied or verified configuration.

Then verify FC-origin system1/component1 distance reports for forward0 and downward25 separately, plus continuing optical flow and the Pi link. Both sensors' native system1/component88/ID0 values do not by themselves prevent backend separation: this ArduPilot rangefinder handler matches orientation. Recovery of the downward sensor's MAVLink output must also be verified rather than assumed.

No PARAM_SET, sensor-setting writes, reboot, arm, mode, actuator or mission commands were sent. No baud settings were changed in this check. Every temporary SERIAL_CONTROL lock was released, and fresh disarmed USB heartbeats plus parameter responses were verified afterwards.

Evidence: [integration artifact directory](../../artifacts/mt15-configuration/20260907-fc-integration/), including `usb-live.json`, `pi-live.json`, host clock/network metadata, `all-uart-passive.json`, `fc-uarts.txt`, `offline-decoding.json`, and the exact diagnostic scripts. Different-drone reference data was not used.
