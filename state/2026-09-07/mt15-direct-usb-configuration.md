# MT-15 direct USB configuration — 7 September 2026

The user connected the forward MT-15 through a CP2102 USB-to-UART adapter and explicitly requested checking and configuring it. The adapter appeared as `/dev/ttyUSB0`, stable path `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`. Direct communication worked at 115200 baud. A checksum-valid device15/product16 heartbeat identified MT-15 hardware10, firmware1.1.4.

Original settings were backed up before sending SET1. Only two fields changed:

| Setting | Before | After |
| --- | --- | --- |
| Range orientation | 9: downward | 0: forward |
| Sensor ID | 0 | 1 |

Fresh GET1 readback verified both changes and byte-for-byte preservation of unrelated parameter bytes. Existing output protocol2 (MAVLink/APM), UART interface0, baud index0 (observed communication115200), range frequency index4 (50Hz), query mode0, MAVLink system1/component88, and other fields remained unchanged. The project's downward sensor previously transmitted sensor ID0 and orientation25. The MT-15 saved sensor ID1, but its native MAVLink output still uses ID0; see verification below.

After host configuration heartbeats stopped, an eight-second sample still contained MicoLink configuration-format range frames, not MAVLink. It contained417 checksum-valid range frames, all sensor ID1, with distances105–1878mm and about52Hz observed rate. These demonstrate live measurement output, not calibrated accuracy.

The user then physically power-cycled the sensor. An eight-second passive capture, taken before any new configuration heartbeat, decoded405 valid MAVLink DISTANCE_SENSOR messages from system1/component88. All had forward orientation0. Distances were164–168cm, with approximately52.04Hz timestamp-derived rate and advertised limits2–1500cm. Those advertised limits are not a calibrated range test. Two BAD_DATA fragments and one UNKNOWN_88 decode were excluded from conclusions; acquisition began midstream.

Fresh identification after passive capture confirmed firmware1.1.4 and lower device uptime; GET1 returned exactly the saved parameter bytes, verifying persistence across the power cycle. A firmware discrepancy remains: configured sensor ID1 appears in MicoLink range frames, but native MAVLink DISTANCE_SENSOR.id stays0. Thus the combined orientation-and-ID stream assertion is false, while forward MAVLink output and persistence are verified. The local ArduPilot `AP_RangeFinder_MAVLink::handle_msg` matches readings by orientation, not packet ID, so this does not by itself prevent separate forward/downward rangefinder instances. Live FC integration is still required.

The final GET1 required configuration heartbeats, so power-cycle the sensor when moving it back to the FC to restore its demonstrated native startup output. The host serial descriptor is closed.

No flight controller was connected during this operation; FC wiring, UART configuration, and forward rangefinder integration remain unverified. No FC parameters or actuator commands were sent.

Evidence and scripts: [direct USB artifacts](../../artifacts/mt15-configuration/20260907-usb/). Protocol implementation follows the previously inspected manufacturer MicoAssistant asset, documented in [protocol notes](../../artifacts/mt15-configuration/20260907/mt15-protocol-notes.md). The manufacturer's [guide](https://micoair.cn/en/docs/software-tutorial/micoassistant-guide) recommends reading settings back after saving, with a power cycle when needed.
