**Forward MT-15 passthrough attempt — 7 September 2026**

**Superseded by the 14:18 CEST [diagram review and all-port recheck](mt15-all-ports-recheck.md).** The later check includes UART3 and all other hardware UARTs, identifies GPS on UART6, and records the newer battery state.

The user reports that an MT-15 is now installed facing forward and requested connection/configuration through the FC. This supersedes older project descriptions of a planned or absent forward sensor, but its physical UART and electrical connection have not yet been confirmed in this session.

The direct FlywooF745 USB connection worked and the aircraft remained disarmed. Before any passthrough attempt, the complete [live parameter backup](../../artifacts/mt15-configuration/20260907/fc-before.json) received all 1,172 parameters, SHA-256 `9e71d31617c2cdc105e22c9824fb72dd2c946d3ab783ebe130a78122f9246831`. It records `RNGFND1_TYPE=10`, downward orientation25, `RNGFND2_TYPE=0`, `SERIAL_PASS1=0`, `SERIAL_PASS2=-1`, `SERIAL_PASSTIMO=60`, and `ARMING_SKIPCHK=4`.

The pinned Flywoo build includes MAVLink SERIAL_CONTROL. This method leaves USB MAVLink available; raw SERIAL_PASS was not enabled. Flywoo's SERIAL numbering maps directly to hardware UART numbers, and SERIAL_CONTROL device numbers are100+SERIALn. The confirmed Pi UART4 and downward sensor UART5 were excluded.

| FC serial port | Existing configuration | Probe result |
| --- | --- | --- |
| 1 | MAVLink1, 115200 | No received bytes in the initial passive read or vendor identity probe |
| 2 | MAVLink2, 460800 | Read225 bytes; several valid RADIO_STATUS frames from system255/component68 identify telemetry-radio traffic. No sensor discovery commands were sent here |
| 3 | Tramp VTX, 115200 | Not probed |
| 4 | Pi MAVLink2, 115200 | Not disturbed |
| 5 | Downward sensor MAVLink1, 115200 | Not disturbed |
| 6 | GPS protocol, parameter baud57600 | Not probed |
| 7 | MAVLink1, 115200 | One zero byte in passive read; no received bytes in vendor identity probe |

The first scan used SERIAL_CONTROL RESPOND, timeout0, baudrate0, count0: no bytes were written to attached devices. The second bounded probe used the [manufacturer configurator's](https://micoair.cn/assistant/) normal sensor-identification heartbeat on ports1 and7 at their unchanged rates, with brief MAVLink consumer exclusivity. No radio-discovery fallback, settings SET, reset, firmware update, or baud change was sent. Its optional GET-settings branch could run only after a valid MT-15 identity; that branch was never reached. Release requests were sent for both ports, followed by a newly observed disarmed USB heartbeat.

The exact manufacturer's protocol was checked against its [public application JavaScript](https://micoair.cn/assistant/assets/index-B-zxJ0A-.js) and [MicoLink documentation](https://micoair.cn/zh/docs/sensors/micolink). MT-15 identity requires a checksum-valid device15/product16 heartbeat. Neither probe produced such an identity. Silence does not establish a broken or absent sensor: wiring, power, baud, output/interface mode, and a different UART remain possible causes.

**Result:** MT-15 not identified, no sensor settings read, no sensor or FC configuration written. All permanent values remain as found. The latest sampled battery voltage during this attempt was12.511 V, below the application's flight-test minimum. The pre-existing compass-check bypass was not altered.

**Information needed to proceed:** actual FC TX/RX pad names for the forward MT-15, and confirmation of regulated5 V power/common ground. An asynchronous question requesting these facts was sent while the independent checks ran; no answer had been received when this note was written. Do not guess another device's UART or configure a forward rangefinder from the downward sensor's packets.

Once the connection is identified, read and preserve the MT-15's complete settings, select its ArduPilot MAVLink output and forward orientation, retain or deliberately review baud/IDs/rate, and verify sensor readback before configuring the corresponding FC serial port and separate rangefinder instance. FC integration must be validated by fresh forward-orientation distance telemetry. Sensor configuration, reboot requirements, range limits, and persistence remain unverified and were not applied speculatively.

Evidence and prepared protocol helpers are in [the local MT-15 artifact directory](../../artifacts/mt15-configuration/20260907/): `passive-uart-scan.json`, `identity-probe.json`, `mico_probe_protocol.py`, `mt15-identify-serial-control.py`, and `mt15-protocol-notes.md`. The pure byte constructors were validated against the manufacturer's extracted pure JavaScript functions; no manufacturer app was run against hardware.
