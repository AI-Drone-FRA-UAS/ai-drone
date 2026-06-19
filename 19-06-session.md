# Development Session — 19 June 2026

This document records the complete MAVLink, Raspberry Pi UART, and MTF-01P
bring-up performed on 19 June 2026. It includes the commands that were run,
the failures encountered, how each failure was diagnosed, the fixes applied,
and the final verified state.

## Final result

At the end of the session:

- The developer machine communicates with the FlywooF745 flight controller
  over USB.
- The Raspberry Pi communicates bidirectionally with the flight controller
  over physical UART4.
- The MTF-01P rangefinder and optical-flow telemetry reaches the Raspberry Pi
  through the flight controller.
- The Raspberry Pi can send MAVLink commands and receive acknowledgements.
- The project provides:
  - `uv run drone-console` for an interactive MAVProxy console.
  - `uv run drone-health` for checking both MAVLink connections.
- Autonomous or remote flight from the Pi is **not yet considered flight-ready**
  because important ArduPilot safety and failsafe settings are disabled.

No arming, motor-test, movement, flight-mode, or actuator command was issued
during this session.

## Hardware and connection summary

### Flight controller

- Product: Flywoo GOKU GN745 AIO / FlywooF745
- Firmware: ArduCopter 4.6.3
- USB ID: `1209:5741`
- USB device: `/dev/ttyACM0`
- Stable USB path:
  `/dev/serial/by-id/usb-ArduPilot_FlywooF745_200023000451333436353531-if00`
- MAVLink system ID: `1`
- MAVLink baud rate: `115200`

### Raspberry Pi

- Hostname: `seb-is-pm`
- SSH target: `seb@seb-is-pm`
- Pi UART device: `/dev/serial0`
- `/dev/serial0` resolves to `/dev/ttyAMA0`
- Pi GPIO14 and GPIO15 are configured as `TXD0` and `RXD0`
- The `seb` user belongs to the `dialout` group

The SSH password was entered interactively during testing. It is intentionally
not stored in this repository or in the project scripts.

### Final working Pi wiring

The final wiring after swapping the green and blue cables on the Pi is:

| Pi physical pin | Pi function | Cable color | Flight-controller endpoint |
|---|---|---|---|
| Pin 2 | 5 V | Red | Regulated FC 5 V supply |
| Pin 6 | GND | Black | FC GND |
| Pin 8 | GPIO14 / TXD0 | Blue | FC `R4` / UART4 receive |
| Pin 10 | GPIO15 / RXD0 | Green | FC `T4` / UART4 transmit |

UART signals must cross:

```text
Pi TXD -> FC RX
Pi RXD <- FC TX
```

The Pi is powered from the flight controller's regulated 5 V output. The red
wire must never be connected to raw battery voltage. The FC 5 V supply capacity
must also be considered when powering the Pi and AI camera together.

### ArduPilot serial-port mapping

The official ArduPilot FlywooF745 board definition uses this direct mapping:

| ArduPilot parameter group | Physical board UART |
|---|---|
| `SERIAL0` | USB |
| `SERIAL1` | UART1 |
| `SERIAL2` | UART2 |
| `SERIAL3` | UART3 |
| `SERIAL4` | UART4 |
| `SERIAL5` | UART5 |
| `SERIAL6` | UART6 |
| `SERIAL7` | UART7 |

Official references:

- [FlywooF745 board documentation](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/FlywooF745/README.md)
- [FlywooF745 hardware definition](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/FlywooF745/hwdef.dat)

Relevant assignments on this drone:

| Port | Use | Live configuration |
|---|---|---|
| `SERIAL0` | Developer-machine USB | MAVLink2, 115200 |
| `SERIAL2` | RC input | Protocol `23` |
| `SERIAL3` | VTX control | Protocol `44` |
| `SERIAL4` | Raspberry Pi companion link | MAVLink2, 115200 |
| `SERIAL5` | MTF-01P range/optical flow | Protocol `1`, 115200, options `1024` |
| `SERIAL6` | GPS | Protocol `5`, 57600 |

## 1. Initial developer-machine USB check

The first requested check opened `/dev/ttyACM0` with `pymavlink` and waited for
a heartbeat.

### Initial obstacle: the device appeared to be missing

Inside the restricted command environment:

```text
ls: cannot access '/dev/ttyACM0': No such file or directory
unable to initialize libusb: -99
```

Running the Python script there produced:

```text
serial.serialutil.SerialException:
[Errno 2] could not open port /dev/ttyACM0:
[Errno 2] No such file or directory
```

### Cause

The restricted execution environment did not expose the host USB devices.
This was not evidence that the physical controller was disconnected.

### Fix

The USB checks and serial script were rerun with direct host-device access.
The host then showed:

```text
1209:5741 Generic FlywooF745
/dev/ttyACM0
/dev/serial/by-id/usb-ArduPilot_FlywooF745_200023000451333436353531-if00
```

### Verified heartbeat

```text
Heartbeat received!
target_system    = 1
target_component = 0
vehicle type     = 2
autopilot        = 3
base_mode        = 89
custom_mode      = 0
system_status    = 1
```

Interpretation:

- Vehicle type `2`: quadrotor
- Autopilot `3`: ArduPilot
- Target system: `1`
- The test only listened for telemetry and did not write anything.

## 2. Read-only parameter checks

### `SYSID_THISMAV`

A `PARAM_REQUEST_READ` was sent for `SYSID_THISMAV`.

Result:

```text
PARAM_VALUE {
    param_id : SYSID_THISMAV,
    param_value : 1.0,
    param_type : 4,
    param_count : 1152,
    param_index : 65535
}
```

This proved both communication directions:

1. The developer machine sent a parameter request.
2. The flight controller returned the requested parameter.

### Full parameter download

The complete parameter table was requested and serial-related parameters were
filtered.

Important live values:

```text
SERIAL0_BAUD             115
SERIAL0_PROTOCOL         2
SERIAL1_PROTOCOL         -1
SERIAL1_BAUD             115
SERIAL2_PROTOCOL         23
SERIAL2_BAUD             115
SERIAL3_PROTOCOL         44
SERIAL3_BAUD             115
SERIAL4_PROTOCOL         2
SERIAL4_BAUD             115
SERIAL4_OPTIONS          0
SERIAL5_PROTOCOL         1
SERIAL5_BAUD             115
SERIAL5_OPTIONS          1024
SERIAL6_PROTOCOL         5
SERIAL6_BAUD             57
SERIAL7_PROTOCOL         -1
```

ArduPilot stores common baud rates in thousands:

```text
115 -> 115200 baud
57  -> 57600 baud
```

### Important historical difference

The saved parameter backup from 9 June contains:

```text
SERIAL4_PROTOCOL=-1
SERIAL4_BAUD=230
SERIAL4_OPTIONS=0
```

The live controller on 19 June contained:

```text
SERIAL4_PROTOCOL=2
SERIAL4_BAUD=115
SERIAL4_OPTIONS=0
```

Therefore, the June 9 backup is not a complete representation of the live
controller state on June 19. The session used fresh live reads before making
decisions.

## 3. MAVProxy startup failure

The initial one-off MAVProxy command was:

```bash
uv run --with MAVProxy mavproxy.py \
  --master=/dev/ttyACM0 \
  --baudrate 115200
```

It failed with:

```text
ModuleNotFoundError: No module named 'future'
```

### Cause

MAVProxy 1.8.74 imports:

```python
from future.builtins import input
```

However, its base package metadata did not install `future`.

### Temporary fix

The corrected one-off invocation was:

```bash
uv run --with MAVProxy --with future mavproxy.py \
  --master=/dev/ttyACM0 \
  --baudrate 115200
```

This successfully loaded the MAVProxy help output.

### Second packaging obstacle: missing Pillow

After `future` was added, MAVProxy connected but printed:

```text
Failed to load module: No module named 'adsb'
```

The actual nested import failure was:

```text
ModuleNotFoundError: No module named 'PIL'
```

MAVProxy's default ADS-B module imports Pillow, but Pillow is only part of
MAVProxy's optional recommended dependencies.

### Permanent project fix

The project now declares:

```toml
dependencies = [
    "future>=1.0.0",
    "mavproxy>=1.8.74",
    "pillow>=12.2.0",
    "pymavlink>=2.4.49",
    "pyserial>=3.5",
]
```

The lockfile was updated with:

- `future==1.0.0`
- `mavproxy==1.8.74`
- `pillow==12.2.0`
- MAVProxy transitive dependencies such as `pynmeagps`

For another minimal project, use:

```bash
uv add MAVProxy future Pillow
```

## 4. Dependency-installation and `uv` obstacles

### DNS failures

Some dependency operations failed inside the restricted environment:

```text
Temporary failure in name resolution
Failed to fetch: https://pypi.org/simple/mavproxy/
```

### Fix

The same `uv add` or `uv sync` operation was rerun with approved network access.

### Read-only uv cache

Some `uv` commands failed with:

```text
Could not acquire lock
Could not create temporary file
Read-only file system at /home/abaris/.cache/uv/...
```

### Fix

For non-network checks, the project used:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv ...
```

For required package installations, the operation was run with direct access
to the normal uv cache.

## 5. MAVProxy live verification

Both forms were tested against the physical controller:

```bash
uv run --with MAVProxy mavproxy.py \
  --master=/dev/ttyACM0 \
  --baudrate 115200

uv run mavproxy.py \
  --master=/dev/ttyACM0 \
  --baudrate 115200
```

Successful startup:

```text
Connect /dev/ttyACM0 source_system=255
Waiting for heartbeat from /dev/ttyACM0
Detected vehicle 1:1 on link 0
online system 1
Mode STABILIZE
```

The following MAVProxy commands were exercised:

```text
status
param fetch
param show SERIAL*
param show MAV*
```

### `status`

`status` showed normal ArduCopter telemetry and:

```text
MAV Errors: 0
HEARTBEAT type=2 autopilot=3
```

### `param fetch`

MAVProxy downloaded:

```text
Received 1152 parameters (ftp)
Saved 1152 parameters to mav.parm
```

It also printed:

```text
Failed to download /SRTM3/filelist_python :
'utf-8' codec can't decode byte 0x80 ...
```

This was a MAVProxy terrain/SRTM cache decoding issue. It did not prevent the
parameter table from being downloaded.

### `param show SERIAL*`

This returned the expected serial configuration, including:

```text
SERIAL4_BAUD      115
SERIAL4_OPTIONS   0
SERIAL4_PROTOCOL  2
SERIAL5_BAUD      115
SERIAL5_OPTIONS   1024
SERIAL5_PROTOCOL  1
```

### `param show MAV*`

The current parameter set contained no names matching `MAV*`. An empty result
was valid and was documented as such.

### MAVProxy shutdown issue

`exit` and `quit` were not recognized as console commands:

```text
Unknown command 'exit'
Unknown command 'quit'
```

`Ctrl-D` unloaded the modules and exited. MAVProxy 1.8.74 sometimes printed a
`log_writer` thread traceback during shutdown.

This did not leave the serial port open. It was verified afterward with:

```bash
fuser /dev/ttyACM0
```

Result:

```text
/dev/ttyACM0 is released
```

Generated MAVProxy files are now ignored:

```text
mav.tlog
mav.tlog.raw
mav.parm
```

### MAVProxy `--version` issue

During validation:

```bash
uv run mavproxy.py --version
```

failed because MAVProxy attempted to import `pkg_resources`, then fell back to
a missing `~/.mavproxy/version.txt`.

Normal startup and `--help` worked. Version verification was performed with:

```python
from importlib.metadata import version
print(version("MAVProxy"))
```

This returned `1.8.74`. No unnecessary `setuptools` dependency was added solely
for the upstream `--version` implementation.

## 6. Project command: `drone-console`

The repeated MAVProxy command was too long, so a standard Python console entry
point was added instead of a shell alias:

```toml
[project.scripts]
drone-console = "ai_drone.console:main"
```

Normal use:

```bash
uv run drone-console
```

The command:

1. Prefers the stable ArduPilot `/dev/serial/by-id/...` device.
2. Falls back to an ArduPilot by-id match.
3. Falls back to `/dev/ttyACM0` or another `/dev/ttyACM*`.
4. Uses 115200 baud.
5. Forwards additional arguments to MAVProxy.

Examples:

```bash
uv run drone-console
uv run drone-console --device /dev/ttyACM0 --baud 115200
uv run drone-console --show-errors
```

### Build-system obstacle

Adding `[project.scripts]` required the repository to become an installable
Python project. The initial `uv_build` configuration failed:

```text
Expected a Python module at: src/ai_drone/__init__.py
```

### Cause

`uv_build` defaults to a `src/` package layout, while this repository already
uses a top-level `ai_drone/` directory.

### Fix

The existing layout was declared explicitly:

```toml
[build-system]
requires = ["uv_build>=0.11.17,<0.12.0"]
build-backend = "uv_build"

[tool.uv.build-backend]
module-root = ""
```

After this change, `uv sync` installed the project entry point successfully.

### Live verification

```bash
uv run drone-console
```

selected:

```text
/dev/serial/by-id/usb-ArduPilot_FlywooF745_200023000451333436353531-if00
```

and reached:

```text
Detected vehicle 1:1
online system 1
```

## 7. Identifying the Raspberry Pi flight-controller port

The goal was to determine which ArduPilot `SERIALx` belongs to the already
soldered Pi cable.

### Available evidence

- The repository recorded the MTF-01P on UART5 / `SERIAL5`.
- The Raspberry Pi cable's FC end was soldered, but its UART number was not
  recorded.
- The FlywooF745 official board definition maps `SERIAL4` directly to physical
  `TX4/RX4`.
- The project handbook's recommended allocation leaves UART4 available for
  companion hardware.
- Live parameters showed `SERIAL4` as the only available user UART already
  configured as bidirectional MAVLink2:

```text
SERIAL4_PROTOCOL=2
SERIAL4_BAUD=115
SERIAL4_OPTIONS=0
```

- UART1 is receive-only on this board revision, so it is not suitable for a
  bidirectional Pi companion link.

### Conclusion

The intended Raspberry Pi port is:

```text
physical UART4
ArduPilot SERIAL4
```

### No redundant parameter write

The proposed commands were:

```text
param set SERIAL4_PROTOCOL 2
param set SERIAL4_BAUD 115
param set SERIAL4_OPTIONS 0
reboot
```

The three live values were already exactly correct. They were not rewritten.

### Safe reboot

The flight controller was confirmed disarmed:

```text
system=1 component=0 armed=False
```

A flight-controller reboot command was sent:

```text
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
```

It returned:

```text
COMMAND_ACK command=246 result=0
```

The USB heartbeat returned immediately after reboot:

```text
USB_HEARTBEAT_OK system=1 component=0
```

## 8. Raspberry Pi UART troubleshooting

### Pi UART configuration

The Pi reported:

```text
/dev/serial0 -> /dev/ttyAMA0
GPIO14 = TXD0
GPIO15 = RXD0
enable_uart=1
```

The serial device was unused, and the user belonged to `dialout`.

### First failure: no heartbeat

The initial Pi test opened:

```text
/dev/serial0 at 115200
```

but returned:

```text
NO_HEARTBEAT
```

A raw five-second serial read returned:

```text
bytes_received=0
```

This showed that the problem was below the MAVLink parser: no bytes were
arriving at the Pi UART.

### Bidirectional probe

The Pi sent temporary MAVLink heartbeats using:

```text
source_system=42
source_component=191
```

The developer-machine USB listener saw only flight-controller heartbeats from
system `1:1`; it did not see the Pi's heartbeat. This showed that the other
UART direction also was not reaching the FC.

### Incorrect hostname obstacle

An attempted connection used the obsolete hostname `seb-is-pm2`.

It failed with:

```text
Could not resolve hostname seb-is-pm2
```

The user clarified that the current hostname is:

```text
seb-is-pm
```

All later commands used:

```bash
ssh seb@seb-is-pm
```

### Wiring interpretation

The initially reported Pi wiring was:

```text
Pin 2:  Red   5 V
Pin 6:  Black GND
Pin 8:  Green TXD
Pin 10: Blue  RXD
```

The Pi UART must be crossed with the FC:

```text
Pi TXD -> FC R4
Pi RXD <- FC T4
```

With green on Pi TXD and blue on Pi RXD, the link still produced no heartbeat.

### Fix: swap green and blue on the Pi

After the user swapped green and blue:

```text
Pin 8:  Blue  Pi TXD -> FC R4
Pin 10: Green Pi RXD <- FC T4
```

the next test succeeded:

```text
PI_HEARTBEAT_OK system=1 component=0
type=2 autopilot=3 base_mode=81 status=3
```

A read-only parameter request over the Pi cable also succeeded:

```text
PI_PARAM_OK SYSID_THISMAV=1
```

This proved that the UART works in both directions.

## 9. Project command: `drone-health`

A second project entry point was added:

```toml
[project.scripts]
drone-health = "ai_drone.health:main"
```

Normal use:

```bash
uv run drone-health
```

It checks:

1. Developer machine -> FC over USB.
2. Developer machine -> SSH -> Pi `/dev/serial0` -> FC over UART4.

Each path must:

1. Receive a flight-controller heartbeat.
2. Send a `PARAM_REQUEST_READ`.
3. Receive `SYSID_THISMAV`.

Therefore, a passing result confirms bidirectional communication, not only
incoming telemetry.

The Pi-side test is sent as a small encoded inline Python program over SSH.
This avoids requiring the latest health-check module to have already been
deployed to the Pi.

### Successful live result

```text
PASS Developer USB:
device=/dev/serial/by-id/usb-ArduPilot_FlywooF745_200023000451333436353531-if00
system=1 component=0 SYSID_THISMAV=1 armed=False

PASS Pi UART:
device=/dev/serial0
system=1 component=0 SYSID_THISMAV=1 armed=False

PASS All requested MAVLink connections are working.
```

Individual checks:

```bash
uv run drone-health --usb-only
uv run drone-health --pi-only
```

## 10. MTF-01P lidar and optical-flow test through the Pi

The existing passive sensor test was run remotely:

```bash
ssh seb@seb-is-pm
cd ~/ai-drone
.venv/bin/python test_lidar.py \
  --device /dev/serial0 \
  --duration 10 \
  --output /tmp/mtf-01p-pi-check.csv
```

The script:

- Waited for a heartbeat.
- Confirmed the vehicle was disarmed.
- Requested rangefinder and optical-flow messages.
- Did not arm, change mode, move motors, or write parameters.
- Saved the received messages to CSV.

### Successful result

```text
Vehicle is DISARMED. Requesting sensor telemetry only.
Saved 296 sensor messages to /tmp/mtf-01p-pi-check.csv
Rangefinder: working, samples=198,
median=0.95 m, min=0.91 m, max=0.98 m
Optical flow: samples=98,
median quality=62/255, max=74/255
```

The CSV contained 297 lines including the header.

Example records:

```text
RANGEFINDER      distance approximately 0.93-0.94 m
DISTANCE_SENSOR  distance approximately 0.93-0.94 m
OPTICAL_FLOW     quality approximately 63-66
```

Because the drone was stationary, the sampled optical-flow X/Y values were
mostly zero. The `OPTICAL_FLOW.ground_distance` field was also `0.0`, while the
separate `RANGEFINDER` and `DISTANCE_SENSOR` messages contained valid distance
measurements. This proves that both sensors are reaching ArduPilot and the Pi,
but it is not yet an in-flight validation of optical-flow navigation or
position hold.

This is a major improvement over the earlier June 9 test, where range messages
were zero and no optical-flow messages arrived. During this session, the sensor
was powered and produced meaningful data.

## 11. Can the Raspberry Pi control the drone?

### What was proven

The Pi can:

- Receive flight-controller heartbeats.
- Receive normal telemetry.
- Request and receive parameters.
- Receive rangefinder and optical-flow telemetry.
- Send a MAVLink `COMMAND_LONG`.
- Receive a `COMMAND_ACK`.
- Receive the message requested by that command.

A safe read-only command requested `AUTOPILOT_VERSION`:

```text
MAV_CMD_REQUEST_MESSAGE
```

Result:

```text
HEARTBEAT_OK system=1 component=0 armed=False
COMMAND_ACK command=512 result=0
AUTOPILOT_VERSION_OK
```

`result=0` means the command was accepted.

Therefore, the Raspberry Pi has a functional command channel to ArduPilot. It
is technically capable of sending mode, guided-movement, mission, and arming
commands through `pymavlink`.

### What was deliberately not tested

The session did not test:

- Arming from the Pi
- Motor output
- Flight-mode changes
- Guided movement
- Takeoff or landing
- Mission upload or execution
- RC takeover behavior
- Loss-of-Pi behavior
- In-flight optical-flow performance

### Current safety blockers

Live parameter reads showed:

```text
ARMING_CHECK=0
FENCE_ENABLE=0
FS_GCS_ENABLE=0
FS_GCS_TIMEOUT=5
SYSID_MYGCS=255
GUID_OPTIONS=0
```

The EKF source configuration includes:

```text
EK3_SRC1_POSXY=0
EK3_SRC1_VELXY=5
EK3_SRC1_POSZ=2
EK3_SRC1_VELZ=0
EK3_SRC1_YAW=1
FLOW_TYPE=5
RNGFND1_TYPE=10
```

Interpretation:

- Optical flow is configured as the horizontal velocity source.
- The rangefinder is configured for vertical position/height support.
- Arming checks are disabled.
- The geofence is disabled.
- The GCS/companion-link failsafe is disabled.

The command transport is working, but autonomous flight is not yet considered
safe or complete.

Before attempting Pi-controlled flight:

1. Restore and validate appropriate arming checks.
2. Configure a deliberate GCS/companion-link loss failsafe.
3. Review geofence requirements.
4. Confirm RC pilot override and emergency disarm behavior.
5. Test optical-flow and rangefinder estimates while restrained and
   propeller-safe.
6. Test mode changes without arming.
7. Test guided setpoints in simulation or with motors disabled.
8. Only then perform a controlled flight test in a safe area.

## 12. Repository changes

### New files

- `ai_drone/console.py`
  - Implements `drone-console`.
  - Selects the stable USB path automatically.
  - Launches MAVProxy with project defaults.

- `ai_drone/health.py`
  - Implements `drone-health`.
  - Checks USB and Pi UART paths.
  - Requires heartbeat plus `SYSID_THISMAV` response.

- `docs/DEVELOPER_MACHINE_DRONE_CONNECTION.md`
  - Detailed developer-machine connection and troubleshooting workflow.

- `19-06-session.md`
  - This complete session report.

### Updated files

- `pyproject.toml`
  - Added MAVProxy runtime dependencies.
  - Added `drone-console` and `drone-health`.
  - Added the `uv_build` backend configuration.

- `uv.lock`
  - Locked the new dependencies.

- `README.md`
  - Added short connection and health-check commands.
  - Updated the Pi UART status from pending to verified.

- `docs/DRONE_CONFIGURATION.md`
  - Recorded UART4, Pi pin assignments, and health-check command.

- `docs/drone-project.md`
  - Updated the Pi integration status to connected and verified.

- `.gitignore`
  - Ignores MAVProxy-generated telemetry and parameter-cache files.

- `tests/test_drone_tools.py`
  - Added command construction, device selection, and SSH health-check tests.

## 13. Validation performed

The final project checks were:

```bash
uv run --group dev ruff check .
uv run --group dev ruff format --check .
uv run --group dev pytest
uv lock --check
uv run --group dev ty check
```

Results:

```text
Ruff: passed
Formatting: passed
Pytest: 7 passed
Lockfile: passed
```

`ty check` reported two existing non-fatal warnings in
`ai_drone/nearest_person.py`:

```text
Unused ty: ignore directive for numpy imports
```

These warnings appeared because NumPy became available through MAVProxy's
dependency set. They are unrelated to the MAVLink work and were not changed
during this session.

## 14. Normal workflow after this session

### Install or update the project

```bash
cd ~/ai-drone
uv sync
```

### Verify both communication paths

```bash
uv run drone-health
```

Expected:

```text
PASS Developer USB: ...
PASS Pi UART: ...
PASS All requested MAVLink connections are working.
```

### Open an interactive MAVProxy console

```bash
uv run drone-console
```

Useful read-only commands:

```text
status
param fetch
param show SERIAL*
param show MAV*
```

### Check only the developer USB path

```bash
uv run drone-health --usb-only
```

### Check only the Pi UART path

```bash
uv run drone-health --pi-only
```

### Run the lidar test through the Pi

From the developer machine:

```bash
./deploy.sh --lidar
```

Or directly on the Pi:

```bash
cd ~/ai-drone
.venv/bin/python test_lidar.py --device /dev/serial0
```

## 15. Troubleshooting summary

| Symptom | Cause found in this session | Resolution |
|---|---|---|
| `/dev/ttyACM0` missing in a command | Restricted environment hid host USB | Run hardware check with host-device access |
| `libusb: -99` | USB unavailable inside restricted environment | Inspect host USB directly |
| `No module named future` | MAVProxy packaging omitted runtime import | Add `future` |
| ADS-B module failed to load | Pillow was not installed | Add `Pillow` |
| `uv` DNS failure | Restricted network | Rerun approved network operation |
| `uv` cache read-only | Normal cache unavailable in restricted run | Use `UV_CACHE_DIR=/tmp/uv-cache` |
| Build expected `src/ai_drone` | `uv_build` defaulted to src layout | Set `module-root = ""` |
| MAVProxy SRTM UTF-8 error | Terrain cache/file decoding issue | Parameters still fetched; ignore for serial inspection |
| `exit`/`quit` unknown in MAVProxy | Not valid commands in this console | Use `Ctrl-D` |
| `log_writer` traceback on exit | MAVProxy shutdown-thread behavior | Confirm process exit and port release |
| `seb-is-pm2` did not resolve | Hostname was obsolete | Use `seb-is-pm` |
| Pi heartbeat timed out | TX/RX cable colors were reversed at Pi | Swap green and blue |
| Pi raw serial read returned zero bytes | Same physical TX/RX reversal | Correct crossing: Pi TX->FC R4, Pi RX<-FC T4 |
| Concern that Pi only receives telemetry | Needed proof of transmit path | Request parameter and `AUTOPILOT_VERSION`; both acknowledged |
| Earlier lidar values were zero | Sensor was not producing powered data | Power sensor and test through working Pi/FC link |

## 16. Safety notes

- Keep the vehicle disarmed during development checks.
- Remove propellers whenever possible.
- Do not run `arm`, motor-test, movement, or mode-change commands as casual
  connectivity tests.
- Do not connect the Pi's red wire to raw battery voltage.
- Do not power the Pi from FC 5 V and a second 5 V source at the same time
  unless the power design explicitly supports it.
- Only one process should own a serial device at a time.
- Close QGroundControl before opening the same USB device in MAVProxy.
- The current disabled arming checks and failsafes must be addressed before
  flight testing.
