# Developer Machine Drone Connection

This workflow connects a Linux developer machine to the FlywooF745 flight
controller over USB and inspects ArduPilot through MAVLink. It does not require
the Raspberry Pi.

## Safety boundary

The commands in this guide are intended for telemetry and parameter inspection.

- Keep the vehicle disarmed.
- Remove the propellers when possible. If they cannot be removed, secure the
  frame, keep clear of the motors, and do not use arming, motor-test, flight-mode,
  parameter-write, or reboot commands.
- Start with USB power only. Connect the flight battery only when a peripheral,
  such as the MTF-01P, must be powered for a sensor test.
- Do not leave QGroundControl, another MAVProxy process, or another serial tool
  connected to the same device.
- Do not run `param set`, `arm`, `mode`, `reboot`, or motor-test commands as part
  of this inspection workflow.

## Known flight-controller connection

The verified controller is:

- USB ID: `1209:5741`
- Product: `FlywooF745`
- Current short device path: `/dev/ttyACM0`
- Stable device path:
  `/dev/serial/by-id/usb-ArduPilot_FlywooF745_200023000451333436353531-if00`
- MAVLink baud rate: `115200`
- MAVLink system ID: `1`

Prefer the stable `/dev/serial/by-id/...` path in scripts. The kernel can assign
a different `ttyACM` number after a reconnect.

## 1. Prepare the project

Install `uv`, if needed:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Install the locked project dependencies:

```bash
cd ~/ai-drone
uv sync
```

The project permanently declares `MAVProxy`, `future`, and `Pillow`. MAVProxy
1.8.74 imports `future`, but its published package metadata does not install
that dependency. Its default ADS-B module also imports Pillow, which is only
included in MAVProxy's optional recommended dependencies. If adding MAVProxy to
another minimal project, use:

```bash
uv add MAVProxy future Pillow
```

Using only `uv add MAVProxy` can otherwise fail at startup with:

```text
ModuleNotFoundError: No module named 'future'
```

## 2. Make the physical connection

1. Confirm the vehicle is disarmed and place it on a stable surface.
2. Leave the flight battery disconnected for the initial controller check.
3. Connect a data-capable USB cable from the developer machine to the flight
   controller USB port.
4. Wait a few seconds for Linux to enumerate the controller.
5. Do not open QGroundControl while MAVProxy is using the serial port.

USB alone powers the flight controller and is sufficient for heartbeat and
parameter inspection. It does not necessarily power attached sensors. Connect
the flight battery only when the test explicitly requires those sensors.

## 3. Verify Linux detected the controller

Check the USB identity:

```bash
lsusb | grep -i '1209:5741\|Flywoo'
```

Expected identity:

```text
1209:5741 Generic FlywooF745
```

Check the serial paths:

```bash
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
```

The stable symlink should resolve to the current `ttyACM` device:

```bash
readlink -f /dev/serial/by-id/usb-ArduPilot_FlywooF745_200023000451333436353531-if00
```

If opening the port reports `Permission denied`, inspect its ownership:

```bash
ls -l /dev/ttyACM0
id
```

On Arch Linux, serial access commonly uses the `uucp` group:

```bash
sudo usermod -aG uucp "$USER"
```

On Debian or Ubuntu, it commonly uses `dialout`:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership. Do not use a permanent
world-writable `chmod` as the access fix.

## 4. Start MAVProxy

The normal project command is:

```bash
uv run drone-console
```

`drone-console` automatically prefers the stable ArduPilot
`/dev/serial/by-id/...` path, falls back to `/dev/ttyACM*`, and uses `115200`
baud. Override either value when needed:

```bash
uv run drone-console --device /dev/ttyACM0 --baud 115200
```

Additional unrecognized options are forwarded to MAVProxy:

```bash
uv run drone-console --show-errors
```

The underlying commands remain available for debugging:

```bash
uv run mavproxy.py --master=/dev/ttyACM0 --baudrate 115200
uv run --with MAVProxy mavproxy.py --master=/dev/ttyACM0 --baudrate 115200
```

Successful startup should report a heartbeat and identify system `1`. If it
waits indefinitely, exit with `Ctrl-C` and follow the troubleshooting section.
MAVProxy writes local `mav.tlog`, `mav.tlog.raw`, and `mav.parm` files while it
runs; these generated files are ignored by this repository.

## 5. Inspect the connection in MAVProxy

Run these commands at the `MAV>` prompt:

```text
status
param fetch
param show SERIAL*
param show MAV*
```

What they do:

- `status` shows the current vehicle and MAVLink state.
- `param fetch` downloads the full parameter table. Wait for it to finish.
- `param show SERIAL*` lists serial-port protocol, baud, and option settings.
- `param show MAV*` lists parameters whose names begin with `MAV`, if present.
  This controller's current parameter set returns no matches for that pattern.

This controller was verified with these relevant values:

```text
SERIAL0_BAUD       115
SERIAL0_PROTOCOL   2
SERIAL4_BAUD       115
SERIAL4_PROTOCOL   2
SERIAL5_BAUD       115
SERIAL5_PROTOCOL   1
SERIAL5_OPTIONS    1024
```

ArduPilot stores common baud rates in thousands, so `115` means `115200`.
Protocol numbers are ArduPilot parameter values; do not change them solely
because they differ between ports.

## Verify both developer and Pi connections

From the developer machine:

```bash
uv run drone-health
```

This checks the direct USB connection, then SSHes to `seb@seb-is-pm` and checks
the Pi's `/dev/serial0` UART connection. Both paths must receive a heartbeat and
receive a read-only `SYSID_THISMAV` parameter response.

## 6. Disconnect cleanly

1. Exit MAVProxy with `Ctrl-D`.
2. Confirm the MAVProxy process has stopped.
3. If a flight battery was connected for a sensor test, disconnect it first.
4. Disconnect the USB cable.

## Troubleshooting

### `No module named 'future'`

Synchronize the repository and retry:

```bash
uv sync
uv run drone-console
```

For a one-off environment outside this repository:

```bash
uv run --with MAVProxy --with future --with Pillow mavproxy.py \
  --master=/dev/ttyACM0 \
  --baudrate 115200
```

### `/dev/ttyACM0` does not exist

Reconnect the USB cable and inspect the stable path:

```bash
lsusb
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM*
```

Use the `/dev/serial/by-id/...` path when it exists. A changed `ttyACM` number
does not imply a driver failure.

### `Device or resource busy`

Close QGroundControl, MAVProxy, serial monitors, and any scripts using the
flight controller. To find a process holding the port:

```bash
fuser /dev/ttyACM0
```

### No heartbeat

Check that:

- the USB cable carries data, not only power;
- the controller appears in `lsusb`;
- the selected serial path exists;
- no other process owns the port; and
- the baud rate is `115200`.

USB MAVLink normally works without the flight battery. A missing sensor reading
is a separate issue: peripherals such as the MTF-01P may require battery power
even while the flight controller itself is communicating over USB.

### Traceback while exiting MAVProxy

MAVProxy 1.8.74 can print a `log_writer` thread traceback after `Ctrl-D` while
its logging thread shuts down. If the process has exited and the serial port can
be opened again, the connection was released; this does not indicate a flight
controller failure.
