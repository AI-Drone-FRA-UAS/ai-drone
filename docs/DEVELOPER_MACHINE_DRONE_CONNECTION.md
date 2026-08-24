# Direct flight-controller USB connection

This workflow connects a Linux developer machine directly to the FlywooF745
flight controller for disarmed MAVLink inspection. It does not require the Pi.

## Safety boundary

- Keep the vehicle disarmed and start with the flight battery disconnected.
- Remove propellers when practical; otherwise secure the frame and stay clear.
- Connect the battery only when a peripheral needs power for an explicitly
  authorized sensor check.
- Close QGroundControl and other serial tools before opening the port.
- Do not use `param set`, arm, mode, reboot, motor-test, or actuator commands
  during this inspection workflow.

## Connect

The known controller identity is:

```text
USB ID: 1209:5741
Product: FlywooF745
Baud: 115200
MAVLink system ID: 1
```

After connecting a data-capable USB cable, inspect the stable serial path:

```bash
lsusb
ls -l /dev/serial/by-id/ /dev/ttyACM*
```

Prefer a `/dev/serial/by-id/usb-ArduPilot_FlywooF745_...-if00` symlink. A
`ttyACM` number can change after reconnecting.

If the port is not accessible, inspect its group:

```bash
ls -l /dev/ttyACM0
id
```

Serial access commonly uses `uucp` on Arch Linux and `dialout` on Debian or
Ubuntu. Add the user to the applicable group and log in again; do not solve the
problem with a permanent world-writable `chmod`.

## Inspect

Capture a best-effort read-only report over direct USB:

```bash
uv run drone-inspect --device /dev/ttyACM0 --duration 10
```

To inspect through the Pi UART instead, deploy and run the same inspector:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --run inspect -- --duration 10
```

The manifest distinguishes a live flight-controller heartbeat, downward and
forward range streams, optical flow, camera frames, and AprilTags. Missing
components are reported rather than failing the run.

MAVProxy remains available as a separately installed expert tool:

```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200
```

Useful read-only MAVProxy commands are:

```text
status
param fetch
param show SERIAL*
```

Exit with `Ctrl-D`. MAVProxy writes ignored `mav.tlog`, `mav.tlog.raw`, and
`mav.parm` files in the working directory.

## Troubleshooting

### Device missing

Reconnect the cable and inspect both USB and serial enumeration:

```bash
lsusb
ls -l /dev/serial/by-id/ /dev/ttyACM*
```

A missing sensor reading is separate from USB MAVLink: USB can power the
controller while peripherals such as the MTF-01P still require the flight
battery.

### Device busy

Close other ground-control or serial programs and identify the owner:

```bash
fuser /dev/ttyACM0
```

### No heartbeat

Confirm the cable carries data, the selected path exists, no process owns it,
and the baud rate is 115200.

### Traceback during shutdown

MAVProxy may print a logging-thread traceback after `Ctrl-D`. If the process
exited and the serial port can be reopened, the connection was released.

## Disconnect

Exit the console, confirm the process stopped, disconnect any flight battery,
then unplug USB.
