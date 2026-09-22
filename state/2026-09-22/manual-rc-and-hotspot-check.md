# Manual RC and hotspot check, 2026-09-22

The user confirmed the transmitter is on and linked, USB is connected, and the
teammate uses a dedicated arm switch. The requested flight is manually piloted;
the Pi should only record and open the payload mount on AprilTag 6.

## Actions and connection

- Read Pi and flight-controller state through USB at 192.168.7.2, preserving the
  normal SSH host identity `seb-is-pm.tail59e6a4.ts.net`.
- Activated the existing NetworkManager `Hotspot` profile. It advertises
  `AI-Drone-Zero`, with Pi address 192.168.4.1/24. The PC scan saw signal 100 on
  channel 11. SSH is active and listening on all addresses. The PC was not joined
  to the hotspot, so an actual SSH connection through the Wi-Fi AP is not yet
  verified. USB remained connected throughout.
- Preserved hotspot credentials and autoconnect=no. Activation is for this boot;
  this did not make the AP start automatically after reboot.
- Did not change flight parameters, arm, switch flight modes, run motors, or
  activate the camera/release watcher. All inspected runtime/capture units were
  inactive before the check.

After manually joining the hotspot, use:

```sh
ssh -o Hostname=192.168.4.1 -o HostKeyAlias=seb-is-pm.tail59e6a4.ts.net seb@seb-is-pm
cd ~/ai-drone
```

## Wi-Fi failure evidence

NetworkManager connected to eduroam at 16:24:18, obtained 10.53.135.29, then
wpa_supplicant reported a locally generated disconnect at 16:24:29. Reassociation
to two APs failed repeatedly with `CTRL-EVENT-ASSOC-REJECT`, status_code=16, and
`CONN_FAILED`; NetworkManager eventually reported `supplicant-timeout`.

The underlying reason for these association failures remains unproven. This
does not establish a bad password, RF interference, hardware failure, or power
fault. `vcgencmd get_throttled` returned 0x0 and temperature was 58 C. Wi-Fi
power saving was already disabled. Tailscaled remained active; without Wi-Fi
there was no Internet route. Direct hotspot SSH does not require Tailscale.

## Readback and arming

Raw evidence is under `artifacts/manual-rc-check-20260922/`:

- `usb-probe.json`: initial burst query; incomplete parameter replies.
- `usb-probe-paced.json`: individually paced reads, all requested supported
  parameters returned. SERIAL8 parameters were absent.
- `usb-prearm-final.json`: final `drone check --duration 8 --prearm --json`.
  Exit 1 accurately indicates failed sensor/pre-arm checks, not an SSH failure.

The controller is ArduCopter 4.7.1, identity dbe79216, disarmed in STABILIZE.

| Parameter | Observed value | Meaning |
| --- | --- | --- |
| MOT_SPIN_MAX | 0.40 | Motor actuator output limit, including manual flight; not a measurement of 40% thrust |
| MOT_PWM_MIN / MOT_PWM_MAX | 1000 / 2000 | Configured output range |
| MOT_PWM_TYPE | 6 | DShot600 |
| MOT_BAT_CURR_MAX | 0 | Current-based throttle limiting disabled |
| ATC_ANGLE_MAX | 15 | Maximum requested lean angle in degrees |
| RC8_OPTION | 154 | Arm/disarm with AirMode |
| ARMING_RUDDER | 0 | Stick arming disabled; consistent with using the dedicated switch |
| ARMING_SKIPCHK | 0 | Checks enabled |
| FS_THR_ENABLE | 0 | Radio-loss failsafe disabled |
| FS_GCS_ENABLE / FS_GCS_TIMEOUT | 5 / 5 | GCS-heartbeat-loss Land action, five seconds |
| FS_OPTIONS | 8 | Continue landing on failsafe; no manual-mode GCS exemption |

RC3 and RC8 were 988 (throttle low and arm switch low). SYS_STATUS reported the
RC receiver healthy, but RC_CHANNELS.chancount was 0. Do not equate this field
alone with proof that the bound receiver is absent or with proof of working
pilot control; stick movement and receiver-loss behavior were not tested.

Final explicit FC messages:

- `PreArm: Check mag field: 1364, max 875, min 185` (also 1362).
- `PreArm: Rangefinder 1: No Data`.

Optical-flow and rangefinder health were false in the final check, despite some
range messages being received (downward 0.02 m, forward 0.45 m). Earlier in this
check the compass instead reported `z diff:207>200`; the exact magnetic error
varied. Battery was 15.772 V. Earlier GCS-failsafe pre-arm messages from before
the reboot did not recur in this final sample.

Flight readiness has not been established. Moving away from environmental metal
and diagnosing the sensor data interruptions are preferable to bypassing these
failures. A manual RC configuration needs receiver-loss handling reviewed; no
failsafe settings were changed. The current tag-mount recorder also requires
exact ARMING_SKIPCHK=0 and refuses startup if checks are skipped.

References consulted:

- https://ardupilot.org/copter/docs/common-prearm-safety-checks.html
- https://ardupilot.org/copter/docs/stabilize-mode.html
- https://ardupilot.org/copter/docs/motor-thrust-scaling.html
- https://ardupilot.org/copter/docs/gcs-failsafe.html
- https://ardupilot.org/copter/docs/radio-failsafe.html
- Locally archived matching ArduPilot source under
  `artifacts/software-update-20260909/firmware/ardupilot/`.
