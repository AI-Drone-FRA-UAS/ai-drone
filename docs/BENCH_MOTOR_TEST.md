# Guarded bench motor test

This test is only for proving motor numbering, ESC response, and rotation. It
does not test flight control.

## Hard prerequisites

1. Remove every propeller from every motor.
2. Secure the frame to a stable, non-conductive bench.
3. Keep hands, cables, the camera ribbon, and clothing outside all motor bells.
4. Turn on the RC transmitter and keep a second person ready to disconnect the
   LiPo.
5. Restore `ARMING_CHECK=1` and resolve every reported pre-arm problem. The
   utility intentionally refuses to run with any other value, including a
   partial check mask.

The commands below prefer Tailscale MagicDNS. When Tailscale is offline and the
laptop is joined to `AI-Drone-Zero`, use `PI_HOST=seb@192.168.4.1` instead.

Test one motor for half a second at 7%:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --motor-test \
  --motor 1 \
  --duration 0.5 \
  --throttle-percent 7 \
  --confirm-props-removed PROPS_REMOVED \
  --confirm-vehicle-secured VEHICLE_SECURED
```

After validating motors individually, test all configured motors sequentially:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --motor-test \
  --all-motors \
  --duration 0.5 \
  --throttle-percent 7 \
  --confirm-props-removed PROPS_REMOVED \
  --confirm-vehicle-secured VEHICLE_SECURED
```

The program caps each motor at 1 second and 10%, performs a five-second
countdown, checks that the initial heartbeat is disarmed, verifies the configured
motor outputs, and uses `MAV_CMD_DO_MOTOR_TEST`. ArduPilot temporarily soft-arms
the outputs internally during this command; the utility never sends the normal
vehicle-arm command. It sends a zero-duration stop request in cleanup and waits
for a disarmed heartbeat.

The 7% default is deliberately near the lowest expected spin point. Confirm
the live `MOT_PWM_*` and `MOT_SPIN_MIN` values before testing. If a motor does
not move, do not immediately raise the limit: first check its ESC signal,
power, wiring, and ArduPilot status text.

Official safety/setup reference:
[ArduPilot motor range test](https://ardupilot.org/copter/docs/set-motor-range.html).
