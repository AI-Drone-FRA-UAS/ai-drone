# Arduino Servo Test

This folder contains small Arduino CLI sketches for testing the Miuzei SG90 / MS18-F servo.

## Hardware

Board target:

```text
Arduino Leonardo-compatible / Olimexino 32U4
FQBN: arduino:avr:leonardo
```

Servo wiring:

```text
Servo red wire    -> Arduino 5V
Servo brown wire  -> Arduino GND
Servo yellow wire -> Arduino D10
```

Use USB power only for light no-load tests. The datasheet says the servo can draw much more current when moving or stalled, so use an external 5V supply with a shared GND for loaded tests.

## Servo Specs

```text
Model: MS18-F
Operating voltage: 4.8V to 6.0V
Operating travel: 120 degrees +/- 10 degrees
Pulse width range: 900 us to 2100 us
Neutral position: 1500 us
Mechanical limit angle: 200 degrees +/- 1 degree
No-load running current: 400 mA at 4.8V, 500 mA at 6.0V
Stall current: 1300 mA at 4.8V, 1600 mA at 6.0V
Direction: 900 us -> 2100 us moves counter-clockwise
```

Important: command only the normal `900` to `2100` microsecond range. The 200 degree value is the mechanical limit, not the safe commanded travel.

## Setup Commands

Install the Arduino CLI and board core:

```fish
sudo pacman -Syu arduino-cli
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install Servo
```

Check that the board is connected:

```fish
arduino-cli board list
```

The port was `/dev/ttyACM0` here.

## Compile And Upload

Compile only:

```fish
arduino-cli compile --fqbn arduino:avr:leonardo ~/arduino/servo_range_test
```

Compile and upload in one step:

```fish
arduino-cli compile --upload -p /dev/ttyACM0 --fqbn arduino:avr:leonardo ~/arduino/servo_range_test
```

If the board appears on a different port, replace `/dev/ttyACM0` with the port from:

```fish
arduino-cli board list
```

## Stop The Servo Program

Immediate stop: disconnect USB power or remove the servo red 5V wire.

To stop it by software, upload an empty sketch:

```fish
mkdir -p ~/arduino/servo_stop
nvim ~/arduino/servo_stop/servo_stop.ino
```

Use:

```cpp
void setup() {
}

void loop() {
}
```

Then upload:

```fish
arduino-cli compile --upload -p /dev/ttyACM0 --fqbn arduino:avr:leonardo ~/arduino/servo_stop
```

## Program Values

The safer reference sketch is:

```text
~/arduino/servo_range_test/servo_range_test.ino
```

Config values:

```cpp
const int SERVO_PIN = 10;
const int MIN_US = 900;
const int CENTER_US = 1500;
const int MAX_US = 2100;
const int STEP_US = 10;
const int STEP_DELAY_MS = 20;
```

What they mean:

```text
SERVO_PIN       Arduino digital pin used for the yellow servo signal wire.
MIN_US          One safe endpoint from the datasheet.
CENTER_US       Neutral center position from the datasheet.
MAX_US          Other safe endpoint from the datasheet.
STEP_US         Pulse-width change per movement step. Smaller is smoother.
STEP_DELAY_MS   Delay after each step. Larger is slower.
```

Startup behavior:

```cpp
servo.attach(SERVO_PIN, MIN_US, MAX_US);
servo.writeMicroseconds(CENTER_US);
delay(2000);
```

This attaches the servo signal to D10, starts at neutral `1500 us`, and waits 2 seconds.

Loop behavior:

```cpp
moveToMicroseconds(CENTER_US, MIN_US);
delay(1500);

moveToMicroseconds(MIN_US, MAX_US);
delay(1500);

moveToMicroseconds(MAX_US, CENTER_US);
delay(3000);
```

This moves:

```text
center -> minimum endpoint
minimum endpoint -> maximum endpoint
maximum endpoint -> center
```

## How To Set Positions

For this servo, prefer `writeMicroseconds()` because the datasheet gives pulse widths directly.

Useful positions:

```text
900 us   approx one endpoint
1500 us  center / neutral
2100 us  approx other endpoint
```

Approximate angle mapping across the normal 120 degree travel:

```text
900 us   ~= 0 degrees of servo travel
1500 us  ~= 60 degrees of servo travel
2100 us  ~= 120 degrees of servo travel
```

Rule of thumb:

```text
10 us ~= 1 degree of this servo's normal travel
```

Example positions:

```text
30 degrees from the low endpoint  -> 900 + 30 * 10 = 1200 us
60 degrees from the low endpoint  -> 1500 us
90 degrees from the low endpoint  -> 900 + 90 * 10 = 1800 us
```

Example code:

```cpp
servo.writeMicroseconds(1200);
delay(1000);

servo.writeMicroseconds(1800);
delay(1000);
```

To change where the servo starts, edit:

```cpp
servo.writeMicroseconds(CENTER_US);
```

For example:

```cpp
servo.writeMicroseconds(1200);
```

To change the movement range, edit the calls in `loop()`:

```cpp
moveToMicroseconds(1200, 1800);
delay(1500);
moveToMicroseconds(1800, 1200);
delay(1500);
```

Do not use values below `900` or above `2100` unless you are deliberately calibrating very carefully and are ready to disconnect power.

## About `servo.write()`

Arduino also supports:

```cpp
servo.write(90);
```

That API uses a 0 to 180 degree scale, but this servo's safe commanded range is about 120 degrees. For this exact servo, `writeMicroseconds()` is clearer because it matches the PDF.
