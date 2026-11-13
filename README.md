# byt-x4-drone

Quadcopter flight controller for ECE 5, running MicroPython on a Raspberry Pi Pico 2 (RP2350).

Step 1 (this commit): read an MPU6050 IMU and print live accel, gyro, roll and pitch.
There is no motor or receiver code yet.

## Wiring (IMU only)

| MPU6050 | Pico 2 pin | Header pin |
|---------|------------|------------|
| VCC     | 3V3(OUT)   | 36         |
| GND     | any GND    | 38         |
| SDA     | GP4        | 6          |
| SCL     | GP5        | 7          |

I2C0 at 400 kHz, address 0x68. If your wires are on different pins, run
`tools/i2c_scan.py` (below) and update `config.py`.

GP10-GP13 go to the ESC signal inputs and one more GPIO goes to the receiver.
Nothing here configures or drives those pins. Keep the battery unplugged for this step.

## Setup (Mac)

```
brew install git gh pipx
pipx install mpremote
pipx ensurepath
```

Use a data USB cable. Flash MicroPython once: hold BOOTSEL while plugging in the Pico,
then copy the `RPI_PICO2` `.uf2` from https://micropython.org/download/RPI_PICO2/
onto the `RP2350` drive that appears. The board reboots into MicroPython.

## Run

Find the IMU pins first:

```
mpremote run tools/i2c_scan.py
```

It tries every valid I2C0/I2C1 pin pair (skipping GP10-GP13) and prints the pins and
addresses that answer. If the IMU is on other pins, edit `SDA_PIN`, `SCL_PIN` and `I2C_ID`
in `config.py`.

Live readings, with output in your terminal (Ctrl-C to stop):

```
tools/run.sh
```

Run on every power-up instead:

```
tools/deploy.sh
```

To watch a deployed board: `mpremote connect auto repl` (Ctrl-] to leave).

## What gets printed

```
+0.004 -0.002 +1.000 |    +0.00    -0.00    -0.00 |   -0.14   -0.21 |   812 Hz  err 0
```

| Field | Meaning |
|-------|---------|
| ax ay az | acceleration in g. Flat and still, az is about +1.0 |
| gx gy gz | rotation rate in deg/s after bias removal. About 0 when still |
| roll, pitch | angle in degrees from the complementary filter. Flat is about 0, on its edge about +/-90 |
| Hz | measured loop rate, averaged over the last print interval |
| err | count of I2C read errors since start (the loop keeps running) |

On boot it prints "keep still" and averages 500 gyro samples to get the gyro bias. Do not move
the board until the readings start.

## How it works

- `config.py`: pins, I2C frequency, addresses, filter constants and print rate.
- `mpu6050.py`: driver. Wakes the chip, checks WHO_AM_I, sets +/-4 g and +/-500 deg/s, enables
  the ~42 Hz digital low-pass filter, and reads all 14 data bytes in one burst.
- `main.py`: gyro calibration, complementary filter, print loop.
- Filter: `angle = 0.98 * (angle + gyro * dt) + 0.02 * accel_angle`, with `dt` from `time.ticks_us()`.
  Roll is `atan2(ay, az)` and pitch is `atan2(-ax, sqrt(ay^2 + az^2))`.
- The MPU6050 produces new data at 1 kHz with the low-pass filter on, so the loop rate can be
  higher than the true sample rate.
