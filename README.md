# byt-x4-drone

Quadcopter flight controller for ECE 5, running MicroPython on a Raspberry Pi Pico 2 (RP2350).

Current stage: bench test with props off. It reads an MPU6050, reads one RC channel
(throttle) from a FlySky receiver, and drives the ESC outputs through an arming switch
with failsafes. There is no stabilization yet: all motors get the same throttle.

## Wiring

| Part | Wire | Pico 2 pin | Header pin |
|------|------|------------|------------|
| MPU6050 | VCC | VSYS | 39 |
| MPU6050 | GND | any GND | 38 |
| MPU6050 | SDA | GP8 | 11 |
| MPU6050 | SCL | GP9 | 12 |
| Receiver | throttle channel signal | GP7 | 10 |
| Receiver | 5 V | VBUS | 40 |
| Receiver | GND | any GND | |
| ESC 1 | signal | GP10 | 14 |
| ESC 2 | signal | GP11 | 15 |
| ESC 3 | signal | GP12 | 16 |
| ESC 4 | signal | GP13 | 17 |
| ESC (each) | signal ground | any GND | e.g. 18 |
| Arming switch | one leg | GP15 | 20 |
| Arming switch | other leg | any GND | e.g. 18 |

Notes:

- **IMU on VSYS:** VSYS is about 4.7 V from USB. This only works if the MPU6050 breakout has an
  onboard 3.3 V regulator (GY-521 boards do) and its VCC input is rated for 5 V. A bare
  3.3 V-only module must go on 3V3(OUT), pin 36.
- **Grounds:** any GND pin works. They are all the same net (pins 3, 8, 13, 18, 23, 28, 38;
  pin 33 is AGND, avoid it for digital use). What matters is that the IMU, receiver, arming switch
  and every ESC signal ground share one common ground with the Pico, and the ESC battery
  negative is the same ground.
- The receiver pin must be set to give a plain RC PWM channel (a 1000-2000 us pulse about every
  20 ms), for example the throttle channel (CH3) of an FS-iA6B. PPM and iBUS are not decoded here.
- The arming switch is a plain toggle switch between GP15 and GND. Open or unplugged means off.

## Arming and safety

Motors only run when armed. To arm:

1. Switch off, throttle stick down, receiver on and bound.
2. Flip the arming switch on. It arms only if the switch was seen off since power-up, the
   receiver signal is live, throttle is below 1050 us, and the IMU is working and still.
3. Raise throttle. Output goes from 1000 us up to a capped maximum
   (`TEST_THROTTLE_MAX_PCT`, 20% by default).

Anything below cuts the motors to 1000 us:

| Cause | Result | Reset |
|-------|--------|-------|
| Arming switch off | DISARM | flip on again |
| Jolt: total acceleration above `JOLT_G` (3.0 g) or any gyro axis above `JOLT_DPS` (450 deg/s) for 2 samples in a row | TRIPPED (jolt) | switch off, then on |
| Receiver signal missing for 500 ms | TRIPPED (rx lost) | switch off, then on |
| IMU reads fail 10 times in a row | TRIPPED (imu lost) | switch off, then on |
| Ctrl-C or error in the program | motors set to 1000 us | |

Tune `JOLT_G` and `JOLT_DPS` in `config.py`. Spinning props can add vibration, so test on the
bench before trusting the values.

**Props off for every bench test.** Clamp the motor down. Connect the LiPo last and unplug it first.

## Setup (Mac)

```
brew install git gh pipx
pipx install mpremote
pipx ensurepath
```

Use a data USB cable. Flash MicroPython once: hold BOOTSEL while plugging in the Pico, then copy
the `RPI_PICO2` `.uf2` from https://micropython.org/download/RPI_PICO2/ onto the drive that
appears. The board reboots into MicroPython.

## Run with Thonny

Six files must be on the Pico: `config.py`, `mpu6050.py`, `motors.py`, `receiver.py`,
`safety.py` and `main.py`.

1. Install Thonny, plug in the Pico, and set the interpreter (bottom right) to
   **MicroPython (Raspberry Pi Pico)**.
2. Turn on **View > Files**. The top pane shows this computer, the bottom pane shows the Pico.
3. In the top pane, go to the `byt-x4-drone` folder.
4. For each of the six files: right-click it and choose **Upload to /**.
5. Open `main.py` and press the green Run button (F5). Press the red Stop button to end it.
   Stopping sets the motors back to 1000 us.
6. To run on every power-up, keep `main.py` on the Pico. It starts when the board gets power.

If the status line scrolls instead of updating in place, set `PRINT_INPLACE = False` in
`config.py` and upload it again. After you change any file, upload it to the Pico again.

## Run with the terminal

```
mpremote run tools/i2c_scan.py   # find which pins the IMU answers on
tools/run.sh                     # copy files and run main.py, output here
tools/deploy.sh                  # copy files so main.py runs on boot
```

`tools/i2c_scan.py` tries every valid I2C pin pair (skipping GP7, GP10-GP13 and GP15) and prints
the pins and addresses that answer. If the IMU answers on other pins than GP8/GP9, edit
`SDA_PIN`, `SCL_PIN` and `I2C_ID` in `config.py`.

## What gets printed

```
+0.004 -0.002 +1.000 |    +0.00    -0.00    -0.00 |   -0.14   -0.21 | rx 1000 | m 1000 | DISARM  throttle up | 812 Hz err 0
```

| Field | Meaning |
|-------|---------|
| ax ay az | acceleration in g. Flat and still, az is about +1.0 |
| gx gy gz | rotation rate in deg/s after bias removal. About 0 when still |
| roll, pitch | angle in degrees from the complementary filter. Flat is about 0, on its edge about +/-90 |
| rx | throttle pulse from the receiver in us, or ---- if there is no signal |
| m | pulse sent to the motors in us (1000 = off) |
| state | DISARM, ARMED or TRIPPED, plus the reason arming is blocked or why it tripped |
| Hz | measured loop rate |
| err | count of I2C read errors since start |

On boot it prints "keep still" and averages 500 gyro samples to get the gyro bias. Do not move
the board until the readings start. The motors get the 1000 us off pulse from the first moment
so the ESC can arm.

## Code layout

- `config.py`: pins, I2C frequency, filter constants, throttle cap, jolt thresholds, print rate.
- `mpu6050.py`: IMU driver. Wakes the chip, checks WHO_AM_I, sets +/-4 g and +/-500 deg/s, enables
  the ~42 Hz low-pass filter, reads all 14 data bytes in one burst.
- `receiver.py`: measures one RC PWM channel with a pin interrupt.
- `motors.py`: 50 Hz ESC pulses on GP10-GP13.
- `safety.py`: arming switch, receiver failsafe, jolt cutoff.
- `main.py`: gyro calibration, complementary filter (`0.98 * (angle + gyro * dt) + 0.02 * accel_angle`),
  safety, motor output and the status line.
