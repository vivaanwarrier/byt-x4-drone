import math
import time
from machine import I2C, Pin

import config
from mpu6050 import MPU6050
from motors import Motors
from receiver import Receiver
from safety import Safety, ARMED, STATE_NAMES

RAD2DEG = 180.0 / math.pi
MOTOR_MAX_US = config.MOTOR_MIN_US + (1000 * config.TEST_THROTTLE_MAX_PCT) // 100


def make_sensor():
    i2c = I2C(config.I2C_ID, sda=Pin(config.SDA_PIN), scl=Pin(config.SCL_PIN),
              freq=config.I2C_FREQ)
    return MPU6050(i2c, config.MPU_ADDR, config.ACCEL_RANGE_G,
                   config.GYRO_RANGE_DPS, config.DLPF_CFG)


def connect():
    while True:
        try:
            return make_sensor()
        except OSError as e:
            print("IMU not responding (%s). Retrying..." % e)
            time.sleep_ms(1000)


def calibrate_gyro(imu, n):
    print("keep still")
    time.sleep_ms(500)
    sx = sy = sz = 0.0
    got = 0
    tries = 0
    while got < n and tries < n * 4:
        tries += 1
        try:
            _ax, _ay, _az, gx, gy, gz = imu.read()
        except OSError:
            continue
        sx += gx
        sy += gy
        sz += gz
        got += 1
        time.sleep_ms(2)
    if got == 0:
        raise OSError("gyro calibration got no samples")
    print("gyro bias: %.3f %.3f %.3f deg/s (%d samples)" % (sx / got, sy / got, sz / got, got))
    return sx / got, sy / got, sz / got


def accel_angles(ax, ay, az):
    roll = math.atan2(ay, az) * RAD2DEG
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * RAD2DEG
    return roll, pitch


def throttle_to_us(rx_us):
    t = rx_us - 1000
    if t < 0:
        t = 0
    elif t > 1000:
        t = 1000
    return config.MOTOR_MIN_US + t * (MOTOR_MAX_US - config.MOTOR_MIN_US) // 1000


def run(motors):
    imu = connect()
    bx, by, bz = calibrate_gyro(imu, config.CAL_SAMPLES)
    rx = Receiver(config.RX_PIN)
    safety = Safety()

    alpha = config.ALPHA
    print_period_us = 1000000 // config.PRINT_HZ
    end = "\r" if config.PRINT_INPLACE else "\n"

    # Seed the filter from the accelerometer.
    while True:
        try:
            ax, ay, az, gx, gy, gz = imu.read()
            break
        except OSError:
            pass
    roll, pitch = accel_angles(ax, ay, az)

    t_prev = time.ticks_us()
    t_print = t_prev
    loops = 0
    errors = 0
    consecutive = 0

    print("ax(g) ay(g) az(g) | gx gy gz (deg/s) | roll pitch (deg) | rx us | motor us | state | Hz")
    while True:
        need_reinit = False
        try:
            ax, ay, az, gx, gy, gz = imu.read()
            sample = (ax, ay, az, gx - bx, gy - by, gz - bz)
            consecutive = 0
        except OSError:
            sample = None
            errors += 1
            consecutive += 1
            if consecutive >= config.ERR_REINIT:
                consecutive = 0
                need_reinit = True

        now = time.ticks_us()

        if sample is not None:
            dt = time.ticks_diff(now, t_prev) * 1e-6
            t_prev = now
            if dt <= 0 or dt > 0.1:
                dt = 0.001  # skip a long gap instead of integrating it
            gx = sample[3]
            gy = sample[4]
            acc_roll, acc_pitch = accel_angles(ax, ay, az)
            roll = alpha * (roll + gx * dt) + (1.0 - alpha) * acc_roll
            pitch = alpha * (pitch + gy * dt) + (1.0 - alpha) * acc_pitch
            loops += 1

        rx_us, rx_age = rx.read()
        state = safety.update(time.ticks_ms(), rx_us, rx_age, sample)
        if state == ARMED:
            motors.set_all(throttle_to_us(rx_us))
        else:
            motors.stop()

        if need_reinit:
            try:
                imu.init()
                t_prev = time.ticks_us()
            except OSError:
                pass

        elapsed = time.ticks_diff(now, t_print)
        if elapsed >= print_period_us:
            hz = loops * 1000000 // elapsed
            rx_txt = "----" if rx_us is None or rx_age > config.RX_TIMEOUT_MS else "%4d" % rx_us
            why = safety.reason
            print("%+6.3f %+6.3f %+6.3f | %+8.2f %+8.2f %+8.2f | %+7.2f %+7.2f | rx %s | m %4d | %-6s %-11s | %5d Hz err %-4d"
                  % (ax, ay, az, gx, gy, sample[5] if sample else 0.0, roll, pitch,
                     rx_txt, motors.us, STATE_NAMES[state], why, hz, errors), end=end)
            loops = 0
            t_print = now


def main():
    # Output the off pulse first so the ESC sees a valid signal while the IMU starts.
    motors = Motors(config.MOTOR_PINS, config.PWM_HZ, config.MOTOR_MIN_US, MOTOR_MAX_US)
    try:
        run(motors)
    except KeyboardInterrupt:
        print("")
        print("stopped")
    finally:
        motors.stop()


main()
