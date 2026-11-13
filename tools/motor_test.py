# Motor wiring test. No receiver, IMU or safety logic: just the Pico, ESC and motor.
#
# PROPS OFF. Motor clamped. Hands clear.
#
# Run it from Thonny: File > Open > This computer > tools/motor_test.py, then F5.
# (Or: mpremote run tools/motor_test.py)
#
# It holds 1000 us on GP10-GP13 so the ESC can arm, then slowly ramps ONE pin at a
# time from 1000 up to RAMP_MAX_US and back, printing the pulse width. Whichever pin
# makes the motor spin is the signal wire that belongs to that motor.
# Stop with the red Stop button or Ctrl-C: every output goes back to 1000 us.

from machine import Pin, PWM
import time

PINS = (10, 11, 12, 13)
PWM_HZ = 50            # try 400 if the ESC never reacts at 50
MIN_US = 1000
RAMP_MAX_US = 1300     # hard top for this test (30% throttle). Some ESCs only start above ~1100
STEP_US = 5
STEP_MS = 150
ARM_WAIT_S = 10        # time at 1000 us while you plug in the LiPo


def main():
    pwms = [PWM(Pin(p), freq=PWM_HZ, duty_ns=MIN_US * 1000) for p in PINS]

    def set_us(pwm, us):
        if us > RAMP_MAX_US:
            us = RAMP_MAX_US
        pwm.duty_ns(us * 1000)

    try:
        print("Sending 1000 us (off) on GP%s." % ", GP".join(str(p) for p in PINS))
        print("Plug in the LiPo NOW if it is not already. Wait for the ESC beeps.")
        for s in range(ARM_WAIT_S, 0, -1):
            print("  starting in %d s..." % s)
            time.sleep(1)

        for pin, pwm in zip(PINS, pwms):
            print("")
            print("=== GP%d: ramping up. Watch the motor. ===" % pin)
            us = MIN_US
            while us < RAMP_MAX_US:
                us += STEP_US
                set_us(pwm, us)
                print("GP%d  %d us" % (pin, us))
                time.sleep_ms(STEP_MS)
            time.sleep(1)
            while us > MIN_US:
                us -= STEP_US * 2
                if us < MIN_US:
                    us = MIN_US
                set_us(pwm, us)
                time.sleep_ms(STEP_MS // 2)
            print("GP%d back to 1000 us" % pin)
            time.sleep(2)
        print("")
        print("Done. If the motor never moved on any pin, see the checklist.")
    except KeyboardInterrupt:
        print("stopped")
    finally:
        for pwm in pwms:
            pwm.duty_ns(MIN_US * 1000)


main()
