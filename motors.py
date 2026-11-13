from machine import Pin, PWM


class Motors:
    """Standard 50 Hz ESC pulses (1000-2000 us) on one or more pins."""

    def __init__(self, pins, freq, min_us, max_us):
        self.min_us = min_us
        self.max_us = max_us
        self.us = min_us
        # Start at the off pulse immediately so the ESC can arm.
        self.pwms = [PWM(Pin(p), freq=freq, duty_ns=min_us * 1000) for p in pins]

    def set_all(self, us):
        if us < self.min_us:
            us = self.min_us
        elif us > self.max_us:
            us = self.max_us
        if us != self.us:
            self.us = us
            ns = us * 1000
            for pwm in self.pwms:
                pwm.duty_ns(ns)

    def stop(self):
        self.set_all(self.min_us)
