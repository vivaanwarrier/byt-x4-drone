import time
from machine import Pin


class Receiver:
    """Measures one RC PWM channel (a 1000-2000 us high pulse about every 20 ms)."""

    def __init__(self, pin_num):
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_DOWN)
        self.width = 0
        self.t_rise = 0
        self.t_last = None
        self.pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._edge)

    def _edge(self, pin):
        now = time.ticks_us()
        if pin.value():
            self.t_rise = now
        else:
            w = time.ticks_diff(now, self.t_rise)
            if 800 <= w <= 2200:
                self.width = w
                self.t_last = now

    def read(self):
        """Return (pulse_us, age_ms). pulse_us is None until a pulse was seen."""
        t = self.t_last
        if t is None:
            return None, 99999
        return self.width, time.ticks_diff(time.ticks_us(), t) // 1000
