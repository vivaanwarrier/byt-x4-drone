import config

DISARMED = 0
ARMED = 1
TRIPPED = 2
STATE_NAMES = ("DISARM", "ARMED", "TRIPPED")


class Safety:
    """Arming switch, signal-loss failsafe and jolt cutoff.

    Rules:
    - Motors only run in ARMED.
    - Arming needs: the switch turned off then on since power-up, a live
      receiver signal, throttle low, a working IMU and no motion right now.
    - Switch off always means DISARMED.
    - A jolt, lost IMU or lost receiver signal while armed trips TRIPPED.
      TRIPPED stays until the arming switch is turned off.
    """

    def __init__(self):
        self.state = DISARMED
        self.reason = ""
        self._switch_was_off = False
        self._jolts = 0
        self._errs = 0
        self._jolt_g2 = config.JOLT_G * config.JOLT_G

    def _trip(self, why):
        self.state = TRIPPED
        self.reason = why

    def update(self, switch_on, rx_us, rx_age_ms, sample):
        """sample is (ax, ay, az, gx, gy, gz) in g and deg/s, or None if the read failed."""
        if sample is None:
            self._errs += 1
        else:
            self._errs = 0
            ax, ay, az, gx, gy, gz = sample
            lim = config.JOLT_DPS
            if (ax * ax + ay * ay + az * az > self._jolt_g2
                    or abs(gx) > lim or abs(gy) > lim or abs(gz) > lim):
                self._jolts += 1
            else:
                self._jolts = 0

        rx_ok = rx_us is not None and rx_age_ms <= config.RX_TIMEOUT_MS

        if not switch_on:
            self._switch_was_off = True
            self.state = DISARMED
            self.reason = ""
            return self.state

        if self.state == TRIPPED:
            return self.state

        if self.state == ARMED:
            if self._jolts >= config.JOLT_SAMPLES:
                self._trip("jolt")
            elif self._errs >= config.IMU_ERR_LIMIT:
                self._trip("imu lost")
            elif not rx_ok:
                self._trip("rx lost")
            return self.state

        # DISARMED with the switch on: check every arming condition.
        if not self._switch_was_off:
            self.reason = "sw off first"
        elif not rx_ok:
            self.reason = "no rx"
        elif rx_us > config.THROTTLE_LOW_US:
            self.reason = "throttle up"
        elif sample is None:
            self.reason = "no imu"
        elif self._jolts > 0:
            self.reason = "moving"
        else:
            self.state = ARMED
            self.reason = ""
        return self.state
