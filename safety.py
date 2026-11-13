import config

DISARMED = 0
ARMED = 1
TRIPPED = 2
STATE_NAMES = ("DISARM", "ARMED", "TRIPPED")


class Safety:
    """Throttle-low arming, signal-loss failsafe and jolt cutoff.

    Rules:
    - Motors only run in ARMED.
    - Arming needs: a live receiver signal, throttle below THROTTLE_LOW_US,
      a working IMU and no motion right now. So if the throttle stick is up
      at power-up, nothing runs until it is brought down.
    - A jolt, lost IMU or lost receiver signal while armed trips TRIPPED.
    - TRIPPED stays until the throttle stick is held low for RESET_LOW_MS.
    """

    def __init__(self):
        self.state = DISARMED
        self.reason = ""
        self._jolts = 0
        self._errs = 0
        self._low_since = None
        self._jolt_g2 = config.JOLT_G * config.JOLT_G

    def _trip(self, why):
        self.state = TRIPPED
        self.reason = why

    def update(self, now_ms, rx_us, rx_age_ms, sample):
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
        low = rx_ok and rx_us <= config.THROTTLE_LOW_US

        if low:
            if self._low_since is None:
                self._low_since = now_ms
            low_held = ((now_ms - self._low_since) & 0x3FFFFFFF) >= config.RESET_LOW_MS
        else:
            self._low_since = None
            low_held = False

        if self.state == TRIPPED:
            if not low_held:
                return self.state
            self.state = DISARMED
            self.reason = ""

        if self.state == ARMED:
            if self._jolts >= config.JOLT_SAMPLES:
                self._trip("jolt")
            elif self._errs >= config.IMU_ERR_LIMIT:
                self._trip("imu lost")
            elif not rx_ok:
                self._trip("rx lost")
            return self.state

        # DISARMED: check every arming condition.
        if not rx_ok:
            self.reason = "no rx"
        elif not low:
            self.reason = "throttle up"
        elif sample is None:
            self.reason = "no imu"
        elif self._jolts > 0:
            self.reason = "moving"
        else:
            self.state = ARMED
            self.reason = ""
        return self.state
