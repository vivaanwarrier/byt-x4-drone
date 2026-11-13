import struct
import time

_PWR_MGMT_1 = 0x6B
_SMPLRT_DIV = 0x19
_CONFIG = 0x1A
_GYRO_CONFIG = 0x1B
_ACCEL_CONFIG = 0x1C
_ACCEL_XOUT_H = 0x3B
_WHO_AM_I = 0x75

# 0x68 on a genuine MPU6050. Common clones and MPU6500-family parts report
# other values but use the same registers.
_KNOWN_IDS = (0x68, 0x70, 0x71, 0x72, 0x73, 0x98)

_ACCEL_SEL = {2: 0, 4: 1, 8: 2, 16: 3}
_ACCEL_LSB = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
_GYRO_SEL = {250: 0, 500: 1, 1000: 2, 2000: 3}
_GYRO_LSB = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}


class MPU6050:
    def __init__(self, i2c, addr=0x68, accel_g=4, gyro_dps=500, dlpf=3):
        self.i2c = i2c
        self.addr = addr
        self._accel_g = accel_g
        self._gyro_dps = gyro_dps
        self._dlpf = dlpf
        self._accel_scale = 1.0 / _ACCEL_LSB[accel_g]
        self._gyro_scale = 1.0 / _GYRO_LSB[gyro_dps]
        self._buf = bytearray(14)
        self.who_am_i = None
        self.init()

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes((val,)))

    def init(self):
        self.who_am_i = self.i2c.readfrom_mem(self.addr, _WHO_AM_I, 1)[0]
        if self.who_am_i not in _KNOWN_IDS:
            raise OSError("unexpected WHO_AM_I 0x%02X at 0x%02X" % (self.who_am_i, self.addr))
        # Reset, then wake from sleep with the gyro X PLL as clock source.
        self._write(_PWR_MGMT_1, 0x80)
        time.sleep_ms(100)
        self._write(_PWR_MGMT_1, 0x01)
        time.sleep_ms(10)
        self._write(_CONFIG, self._dlpf & 0x07)
        self._write(_SMPLRT_DIV, 0)  # 1 kHz output rate with the filter on
        self._write(_GYRO_CONFIG, _GYRO_SEL[self._gyro_dps] << 3)
        self._write(_ACCEL_CONFIG, _ACCEL_SEL[self._accel_g] << 3)
        time.sleep_ms(50)

    def read(self):
        """Return (ax, ay, az) in g and (gx, gy, gz) in deg/s."""
        self.i2c.readfrom_mem_into(self.addr, _ACCEL_XOUT_H, self._buf)
        ax, ay, az, _t, gx, gy, gz = struct.unpack(">hhhhhhh", self._buf)
        a = self._accel_scale
        g = self._gyro_scale
        return ax * a, ay * a, az * a, gx * g, gy * g, gz * g
