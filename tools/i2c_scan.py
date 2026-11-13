# Scan I2C on every valid I2C0/I2C1 pin pair and report which respond.
# Run with:  mpremote run tools/i2c_scan.py
#
# Pins GP10-GP13 (ESC signals), GP7 (receiver) and GP15 (arming switch) are
# never touched by this scan.

from machine import I2C, SoftI2C, Pin
import time

RESERVED = (7, 10, 11, 12, 13, 15)

# Valid hardware I2C pins on the Pico 2 (RP2350) header.
I2C_PINS = {
    0: {"sda": (0, 4, 8, 16, 20), "scl": (1, 5, 9, 17, 21)},
    1: {"sda": (2, 6, 14, 18, 26), "scl": (3, 7, 15, 19, 27)},
}

IMU_ADDRS = (0x68, 0x69)


def scan_hw():
    found = []
    for bus in (0, 1):
        for sda in I2C_PINS[bus]["sda"]:
            for scl in I2C_PINS[bus]["scl"]:
                if sda in RESERVED or scl in RESERVED:
                    continue
                try:
                    i2c = I2C(bus, sda=Pin(sda), scl=Pin(scl), freq=100000)
                    addrs = i2c.scan()
                except OSError:
                    continue
                if addrs:
                    found.append((bus, sda, scl, addrs))
                    print("I2C%d  SDA=GP%d  SCL=GP%d  ->  %s" % (
                        bus, sda, scl, ["0x%02X" % a for a in addrs]))
    return found


def scan_soft():
    # Diagnostic fallback: bit-banged I2C on any free pin pair. Finds the
    # sensor even if SDA and SCL wires are swapped or on non-I2C-capable pins.
    pins = [p for p in list(range(0, 23)) + [26, 27, 28] if p not in RESERVED]
    found = []
    for sda in pins:
        for scl in pins:
            if sda == scl:
                continue
            try:
                i2c = SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=100000, timeout=2000)
                addrs = i2c.scan()
            except OSError:
                continue
            if addrs:
                found.append((sda, scl, addrs))
                print("soft  SDA=GP%d  SCL=GP%d  ->  %s" % (
                    sda, scl, ["0x%02X" % a for a in addrs]))
    return found


print("Scanning hardware I2C pin pairs (GP7, GP10-GP13, GP15 skipped)...")
hw = scan_hw()
imu = [f for f in hw if any(a in IMU_ADDRS for a in f[3])]

if imu:
    print("")
    for bus, sda, scl, addrs in imu:
        a = [x for x in addrs if x in IMU_ADDRS][0]
        print("IMU found at 0x%02X on I2C%d: SDA=GP%d, SCL=GP%d" % (a, bus, sda, scl))
    if len(imu) > 1:
        print("Several pairs answered; a floating or shared line can do this.")
        print("Check wiring and pick the pair that matches your wires.")
else:
    print("")
    print("No MPU6050 (0x68/0x69) on any hardware I2C pin pair.")
    if hw:
        print("Other devices answered above; those are not the IMU.")
    print("Trying bit-banged I2C on free pins (catches swapped SDA/SCL)...")
    time.sleep_ms(100)
    soft = scan_soft()
    soft_imu = [f for f in soft if any(a in IMU_ADDRS for a in f[2])]
    if soft_imu:
        for sda, scl, addrs in soft_imu:
            print("IMU answers with SDA=GP%d, SCL=GP%d (bit-banged)" % (sda, scl))
        print("If SDA/SCL are the opposite of a hardware pair, the two wires are swapped.")
    else:
        print("Nothing found. Check: VCC on 3V3(OUT) pin 36, GND, solder joints,")
        print("SDA/SCL wires, and that the data USB cable powers the board.")
