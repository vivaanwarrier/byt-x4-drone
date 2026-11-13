# All tunable values in one place.

# ---- I2C / IMU (MPU6050) ----
I2C_ID = 0
SDA_PIN = 8          # GP8, header pin 11
SCL_PIN = 9          # GP9, header pin 12
I2C_FREQ = 400000    # 400 kHz
MPU_ADDR = 0x68      # 0x68 (AD0 low) or 0x69 (AD0 high)

ACCEL_RANGE_G = 4        # +/- 4 g
GYRO_RANGE_DPS = 500     # +/- 500 deg/s
DLPF_CFG = 3             # digital low-pass filter, about 42 Hz

CAL_SAMPLES = 500        # gyro bias samples at boot
ALPHA = 0.98             # complementary filter: weight of the gyro angle

# ---- Motors (ESC signal wires) ----
MOTOR_PINS = (10, 11, 12, 13)   # GP10..GP13, header pins 14, 15, 16, 17
PWM_HZ = 50                     # standard ESC PWM, 1000-2000 us pulses
MOTOR_MIN_US = 1000             # pulse for motor off
# Test cap: at full throttle stick the motors only get this % of the range.
TEST_THROTTLE_MAX_PCT = 20

# ---- Receiver (one RC PWM channel, the throttle channel) ----
RX_PIN = 7                      # GP7, header pin 10
RX_TIMEOUT_MS = 500             # no pulse for this long = signal lost
THROTTLE_LOW_US = 1050          # throttle must be below this to arm

# ---- Arming switch (physical toggle between this GPIO and GND) ----
ARM_SWITCH_PIN = 15             # GP15, header pin 20. Closed to GND = armed

# ---- Jolt cutoff ----
JOLT_G = 3.0            # total acceleration above this many g trips the cutoff
JOLT_DPS = 450          # any gyro axis above this many deg/s trips the cutoff
JOLT_SAMPLES = 2        # consecutive bad samples needed
IMU_ERR_LIMIT = 10      # consecutive failed IMU reads while armed = cutoff

# ---- Output ----
PRINT_HZ = 10
PRINT_INPLACE = True    # False prints a new line each time (use in Thonny if the line scrolls)
ERR_REINIT = 50         # consecutive I2C failures before the IMU is re-initialised
