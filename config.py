# All tunable values for the IMU test in one place.

# I2C bus (set these from the result of tools/i2c_scan.py)
I2C_ID = 0
SDA_PIN = 4          # GP4, header pin 6
SCL_PIN = 5          # GP5, header pin 7
I2C_FREQ = 400000    # 400 kHz

# MPU6050 address: 0x68 (AD0 low) or 0x69 (AD0 high)
MPU_ADDR = 0x68

# Sensor ranges
ACCEL_RANGE_G = 4        # +/- 4 g
GYRO_RANGE_DPS = 500     # +/- 500 deg/s
DLPF_CFG = 3             # digital low-pass filter, about 42 Hz

# Gyro bias calibration
CAL_SAMPLES = 500

# Complementary filter: weight of the integrated gyro angle
ALPHA = 0.98

# Terminal output rate
PRINT_HZ = 10

# Consecutive I2C failures before the sensor is re-initialised
ERR_REINIT = 50
