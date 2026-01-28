#!/usr/bin/env python3
import json, time, board, busio
from adafruit_bno055 import BNO055_I2C

# Mode constants (datasheet)
MODE_CONFIG = 0x00
MODE_NDOF   = 0x0C

CAL_PATH = "/home/ros/payload/src/satellite_payload/config/bno055_cal.json"

i2c = busio.I2C(board.SCL, board.SDA)
# ADR tied high → 0x28
bno = BNO055_I2C(i2c, address=0x28)

# Load saved calibration blob
with open(CAL_PATH) as f:
    cal = json.load(f)

# Enter config to write offsets
bno.mode = MODE_CONFIG
time.sleep(0.05)

# Write offsets/radii
bno.offsets_accelerometer = tuple(cal["accel_offset"])
bno.offsets_gyroscope    = tuple(cal["gyro_offset"])
bno.offsets_magnetometer = tuple(cal["mag_offset"])
bno.radius_accelerometer = int(cal["accel_radius"])
bno.radius_magnetometer  = int(cal["mag_radius"])

# Back to fusion mode
bno.mode = MODE_NDOF
time.sleep(0.3)

print("Restored calibration. Status:", bno.calibration_status)
