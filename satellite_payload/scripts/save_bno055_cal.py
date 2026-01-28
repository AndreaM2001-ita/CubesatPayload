#!/usr/bin/env python3
import time, json, board, busio
from adafruit_bno055 import BNO055_I2C

i2c = busio.I2C(board.SCL, board.SDA)
# NOTE: ADR tied high → I²C address 0x28  #chaged to 28
bno = BNO055_I2C(i2c, address=0x28)

print("Waiting for full calibration (3,3,3,3)...")
while True:
    cal = bno.calibration_status  # (sys, gyro, accel, mag)
    print("Cal:", cal)
    if cal == (3, 3, 3, 3):
        break
    time.sleep(0.5)

cal_blob = {
    "accel_offset":  bno.offsets_accelerometer,   # (x, y, z)
    "gyro_offset":   bno.offsets_gyroscope,       # (x, y, z)
    "mag_offset":    bno.offsets_magnetometer,    # (x, y, z)
    "accel_radius":  bno.radius_accelerometer,    # int
    "mag_radius":    bno.radius_magnetometer      # int
}

out_path = "/home/ros/payload/src/satellite_payload/config/bno055_cal.json"
with open(out_path, "w") as f:
    json.dump(cal_blob, f)

print(f"Saved calibration to {out_path}")
