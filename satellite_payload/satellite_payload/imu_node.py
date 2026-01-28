#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import board, busio
from adafruit_bno055 import BNO055_I2C

def safe_tuple(t):
    return t if (t is not None and all(v is not None for v in t)) else None

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Parameters
        self.declare_parameter('frame_id', 'sat_imu_link')
        self.declare_parameter('address', 40)       # 41 = 0x29 #changed to 40 to use address 28
        self.declare_parameter('rate_hz', 2)
        self.declare_parameter('min_calib', 0)      # 0..3 (publish even if under this)

        self.frame_id = self.get_parameter('frame_id').value
        addr_dec = int(self.get_parameter('address').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.min_calib = int(self.get_parameter('min_calib').value)
        period = max(0.001, 1.0 / max(1e-6, rate_hz))

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO055_I2C(i2c, address=addr_dec)
            self.get_logger().info(f'BNO055 initialised at 0x{addr_dec:02X}')
        except Exception as e:
            self.get_logger().error(f'BNO055 init failed: {e}')
            raise
        #publisher
        self.pub = self.create_publisher(Imu, '/payload_imu', 10)

        self.sensor_enabled = False
        #subscription to activate sensor only whn requested
        self.create_subscription(String, '/sensor_enable', self.control_cb, 10)
        self.timer = self.create_timer(period, self.read)
        
        self._last_cal_log = 0.0
    #control request of IMU data
    def control_cb(self, msg: String):
        if msg.data.upper() == "ON":
            self.sensor_enabled = True
        else:
            self.sensor_enabled = False
    def read(self):
        if not self.sensor_enabled:
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # Unknown covariances by default per REP-145
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        try:
            # Read calibration tuple (sys, gyro, accel, mag)
            cal = getattr(self.bno, 'calibration_status', None)
            now = time.time()
            if cal and now - self._last_cal_log > 5.0:
                self.get_logger().info(f'Cal status: {cal}')
                self._last_cal_log = now

            # Orientation (quaternion) — only if valid and meets min calibration
            q = safe_tuple(self.bno.quaternion)
            if q and (not cal or cal[0] >= self.min_calib):
                msg.orientation.w = float(q[0])
                msg.orientation.x = float(q[1])
                msg.orientation.y = float(q[2])
                msg.orientation.z = float(q[3])
                msg.orientation_covariance[0] = 0.1  # nominal

            # Angular velocity (deg/s -> rad/s)
            g = safe_tuple(self.bno.gyro)
            if g and (not cal or cal[1] >= self.min_calib):
                msg.angular_velocity.x = math.radians(float(g[0]))
                msg.angular_velocity.y = math.radians(float(g[1]))
                msg.angular_velocity.z = math.radians(float(g[2]))
                msg.angular_velocity_covariance[0] = 0.1

            # Linear acceleration: prefer linear_acceleration; else raw acceleration
            lin = safe_tuple(self.bno.linear_acceleration)
            if not lin:
                lin = safe_tuple(self.bno.acceleration)  # includes gravity; better than nothing
            if lin and (not cal or cal[2] >= self.min_calib):
                msg.linear_acceleration.x = float(lin[0])
                msg.linear_acceleration.y = float(lin[1])
                msg.linear_acceleration.z = float(lin[2])
                msg.linear_acceleration_covariance[0] = 0.2

            # Only publish if we populated at least one field
            has_any = (
                msg.orientation_covariance[0] != -1.0 or
                msg.angular_velocity_covariance[0] != -1.0 or
                msg.linear_acceleration_covariance[0] != -1.0
            )
            if has_any:
                self.pub.publish(msg)
        except Exception as e:
            # Soft-fail bad reads; keep node alive
            self.get_logger().warn(f'BNO055 read failed: {e}')

def main():
    rclpy.init()
    node = ImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
