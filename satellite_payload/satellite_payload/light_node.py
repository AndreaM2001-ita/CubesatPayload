#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Illuminance
import board
import busio
import adafruit_tsl2561
from std_msgs.msg import String

#Define cam node
class LightNode(Node):
    def __init__(self):
        super().__init__('light_node')

        # Declare parameters
        self.declare_parameter('address', 0x29)  # Default I2C address for TSL2561
        self.declare_parameter('rate_hz', 2.0)  # Default rate
        self.declare_parameter('frame_id', 'light_sensor_link')

        # Get parameters
        address = self.get_parameter('address').get_parameter_value().integer_value
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Init I2C and sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_tsl2561.TSL2561(i2c, address=address)

        # Publisher
        self.pub = self.create_publisher(Illuminance, '/payload_light', 10)

        self.sensor_enabled = False
        self.create_subscription(String, '/sensor_enable', self.control_cb, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_light)

        # Timer for publishing
        #self.timer = self.create_timer(1.0 / rate_hz, self.publish_light)

        self.get_logger().info(f"Light sensor node started on address=0x{address:X}, "
                               f"rate={rate_hz} Hz, frame_id={self.frame_id}")
    #light control    
    def control_cb(self, msg: String):
        if msg.data.upper() == "ON":
            self.sensor_enabled = True
        else:
            self.sensor_enabled = False
    #return light value
    def publish_light(self):
        if not self.sensor_enabled:
            return

        lux = self.sensor.lux
        if lux is None:
            self.get_logger().warn("Light reading invalid")
            #sunlight is recorded as invalid
            # so when invalid the maximum light value should be sent
            lux= 40000
            return

        msg = Illuminance()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.illuminance = lux
        msg.variance = 0.0  # TSL2561 doesn’t provide variance
        self.get_logger().info(f'Illuminance: {msg.illuminance}')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
