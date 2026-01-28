#!/usr/bin/env python3

#ROS2 Environmenttal Sensor Node (env_node)
#reads temperature, humidity, and pressure data from BME280 sensor 
#via i2c interface (address x77) and publishes them as standard ROS2 messages

# /payload_temperature: senor_msgs/Temperature
# /payload_humidity:    senor_msgs/Humidity
# /payload_pressure:    senor_msgs/Presure

#Sensor operation is toggled via /sensor_enable topic:
#    "ON"-start publishing data
#    "OFF"-stops publishing data

#importing essential libraries
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure
from std_msgs.msg import String

#Adafruit BME280 library
import board, busio
from adafruit_bme280 import basic as adafruit_bme280  # BME280 returns °C, %RH, hPa

#safely parse I2C address or fall back to default
def _parse_addr(val, default):
    if val is None:
        return default
    try:
        return int(val, 0) if isinstance(val, str) else int(val)
    except Exception:
        return default

#Safely convert value to float returning None for invalid readings
def _to_float(x):
    try:
        f = float(x)
        if math.isnan(f) or math.isinf(f):
            return None
        return f
    except Exception:
        return None

#Define Environmental Node Class
class EnvNode(Node):
    def __init__(self):
        super().__init__('env_node')

        # Parameters
        self.declare_parameter('frame_id', 'sat_env_link')
        self.declare_parameter('rate_hz', 2.0)            # publish rate (Hz)
        self.declare_parameter('bme_address', 0x77)     # 0x76 or 0x77

        #fetch parameters
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        bme_addr = _parse_addr(self.get_parameter('bme_address').value, 0x77)
        
        #Timer period for data acquisation
        period = max(0.05, 1.0 / max(rate_hz, 1e-3))

        # initialize I2C vbus and BME280 sensor
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=bme_addr)
            self.get_logger().info(f'BME280 initialised at 0x{bme_addr:02X}')
        except Exception as e:
            self.get_logger().error(f'BME280 init failed: {e}')
            raise
        
        # ROS2 Publishers
        self.pub_T = self.create_publisher(Temperature,      '/payload_temperature',   10)
        self.pub_H = self.create_publisher(RelativeHumidity, '/payload_humidity',      10)
        self.pub_P = self.create_publisher(FluidPressure,    '/payload_pressure',      10)
         
       #Subscribers to activate/deactivate the sensor: /sensor_enable 'ON' or 'OFF'
        self.sensor_enabled = False
        self.create_subscription(String, '/sensor_enable', self.control_cb, 10)
        self.timer = self.create_timer(period, self.read)
        self._last_warn = 0.0
        
    #Control callback
    def control_cb(self, msg: String):
        if msg.data.upper() == "ON":    #Sensors start publishing if msg on /sensor_enable = 'ON'
            self.sensor_enabled = True
        else:
            self.sensor_enabled = False

   #Main Sensor Read and Publish Function
    def read(self):
        #Reads BME280 data and publishes to ROS2 topics if sensor is enabled
        if not self.sensor_enabled:
            return  #skip reading if disabled 
        stamp = self.get_clock().now().to_msg()
        any_pub = False  #flag to check if any data was valid

        try:
            # Temperature (°C)
            t_c = _to_float(self.bme.temperature)
            if t_c is not None:
                t = Temperature()
                t.header.stamp = stamp; t.header.frame_id = self.frame_id
                t.temperature = t_c
                t.variance = 0.0
                self.get_logger().info(f'Temperature: {t.temperature}')
        
                self.pub_T.publish(t)
                any_pub = True

            # Relative humidity as fraction [0..1] (ROS expects fraction, not %)
            rh_pct = _to_float(self.bme.humidity)
            if rh_pct is not None:
                h = RelativeHumidity()
                h.header.stamp = stamp; h.header.frame_id = self.frame_id
                h.relative_humidity = max(0.0, min(1.0, rh_pct / 100.0))
                h.variance = 0.0
                self.get_logger().info(f'Relative Humidity: {h.relative_humidity }')
       
                self.pub_H.publish(h)
                any_pub = True

            # Pressure in Pascals (BME gives hPa)
            hpa = _to_float(self.bme.pressure)
            if hpa is not None:
                p = FluidPressure()
                p.header.stamp = stamp; p.header.frame_id = self.frame_id
                p.fluid_pressure = hpa * 100.0
                p.variance = 0.0
                self.get_logger().info(f'Pressure: {p.fluid_pressure }')
                self.pub_P.publish(p)
                any_pub = True

            #warn if no valid data was published
            if not any_pub and (time.time() - self._last_warn) > 5.0:
                self.get_logger().warn('Env read returned no valid values this cycle')
                self._last_warn = time.time()

        except Exception as e:
            #prevent repetitive warning if more than 5 times
            if (time.time() - self._last_warn) > 5.0:
                self.get_logger().warn(f'Env read failed: {e}')
                self._last_warn = time.time()

#Main Entry Point
def main():
    #Initializes and runs the env_node until interrupted
    rclpy.init()
    node = EnvNode()
    try:
        rclpy.spin(node)        #keep the node  running until Ctrl+C is pressed
    finally:
        node.destroy_node()     #clean up
        rclpy.shutdown()

if __name__ == '__main__':
    main()
