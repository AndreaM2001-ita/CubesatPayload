#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial, os, sys

def sensor_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')

        # Publishers
        self.pub = self.create_publisher(String, '/scheduler_cmd', 10)
        qos = sensor_qos()
        self.create_subscription(String, '/ackScheduler', self.send_stop, qos)

        # Timers
        self.timer_period = 30.0   # seconds (for testing — can be changed to 30*60 for 30 min)
        self.watchdog_period = 0.1 # seconds
        self.create_timer(self.timer_period, self.send_go)
        self.create_timer(self.watchdog_period, self.check_watchdog)

        # Serial setup
        self.ser = self.open_port()

        self.get_logger().info("Scheduler node running: GO every 30s, watchdog check every 0.1s.")

    #activate fusion node read
    def send_go(self):
        msg = String()
        msg.data = "GO"
        self.pub.publish(msg)
        now = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.get_logger().info(f"[{now}] Sent GO signal to FusionNode.")

    #acknoledgement hence stopping
    def send_stop(self, ack: String):
        if ack.data.strip() == "ACK":
            now = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            self.get_logger().info(f"[{now}] Received ACK from FusionNode — collection started.")

    #open uart port for stm32 watchdog
    def open_port(self):
        while True:
            try:
                os.system("sudo chmod 666 /dev/ttyAMA0") # require sudo permission
                #use in temrinal sudo usermod -aG dialout $USER
                ser = serial.Serial(
                    '/dev/ttyAMA0',
                    baudrate=9600,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05,   # short read timeout
                )
                self.get_logger().info(f"Opened /dev/ttyAMA0 at 9600 baud.")
                return ser
            except serial.SerialException as e:
                self.get_logger().warn(f"Waiting for STM32... ({e})")
                time.sleep(2)

    def check_watchdog(self):
        """Runs every 0.1s to respond to STM32 watchdog"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("UART not open. Retrying...")
            try:
                self.ser = self.open_port()
            except Exception as e:
                self.get_logger().error(f"Failed to reopen UART: {e}")
            return

        try:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    #respond if somethign was recieved
                    self.get_logger().info(f"STM32 → Pi: {data}")
                    reply = "Yes\r\n"
                    self.ser.write(reply.encode())
                    self.get_logger().info("Pi → STM32: Yes")
        except (serial.SerialException, OSError) as e:
            # Handles device disconnection, IO errors, etc.
            self.get_logger().error(f"UART error ({type(e).__name__}): {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None  # force reopen next cycle

def main():
    rclpy.init()
    node = SchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
