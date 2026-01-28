#!/usr/bin/env python3

# ROS2 telemetry uploader node
# Establish mqtt connection with broker 
# Publish telemetry data to mqtt telemetry topic
# Publish image captured on command from ground station


# Import Libraries
import os # For file handling
import json                         #Convert to/from python dictionary and json file
import time                         #For timestamp
import math                         #Python math function 
import socket                       #Find local host ip 
from collections import deque       #Provide queing 
from pathlib import Path            #Find filepath
from sensor_msgs.msg import Image   #Standard ROS2 message type for images
from cv_bridge import CvBridge      #Convert between ROS Image message type and OpenCV images
import cv2                          #OpenCV for image reading and conversion
import base64                       #For encoding/decoding base64 image string
import rclpy                        #ROS client library for Python
from rclpy.node import Node         #Base class of all ROS2 nodes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  #Set Quality of Service policies for ROS2 messages

from sensor_msgs.msg import Imu, Temperature, RelativeHumidity, FluidPressure  #Standard ROS2 message type for imu, etmp, humidity, pressure
from std_msgs.msg import Bool, String                                           #For control message FLASH and REMOTE_FLASH, and fusion state json string output

#Importing MQTT Broker
import paho.mqtt.client as mqtt

def ros_time_ns(node):
    t = node.get_clock().now().to_msg()
    return t.sec * 1_000_000_000 + t.nanosec

def to_float(x):
    try: return float(x)
    except: return None

#Define Telemetry Node  
class TelemetryUploader(Node):
    def __init__(self):
        super().__init__('telemetry_uploader')

       # ------------ Parameters ------------
        self.declare_parameter('broker_host', 'localhost')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('topic_root', 'sat/telemetry')

        # Enable/disable optional feeds if not present in your setup
        self.declare_parameter('use_fusion', False)
        self.declare_parameter('use_event', False)

        # Throttle upload rates for streams
        self.declare_parameter('imu_upload_hz', 10.0)   # 10 Hz uplink for IMU
        self.declare_parameter('env_push_hz', 1.0)      # 1 Hz combined env JSON

        # Offline buffering
        self.declare_parameter('ring_max', 2000)
        self.declare_parameter('offline_csv', str(Path.home() / 'sat_offline.csv'))

        # Declare publisher
        self.pub_camera_enable = self.create_publisher(String, '/camera_enable', 10)

        # Read params
        self.host = str(self.get_parameter('broker_host').value)
        self.port = int(self.get_parameter('broker_port').value)
        self.root = str(self.get_parameter('topic_root').value).rstrip('/')
        self.use_fusion = bool(self.get_parameter('use_fusion').value)
        self.use_event  = bool(self.get_parameter('use_event').value)
        self.imu_dt = max(0.01, 1.0 / float(self.get_parameter('imu_upload_hz').value))
        self.env_dt = max(0.1,  1.0 / float(self.get_parameter('env_push_hz').value))
        self.ring_max = int(self.get_parameter('ring_max').value)
        self.offline_csv = Path(str(self.get_parameter('offline_csv').value))

        # ------------ MQTT client ------------
        self.connected = False
        client_id = f"uploader-{socket.gethostname()}-{os.getpid()}"
        self.mq = mqtt.Client(client_id=client_id, clean_session=True)
        self.mq.on_connect = self._on_connect
	    
        self.mq.on_message = self._on_message
        self.mq.on_disconnect = self._on_disconnect
        self.mq.reconnect_delay_set(min_delay=1, max_delay=60)  # exponential backoff
        try:
            self.mq.connect_async(self.host, self.port, keepalive=30)
            self.mq.loop_start()
        except Exception as e:
            self.get_logger().warn(f"MQTT connect_async failed: {e}")

        self.out_q = deque(maxlen=self.ring_max)

        # ------------ ROS subscriptions ------------
        # QoS best_effort for high-rate streams (drop-tolerant)
        qos_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self.create_subscription(String, '/fusion_state', self.onProcessNodeReceived, qos_best)
        self.create_subscription(Image, '/remote_payload_image', self.onProcessRemoteImage, qos_best)
        #publisher for taking remote picture through cam node
        self.remote_ask_picture = self.create_publisher(String, '/remote_camera_enable', 10)

        # Periodic flush of offline CSV if we reconnect
        self.create_timer(2.0, self._drain_offline_csv)

        self.get_logger().info(f"TelemetryUploader started → MQTT {self.host}:{self.port} root={self.root}")
        
    #Publish payload controlled by scheduler
    def onProcessNodeReceived(self, msg: String):
        try:
            js = json.loads(msg.data)
            self._publish(self.root + '/DataReady', js, qos=1)
        except Exception as e:
            self.get_logger().warn(f"Fusion JSON decode error: {e}")
            return
    # ---------- MQTT helpers ----------
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        self.connected = True
        self.get_logger().info(f"MQTT connected rc={rc}")
        # Try to drain any memory queue immediately
        self._drain_ring()
        self.mq.subscribe('sat/cmd/capture')

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8').strip() or '{}'
            data = json.loads(payload)
        except Exception as e:
            self.get_logger().warn(f'Bad JSON on {msg.topic}: {e}')
        # Send control signal to enable camera when remote command is received
        if data["Command"] == "Flash":
            self.remote_ask_picture.publish(String(data="REMOTE_FLASH"))


    # Image process for remote command capture
    # Publish image once image captured 
    def onProcessRemoteImage(self, msg: Image):
        #complted with image taken to earth
        self.get_logger().info("Picture taken")

        # Convert ROS Image to OpenCV image (assuming msg is 'bgr8')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Encode as JPEG to reduce size
        success, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 30])
        if not success:
            self.get_logger().error("Failed to encode image")
            return

        # Convert to base64 string
        img_b64 = base64.b64encode(buffer).decode('utf-8')
        payloadImage={}

        # Add to payload and publish
        payloadImage["ts"]= time.time()
        payloadImage["Image"] = img_b64

        jsonImage=json.dumps(payloadImage)
        try:
            js = json.loads(jsonImage)
            self._publish(self.root + '/DataReady', js, qos=1)
        except Exception as e:
            self.get_logger().warn(f"Fusion JSON decode error: {e}")
            return

    def _on_disconnect(self, client, userdata, rc, properties=None):
        self.connected = False
        self.get_logger().warn(f"MQTT disconnected rc={rc}")

    # Function to publish json payload to specified topic 
    # Message payload placed into queue if not connected to broker
    # Append current topic and payload for data logging in CSV file
    def _publish(self, topic, obj, qos=0):
        payload = json.dumps(obj, separators=(',', ':'))
        if not self.connected:
            self._buffer(topic, payload, qos)
            self._append_offline(topic, payload)
            return
        try:
            res = self.mq.publish(topic, payload, qos=qos)
            if res.rc != mqtt.MQTT_ERR_SUCCESS:
                # Treat as failure; buffer & CSV
                self._buffer(topic, payload, qos)
                self._append_offline(topic, payload)
        except Exception as e:
            self.get_logger().warn(f"MQTT publish error: {e}")
            self._buffer(topic, payload, qos)
            self._append_offline(topic, payload)


    #Append payload to a queue
    #Occurs when mqtt publish error
    def _buffer(self, topic, payload, qos):
        try:
            self.out_q.append((topic, payload, qos))
        except Exception:
            # deque full already; also append to disk
            self._append_offline(topic, payload)

    # Drain memory queue  
    def _drain_ring(self):
        if not self.connected: return
        while self.out_q:
            t, p, q = self.out_q[0]
            res = self.mq.publish(t, p, qos=q)
            if res.rc == mqtt.MQTT_ERR_SUCCESS:
                self.out_q.popleft()
            else:
                break  # stop if we fail again


    # Append topic and payload to csv for logging
    def _append_offline(self, topic, payload):
        try:
            self.offline_csv.parent.mkdir(parents=True, exist_ok=True)
            with self.offline_csv.open('a') as f:
                f.write(f"{int(time.time()*1000)},{topic},{payload}\n")
        except Exception as e:
            self.get_logger().warn(f"Offline CSV append failed: {e}")

    # Drain payload log stored in offline csv file 
    def _drain_offline_csv(self):
        # Only attempt if connected and file exists
        if not self.connected or not self.offline_csv.exists():
            return
        try:
            lines = self.offline_csv.read_text().splitlines()
            if not lines: return
            ok = True
            for ln in lines:
                try:
                    _, topic, payload = ln.split(',', 2)
                    res = self.mq.publish(topic, payload, qos=1)
                    if res.rc != mqtt.MQTT_ERR_SUCCESS:
                        ok = False
                        break
                except Exception:
                    continue
            if ok:
                # Truncate after successful flush
                self.offline_csv.write_text('')
                self.get_logger().info('Flushed offline CSV.')
        except Exception as e:
            self.get_logger().warn(f"Offline CSV drain failed: {e}")


def main():
    rclpy.init()
    node = TelemetryUploader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
