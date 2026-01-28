#!/usr/bin/env python3
import math, time, json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity, FluidPressure, Illuminance
from sensor_msgs.msg import Image
from std_msgs.msg import String

#image compression libs
import numpy as np
import cv2
import base64
from cv_bridge import CvBridge

def sensor_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,  # tolerate lossy sources
    )

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        #Parameters (tweak in launch)
        self.declare_parameter('accel_thresh', 1.5)   # m/s^2
        self.declare_parameter('gyro_thresh',  0.7)   # rad/s
        self.declare_parameter('temp_high',    35.0)  # °C
        self.declare_parameter('press_drop',   500.0) # Pa delta for event
        self.declare_parameter('publish_hz',   2.0)   # state publish rate
        self.declare_parameter('stale_after',  3.0)   # seconds => data stale
        self.declare_parameter('strict',       False) # if True, require ALL sensors #keep it false to be have wokring 
                                                        #code that works even without some sensor values

        # State holders
        self.last_imu = None             # (ax, ay, az, gx, gy, gz)
        self.last_temp = None            # float (°C)
        self.last_hum = None             # fraction 0..1
        self.last_press = None           # Pa
        self.prev_press = None           # Pa
        self.last_light = None           # float (lux)
        self.prev_light = None
        self.cloud_confidence=None 
        self.cloud_presence=None

        self.t_imu = self.t_temp = self.t_hum = self.t_press = self.t_light= None

        self._state = 'UNKNOWN'
        self._last_event_time = {}
        self._event_cooldown = 3.0       # seconds between repeated same events

        self._collecting = False
        self._data_buffer = []
        self._collection_timer = None
        #self._collection_duration = 10.0  # seconds
        self._collection_start = None
        self._collection_period = 0.5  # seconds between readings
        self._samples_collected = 0


        # ---- Subscriptions (work even if publishers are absent) ----
        qos = sensor_qos()
        self.create_subscription(Imu,          '/payload_imu',         self.on_imu,   qos)
        self.create_subscription(Temperature,  '/payload_temperature', self.on_temp,  qos)
        self.create_subscription(RelativeHumidity, '/payload_humidity', self.on_hum,  qos)
        self.create_subscription(FluidPressure,    '/payload_pressure', self.on_press, qos)
        self.create_subscription(String,    '/scheduler_cmd', self.on_sched_cmd, qos)
        self.create_subscription(Illuminance,'/payload_light',self.on_light,qos )
        self.create_subscription(Image, '/payload_image', self.onPictureTaken, qos)  # for testing, can be removed

        # ---- Publishers ----
        self.pub_state   = self.create_publisher(String, '/fusion_state', 10)
        self.pub_sensor_enable = self.create_publisher(String, '/sensor_enable', 10)
        self.pub_ack = self.create_publisher(String, '/ackScheduler', 10)
        self.pub_camera_enable = self.create_publisher(String, '/camera_enable', 10)
        self.pub_trigger = self.create_publisher(String, '/event_trigger', 10)  # simple String events

        self.payload = {}

        self.get_logger().info('Fusion node up: tolerant to missing/stale sensors.')

    # ------------- Sub callbacks -------------
    def on_imu(self, msg: Imu):
        if not self._collecting:
            return
        la, gv = msg.linear_acceleration, msg.angular_velocity
        self.last_imu = (la.x, la.y, la.z, gv.x, gv.y, gv.z)
        self.t_imu = time.monotonic()

    def on_temp(self, msg: Temperature):
        if not self._collecting:
            return
        self.last_temp = float(msg.temperature)
        self.t_temp = time.monotonic()

    def on_hum(self, msg: RelativeHumidity):
        if not self._collecting:
            return
        self.last_hum = float(msg.relative_humidity)
        self.t_hum = time.monotonic()

    def on_press(self, msg: FluidPressure):
        if not self._collecting:
            return
        self.prev_press = self.last_press
        self.last_press = float(msg.fluid_pressure)
        self.t_press = time.monotonic()

    def on_light(self, msg: Illuminance):
        if not self._collecting:
            return
        self.prev_light=self.last_light
        self.last_light = float(msg.illuminance)
        self.t_light = time.monotonic()

    # ------------- Timer compute -------------
    def on_sched_cmd(self, msg: String):
        
        self.get_logger().info("Scheduler GO received: starting 10-second collection.")
          
        if msg.data.upper() == "GO" and not self._collecting:
            self._data_buffer = []
            self._collecting = True
            self._collection_start = time.monotonic()
            self._samples_collected = 0 

            self.pub_ack.publish(String(data="ACK"))
            self.pub_sensor_enable.publish(String(data="ON"))

            self.last_imu = self.last_temp = self.last_hum = self.last_press = self.last_light = None
            self.prev_press = self.prev_light = None
            #Create a separate 0.5s timer for collection
            self._collection_timer = self.create_timer(self._collection_period, self.timer_cb)
    
    def onPictureTaken(self, msg: Image):
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

        # Add to payload and publish
        self.payload["Image"] = img_b64
        self.publish_state()


    def analyze_cloud_presence(self):
        beta = 17.62
        lambda_ = 243.12
        cloud_votes = 0
        no_cloud_votes = 0
        valid_samples = 0

        #  Primary check: Temp + Humidity 
        for sample in self._data_buffer:
            T = sample.get('temp')
            RH = sample.get('hum')

            if T is None or RH is None:  
            #if some of the values are missing /sensor broken for loop will only consider present values
                continue

            valid_samples += 1
            RH_percent = RH * 100.0  # assumes sensor gives 0..1

            try:
                ln_rh = math.log(RH_percent / 100.0)
                dp = (lambda_ * (ln_rh + (beta * T) / (lambda_ + T))) / \
                    (beta - (ln_rh + (beta * T) / (lambda_ + T)))
            except Exception:
                continue

            # Cloud likely: near saturation
            if abs(T - dp) <= 5.0 or RH_percent >= 70.0:
                cloud_votes += 1
            # No cloud likely: dry air or dewpoint far from T
            elif RH_percent < 70.0 or (T - dp) > 5.0:
                no_cloud_votes += 1

        if valid_samples == 0:
            return 0.0, False

        #Secondary check: light
        light=[s for s in self._data_buffer[:] if s.get('light') is not None]
        #in case ligth sensor is broken
        if light:  
            avglight = sum(s['light'] for s in light) / len(light)
            light_threshold = 13000
            max_votes = 3  # max boost for low light
            if avglight < light_threshold:
                proportion = (light_threshold - avglight) / light_threshold
                boost = 1 + proportion * (max_votes - 1)  # scales 1..3
                cloud_votes += boost
            valid_samples += boost
                
        else:
            avglight = None

        # Base confidence from humidity/temp
        if cloud_votes > no_cloud_votes:
            confidence = cloud_votes / valid_samples

            confidence = min(confidence, 1.0)
            presence = True
        else:
            confidence = no_cloud_votes / valid_samples
            presence = False

        #  third check: Pressure trend
        # Only used if confidence is not strong (<0.7)
        if confidence < 0.7:
            recent_pressures = [s['press'] for s in self._data_buffer[:] if s.get('press') is not None]
            #make sure that pressure varuibales are availble
            if len(recent_pressures) >= 2:
                dp = recent_pressures[-1] - recent_pressures[0]

                # Falling pressure supports cloud formation
                if dp < -200:  # drop of >=200 Pa across window
                    confidence = min(1.0, confidence + 0.2)

        # Round confidence to 2 decimal places
        confidence = round(confidence, 2)

        return confidence, presence
    def timer_cb(self):
        #check if in collection state otherwise skip
        if not self._collecting:
            # Skip if not collecting
            return

        now = time.monotonic()
        stale_after = float(self.get_parameter('stale_after').value)
        strict = bool(self.get_parameter('strict').value)

        missing = []
        fresh = {}

        #get fresh data
        fresh['imu'] = (self.t_imu is not None) and (now - self.t_imu <= stale_after)
        fresh['temp'] = (self.t_temp is not None) and (now - self.t_temp <= stale_after)
        fresh['hum'] = (self.t_hum is not None) and (now - self.t_hum <= stale_after)
        fresh['press'] = (self.t_press is not None) and (now - self.t_press <= stale_after)
        fresh['light'] = (self.t_light is not None) and (now - self.t_light <= stale_after)

        #sort out if missing
        for k in ['imu', 'temp', 'hum', 'press', 'light']:
            if not fresh[k]:
                missing.append(k)

        if strict and missing:
            state = f'DEGRADED ({",".join(sorted(missing))} missing/stale)'
            reasons = missing  # reasons = missing sensors

            self.payload = {
                'state': state,
                'reasons': sorted(reasons) if reasons else [],
                'fresh': None,
                'has': {
                    'imu':   self.last_imu   is not None,
                    'temp':  self.last_temp  is not None,
                    'hum':   self.last_hum   is not None,
                    'press': self.last_press is not None,
                    'light': self.last_light is not None
                },
                'values': {
                    'confidence': self.cloud_confidence,
                    'presence': self.cloud_presence,
                    'temp':  self.last_temp,
                    'hum':   self.last_hum,
                    'press': self.last_press,
                    'imu':   self.last_imu,
                    'light': self.last_light
                },
                'ts': time.time(),
                # 'Image': Image
            }

            self.publish_state()
            return

        state = 'UNKNOWN'
        reasons = []
        #determine IMU satellite motion ->do not take picture
        if fresh['imu'] and self.last_imu is not None:
            ax, ay, az, gx, gy, gz = self.last_imu
            acc_mag = math.sqrt(ax * ax + ay * ay + az * az)
            gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
            a_th = float(self.get_parameter('accel_thresh').value)
            g_th = float(self.get_parameter('gyro_thresh').value)
            if acc_mag < a_th and gyro_mag < g_th:
                state = 'NOMINAL'
            else:
                state = 'MOTION'
                if gyro_mag >= g_th * 2 or acc_mag >= a_th * 2:
                    self.maybe_trigger('EXCESS_MOTION')
        else:
            state = 'NOMINAL_ENV'
            reasons.append('imu_stale')
        
        if missing:
            state = f'{state}+DEGRADED'

        # if fresh['temp'] and self.last_temp is not None and self.last_temp >= float(self.get_parameter('temp_high').value):
        #     reasons.append('temp_high')
        #     self.maybe_trigger('TEMP_HIGH')

        # if fresh['press'] and self.last_press is not None and self.prev_press is not None:
        #     dp = abs(self.last_press - self.prev_press)
        #     if dp >= float(self.get_parameter('press_drop').value):
        #         reasons.append(f'pressure_jump|Δ={dp:.0f}Pa')
        #         self.maybe_trigger('PRESSURE_JUMP')
        
        #Record data for collection
        self._data_buffer.append({
            'ts': time.time(),
            'state': state,
            'reasons': reasons,
            'imu': self.last_imu,
            'temp': self.last_temp,
            'hum': self.last_hum,
            'press': self.last_press,
            'light': self.last_light
        })
        self._samples_collected += 1
        self.get_logger().info(f"Collected sample #{self._samples_collected}")

        # Stop collection after 20 samples (10s)
        if self._samples_collected >= 20:
            self._collecting = False
            self.get_logger().info("10-second collection complete, returning buffer.")
            print(json.dumps(self._data_buffer, indent=2))  # collected buffer
            # Destroy timer to stop further calls
            self.cloud_confidence, self.cloud_presence = self.analyze_cloud_presence()  #CLOUD DETECTION WITH SENSOR

            self.payload = {
                'state': state,
                'reasons': sorted(reasons) if reasons else [],
                'fresh': fresh,
                'has': {
                    'imu':   self.last_imu   is not None,
                    'temp':  self.last_temp  is not None,
                    'hum':   self.last_hum   is not None,
                    'press': self.last_press is not None,
                    'light': self.last_light is not None
                },
                'values': {
                    'confidence': self.cloud_confidence,
                    'presence': self.cloud_presence,
                    'temp':  self.last_temp,
                    'hum':   self.last_hum,
                    'press': self.last_press,
                    'imu':   self.last_imu,
                    'light': self.last_light
                },
                'ts': time.time(),
                #'Image': None
            }

            #TO BE USED IF EVERY READ A PICTURE IS REQUESTED
            # self.pub_camera_enable.publish(String(data="FLASH"))
            # self.get_logger().info(f"Take Picture, I think there are clouds! Confidence: {self.cloud_confidence}")


            if self.cloud_presence and self.cloud_confidence is not None and self.cloud_confidence >= 0.7:
                if state !="MOTION":
                    self.pub_camera_enable.publish(String(data="FLASH"))
                    self.get_logger().info(f"Take Picture, I think there are clouds! Confidence: {self.cloud_confidence}")
                else:
                    self.publish_state()
                    self.get_logger().info(f"Clouds were detected but the satellite is shaking")
            else:
                self.publish_state()
                self.get_logger().info(f"No clouds detected. Confidence: {self.cloud_confidence}")
            
            self._samples_collected=0
            self.pub_sensor_enable.publish(String(data="OFF"))
            self.destroy_timer(self._collection_timer)
            self._collection_timer = None
            self.get_logger().info("timer destroyed, waiting for next GO command.")
                

    # PUBLISHER
    def publish_state(self):
        self.pub_state.publish(String(data=json.dumps(self.payload)))
           

    def maybe_trigger(self, name: str):
        now = time.monotonic()
        last = self._last_event_time.get(name, 0.0)
        if now - last >= self._event_cooldown:
            self._last_event_time[name] = now
            self.pub_trigger.publish(String(data=name))
            self.get_logger().warn(f'event_trigger: {name}')

def main():
    rclpy.init()
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
