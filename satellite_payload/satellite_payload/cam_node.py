#!/usr/bin/env python3

#ROS2 Camera Node interfaced to Pi cam v2
#Captures image using rpicam-jpeg and publises them as ROS2 image messages
#Image capture  happens two way: Scheduled and Remote Request

#Import libraries 
import rclpy                        #ROS2 Python client library
from rclpy.node import Node         #Base class all Nodes
from sensor_msgs.msg import Image   #Standard ROS2 message type for images
from std_msgs.msg import String     #For control messages "FLASH" or "REMOTE_FLASH"
from cv_bridge import CvBridge      #Bridge between OpenCV and ROS2 image messages
import subprocess                   #To execute shell commands
import cv2                          #OpenCV for image reading and conversion
import time                         #for timing and timestamps
import os                           #for  file handling

#Define Camera Node class
class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node') #initialize ROS2 nodee named 'cam_node'

        # Declare node Parameters
        self.declare_parameter('width', 640)  #image size initialized at 640x480pixels
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 1)  #Frme rate = 1 frame per second
        
        #Retrieve parameters from ROS2 prameter server
        w = self.get_parameter('width').get_parameter_value().integer_value
        h = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value
        period = 1.0 / max(1, fps)

        #Create ROS2 Publishers for sendinf images
        self.pub = self.create_publisher(Image, '/payload_image', 10)
        self.remote_pub = self.create_publisher(Image, '/remote_payload_image', 10)
        self.bridge = CvBridge() #initialize CvBridge to convert OpenCV imge into ROS2 message
        
        #temporary file path to store captured images
        self.temp_file = "/tmp/capture.jpg"

       #Create ROS2 Subscribers for Control Signals
        self.create_subscription(String, '/camera_enable', self.control_cb, 10)
        self.create_subscription(String, '/remote_camera_enable', self.remote_control_cb, 10)

        self.get_logger().info(f"Camera node ready: {w}x{h} publishing to /payload_image ON REQUEST")

    #Callback for Scheduled camera trigger
    def control_cb(self, msg: String):
        #Triggered when a message received on /camera_enable topic
        self.get_logger().info("Recived msg")
        if msg.data.upper() == "FLASH":    #Camera is turned on only if the msg='FLASH'
            self.get_logger().info("flashing camera")
            self.camera_enabled = True
            self.capture()
        else:
            self.camera_enabled = False

   #Callback for Remote Camera Trigger
    def remote_control_cb(self, msg: String):
         #Triggered when a message received on /remote_camera_enable topic
        self.get_logger().info("Recived msg")
        if msg.data.upper() == "REMOTE_FLASH": #Camera is turned on only if the msg='FLASH'
            self.get_logger().info("flashing camera")
            self.remote_camera_enabled = True
            self.remote_capture()
        else:
            self.remote_camera_enabled = False

   #Local Image Capture Function
    def capture(self):
        #Captures image locally using the Pi camera v2 and publishes it to /payload_image
        try:
            if self.camera_enabled is False:
                return
            # Run the rpicam-jpeg command
            cmd =[
                "rpicam-jpeg",
                "-o", self.temp_file,
                "--width", "640",      # image width
                "--height", "480",     # image height
                "--quality", "30",     # image compression of 30%
                "--nopreview"
            ]
            subprocess.run(cmd, check=True)

            # Read the captured image using OpenCV
            frame = cv2.imread(self.temp_file)
            if frame is None:
                self.get_logger().warn("Failed to read captured image")
                return

            # Convert to ROS2 Image message and publish
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8') #ROS2 image in BGR 8-bit colour format
            msg.header.stamp = self.get_clock().now().to_msg()      #Timestamp the current time
            msg.header.frame_id = 'sat_cam_optical_frame'            #frame ID for camera reference
            self.pub.publish(msg)  #publish to ROS2 topic
            self.camera_enabled = False #reset flag

        except Exception as e:
            self.get_logger().warn(f"Capture failed: {e}")
    
   #Remote Image Capture Function
    def remote_capture(self):
        #Captures image locally using the Pi camera v2 and publishes it to /remote_payload_image
        try:
            if self.remote_camera_enabled is False:
                return
            # Run the rpicam-jpeg command
            cmd = [
                "rpicam-jpeg",
                "-o", self.temp_file,
                "--width", "640",      # small width
                "--height", "480",     # small height
                "--quality", "30",     # high compression
                "--nopreview"
            ]
            subprocess.run(cmd, check=True)

            # Read the captured image using OpenCV
            frame = cv2.imread(self.temp_file)
            if frame is None:
                self.get_logger().warn("Failed to read captured image")
                return

            # Convert to ROS2 Image message andd publish
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'sat_cam_optical_frame'
            self.remote_pub.publish(msg)
            self.remote_camera_enabled = False  #reset flag

        except Exception as e:
            self.get_logger().warn(f"Capture failed: {e}")   #If the image capture fails

   #Node Cleanup Function
    def destroy_node(self):
        #cleans up temporary files andd safely shuts down the node
        try:
            if os.path.exists(self.temp_file):
                os.remove(self.temp_file)
        except Exception:
            pass
        super().destroy_node()

#Main Entry Point
def main():
    #Main Function to initialize and spin the ROS2 camera  node
    rclpy.init()
    node = CamNode()
    try:
        rclpy.spin(node)    #keep node running until interrupted
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()    #clean shutdown


if __name__ == '__main__':
    main()
