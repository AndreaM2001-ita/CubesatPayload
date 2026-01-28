# CubeSat Payload Project

A comprehensive CubeSat satellite system for environmental monitoring and imaging, featuring ROS2-based sensor fusion with ground station telemetry analysis.

## Project Overview

This project implements a complete CubeSat payload system with:
- **Satellite Payload**: Multi-sensor ROS2 nodes for data collection and transmission
- **Ground Station**: MQTT-based telemetry reception, cloud detection, and image analysis
- **Health Monitoring**: STM32-based health monitoring system

## Directory Structure

### `/satellite_payload`
ROS2-based satellite payload system with multiple sensor nodes:

**Sensor Nodes:**
- `cam_node.py` - Captures images via Raspberry Pi camera (rpicam-jpeg), supports scheduled and remote capture
- `imu_node.py` - Interfaces BNO055 IMU sensor for acceleration/gyroscope data
- `env_node.py` - Environmental sensors (temperature, humidity, pressure)
- `light_node.py` - Light/illuminance sensor data collection
- `fusion_node.py` - Fuses multi-sensor data, detects anomalies (acceleration spikes, temperature alerts, pressure drops)

**Data Transmission:**
- `telemetry_uploader.py` - MQTT bridge publishing sensor/image data to ground station
- `scheduler_node.py` - Schedules periodic sensor polling and image capture

**Configuration:**
- `config/bno055_cal.json` - IMU calibration data
- `launch/launch_payload.py` - ROS2 launch configuration
- `package.xml` - ROS2 package metadata

### `/GroundStation`
Ground station receiver and analysis tools:

**Main Components:**
- `telemetry_reader.py` - MQTT subscriber receiving satellite telemetry, decodes JSON payload, appends data to CSV logs
- `imageAnalysis.py` - OpenCV-based cloud cover detection using HSV thresholding, generates cloud masks and contours
- `payload_records_1.csv` - Telemetry data log
- Sample image analysis results included

### `/STM32 Code - Health system`
- `STM_Health_Monitor.txt` - STM32 microcontroller firmware for CubeSat health monitoring

## Key Features

- **Multi-Sensor Fusion**: Combines IMU, environmental, and optical data with configurable anomaly thresholds
- **Robust Communication**: MQTT-based telemetry with QoS policies and base64 image encoding
- **Cloud Detection**: Automated cloud cover analysis on ground-received images
- **Flexible Configuration**: ROS2 parameters for sensor thresholds and publish rates
- **Graceful Degradation**: System functions with partial sensor availability

## Dependencies

**Satellite Payload:**
- ROS2 (rclpy, sensor_msgs, std_msgs, launch_ros)
- Python3-OpenCV, cv_bridge
- Adafruit BNO055 library (for IMU)

**Ground Station:**
- OpenCV (cv2)
- paho-mqtt
- PyYAML
- NumPy

**Hardware:**
- Raspberry Pi (camera & compute)
- BNO055 IMU sensor
- Environmental sensors (temp, humidity, pressure)
- Light sensor
- STM32 health monitoring module

## Usage

**Launch Satellite Payload:**
```bash
colcon build
source install/setup.bash
ros2 launch satellite_payload launch_payload.py
```

**Start Ground Station Telemetry Receiver:**
```bash
python3 telemetry_reader.py
```
