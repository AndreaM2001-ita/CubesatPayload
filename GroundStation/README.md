# CubeSat Ground Station

A ground station application for receiving and processing telemetry data and images from a CubeSat satellite via MQTT. The system performs real-time cloud detection and analysis on satellite imagery while logging sensor telemetry.

## Overview

This Ground Station project handles communication with a CubeSat satellite through MQTT messaging. It receives telemetry data including environmental sensors and satellite images, performs cloud cover analysis on images, and stores all data locally for analysis and archiving.

## Project Structure

```
GroundStation/
├── telemetry_reader.py          # Main MQTT subscriber and data processor
├── imageAnalysis.py             # Cloud detection and image processing
├── payload_records_1.csv        # Historical telemetry data log
├── Sample_json_payload          # Example JSON payload structure
├── README.md                    # This file
└── Sample images/               # Example processed satellite images
    ├── Picture Sample result Inside.jpg
    └── Sample Picture Result Outside.jpg
```

## Features

### Telemetry Reception (`telemetry_reader.py`)

- **MQTT Client**: Subscribes to satellite telemetry topics via MQTT broker
- **JSON Parsing**: Decodes and extracts telemetry data from satellite payloads
- **Data Logging**: Appends sensor readings to a CSV file for historical tracking
- **Image Processing**: Receives base64-encoded satellite images and converts them to JPEG format
- **Dual Topic Support**:
  - `sat/telemetry/DataReady`: Scheduled sensor data with optional images
  - `sat/telemetry/RemoteReady`: Remote command responses with images

### Image Analysis (`imageAnalysis.py`)

- **Cloud Detection**: Uses OpenCV HSV color space thresholding to detect cloud cover
- **Sky/Cloud Segmentation**: Separates sky and cloud pixels using configurable thresholds
- **Contour Generation**: Creates contour outlines of detected clouds
- **ROI Configuration**: Supports configurable region-of-interest cropping
- **Output Artifacts**: Generates debug images showing:
  - ROI preview with crop boundaries
  - Cloud and sky mask overlay
  - Cloud contour visualization

## Telemetry Data

The system logs the following sensor readings to CSV:

| Field       | Description                                  |
| ----------- | -------------------------------------------- |
| Timestamp   | Human-readable timestamp of data collection  |
| Confidence  | Confidence level of presence detection (0-1) |
| Presence    | Boolean presence detection result            |
| Temperature | Ambient temperature (°C)                     |
| Humidity    | Relative humidity (%)                        |
| Pressure    | Atmospheric pressure (Pa)                    |
| Light       | Light intensity level                        |

## Configuration

The system uses YAML configuration files for cloud detection thresholds. Configuration is expected at:

```
~/groundstation/config/thresholds.yaml
```

### Threshold Parameters

- `roi_top_frac`: Fraction of image from top to exclude (0.0-1.0)
- `sky_h_min`, `sky_h_max`: Hue range for sky detection (HSV)
- `sky_s_min`: Saturation threshold for sky
- `sky_v_min`: Value threshold for sky
- `cloud_s_max`: Maximum saturation for cloud detection
- `cloud_v_min`: Minimum value for cloud detection
- `min_sky_pixels`: Minimum valid pixels for analysis

## Dependencies

- **paho-mqtt**: MQTT client library for Python
- **OpenCV (cv2)**: Image processing and computer vision
- **numpy**: Numerical operations
- **PyYAML**: Configuration file handling
- **Python 3.7+**

## Installation

```bash
pip install paho-mqtt opencv-python numpy pyyaml
```

## Usage

### Starting the Ground Station

```bash
python telemetry_reader.py
```

The application will:

1. Connect to the MQTT broker (default: localhost:1883)
2. Subscribe to telemetry topics
3. Listen for incoming satellite data
4. Process images and log telemetry automatically

### MQTT Broker Configuration

Edit the broker address in `telemetry_reader.py`:

```python
broker = "localhost"  # Change to satellite's IP address
port = 1883
```

### Output Directories

When images are received, the system creates a directory structure:

```
~/groundstation/
├── [timestamp]/
│   ├── roi_preview.jpg        # ROI boundary visualization
│   ├── newoverlay.jpg         # Cloud/sky mask overlay
│   └── cloud_contours.jpg     # Cloud contour outlines
└── payload_records.csv        # All historical telemetry data
```

## Data Flow

```
Satellite
    ↓
MQTT Broker (sat/telemetry/*)
    ↓
telemetry_reader.py (subscribes)
    ↓
    ├─→ JSON Parsing
    ├─→ Image Decoding (base64 → JPEG)
    ├─→ imageAnalysis.py (cloud detection)
    ├─→ CSV Logging
    ├─→ Debug Image Generation
    └─→ Local Storage
```

## Sample Data

The project includes sample telemetry records in `payload_records_1.csv` with sensor data from October 27, 2025, showing environmental monitoring from the satellite.

## Troubleshooting

- **MQTT Connection Failed**: Verify broker address and port are correct
- **JSON Decode Error**: Check that satellite is sending valid JSON payloads
- **Image Processing Fails**: Ensure OpenCV is properly installed and image data is valid base64
- **Config Not Found**: Create `~/groundstation/config/thresholds.yaml` with appropriate threshold values

## Future Enhancements

- Real-time data visualization dashboard
- Cloud fraction statistics and trends
- Integration with satellite command system
- Network redundancy and failover
- Enhanced image analysis (cloud type classification, vegetation indices)

## License

[Add your license information here]

## Contact

For questions or support, contact the CubeSat project team.
