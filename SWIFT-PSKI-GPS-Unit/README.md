# Swift PSKI GPS Module Integration

## Overview
This part of the repository focuses on the integration and utilization of the **Swift PSKI GPS module** for obtaining accurate geographical coordinates. The work involves using the `libsbp` library to extract "LLH" (Latitude, Longitude, Height) data from the evaluation board, integrating the GPS module with ROS 2, and understanding the concept of **RTK correction** for improved accuracy. The ongoing work involves setting up a base station for RTK correction.

## Objectives
- Capture real-time GPS coordinates in "LLH" format.
- Integrate the GPS module with ROS 2 for seamless data processing.
- Understand and implement RTK correction to enhance GPS accuracy.

## Project Details
### 1. **Extracting GPS Data Using `libsbp`**
- **Library Used**: `libsbp` from Swift Navigation.
- **Purpose**: To interface with the Swift PSKI evaluation board and extract precise geographic coordinates.
- **Data Format**: "LLH" (Latitude, Longitude, Height).
- **Description**: The `libsbp` library is used to parse GPS data transmitted by the Swift PSKI module. The extracted coordinates are essential for autonomous navigation tasks that require high accuracy.

### Code Snippet for Parsing GPS Data:
```python
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED, SBP_MSG_POS_LLH

# Initialize the serial connection to the evaluation board
with PySerialDriver('/dev/ttyUSB0', baud=115200) as driver:
      with Handler(Framer(driver.read, None, verbose=True)) as source:
          try:
              for msg, metadata in source.filter(SBP_MSG_POS_LLH):                
                #print(f"TOW: {msg.tow}, latitude: {msg.lat}, longitude: {msg.lon}, height: {msg.height}, h_accuracy: {msg.h_accuracy}, v_accuracy: {msg.v_accuracy}, n_stats: {msg.n_sats}, flags: {msg.flags}")
                node.publish_gps(msg.lat, msg.lon)
                time.sleep(0.1)
          except KeyboardInterrupt:
            pass
            node.destroy_node()
            rclpy.shutdown()
```

### 2. **Base Station Setup for RTK Correction**
**What is RTK Correction?**
- **RTK (Real-Time Kinematic)** correction is a technique used to enhance the accuracy of satellite-based positioning systems (e.g., GPS).
- **Basic Concept**: RTK works by using a base station that knows its exact location. It compares the satellite data it receives to its known position and calculates correction factors. These corrections are then sent to the rover (e.g., the autonomous tractor) to correct its GPS data in real-time, improving accuracy from meters to centimeters.

**Steps for RTK Correction**:
1. **Base Station Setup**:
   - Install the base station at a known fixed location.
   - Configure the station to receive satellite signals and compute corrections.
2. **Transmission of Corrections**:
   - Transmit the correction data via a communication link (e.g., radio, cellular) to the rover.
3. **Rover Integration**:
   - The rover receives the corrections and adjusts its GPS data for high-precision positioning.

**Current Status**:
- The base station setup and configuration are ongoing, with plans to complete the RTK correction pipeline for improved localization.

### 3. **Integration with ROS 2**
- **ROS 2 Node**: The GPS data parsed using the `libsbp` library is published as a ROS 2 topic for real-time processing.
- **Integration**: Ensures the GPS data is available to other nodes in the ROS 2 system for tasks such as autonomous navigation and sensor fusion.
- **Topic Details**:
  - **Topic Name**: `/gps/coordinates`
  - **Message Type**: `sensor_msgs/NavSatFix`

### Example ROS 2 Node Code:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher = self.create_publisher(NavSatFix, '/gps/coordinates', 10)
        self.get_logger().info('GPS Publisher Node has been started.')

    def publish_coordinates(self, lat, lon, alt):
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        self.publisher.publish(msg)
        self.get_logger().info(f'Published GPS Data: {lat}, {lon}, {alt}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    # Replace with actual logic to read data and call `publish_coordinates`
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Reports
All progress reports and documentation related to the GPS module integration and RTK correction setup are included in this repository for further reference.

## What I Learned
- **GPS Data Handling**: Understanding and parsing GPS data in "LLH" format.
- **RTK Correction Basics**: Concepts and steps involved in setting up RTK correction for enhanced GPS accuracy.
- **ROS 2 Integration**: Developing and deploying ROS 2 nodes for real-time data publication.

**Thanks Charles Raines for Ros2 integration for GPS module.**

## Future Work
- Complete the base station setup for RTK correction.
- Implement the communication link between the base station and the rover for RTK data transmission.
- Enhance the ROS 2 node to process and adjust GPS data using RTK corrections.