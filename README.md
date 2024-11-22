# Ros2-Autonomous-Tractor

## Overview
This repository contains all code and related project work for the **ABE 6990 Autonomous Tractor Course** at **Mississippi State University**. The course focuses on developing and integrating autonomous systems using ROS 2 for agricultural machinery. The code and documentation in this repository demonstrate the concepts, techniques, and hands-on projects completed during the course.

## Course Objectives
- **Understanding Autonomous Systems**: Learning how to develop autonomous control systems for tractors and other agricultural machinery.
- **ROS 2 Integration**: Gaining proficiency in using ROS 2 for communication between various sensors, cameras, and control modules.
- **GPS and Localization**: Utilizing the Swift PSKI GPS module for precise localization and navigation.
- **Base Station Setup**: Setting up and configuring a base station for remote communication and control.
- **Practical Applications**: Applying theoretical knowledge in real-world scenarios to control and automate machinery.

## Projects and Code Included
### 1. **Camera Work**
- **Description**: Code for integrating and processing data from an RGB-D camera using ROS 2.
- **Key Features**:
  - Capturing RGB and depth images.
  - Calculating distance from the camera to objects.
  - Publishing image data and distance measurements as ROS 2 topics.
- **Technologies Used**: ROS 2, Python, DepthAI library, OpenCV.

### 2. **Swift PSKI GPS Module Work**
- **Description**: Code for integrating the Swift PSKI GPS module for accurate GPS-based localization.
- **Key Features**:
  - Reading and processing GPS data.
  - Integrating GPS data with ROS 2 for real-time positioning.
- **Technologies Used**: ROS 2, Python.

### 3. **Base Station Setup**
- **Description**: Configuration and setup for a base station to monitor and control the autonomous tractor remotely.
- **Key Features**:
  - Establishing communication between the tractor and base station.
  - Configuring ROS 2 nodes for data relay and control commands.
- **Technologies Used**: ROS 2, Python, network configuration tools.

## Reports
This repository also includes detailed reports that document the project workflows, methodologies, results, and reflections on lessons learned throughout the course. These reports provide deeper insights into the approaches taken and the challenges faced during the implementation of each project.

## What I Learned
- **ROS 2 Workflow**: Developing and managing ROS 2 nodes for real-time data streaming and processing.
- **Sensor Integration**: Combining different sensor data (camera and GPS) for autonomous navigation.
- **Communication Systems**: Setting up and managing communication between autonomous machinery and base stations.
- **Practical Problem Solving**: Applying academic learning to solve real-world challenges in autonomous system development.

## Getting Started
To get started with the code:
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/Ros2-Autonomous-Tractor.git
   cd Ros2-Autonomous-Tractor
   ```
2. **Install Dependencies**:
   Ensure you have ROS 2 installed and install any required Python packages using:
   ```bash
   pip install -r requirements.txt
   ```
3. **Run the Code**:
   Follow the instructions provided in individual project folders for running the code.

## Contributing
Contributions are welcome! If you find any issues or have suggestions for improvements, feel free to open an issue or submit a pull request.

## License
This project is licensed under the [MIT License](LICENSE).