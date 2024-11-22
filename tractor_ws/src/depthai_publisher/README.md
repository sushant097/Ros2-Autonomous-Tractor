
# DepthAI Publisher ROS 2 Package

## Description
This package publishes RGB and depth images from a Luxonis RGB-D camera, along with distance measurements, to ROS 2 topics.

## Installation and Setup
1. Clone this repository:
   ```bash
   git clone <repository-link> ~/ros2_ws/src/depthai_publisher
   ```

2. Navigate to your ROS 2 workspace and build the package:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Running the Node
Run the node with:
```bash
ros2 run depthai_publisher depthai_publisher
```

## Viewing Image Streams
Use `rqt_image_view` to view the image topics:
```bash
ros2 run rqt_image_view rqt_image_view
```
* Once `rqt_image_view` opens, select the desired image topic from the dropdown menu to display it (e.g., `/camera/rgb` or `/camera/depth`).

## Dependencies
- ROS 2 (e.g., Humble or Foxy)
- Python packages: `depthai`, `cv_bridge`

Ensure these dependencies are installed:
```bash
pip install depthai
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport
```

Replace `$ROS_DISTRO` with your ROS 2 distribution (e.g., `humble`).
```