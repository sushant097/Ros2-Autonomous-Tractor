
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

## New Update:
To efficiently detect obstacles within 5 meters in a **wide area** covering the front and sides of the tractor (e.g., ±0.5 meters to the left and right), we can:

1. **Define a Region of Interest (ROI)**:
   - Focus on the central horizontal strip of the depth frame.
   - Adjust the width of the ROI to cover the desired left and right areas.
   
2. **Use NumPy for Masking and Efficient Computation**:
   - Apply a threshold on the ROI to check for depth values ≤ 5 meters.
   - Count the pixels satisfying the condition in this ROI.

3. **Determine Obstacle Presence**:
   - If the count of such pixels exceeds a threshold, it indicates an obstacle.


### Key Updates
1. **ROI Selection**:
   - The **region of interest (ROI)** focuses on the central horizontal strip of the depth frame to account for the left and right sides of the tractor.
   - **Horizontal Coverage**: Calculated as ±0.5 meters from the camera's center.
   - **Vertical Coverage**: Focused on the lower half of the frame to align with ground-level obstacles.

2. **NumPy Operations**:
   - The `roi` is extracted and processed as a subset of the depth frame.
   - Boolean masking and summing efficiently count pixels within the 5-meter threshold.

3. **Obstacle Threshold**:
   - If the number of pixels satisfying the depth condition exceeds `obstacle_threshold`, it triggers a warning to stop the tractor.

4. **Logging**:
   - Logs warnings or informational messages based on the presence of obstacles.

---

### Adjustments
- **ROI Dimensions**:
   - Tweak `top_bound`, `left_bound`, and `right_bound` to adjust the region's size to match the tractor's width and height requirements.
   
- **Obstacle Threshold**:
   - Set `obstacle_threshold` based on the environment and application sensitivity.

---

### Advantages of This Approach
- **Real-Time Efficiency**:
   - Uses a focused region to reduce unnecessary computation and vectorized operations for efficiency.
   
- **Wide Area Coverage**:
   - Ensures obstacles on both sides of the tractor are detected.
   
- **Customizable**:
   - ROI and threshold can be easily adjusted for different camera setups or environments.