# **ROS 2 Autonomous Tractor with PID Control**


## **Overview**

This repository includes the complete codebase, documentation, and reports for the **Autonomous Tractor Project** developed during the ABE 6990 course at Mississippi State University. The project aims to create an autonomous tractor capable of:
1. **Precise Navigation**: Reaching a target goal using GPS localization.
2. **Obstacle Avoidance**: Using an RGB-D camera for real-time detection of obstacles.
3. **Actuator Control**: Controlling acceleration (linear actuator) and steering (stepper motor) with commands derived from PID-based control logic.

The tractor system is developed and integrated using **ROS 2**, **Python**, and **Arduino** to achieve seamless communication between sensors, actuators, and computational modules.

---

## **System Features**

### **PID-Based Goal Navigation**
The tractor uses a **Proportional-Integral-Derivative (PID) Controller** to:
1. Compute **linear velocity** (`linear.x`) to drive the tractor forward or backward based on the distance to the target.
2. Compute **angular velocity** (`angular.z`) to adjust the steering wheel and correct heading errors.

### **Obstacle Detection**
- An RGB-D camera calculates the distance to obstacles and publishes it to the `camera/distance` topic.
- If an obstacle is detected within **5 meters**, the system halts all movement to ensure safety.

### **System Control Flow**
The following diagram illustrates the **overall system architecture**:

![abe_tractor](https://github.com/user-attachments/assets/2eeb0c0b-cb34-487b-b1fa-6d77914fae9e)


---

## **How It Works**

### **1. Navigation with GPS**
The tractor uses **GPS coordinates** (latitude, longitude, altitude) for navigation:
1. The target goal and the tractor’s current position are converted from **LLA (Latitude, Longitude, Altitude)** to **ENU (East-North-Up)** coordinates using `ecef_to_enu`.
2. The PID algorithm computes:
   - **Distance Error**: The Euclidean distance between the current position and the target.
   - **Heading Error**: The angular difference between the desired orientation and the current orientation.

The system continuously updates the tractor's linear and angular velocities until it reaches the goal or an obstacle is detected.

### **2. Obstacle Detection**
The **RGB-D camera module** publishes:
- **RGB Images**: Published on `camera/rgb` for visualization and debugging.
- **Obstacle Distance**: Published on `camera/distance` for real-time safety checks.

If the obstacle distance is ≤ **5 meters**, the tractor stops and waits until the path is clear.

### **3. Actuator Control**
- **Linear Actuator**: Controls forward/backward acceleration based on `linear.x` velocity.
- **Stepper Motor**: Controls steering adjustments based on `angular.z` velocity.

The control commands are processed by an Arduino, which executes low-level motor control for smooth operation.

---

## **Code Description**

### **PID Control for Navigation**

The tractor's navigation logic is implemented using the **PID algorithm**. Below is an explanation of the included code:

#### **Key Components**
1. **GPS Conversion**:
   - The goal and tractor positions are converted to local ENU coordinates using `lla_to_ecef` and `ecef_to_enu`.
   - Example:
     ```python
     goal_ecef = lla_to_ecef(lat_goal, lon_goal, alt_goal)
     goal_enu = ecef_to_enu(goal_ecef, orig_ecef)
     ```

2. **PID Logic**:
   - **Linear Velocity**:
     ```python
     v = Kp_linear * e_distance  # Proportional control for forward/backward motion
     ```
   - **Angular Velocity**:
     ```python
     omega = Kp_angular * e_theta  # Proportional control for steering
     ```
   - These values are updated at every timestep and published to `/cmd_vel`.

3. **Trajectory Plotting**:
   - The trajectory and error values are logged and plotted in real-time for debugging and analysis.

#### **Simulation and Results**
The PID algorithm ensures smooth and accurate navigation:
- The tractor follows the **shortest path** to the goal.
- Errors are reduced dynamically using proportional control.

#### **Code Example**
```python
for t in np.arange(0, T, dt):
    e_distance = np.sqrt((x_target - x) ** 2 + (y_target - y) ** 2)
    desired_theta = np.arctan2(y_target - y, x_target - x)
    e_theta = wrap_to_pi(desired_theta - theta)

    # PID control
    v = Kp_linear * e_distance
    omega = Kp_angular * e_theta

    # Update state
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt
```

---

## **Camera Module Data Flow**

The RGB-D camera module processes image data and publishes it to ROS 2 topics. The data flow is illustrated below:

<img width="739" alt="Screenshot 2024-11-21 at 11 32 11 PM" src="https://github.com/user-attachments/assets/4d40642f-627b-4b97-8c1a-a87411326a94">


- **Camera Input**:
  - Captures RGB images and depth data in real-time.
- **ROS 2 Topics**:
  - `camera/rgb`: RGB images for visualization.
  - `camera/depth`: Depth data to calculate the distance to obstacles.
- **Obstacle Detection**:
  - The obstacle distance is published on `camera/distance`. If the distance ≤ **5 meters**, the tractor halts.

---

## **Key Features**

| **Feature**             | **Description**                                                                                 |
|--------------------------|-------------------------------------------------------------------------------------------------|
| **GPS-Based Navigation** | Precise localization using ENU coordinates and PID control.                                    |
| **Obstacle Detection**   | Real-time safety checks using RGB-D camera depth data.                                         |
| **Actuator Control**     | Smooth control of acceleration (linear actuator) and steering (stepper motor).                 |
| **ROS 2 Integration**    | Seamless communication between sensors and actuators using ROS 2 nodes and topics.             |

---

## **Getting Started**

### **1. Clone the Repository**
```bash
git clone https://github.com/yourusername/Ros2-Autonomous-Tractor.git
cd Ros2-Autonomous-Tractor
```

### **2. Install Dependencies**
Install required Python packages:
```bash
pip install -r requirements.txt
```

### **3. Run the ROS 2 Nodes**
- **Start the Camera Module**:
  ```bash
  ros2 run depthai_publisher depthai_publisher_node
  ```
- **Start the Navigation Controller**:
  ```bash
  ros2 run tractor_navigation pid_controller_node
  ```

---

## **Results**

### **Output**
The tractor reached to goal point autonomously using PID algorithm and data from GPS and help of actuators.



### **Performance Metrics**
- **Goal Accuracy**: Reaches the goal within a **5-meter tolerance**.
- **Obstacle Detection**: Detects and halts for obstacles ≤ **5 meters**.
- **PID Stability**: Smooth navigation with minimal overshoot.

---

## **Contributors**

* **Charles Raines**
* **Sushant Gautam**
* **Andres Arias Londono**


## **License**

This project is licensed under the MIT License.

