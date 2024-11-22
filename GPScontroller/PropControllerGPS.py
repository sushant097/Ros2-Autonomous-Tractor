for name in dir():
    if not name.startswith('_'):
        del globals()[name]

import numpy as np
import matplotlib.pyplot as plt
import os
os.system('cls' if os.name == 'nt' else 'clear')


# Function to wrap angle to [-pi, pi]
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def lla_to_ecef(lat, lon, alt):
    # WGS84 constants
    a = 6378137.0  # Equatorial radius in meters
    e2 = 6.69437999014e-3  # Square of eccentricity

    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    N = a / np.sqrt(1 - e2 * (np.sin(lat_rad) ** 2))

    x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (N * (1 - e2) + alt) * np.sin(lat_rad)

    return np.array([x, y, z])

# Function to convert ECEF to ENU
def ecef_to_enu(ecef, ref_ecef):
    # ENU transformation
    x, y, z = ecef
    ref_x, ref_y, ref_z = ref_ecef

    # Calculate differences
    dx = x - ref_x
    dy = y - ref_y
    dz = z - ref_z

    # Reference latitude and longitude in radians
    ref_lat_rad = np.radians(lat_orig)
    ref_lon_rad = np.radians(lon_orig)

    # Calculate ENU
    e = -np.sin(ref_lon_rad) * dx + np.cos(ref_lon_rad) * dy
    n = -np.sin(ref_lat_rad) * np.cos(ref_lon_rad) * dx - np.sin(ref_lat_rad) * np.sin(ref_lon_rad) * dy + np.cos(ref_lat_rad) * dz
    u = np.cos(ref_lat_rad) * np.cos(ref_lon_rad) * dx + np.cos(ref_lat_rad) * np.sin(ref_lon_rad) * dy + np.sin(ref_lat_rad) * dz

    return np.array([e, n, u])


# Origin point in LLA, taken prior running the tractor (outside of lab)
lat_orig = 33.448058 # Latitude of origin
lon_orig = -88.797013  # Longitude of origin
alt_orig = 0  # Altitude of origin in meters
# Convert origin point to ECEF
orig_ecef = lla_to_ecef(lat_orig, lon_orig, alt_orig)
orig_enu = ecef_to_enu(orig_ecef,orig_ecef)

# Goal point in LLA, taken prior running the tractor (outside of lab)

# Goal 1
# lat_goal =  33.446560  # Latitude of goal
# lon_goal =  -88.794816 # Longitude of goal
# alt_goal = 0  # Altitude of goal in meters

# Goal 2
lat_goal =  33.449573  # Latitude of goal
lon_goal = -88.795678  # Longitude of goal
alt_goal = 0  # Altitude of goal in meters


# Convert goal point to ECEF
goal_ecef = lla_to_ecef(lat_goal, lon_goal, alt_goal)
goal_enu = ecef_to_enu(goal_ecef,orig_ecef)


# # Tractor position to transform from LLA to local coordinates. This includes the pose of tractor
# # Actually the tractor starts from the same origin
# lat = orig_ecef[0] # Latitude of the point
# lon = orig_ecef[1] # Longitude of the point
# alt = orig_ecef[2]  # Altitude of the point in meters

# # Convert the point to ECEF
# point_ecef = lla_to_ecef(lat, lon, alt)
# print(point_ecef)
# # Convert the ECEF coordinates to ENU
# enu_coordinates = ecef_to_enu(point_ecef, orig_ecef)

########################
#### Controller part ###
########################

# Parameters
Kp_linear = 1.0  # Gain for linear velocity
Kp_angular = 5.0  # Gain for angular velocity
dt = 0.1  # Time step [s]
T = 50  # Total simulation time [s]

# Initial state [x, y, theta]
x = orig_enu[0]
y = orig_enu[1]
theta = 0  # Orientation in radians

# Target position
x_target = goal_enu[0]
y_target = goal_enu[1]

# Data storage for plotting
trajectory = []
time_log = []
cte_log = []  # Cross-Track Error
heading_error_log = []
v_log = []
omega_log = []

# Simulation loop
for t in np.arange(0, T, dt):
    # Store current state
    trajectory.append([x, y])
    time_log.append(t)

    # Compute position error (Euclidean distance)
    e_distance = np.sqrt((x_target - x) ** 2 + (y_target - y) ** 2)

    # Compute desired orientation (angle to the target)
    desired_theta = np.arctan2(y_target - y, x_target - x)

    # Compute orientation error
    e_theta = wrap_to_pi(desired_theta - theta)

    # Log errors
    cte_log.append(e_distance)
    heading_error_log.append(e_theta)

    # Proportional control for linear velocity
    v = Kp_linear * e_distance

    # Proportional control for angular velocity
    omega = Kp_angular * e_theta

    # Log control inputs
    v_log.append(v)
    omega_log.append(omega)

    # Update state using unicycle model
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt

    # Ensure theta stays within [-pi, pi]
    theta = wrap_to_pi(theta)

    # Plotting (live update)
    plt.clf()
    plt.grid(True)
    plt.axis('equal')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Tractor Path Planning using Proportional Controller')

    # Plot desired straight line
    plt.plot([0, x_target], [0, y_target], 'r--', linewidth=2, label='Desired Path')

    # Plot trajectory
    trajectory_np = np.array(trajectory)
    plt.plot(trajectory_np[:, 0], trajectory_np[:, 1], 'b-', linewidth=2, label='Trajectory')

    # Plot current position
    plt.plot(x, y, 'bo', markersize=8, markerfacecolor='b', label='Current Position')

    # Plot target position
    plt.plot(x_target, y_target, 'gx', markersize=12, linewidth=2, label='Target Position')

    plt.legend()
    plt.draw()
    plt.pause(0.01)  # Pause to update the plot

    # Check if target is reached within a tolerance
    distance_to_target = np.sqrt((x - x_target) ** 2 + (y - y_target) ** 2)
    if distance_to_target < 5:
        print(f"Target reached at time t = {t} seconds.")
        break

# Plot Final Trajectory
plt.figure()
plt.grid(True)
plt.axis('equal')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Final Trajectory of the Tractor')

# Desired path
plt.plot([0, x_target], [0, y_target], 'r--', linewidth=2, label='Desired Path')

# Actual trajectory
plt.plot(trajectory_np[:, 0], trajectory_np[:, 1], 'b-', linewidth=2, label='Actual Trajectory')

# Start and target points
plt.plot(0, 0, 'go', markersize=8, markerfacecolor='g', label='Start Position')
plt.plot(x_target, y_target, 'gx', markersize=12, linewidth=2, label='Target Position')

plt.legend()
plt.show()

# # Plot Errors Over Time
# plt.figure()

# # Cross-Track Error
# plt.subplot(2, 1, 1)
# plt.plot(time_log, cte_log, 'b', linewidth=2)
# plt.xlabel('Time [s]')
# plt.ylabel('Distance Error [m]')
# plt.title('Cross-Track Error Over Time')
# plt.grid(True)

# # Heading Error
# plt.subplot(2, 1, 2)
# plt.plot(time_log, np.degrees(heading_error_log), 'r', linewidth=2)
# plt.xlabel('Time [s]')
# plt.ylabel('Heading Error [deg]')
# plt.title('Heading Error Over Time')
# plt.grid(True)

# plt.tight_layout()
# plt.show()

# # Plot Control Inputs Over Time
# plt.figure(figsize=(10, 8))

# # Linear Velocity
# plt.subplot(2, 1, 1)
# plt.plot(time_log, v_log, 'b', linewidth=2)
# plt.xlabel('Time [s]')
# plt.ylabel('Linear Velocity v [m/s]')
# plt.title('Linear Velocity Over Time')
# plt.grid(True)

# # Angular Velocity
# plt.subplot(2, 1, 2)
# plt.plot(time_log, np.degrees(omega_log), 'r', linewidth=2)
# plt.xlabel('Time [s]')
# plt.ylabel('Angular Velocity ω [deg/s]')
# plt.title('Angular Velocity Over Time')
# plt.grid(True)

# plt.tight_layout()
# plt.show()
