import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import math
import numpy as np

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

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Subscriber for GPS
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.twist = Twist()

        # Publisher for velocity command (Twist)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Inition Position
        self.initial_ecef = None
        self.initial_lla = None

        # Target (goal) position
        self.goal = [10, 0]

        self.current_pos = None

    def gps_callback(self, msg):
        if self.initial_enu == None:
            self.initial_lla = [msg.latitude, msg.longitude, 0]
            self.initial_ecef = lla_to_ecef(self.initial_lla[0], self.initial_lla[1], 0)
        if self.current_pos == None:
            self.current_pos = self.ecef_to_enu(self.initial_ecef, self.initial_ecef)
        else:
            self.current_pos = self.ecef_to_enu(lla_to_ecef(msg.latitude, msg.longitude), self.initial_ecef)
        self.control_loop()

    def control_loop(self):
        if self.current_pose:
            x = self.current_pos[0]
            y = self.current_pos[1]

            # Calculate distance to goal
            distance = math.sqrt((self.goal_x - x)**2 + (self.goal_y - y)**2)

            if distance > 1:
                # Proportional control for linear and angular velocity
                linear_vel = 0.5 * distance  # Adjust this factor to tune the speed

                # Create and publish the Twist message
                self.twist.linear.x = linear_vel
                self.vel_pub.publish(self.twist)
            else:
                # If within threshold, stop
                self.twist.linear.x = 0.0
                self.vel_pub.publish(self.twist)
        print(f"Current Pos: {self.current_pos[0]}, {self.current_pos[1]} | Goal Pos: {self.goal[0]}, {self.goal[1]} | Twist Linear: {self.twist.linear.x}\n")

    def ecef_to_enu(self, ecef, ref_ecef):
        # ENU transformation
        x, y, z = ecef
        ref_x, ref_y, ref_z = ref_ecef

        # Calculate differences
        dx = x - ref_x
        dy = y - ref_y
        dz = z - ref_z

        # Reference latitude and longitude in radians
        ref_lat_rad = np.radians(self.initial_lla[0])
        ref_lon_rad = np.radians(self.initial_lla[1])

        # Calculate ENU
        e = -np.sin(ref_lon_rad) * dx + np.cos(ref_lon_rad) * dy
        n = -np.sin(ref_lat_rad) * np.cos(ref_lon_rad) * dx - np.sin(ref_lat_rad) * np.sin(ref_lon_rad) * dy + np.cos(ref_lat_rad) * dz
        u = np.cos(ref_lat_rad) * np.cos(ref_lon_rad) * dx + np.cos(ref_lat_rad) * np.sin(ref_lon_rad) * dy + np.sin(ref_lat_rad) * dz

        return np.array([e, n, u])

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleController()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
