import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelSerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_cmd_vel_publisher')
        
        # Initialize the serial connection (modify with your correct serial port)
        self.serial_port = '/dev/ttyACM0'  # Adjust for your serial device
        self.baud_rate = 9600
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        # Subscriber to the cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # Get linear and angular velocities
        linear_vel = msg.linear.x  # You can adjust this for other dimensions if needed
        angular_vel = msg.angular.z

        # Format the data as a string "linear_vel angular_vel"
        data = f"{linear_vel} {angular_vel}\n"
        
        # Send the data over the serial connection
        self.ser.write(data.encode('utf-8'))
        self.get_logger().info(f"Sent data: {data.strip()}")


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelSerialPublisher()

    rclpy.spin(node)

    node.ser.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
