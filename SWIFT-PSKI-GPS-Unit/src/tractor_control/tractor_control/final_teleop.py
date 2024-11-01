import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty




class TractorTeleOp(Node):
    def __init__(self):
        super().__init__('tractor_teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # Key to twist command mapping
        self.node_key = None
        self.key_map = {'w': [1.0, 0.0], 's': [-1.0, 0.0], ' ': [0.0, 0.0], 'a': [0.0, -1.0], 'd': [0.0, 1.0]}

        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        # Set terminal to raw mode to capture keypresses without Enter
        tty.setcbreak(sys.stdin.fileno())

        # Timer to check for key press and publish twist messages
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.key_timer = self.create_timer(0.03, self.get_key)
        
    def get_key(self):
        # Non-blocking key press capture
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            self.node_key = sys.stdin.read(1)
            #return key
        else:
            self.node_key = None
        return None

    def generate_twist(self):
        msg = Twist()
        key = self.node_key
        if key in self.key_map:
            msg.linear.x = self.key_map[key][0]
            msg.angular.z = self.key_map[key][1]
            print(f"Key: {key} Linear: {msg.linear.x} Angular: {msg.angular.z}")
            self.publisher_.publish(msg)
        else:
            print("Key not in map. Publishing stop command.")
            self.stop_twist()

    def stop_twist(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def timer_callback(self):
        try:
            key = self.node_key
            if key:
                self.generate_twist()
            else:
                self.stop_twist()
        except Exception as e:
            print(f"Exception in timer_callback: {e}")
            self.stop_twist()


def main(args=None):
    rclpy.init(args=args)
    tractor_teleop_node = TractorTeleOp()

    try:
        rclpy.spin(tractor_teleop_node)
    except KeyboardInterrupt:
        # Restore terminal settings on exit
        print("Exiting with Ctrl-C.")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, tractor_teleop_node.settings)
    finally:
        tractor_teleop_node.destroy_node()
        rclpy.shutdown()
        # Restore terminal settings on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, tractor_teleop_node.settings)


if __name__ == "__main__":
    main()

