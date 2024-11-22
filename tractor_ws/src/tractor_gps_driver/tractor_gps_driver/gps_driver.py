import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import time

from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED, SBP_MSG_POS_LLH
import argparse

class gps_driver(Node):

    def __init__(self):
        super().__init__('gps_driver')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)

    def publish_gps(self, lat, long):
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_frame"
        msg.latitude = lat
        msg.longitude = long
        msg.altitude = 0.0
        msg.position_covariance_type = 0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published GPS data: {msg.latitude}, {msg.longitude}")


def main(args=None):
    rclpy.init(args=args)
    node = gps_driver()
    # Open a connection to Piksi using the default baud rate (1Mbaud)
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

if __name__ == "__main__":
    main()
