import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class DepthAIPublisherNode(Node):
    def __init__(self):
        super().__init__('depthai_publisher')

        # ROS publishers
        self.rgb_pub = self.create_publisher(Image, 'camera/rgb', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth', 10)

        # Bridge for OpenCV to ROS image conversion
        self.bridge = CvBridge()

        # Setup DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Define mono cameras for depth input
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # Define stereo depth node
        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setConfidenceThreshold(200)
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        # Link mono cameras to stereo depth node
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Output streams
        color_cam = self.pipeline.create(dai.node.ColorCamera)
        color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        color_cam.setFps(30)

        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        color_cam.video.link(xout_rgb.input)

        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # Start DepthAI device
        self.device = dai.Device(self.pipeline)
        self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=8, blocking=False)
        self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=8, blocking=False)

        # Timer to periodically publish frames
        self.timer = self.create_timer(0.1, self.publish_frames)

    def publish_frames(self):
        # Get RGB frame
        in_rgb = self.rgb_queue.get()
        rgb_frame = in_rgb.getCvFrame()

        # Get depth frame
        in_depth = self.depth_queue.get()
        depth_frame = in_depth.getFrame()

        # Convert depth values from millimeters to meters
        depth_in_meters = depth_frame / 1000.0  # Convert entire frame to meters

        # Define Region of Interest (ROI)
        height, width = depth_in_meters.shape
        center_x = width // 2
        left_bound = max(0, center_x - int(0.5 * width / 2))  # Adjust to cover ±0.5 meters
        right_bound = min(width, center_x + int(0.5 * width / 2))
        top_bound = int(height * 0.4)  # Focus on the lower half of the frame
        bottom_bound = height

        # Extract ROI for obstacle detection
        roi = depth_in_meters[top_bound:bottom_bound, left_bound:right_bound]

        # Check for obstacles in the ROI
        obstacle_pixels = roi <= 5.0  # Boolean mask for pixels ≤ 5 meters
        obstacle_count = np.sum(obstacle_pixels)  # Count the number of such pixels

        # Obstacle detection logic
        obstacle_threshold = 1000  # Threshold for obstacle pixel count
        if obstacle_count > obstacle_threshold:
            self.get_logger().warn(
                f"Stop, Object detected within 5 meters in ROI: {obstacle_count} pixels"
            )
        else:
            self.get_logger().info(
                f"Safe: Only {obstacle_count} pixels within 5 meters in ROI"
            )

        # Convert RGB frame to ROS Image and publish
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
        self.rgb_pub.publish(rgb_msg)

        # Convert depth frame to ROS Image and publish
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="mono16")
        self.depth_pub.publish(depth_msg)

 
def main(args=None):
    rclpy.init(args=args)
    node = DepthAIPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
