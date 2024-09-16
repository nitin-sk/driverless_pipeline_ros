import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_left = self.create_publisher(Image, '/camera/left_image', 10)
        self.publisher_right = self.create_publisher(Image, '/camera/right_image', 10)
        self.publisher_depth = self.create_publisher(Image, '/camera/depth_image', 10)
        self.bridge = CvBridge()

        # Initialize RealSense camera
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.timer = self.create_timer(0.1, self.publish_frames)  # 10 Hz frame rate

    def publish_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return

        # Convert frames to ROS Image messages
        depth_image = self.bridge.cv2_to_imgmsg(np.asanyarray(depth_frame.get_data()), encoding="passthrough")
        left_image = self.bridge.cv2_to_imgmsg(np.asanyarray(color_frame.get_data()), encoding="bgr8")
        
        # Publish the images
        self.publisher_left.publish(left_image)
        self.publisher_depth.publish(depth_image)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

