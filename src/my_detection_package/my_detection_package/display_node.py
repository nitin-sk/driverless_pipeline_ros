import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Display Node has been started.')
    
    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Display the image
        cv2.imshow('Camera Feed', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

