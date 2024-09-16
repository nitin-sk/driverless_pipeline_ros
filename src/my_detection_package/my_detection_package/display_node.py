import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from perception_msgs.msg import ConeArray
import cv2
from cv_bridge import CvBridge

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.image_sub = self.create_subscription(Image, '/camera/left_image', self.image_callback, 10)
        self.cone_sub = self.create_subscription(ConeArray, '/detected_cones', self.cone_callback, 10)
        self.bridge = CvBridge()
        self.image = None
        self.cones = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.display()

    def cone_callback(self, msg):
        self.cones = msg
        self.display()

    def display(self):
        if self.image is not None and self.cones is not None:
            for cone in self.cones.blue_cones:
                cv2.circle(self.image, (int(cone.x), int(cone.y)), 5, (255, 0, 0), -1)
            cv2.imshow('Detections', self.image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

