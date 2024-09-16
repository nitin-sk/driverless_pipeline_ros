import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from perception_msgs.msg import ConeArray
from cv_bridge import CvBridge

class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        self.depth_sub = self.create_subscription(Image, '/camera/depth_image', self.depth_callback, 10)
        self.cone_sub = self.create_subscription(ConeArray, '/detected_cones', self.cone_callback, 10)
        self.bridge = CvBridge()
        self.depth_image = None
        self.detected_cones = None

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def cone_callback(self, msg):
        self.detected_cones = msg

    def process_data(self):
        if self.depth_image is not None and self.detected_cones is not None:
            # Process YOLO detections and depth image to calculate 3D positions
            for cone in self.detected_cones.blue_cones:
                # Use cone.x, cone.y to index into depth image and find z (depth)
                depth = self.depth_image[int(cone.y), int(cone.x)]
                print(f'Cone position: ({cone.x}, {cone.y}, {depth})')

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

