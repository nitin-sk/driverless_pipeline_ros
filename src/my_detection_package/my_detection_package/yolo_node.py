import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'camera/detections', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.model = YOLO('/home/teamojasdv/ros2_ws/src/my_detection_package/my_detection_package/best.pt')

    def listener_callback(self, msg):
        # Process the incoming image message
        color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(color_image)
        detections = []

        for r in results:
            for box in r.boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                label = self.model.names[int(c)]
                detections.append(f"Detected {label} at {b}")

                # Optionally, log detection
                self.get_logger().info(f"Detected {label}")

        # Publish detections
        detections_msg = String()
        detections_msg.data = "\n".join(detections)
        self.publisher_.publish(detections_msg)

    def timer_callback(self):
        # Optionally, handle timed events here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

