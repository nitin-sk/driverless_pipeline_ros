import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_publisher_ = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.detection_publisher_ = self.create_publisher(String, 'camera/detections', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        W, H = 640, 480
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.model = YOLO('/home/teamojasdv/ros2_ws/src/my_detection_package/my_detection_package/best.pt')

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        results = self.model(color_image)
        for r in results:
            for box in r.boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                label = self.model.names[int(c)]
                self.get_logger().info(f"Detected {label}")

                # Draw bounding box on color_image
                x1, y1, x2, y2 = map(int, b)
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                # Calculate 3D coordinates if needed
                center_x = int((b[0] + b[2]) / 2)
                center_y = int((b[1] + b[3]) / 2)
                depth = depth_frame.get_distance(center_x, center_y)
                if depth > 0:
                    self.get_logger().info(f"Depth: {depth:.2f} meters")

        self.publisher_.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))
        self.depth_publisher_.publish(self.bridge.cv2_to_imgmsg(depth_image, "mono16"))

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

