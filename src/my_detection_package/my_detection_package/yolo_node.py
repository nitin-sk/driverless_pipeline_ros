import rclpy
from rclpy.node import Node
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from perception_msgs.msg import ConeArray
from geometry_msgs.msg import Point

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.subscriber = self.create_subscription(Image, '/camera/left_image', self.image_callback, 10)
        self.publisher = self.create_publisher(ConeArray, '/detected_cones', 10)
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5_model.pt')
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)
        
        blue_cones, yellow_cones = [], []
        for detection in results.xyxy[0]:  # [x1, y1, x2, y2, confidence, class]
            cone_class = int(detection[5].item())
            if cone_class == 0:  # Assuming class 0 is blue cones
                blue_cones.append(self.get_cone_coordinates(detection))
            elif cone_class == 1:  # Assuming class 1 is yellow cones
                yellow_cones.append(self.get_cone_coordinates(detection))

        # Publish detected cones
        cone_msg = ConeArray()
        cone_msg.blue_cones = self.np2points(blue_cones)
        cone_msg.yellow_cones = self.np2points(yellow_cones)
        self.publisher.publish(cone_msg)

    def get_cone_coordinates(self, detection):
        x1, y1, x2, y2 = detection[:4]
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return [center_x, center_y, 0.0]  # 2D for now, depth can be added later

    def np2points(self, cones):
        points = []
        for cone in cones:
            p = Point(x=float(cone[0]), y=float(cone[1]), z=float(cone[2]))
            points.append(p)
        return points

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

