import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.subscription = self.create_subscription(
            String,
            'camera/detections',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()