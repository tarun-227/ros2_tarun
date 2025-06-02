# number_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import rclpy.qos

class NumberSubscriber(Node):
    def __init__(self):
        super().__init__('number_subscriber')
        qos = rclpy.qos.QoSProfile(depth=10)
	qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        self.subscription = self.create_subscription(
            Int32,
            'number',
            self.listener_callback,
            qos
        )

    def listener_callback(self, msg):
        squared = msg.data ** 2
        self.get_logger().info(f'Received: {msg.data}, Squared: {squared}')

def main(args=None):
    rclpy.init(args=args)
    node = NumberSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
