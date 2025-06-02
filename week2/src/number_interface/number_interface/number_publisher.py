import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import rclpy

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        qos = rclpy.qos.QoSProfile(depth=10)
	qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        self.publisher_ = self.create_publisher(Int32, 'number', qos)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 1

    def timer_callback(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
