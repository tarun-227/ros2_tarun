import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ConstantSpeedPublisher(Node):
    def __init__(self):
        super().__init__('constant_speed_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.2  # Constant forward speed (m/s)
        msg.angular.z = 0.0  # No rotation
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing constant velocity')

def main(args=None):
    rclpy.init(args=args)
    node = ConstantSpeedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
