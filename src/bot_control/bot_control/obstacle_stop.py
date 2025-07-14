import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleStopper(Node):
    def __init__(self):
        super().__init__('obstacle_stopper')
        self.sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def lidar_callback(self, msg):
        front_ranges = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
        if any(r < 0.5 for r in front_ranges if r > 0.0):
            self.get_logger().warn('Obstacle detected')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

