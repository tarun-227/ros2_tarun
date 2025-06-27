import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleStopNode(Node):
    def __init__(self):
        super().__init__('obstacle_stop_node')

        # Subscriber to /scan (LIDAR topic)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher to /cmd_vel (velocity commands)
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.obstacle_distance_threshold = 0.5  # in meters

    def scan_callback(self, msg):
        # Ignore NaNs and inf, get the minimum range value
        valid_ranges = [r for r in msg.ranges if r > 0.0 and r < float('inf')]

        if not valid_ranges:
            self.get_logger().warn('No valid LIDAR data')
            return

        min_distance = min(valid_ranges)

        self.get_logger().info(f'Closest obstacle: {min_distance:.2f} meters')

        twist_msg = Twist()

        if min_distance < self.obstacle_distance_threshold:
            self.get_logger().warn('Obstacle too close! Stopping the robot.')
            twist_msg.linear.x = 0.0
        else:
            twist_msg.linear.x = 0.2  # Move forward if no obstacle

        self.cmd_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

