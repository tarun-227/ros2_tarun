import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class NumberSubscriber(Node):

    def __init__(self):
        super().__init__('number_subscriber')
        self.subscription = self.create_subscription(
            Int64,
            'number',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        squared_number = msg.data * msg.data
        self.get_logger().info(f'Received: {msg.data}, Squared: {squared_number}')

def main(args=None):
    rclpy.init(args=args)
    number_subscriber = NumberSubscriber()
    rclpy.spin(number_subscriber)
    number_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
