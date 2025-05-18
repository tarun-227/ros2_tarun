# ROS 2 Humble - Week 1 Task Documentation

## Task Overview

**Goal:** Create a basic ROS 2 Python project with:

- A publisher node that sends a number
- A subscriber node that receives this number, calculates its square, and displays it in the terminal

---

## Initial Setup

### 1. **Check if ROS 2 Humble is Installed**

```bash
ls /opt/ros/humble/setup.bash
```

If the above path exists, ROS 2 Humble is installed.

---

## Workspace Setup

```bash
# Create a ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Create a package
ros2 pkg create number_interface --build-type ament_python --dependencies rclpy std_msgs
```

---

## Publisher Node (`number_publisher.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberPublisher(Node):
    def __init__(self):
        """
        Constructor function called once when the node object is created.
        Initializes the publisher, timer, and the starting number.
        """
        super().__init__('number_publisher')
        # Create a publisher on topic 'number' with message type Int32 and queue size 10
        self.publisher_ = self.create_publisher(Int32, 'number', 10)
        # Create a timer to call 'timer_callback' every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        # Initialize the number to be published
        self.i = 0

    def timer_callback(self):
        """
        Called every 1 second by the timer.
        Creates a message, sets the data to current number,
        publishes the message, logs info, and increments the number.
        """
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {self.i}')
        self.i += 1

def main(args=None):
    """
    Main function that initializes ROS 2, creates the node,
    spins it to process callbacks, and finally cleans up.
    """
    rclpy.init(args=args)                # Initialize ROS 2 communications
    node = NumberPublisher()             # Create node instance
    rclpy.spin(node)                     # Keep node alive, processing callbacks
    node.destroy_node()                  # Cleanup node resources
    rclpy.shutdown()                    # Shutdown ROS 2 communications

if __name__ == '__main__':
    main()
```

---

## Subscriber Node (`number_subscriber.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberSubscriber(Node):
    def __init__(self):
        """
        Constructor function called once when the node object is created.
        Initializes the subscriber on the 'number' topic.
        """
        super().__init__('number_subscriber')
        # Create a subscription to 'number' topic with message type Int32
        # Calls listener_callback when message is received
        self.subscription = self.create_subscription(
            Int32,
            'number',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        """
        Called automatically when a new message is received on 'number' topic.
        Calculates the square of the received number and logs the result.
        """
        square = msg.data ** 2
        self.get_logger().info(f'Received: {msg.data}, Square: {square}')

def main(args=None):
    """
    Main function that initializes ROS 2, creates the subscriber node,
    spins it to process callbacks, and cleans up afterward.
    """
    rclpy.init(args=args)               # Initialize ROS 2 communications
    node = NumberSubscriber()           # Create subscriber node instance
    rclpy.spin(node)                    # Keep node alive, processing callbacks
    node.destroy_node()                 # Cleanup node resources
    rclpy.shutdown()                   # Shutdown ROS 2 communications

if __name__ == '__main__':
    main()
```

---

## Build & Run

### 1. Build the workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Open two terminals:

**Terminal 1:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run number_interface number_publisher
```

**Terminal 2:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run number_interface number_subscriber
```

---

## How Do Nodes Communicate?

ROS 2 nodes communicate using a **publish-subscribe** model:

- A **publisher** node sends messages on a **topic**.
- A **subscriber** node listens to the same topic and reacts to those messages.
- Both use the same **message type** (here: `std_msgs/msg/Int32`).

### In Our Task:
- The **publisher** sends a number every second on the topic `/number`.
- The **subscriber** receives that number, squares it, and logs the result.
- The ROS 2 system handles the delivery of messages between the nodes automatically.

This decouples nodes and allows scalable communication across distributed systems.

---

## Key Concepts Explained

### `self`

- Refers to the **instance** of the class
- Used to access variables and methods **inside that class**

### Instance

- A **specific object** created from a class
- Ex: `node = NumberPublisher()` creates an instance of the `NumberPublisher` class

### Why use classes?

Classes are used in ROS 2 nodes to organize related behavior and data in a modular, reusable way. Using classes allows you to:

- **Encapsulate state**: For example, you can store and update a counter (`self.i`) across timer callbacks.
- **Group functionality**: Publishers, subscribers, and timers can all be part of the same object.
- **Improve readability and maintainability**: Especially when nodes become complex.

Although ROS 2 supports function-based nodes, classes are **recommended** for better structure and scalability.

### `rclpy` lifecycle

```python
rclpy.init()          # Initialize ROS
node = Node()         # Create node instance
rclpy.spin(node)      # Keep it alive (process callbacks)
node.destroy_node()   # Cleanup
rclpy.shutdown()      # Shutdown ROS
```

---

## Websites Referred

- [Articulated Robotics Beginner Tutorials](https://articulatedrobotics.xyz/tutorials/)
- [ROS 2 Documentation - Humble](https://docs.ros.org/en/humble/index.html)
- [Python Publisher and Subscriber Tutorial](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/)
