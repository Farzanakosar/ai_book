# Exercise 1 Solution: Create a simple publisher node

## Problem Statement
Create a simple publisher node that publishes "Hello World" messages to a topic called "hello_topic".

## Expected Outcome
A ROS 2 node that publishes "Hello World" messages at regular intervals to the "hello_topic".

## Solution

```python
#!/usr/bin/env python3
# exercise1_solution.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Hello publisher node initialized')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Key Points
- The node creates a publisher for the 'hello_topic' with a queue size of 10
- A timer callback is used to publish messages at regular intervals (1 second)
- The message content is a String type containing "Hello World" with a counter
- Proper ROS 2 lifecycle is followed with init, spin, and shutdown

## Running the Solution
1. Save the code to a file (e.g., `exercise1_solution.py`)
2. Make it executable: `chmod +x exercise1_solution.py`
3. Source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
4. Run the node: `python3 exercise1_solution.py`