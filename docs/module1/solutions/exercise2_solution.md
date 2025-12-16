# Exercise 2 Solution: Create a subscriber that listens to a custom topic

## Problem Statement
Create a subscriber that listens to a custom topic called "custom_topic" and logs the messages.

## Expected Outcome
A ROS 2 node that subscribes to "custom_topic" and logs all received messages.

## Solution

```python
#!/usr/bin/env python3
# exercise2_solution.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CustomSubscriber(Node):
    def __init__(self):
        super().__init__('custom_subscriber')
        self.subscription = self.create_subscription(
            String,
            'custom_topic',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Custom subscriber node initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received from custom_topic: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = CustomSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Key Points
- The node creates a subscription to the 'custom_topic' with a queue size of 10
- A callback function processes incoming messages and logs them
- The subscription object is referenced to prevent garbage collection
- Proper ROS 2 lifecycle is followed with init, spin, and shutdown

## Running the Solution
1. Save the code to a file (e.g., `exercise2_solution.py`)
2. Make it executable: `chmod +x exercise2_solution.py`
3. Source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
4. Run the node: `python3 exercise2_solution.py`
5. To test, run a publisher that sends messages to 'custom_topic' in another terminal