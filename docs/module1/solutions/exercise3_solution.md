# Exercise 3 Solution: Create a parameterized node

## Problem Statement
Create a parameterized node that adjusts its behavior based on ROS parameters.

## Expected Outcome
A ROS 2 node that can be configured with parameters for message content, publish rate, and maximum count.

## Solution

```python
#!/usr/bin/env python3
# exercise3_solution.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # Declare parameters with default values
        self.declare_parameter('message', 'Hello from parameterized node')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('max_count', 5)

        # Get parameter values
        self.message = self.get_parameter('message').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_count = self.get_parameter('max_count').value

        # Validate parameters
        if self.publish_rate <= 0:
            self.get_logger().warn('Publish rate must be positive, using 1.0')
            self.publish_rate = 1.0

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'parameterized_topic', 10)

        # Create timer based on parameter
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.count = 0
        self.get_logger().info(f'Node initialized with message: "{self.message}", rate: {self.publish_rate}Hz, max: {self.max_count}')

    def timer_callback(self):
        if self.count < self.max_count:
            msg = String()
            msg.data = f'{self.message} - {self.count}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: "{msg.data}"')
            self.count += 1
        else:
            self.timer.cancel()
            self.get_logger().info('Max count reached')


def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Solution
1. Save as `exercise3_solution.py`
2. Make executable: `chmod +x exercise3_solution.py`
3. Run with custom parameters: `ros2 run your_package exercise3_solution --ros-args -p message:="Custom message" -p publish_rate:=2.0 -p max_count:=3`

## Key Points
- Parameters are declared with default values using `declare_parameter()`
- Parameter values are accessed with `get_parameter().value`
- Parameters can be set at runtime using ROS 2 command line tools
- Validation ensures parameters have sensible values

## Expected Output
The node will publish messages with the specified content at the specified rate until reaching the maximum count.