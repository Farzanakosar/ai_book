#!/usr/bin/env python3
# exercise3_param_node.py
"""
Exercise 3: Create a parameterized node that adjusts its behavior based on ROS parameters.

Learning objectives:
- Practice using ROS 2 parameters in nodes
- Understand how to declare and access parameters
- Learn parameter validation and error handling

Prerequisites:
- Completed the Python-to-ROS Control chapter
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Exercise3ParamNode(Node):
    """
    Exercise node that demonstrates parameter usage in ROS 2.
    """
    def __init__(self):
        super().__init__('exercise3_param_node')

        # Declare parameters with default values
        self.declare_parameter('message', 'Default Message', 'Message to publish')
        self.declare_parameter('publish_rate', 1.0, 'Publish rate in Hz')
        self.declare_parameter('max_count', 10, 'Maximum number of messages to publish')

        # Access parameter values
        self.message = self.get_parameter('message').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_count = self.get_parameter('max_count').value

        # Validate parameters
        if self.publish_rate <= 0:
            self.get_logger().warn('Publish rate must be positive, using 1.0')
            self.publish_rate = 1.0

        if self.max_count <= 0:
            self.get_logger().warn('Max count must be positive, using 10')
            self.max_count = 10

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'exercise3_topic', 10)

        # Create timer based on publish rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize counter
        self.count = 0

        self.get_logger().info(f'Exercise 3 node initialized with parameters:')
        self.get_logger().info(f'  Message: "{self.message}"')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Max count: {self.max_count}')


    def timer_callback(self):
        """
        Timer callback that publishes messages based on parameters.
        """
        if self.count < self.max_count:
            msg = String()
            msg.data = f'{self.message} - Count: {self.count}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: "{msg.data}"')
            self.count += 1
        else:
            # Stop the timer when max count is reached
            self.timer.cancel()
            self.get_logger().info('Max count reached, stopping publisher')


def main(args=None):
    """
    Main function to run the exercise 3 parameterized node.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = Exercise3ParamNode()
        node.get_logger().info('Exercise 3 node is running. Press Ctrl+C to stop.')

        # Run the node until interrupted
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()