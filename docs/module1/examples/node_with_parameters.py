#!/usr/bin/env python3
# node_with_parameters.py
"""
Node with parameters example for ROS 2.

Learning objectives:
- Understand how to declare and use parameters in ROS 2 nodes
- Learn how to configure nodes externally
- Practice parameter validation and error handling

Prerequisites:
- Basic Python knowledge
- ROS 2 Humble environment
"""

import rclpy
from rclpy.node import Node


class ParameterNode(Node):
    """
    Node that demonstrates parameter usage in ROS 2.
    """
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('frequency', 1.0, 'Publish frequency in Hz')
        self.declare_parameter('message', 'Hello', 'Message to publish')
        self.declare_parameter('count', 10, 'Number of messages to publish before stopping')

        # Access parameter values
        self.frequency = self.get_parameter('frequency').value
        self.message = self.get_parameter('message').value
        self.count = self.get_parameter('count').value

        # Validate parameters
        if self.frequency <= 0:
            self.get_logger().warn('Frequency should be positive, using default value')
            self.frequency = 1.0

        if self.count <= 0:
            self.get_logger().warn('Count should be positive, using default value')
            self.count = 10

        self.get_logger().info(f'Node initialized with parameters:')
        self.get_logger().info(f'  Frequency: {self.frequency} Hz')
        self.get_logger().info(f'  Message: "{self.message}"')
        self.get_logger().info(f'  Count: {self.count}')


def main(args=None):
    """
    Main function to run the parameter node example.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = ParameterNode()

        # Log parameter values
        node.get_logger().info('Parameter node is running. Press Ctrl+C to stop.')

        # Keep the node running
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()