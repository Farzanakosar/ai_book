#!/usr/bin/env python3
# simple_publisher.py
"""
Simple publisher node example for ROS 2.

Learning objectives:
- Understand how to create a basic publisher node
- Learn how to publish messages to a topic
- See the structure of a ROS 2 publisher

Prerequisites:
- Basic Python knowledge
- ROS 2 Humble environment
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    Simple publisher node that publishes "Hello World" messages.
    """
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Simple publisher node initialized')

    def timer_callback(self):
        """
        Callback function that publishes messages at regular intervals.
        """
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    """
    Main function to run the simple publisher example.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = SimplePublisher()

        # Run the node until interrupted
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()