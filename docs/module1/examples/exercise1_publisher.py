#!/usr/bin/env python3
# exercise1_publisher.py
"""
Exercise 1: Create a simple publisher node that publishes "Hello World" messages.

Learning objectives:
- Practice creating a basic publisher node
- Understand the structure of a ROS 2 publisher
- Learn how to publish custom messages

Prerequisites:
- Completed the ROS 2 Core Concepts chapter
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Exercise1Publisher(Node):
    """
    Exercise publisher node that publishes "Hello World" messages.
    """
    def __init__(self):
        super().__init__('exercise1_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Exercise 1 publisher node initialized')

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
    Main function to run the exercise 1 publisher.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = Exercise1Publisher()

        # Run the node until interrupted
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()