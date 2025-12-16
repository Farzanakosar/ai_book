#!/usr/bin/env python3
# exercise2_subscriber.py
"""
Exercise 2: Create a subscriber that listens to a custom topic and logs the messages.

Learning objectives:
- Practice creating a basic subscriber node
- Understand how to subscribe to custom topics
- Learn how to process received messages

Prerequisites:
- Completed the ROS 2 Core Concepts chapter
- Running publisher on 'custom_topic' (or use the exercise1_publisher.py example)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Exercise2Subscriber(Node):
    """
    Exercise subscriber node that subscribes to 'custom_topic' and logs received messages.
    """
    def __init__(self):
        super().__init__('exercise2_subscriber')
        self.subscription = self.create_subscription(
            String,
            'custom_topic',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Exercise 2 subscriber node initialized')

    def listener_callback(self, msg):
        """
        Callback function that is called when a message is received.

        Args:
            msg: The received message of type String
        """
        self.get_logger().info(f'Custom topic message: "{msg.data}"')


def main(args=None):
    """
    Main function to run the exercise 2 subscriber.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = Exercise2Subscriber()

        # Run the node until interrupted
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()