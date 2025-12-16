#!/usr/bin/env python3
# simple_subscriber.py
"""
Simple subscriber node example for ROS 2.

Learning objectives:
- Understand how to create a basic subscriber node
- Learn how to subscribe to a topic and receive messages
- See the structure of a ROS 2 subscriber

Prerequisites:
- Basic Python knowledge
- ROS 2 Humble environment
- Running publisher node (or use the simple_publisher.py example)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    Simple subscriber node that subscribes to 'topic' and logs received messages.
    """
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Simple subscriber node initialized')

    def listener_callback(self, msg):
        """
        Callback function that is called when a message is received.

        Args:
            msg: The received message of type String
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to run the simple subscriber example.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = SimpleSubscriber()

        # Run the node until interrupted
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()