#!/usr/bin/env python3
# timer_publisher.py
"""
Timer-based publisher example for ROS 2.

Learning objectives:
- Understand how to create timers in ROS 2 nodes
- Learn how to publish messages at regular intervals
- Practice timer callback implementation

Prerequisites:
- Basic Python knowledge
- ROS 2 Humble environment
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TimerPublisher(Node):
    """
    Timer-based publisher that publishes messages at regular intervals.
    """
    def __init__(self):
        super().__init__('timer_publisher')

        # Create a publisher for the timer topic
        self.publisher_ = self.create_publisher(String, 'timer_topic', 10)

        # Create a timer that fires every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track published messages
        self.i = 0

        self.get_logger().info(f'Timer publisher initialized, publishing every {timer_period} seconds')

    def timer_callback(self):
        """
        Callback function that is called by the timer at regular intervals.
        """
        # Create and populate the message
        msg = String()
        msg.data = f'Timer message {self.i}: {self.get_clock().now().seconds_nanoseconds()}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Published: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to run the timer publisher example.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = TimerPublisher()
        node.get_logger().info('Timer publisher is running. Press Ctrl+C to stop.')

        # Run the node until interrupted
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()