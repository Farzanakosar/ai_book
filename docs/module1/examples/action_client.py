#!/usr/bin/env python3
# action_client.py
"""
Action client example for ROS 2.

Learning objectives:
- Understand how to create an action client
- Learn how to send goals to action servers
- Practice handling feedback and results

Prerequisites:
- Basic Python knowledge
- ROS 2 Humble environment
- Running action server (or use the action_server.py example)
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """
    Fibonacci action client that sends goals to the Fibonacci action server.
    """
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

        self.get_logger().info('Fibonacci action client initialized')

    def send_goal(self, order):
        """
        Send a goal to the Fibonacci action server.

        Args:
            order: The order of the Fibonacci sequence to calculate
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: calculate Fibonacci sequence up to order {order}')

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Add a callback for when the goal is accepted/rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback for when the goal response is received.

        Args:
            future: The future object containing the goal handle
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server')

        # Request the result when the goal is done
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback for handling feedback from the action server.

        Args:
            feedback_msg: The feedback message from the server
        """
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

    def get_result_callback(self, future):
        """
        Callback for handling the final result from the action server.

        Args:
            future: The future object containing the result
        """
        result = future.result().result
        self.get_logger().info(f'Final result: {result.sequence}')


def main(args=None):
    """
    Main function to run the Fibonacci action client.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        action_client = FibonacciActionClient()

        # Send a goal to calculate Fibonacci sequence up to order 5
        action_client.send_goal(5)

        # Keep the client running to receive feedback and results
        rclpy.spin(action_client)

    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()