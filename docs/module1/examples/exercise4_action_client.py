#!/usr/bin/env python3
# exercise4_action_client.py
"""
Exercise 4: Create an action client that sends goals to an action server.

Learning objectives:
- Practice creating an action client in ROS 2
- Understand how to send goals and handle responses
- Learn to work with action feedback and results

Prerequisites:
- Completed the Python-to-ROS Control chapter
- Running action server (or use the action_server.py example)
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class Exercise4ActionClient(Node):
    """
    Exercise action client that sends Fibonacci calculation goals.
    """
    def __init__(self):
        super().__init__('exercise4_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

        self.get_logger().info('Exercise 4 action client initialized')

    def send_fibonacci_goal(self, order):
        """
        Send a Fibonacci calculation goal to the action server.

        Args:
            order: The order of the Fibonacci sequence to calculate
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return

        # Create the goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending Fibonacci goal: order = {order}')

        # Send the goal asynchronously with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Add callback for goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response from the server.

        Args:
            future: Future object containing the goal handle
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the server')
            return

        self.get_logger().info('Goal was accepted by the server')

        # Request the result when the goal is done
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the action server.

        Args:
            feedback_msg: Feedback message from the server
        """
        sequence = feedback_msg.feedback.sequence
        self.get_logger().info(f'Feedback received - Current sequence length: {len(sequence)}, Last value: {sequence[-1] if sequence else "N/A"}')

    def get_result_callback(self, future):
        """
        Handle the final result from the action server.

        Args:
            future: Future object containing the result
        """
        result = future.result().result
        sequence = result.sequence
        self.get_logger().info(f'Action completed successfully! Final sequence: {sequence}')
        self.get_logger().info(f'Sequence length: {len(sequence)}')


def main(args=None):
    """
    Main function to run the exercise 4 action client.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        action_client = Exercise4ActionClient()

        # Send a goal to calculate Fibonacci sequence up to order 7
        action_client.send_fibonacci_goal(7)

        # Keep the client running to receive feedback and results
        rclpy.spin(action_client)

    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()