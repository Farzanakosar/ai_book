#!/usr/bin/env python3
# action_server.py
"""
Action server example for ROS 2.

Learning objectives:
- Understand how to create an action server
- Learn how to provide feedback during long-running tasks
- Practice goal handling and result reporting

Prerequisites:
- Basic Python knowledge
- ROS 2 Humble environment
"""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """
    Fibonacci action server that calculates Fibonacci sequence up to a given order.
    """
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

        self.get_logger().info('Fibonacci action server initialized')

    def execute_callback(self, goal_handle):
        """
        Execute callback for the Fibonacci action.

        Args:
            goal_handle: The goal handle for the incoming goal

        Returns:
            Fibonacci.Result: The result of the Fibonacci calculation
        """
        self.get_logger().info(f'Executing goal: order = {goal_handle.request.order}')

        # Initialize the sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Check if order is small enough to handle directly
        if goal_handle.request.order <= 1:
            if goal_handle.request.order == 0:
                feedback_msg.sequence = [0] if feedback_msg.sequence else []
            elif goal_handle.request.order == 1:
                feedback_msg.sequence = [0, 1]
        else:
            # Calculate Fibonacci sequence up to the requested order
            for i in range(1, goal_handle.request.order):
                # Check if the goal was canceled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result = Fibonacci.Result()
                    result.sequence = []
                    return result

                # Calculate next Fibonacci number
                if len(feedback_msg.sequence) > i:
                    next_fib = feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
                    feedback_msg.sequence.append(next_fib)

                    # Publish feedback
                    goal_handle.publish_feedback(feedback_msg)
                    self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

                # Simulate some processing time
                time.sleep(0.1)

        # Complete the goal successfully
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Goal succeeded with result: {result.sequence}')
        return result


def main(args=None):
    """
    Main function to run the Fibonacci action server.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        action_server = FibonacciActionServer()
        action_server.get_logger().info('Action server is running. Press Ctrl+C to stop.')

        # Run the action server
        rclpy.spin(action_server)

    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()