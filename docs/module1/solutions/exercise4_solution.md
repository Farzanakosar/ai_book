# Exercise 4 Solution: Create an action client

## Problem Statement
Create an action client that sends goals to an action server.

## Expected Outcome
A ROS 2 action client that sends Fibonacci calculation goals and handles feedback and results.

## Solution

```python
#!/usr/bin/env python3
# exercise4_solution.py

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciClient(Node):
    def __init__(self):
        super().__init__('fibonacci_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: {order}')
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Received feedback: {feedback.feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciClient()

    action_client.send_goal(10)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Solution
1. Save as `exercise4_solution.py`
2. Make executable: `chmod +x exercise4_solution.py`
3. First run an action server (like the provided action_server.py)
4. Then run the client: `python3 exercise4_solution.py`

## Key Points
- Action clients are created with `ActionClient(node, action_type, action_name)`
- Goals are sent asynchronously with `send_goal_async()`
- Feedback is handled with a callback function
- Results are handled when the goal completes
- The client must wait for the server before sending goals

## Expected Output
The client will send a goal to calculate Fibonacci sequence, receive feedback during processing, and display the final result.