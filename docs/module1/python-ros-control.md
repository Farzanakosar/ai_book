---
sidebar_position: 2
---

# Python-to-ROS Control (rclpy)

## Learning Objectives

By the end of this chapter, you will be able to:
- Use the rclpy client library to create ROS 2 nodes in Python
- Implement parameterized nodes with configurable behavior
- Create and use actions for goal-oriented tasks
- Implement timer-based publishers for periodic tasks
- Understand advanced publisher/subscriber patterns in rclpy

## Prerequisites

Before starting this chapter, you should have:
- Completed the ROS 2 Core Concepts chapter
- Basic understanding of ROS 2 nodes, topics, and services
- Python knowledge including classes and callbacks

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl) and enables Python developers to create ROS 2 nodes, publishers, subscribers, services, and actions.

### Key Features of rclpy:
- Object-oriented API design
- Integration with Python's asyncio (optional)
- Support for all ROS 2 communication patterns
- Parameter management
- Logging and lifecycle management

## Node Creation and Management

Creating a node in rclpy involves inheriting from the `Node` class and initializing it with a unique name.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MyPythonNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node created successfully')

def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Parameters in rclpy

Parameters allow nodes to be configured externally. They can be declared with default values and type constraints.

### Parameter Declaration and Usage

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('message', 'Hello')
        self.declare_parameter('count', 10)

        # Access parameter values
        self.frequency = self.get_parameter('frequency').value
        self.message = self.get_parameter('message').value
        self.count = self.get_parameter('count').value

        self.get_logger().info(f'Frequency: {self.frequency}, Message: {self.message}, Count: {self.count}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions in ROS 2

Actions are used for goal-oriented tasks that take time to complete and may be canceled. They provide feedback during execution and result upon completion.

### Action Server Implementation

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client Implementation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal: {order}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()

    try:
        action_client.send_goal(10)
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Timer-Based Publishers

Timers allow nodes to execute callbacks at regular intervals, which is useful for publishing sensor data or control commands.

### Timer Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TimerPublisher(Node):
    def __init__(self):
        super().__init__('timer_publisher')
        self.publisher_ = self.create_publisher(String, 'timer_topic', 10)

        # Create a timer that fires every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Timer message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    timer_publisher = TimerPublisher()
    try:
        rclpy.spin(timer_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        timer_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Publisher/Subscriber Patterns

### Quality of Service (QoS) Settings

QoS settings allow fine-tuning of communication behavior:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Create a QoS profile for sensor data
sensor_qos = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)

publisher = node.create_publisher(String, 'sensor_topic', sensor_qos)
```

### Publisher with Custom Message Types

```python
# Assuming you have a custom message type
from my_package.msg import CustomMessage

class CustomPublisher(Node):
    def __init__(self):
        super().__init__('custom_publisher')
        self.publisher_ = self.create_publisher(CustomMessage, 'custom_topic', 10)

    def publish_custom_message(self, value1, value2):
        msg = CustomMessage()
        msg.field1 = value1
        msg.field2 = value2
        self.publisher_.publish(msg)
```

## Running the Examples

### Setup Instructions

1. Make sure your ROS 2 environment is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Navigate to your workspace directory where you want to run the examples.

3. Create Python files with the examples above and save them with `.py` extensions.

4. Make the files executable:
   ```bash
   chmod +x your_file.py
   ```

### Running Parameter Example

1. Run the parameter node with custom values:
   ```bash
   ros2 run your_package parameter_node --ros-args -p frequency:=2.0 -p message:="Custom Message"
   ```

### Running Action Example

1. Open a terminal and run the action server:
   ```bash
   python3 fibonacci_action_server.py
   ```

2. In another terminal, run the action client:
   ```bash
   python3 fibonacci_action_client.py
   ```

## Summary

In this chapter, you learned advanced Python-to-ROS control techniques:

- **Parameters**: How to configure nodes externally with type-safe parameters
- **Actions**: Goal-oriented communication pattern with feedback and cancellation
- **Timers**: How to execute callbacks at regular intervals
- **QoS Settings**: How to fine-tune communication behavior for different use cases

These advanced patterns are essential for building robust and configurable robotic applications in ROS 2.