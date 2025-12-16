#!/usr/bin/env python3
# service_client.py
"""
Simple service client example for ROS 2.

Learning objectives:
- Understand how to create a basic service client
- Learn how to make service requests to a server
- See the structure of a ROS 2 service client

Prerequisites:
- Basic Python knowledge
- ROS 2 Humble environment
- Running service server (or use the service_server.py example)
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceClient(Node):
    """
    Service client that requests addition of two integers from a server.
    """
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service client node initialized')

    def send_request(self, a, b):
        """
        Send a request to the service server.

        Args:
            a: First integer to add
            b: Second integer to add

        Returns:
            Future: A future object representing the asynchronous call
        """
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.get_logger().info(f'Sending request: {a} + {b}')
        return self.client.call_async(request)


def main(args=None):
    """
    Main function to run the service client example.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = ServiceClient()

        if len(sys.argv) != 3:
            print('Usage: python3 service_client.py <int1> <int2>')
            return

        a = int(sys.argv[1])
        b = int(sys.argv[2])

        future = node.send_request(a, b)

        # Wait for the result
        rclpy.spin_until_future_complete(node, future)

        try:
            response = future.result()
            node.get_logger().info(f'Result: {a} + {b} = {response.sum}')
            print(f'Result: {a} + {b} = {response.sum}')
        except Exception as e:
            node.get_logger().error(f'Service call failed: {e}')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()