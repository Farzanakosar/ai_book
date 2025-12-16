#!/usr/bin/env python3
# service_server.py
"""
Simple service server example for ROS 2.

Learning objectives:
- Understand how to create a basic service server
- Learn how to define and respond to service requests
- See the structure of a ROS 2 service server

Prerequisites:
- Basic Python knowledge
- ROS 2 Humble environment
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServer(Node):
    """
    Service server that adds two integers.
    """
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server node initialized')

    def add_two_ints_callback(self, request, response):
        """
        Callback function that handles service requests.

        Args:
            request: The service request containing two integers
            response: The service response to be filled with the sum

        Returns:
            response: The response with the calculated sum
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    """
    Main function to run the service server example.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = ServiceServer()

        # Run the node until interrupted
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()