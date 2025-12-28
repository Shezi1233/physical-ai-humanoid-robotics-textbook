#!/usr/bin/env python3

"""
Basic ROS 2 Node Example
This example demonstrates how to create a simple ROS 2 node.
"""

import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from the minimal node!')


def main(args=None):
    rclpy.init(args=args)

    minimal_node = MinimalNode()

    rclpy.spin(minimal_node)

    # Destroy the node explicitly
    minimal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()