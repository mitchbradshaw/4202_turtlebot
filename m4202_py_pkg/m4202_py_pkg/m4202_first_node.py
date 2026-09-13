#!/usr/bin/env python3

from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

class M4202Node(Node):
    def __init__(self) -> None:
        super().__init__("m4202_test")
        self.get_logger().info("hello metr")

def main(args=None):
    rclpy.init(args=args)

    node = M4202Node()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()