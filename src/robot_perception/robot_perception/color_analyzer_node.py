#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ColorAnalyzerNode(Node):
    def __init__(self):
        super().__init__('color_analyzer_node')
        self.get_logger().info('Color Analyzer Node Initialized')
        # TODO: Analyze image region of interest when light beacon is active
        # TODO: Return classified block color (e.g., Red, Green, Blue)

def main(args=None):
    rclpy.init(args=args)
    node = ColorAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()