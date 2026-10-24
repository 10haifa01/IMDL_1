#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.get_logger().info('Object Detector Node Initialized')
        # TODO: Subscribe to Intel RealSense RGB-D image/pointcloud topics
        # TODO: Publish /detected_object/pose

    def process_image(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()