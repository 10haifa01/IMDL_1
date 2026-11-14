#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray

class PerceptionSkeletonNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')

        # --- SUBSCRIBERS ---
        self.rgb_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.rgb_cb, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.target_sub = self.create_subscription(
            String, '/brain/target_color', self.target_cb, 10)

        # --- PUBLISHERS ---
        # Tip: You can later replace PoseArray with a custom msg in irobot_create_msgs
        self.blocks_pub = self.create_publisher(
            PoseArray, '/perception/detected_blocks', 10)
        self.obstacles_pub = self.create_publisher(
            PointCloud2, '/perception/block_obstacles', 10)
        self.debug_pub = self.create_publisher(
            Image, '/perception/debug_image', 10)

        self.target_color = ""
        self.get_logger().info("Perception skeleton node initialized.")

    def rgb_cb(self, msg): pass
    def depth_cb(self, msg): pass
    def info_cb(self, msg): pass
    def target_cb(self, msg):
        self.target_color = msg.data
        self.get_logger().info(f"New target color received: {self.target_color}")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSkeletonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()