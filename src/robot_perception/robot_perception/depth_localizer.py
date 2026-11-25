#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs


class DepthLocalizer:  # Renamed from DepthReader; removed (Node) inheritance

    def __init__(self, node: Node, block_size_m: float = 0.03):
        self.node = node
        self.block_size_m = block_size_m

        # Camera calibration parameters (kept from teammate's code)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Added for transforming camera frame -> base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def set_intrinsics(self, msg: CameraInfo):
        """Added: called by ColorDetectorNode.info_cb"""
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.node.get_logger().info("Camera intrinsics loaded in DepthLocalizer.")

    def pixel_to_3d(self, u, v, depth_image, pixel_width=None, encoding='16UC1'):
        """Teammate's patch + projection logic wrapped into a class method"""
        if self.fx is None:
            return None

        height, width = depth_image.shape[:2]
        if not (0 <= u < width and 0 <= v < height):
            return None

        # Extract a 5x5 region (safe bounds)
        patch = depth_image[max(0, v - 2):min(height, v + 3),
                            max(0, u - 2):min(width, u + 3)]

        valid = patch[np.isfinite(patch) & (patch > 0)]
        if len(valid) == 0:
            return None

        depth = float(np.median(valid))

        # Convert depth to meters
        if encoding == '16UC1':
            depth *= 0.001
        elif encoding != '32FC1':
            self.node.get_logger().warn(f"Unsupported encoding: {encoding}")
            return None

        if not math.isfinite(depth) or depth <= 0:
            return None

        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth

        return X, Y, Z

    def to_robot_frame(self, x, y, z, source_frame, stamp, target_frame='base_link'):
        """Added: packages (X, Y, Z) into a PoseStamped and transforms to base_link"""
        pose = PoseStamped()
        pose.header.frame_id = source_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0

        if not target_frame or target_frame == source_frame:
            return pose

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            return tf2_geometry_msgs.do_transform_pose_stamped(pose, transform)
        except Exception:
            # Falls back to camera_color_optical_frame until URDF camera TF is added
            return pose