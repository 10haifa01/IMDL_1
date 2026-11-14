#!/usr/bin/env python3
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_ros
import tf2_geometry_msgs  # Required for transforming PointStamped/PoseStamped


class DepthLocalizer:
    """
    Converts 2D pixel detections + aligned depth frames into 3D coordinates
    and transforms them into the robot/map frame.
    """

    def __init__(self, node: Node, block_size_m: float = 0.03):
        self.node = node
        self.block_size_m = block_size_m  # 3cm x 3cm cubes
        self.intrinsics = None  # Will store (fx, fy, cx, cy)

        # TF2 listener for converting camera_color_optical_frame -> base_link / map
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def set_intrinsics(self, msg: CameraInfo) -> None:
        """Extracts focal lengths and optical center from CameraInfo."""
        if self.intrinsics is None:
            # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.node.get_logger().info("Camera intrinsics loaded in DepthLocalizer.")

    def is_ready(self) -> bool:
        """Returns True once CameraInfo has been received."""
        return self.intrinsics is not None

    def pixel_to_3d(self, u: int, v: int, depth_image: np.ndarray, pixel_width: int = None):
        """
        Projects a 2D pixel (u, v) to 3D (X, Y, Z) in meters in camera_color_optical_frame.
        
        Args:
            u: Horizontal pixel coordinate (center of bounding box)
            v: Vertical pixel coordinate (center of bounding box)
            depth_image: 2D numpy array (16UC1, values in millimeters)
            pixel_width: Optional bounding box width in pixels to validate 3cm physical size

        Returns:
            tuple (X, Y, Z) in meters, or None if depth is invalid or size check fails.
        """
        if not self.is_ready():
            return None

        # TODO:
        # 1. Sample a 3x3 patch around (u, v) in depth_image, filter out 0s, take median, convert mm -> m.
        # 2. Check valid D455 depth range (e.g., 0.35m <= Z <= 2.5m).
        # 3. If pixel_width is given, verify physical width: (pixel_width * Z / fx) ~= 0.03m.
        # 4. Compute X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy.
        return None

    def to_robot_frame(self, x: float, y: float, z: float, source_frame: str,
                       stamp, target_frame: str = 'base_link') -> PoseStamped | None:
        """
        Transforms 3D coordinates from source_frame (e.g., camera_color_optical_frame)
        into target_frame (e.g., 'base_link' or 'map').
        """
        pose = PoseStamped()
        pose.header.frame_id = source_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0  # Default neutral orientation

        if target_frame == source_frame:
            return pose

        # TODO:
        # Use self.tf_buffer.transform(point_stamped, target_frame) with a try/except fallback
        # so it doesn't crash before the URDF camera transform is added.
        return pose