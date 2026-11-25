
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def pixel_to_3d(u, v, depth, fx, fy, cx, cy):
    """Convert pixel coordinates and depth to 3D coordinates."""

    if not math.isfinite(depth) or depth <= 0:
        return None

    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth

    return X, Y, Z


class DepthReader(Node):

    def __init__(self):
        super().__init__('depth_reader')

        self.bridge = CvBridge()
        self.last_print = 0.0

        # Camera calibration parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Subscribe to camera calibration
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_callback,
            qos_profile_sensor_data
        )

        # Subscribe to aligned depth images
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            "Waiting for CameraInfo and depth images..."
        )

    def info_callback(self, msg):
        # Extract camera intrinsics from matrix K
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):

        # Wait until calibration is received
        if self.fx is None:
            return

        # Print every 2 seconds
        if time.monotonic() - self.last_print < 2.0:
            return

        self.last_print = time.monotonic()

        # Convert ROS image to NumPy array
        depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

        height, width = depth_image.shape[:2]

       # Select a pixel away from the image center
        u = width // 2 + 80
        v = height // 2 - 40

        # Extract a 5x5 region
        patch = depth_image[v-2:v+3, u-2:u+3]

        valid = patch[np.isfinite(patch) & (patch > 0)]

        if len(valid) == 0:
            self.get_logger().info("No valid depth")
            return

        depth = float(np.median(valid))

        # Convert depth to meters
        if msg.encoding == '16UC1':
            depth *= 0.001
        elif msg.encoding != '32FC1':
            self.get_logger().warn(
                f"Unsupported encoding: {msg.encoding}"
            )
            return

        # Calculate 3D position
        position = pixel_to_3d(
            u, v, depth,
            self.fx, self.fy, self.cx, self.cy
        )

        if position is not None:
            X, Y, Z = position

            self.get_logger().info(
                f"Pixel ({u}, {v}) | "
                f"X={X:.3f} m, "
                f"Y={Y:.3f} m, "
                f"Z={Z:.3f} m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DepthReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()