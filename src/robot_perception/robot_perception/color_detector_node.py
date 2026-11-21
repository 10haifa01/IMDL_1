#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import message_filters

from perception_msgs.msg import DetectedBlock, DetectedBlockArray
from robot_perception.depth_localizer import DepthLocalizer


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        # CVBridge translates ROS Image messages into OpenCV numpy arrays
        self.bridge = CvBridge()
        self.target_color = ""
        # so it knows the cubes are 3x3cm
        self.localizer = DepthLocalizer(self, block_size_m=0.03)

        # Load HSV thresholds from installed share directory
        pkg_share = get_package_share_directory('robot_perception')
        yaml_path = os.path.join(pkg_share, 'config', 'hsv_thresholds.yaml')
        self.hsv_ranges, self.filter_cfg = self._load_config(yaml_path)

        # --- SUBSCRIBERS ---
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10
        )
        self.target_sub = self.create_subscription(
            String, '/brain/target_color', self.target_cb, 10
        )

        # Synchronize RGB and Aligned Depth frames (pair them by timestamp)
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_image_cb)

        # --- PUBLISHERS ---
        # carries each blocks color string, confidence and 3D PoseStampedlk
        self.blocks_pub = self.create_publisher(
            DetectedBlockArray, '/perception/detected_blocks', 10
        )
        self.obstacles_pub = self.create_publisher(
            PointCloud2, '/perception/block_obstacles', 10
        )
        self.debug_pub = self.create_publisher(
            Image, '/perception/debug_image', 10
        )

        self.get_logger().info("ColorDetectorNode initialized and waiting for frames.")

    # reads hsv_thresholds.yaml and converts [H, S, V] lists into uint8 NumPy arrays
    def _load_config(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        parsed_ranges = {}
        for color, bounds_list in data['hsv_ranges'].items():
            parsed_ranges[color] = [
                (np.array(b['lower'], dtype=np.uint8), np.array(b['upper'], dtype=np.uint8))
                for b in bounds_list
            ]
        return parsed_ranges, data['contour_filter']

    # forwards cameras focal lengths and optical center to self.localizer
    def info_cb(self, msg: CameraInfo):
        self.localizer.set_intrinsics(msg)

    # updates self.target_color whenever robot_brain commands a new target color
    def target_cb(self, msg: String):
        self.target_color = msg.data.strip().lower()
        self.get_logger().info(f"Target color updated: '{self.target_color}'")

    # Main Vision Pipeline
    def sync_image_cb(self, rgb_msg: Image, depth_msg: Image):
        # preprocessing: RGB -> BGR, Gaussian Blur to smooth out wood grain and sensor noise, BGR -> HSV
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')

        blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)

        block_array_msg = DetectedBlockArray()
        block_array_msg.header = rgb_msg.header

        min_area = self.filter_cfg['min_area']
        max_area = self.filter_cfg['max_area']
        min_aspect = self.filter_cfg['min_aspect']
        max_aspect = self.filter_cfg['max_aspect']

        # color masking (HSV ranges), noise cleanup using morphological opening (removes tiny background speckles) and closing (fills small holes inside a cubes face)
        for color_name, bounds_list in self.hsv_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in bounds_list:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # contour filtering -> rejects anything whose pixel area or bounding-box aspect ratio falls outside contour_filter from yaml file
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (min_area <= area <= max_area):
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / float(h)
                if not (min_aspect <= aspect_ratio <= max_aspect):
                    continue

                u = int(x + w / 2)
                v = int(y + h / 2)

                # Call teammate's DepthLocalizer (returns None until her branch is merged)
                coords_3d = self.localizer.pixel_to_3d(u, v, depth_mm, pixel_width=w)

                block_msg = DetectedBlock()
                block_msg.color = color_name
                block_msg.confidence = min(1.0, float(area) / 2000.0)

                if coords_3d is not None:
                    X, Y, Z = coords_3d
                    pose_stamped = self.localizer.to_robot_frame(
                        X, Y, Z,
                        rgb_msg.header.frame_id,
                        rgb_msg.header.stamp,
                        target_frame='base_link'
                    )
                    if pose_stamped is not None:
                        block_msg.pose = pose_stamped
                    label = f"{color_name} ({X:.2f}, {Y:.2f}, {Z:.2f})m"
                else:
                    # Placeholder pose in camera frame until DepthLocalizer is merged
                    pose_stamped = PoseStamped()
                    pose_stamped.header = rgb_msg.header
                    block_msg.pose = pose_stamped
                    label = f"{color_name} [2D: {u},{v}]"

                block_array_msg.blocks.append(block_msg)

                # Highlight target color in green, other blocks in yellow/orange
                box_color = (0, 255, 0) if color_name == self.target_color else (0, 165, 255)
                cv2.rectangle(bgr, (x, y), (x + w, y + h), box_color, 2)
                cv2.circle(bgr, (u, v), 4, (255, 0, 0), -1)
                cv2.putText(
                    bgr, label, (x, max(15, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2
                )

        self.blocks_pub.publish(block_array_msg)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(bgr, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()