#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# To calibrate HSV thresholds -> until only active cube appears in right half of window and floor is completely black
class HsvTunerNode(Node):
    def __init__(self):
        super().__init__('hsv_tuner_gui')
        # To convert ros Image messages into OpenCV NumPy arrays
        self.bridge = CvBridge()
        self.latest_bgr = None

        # Path to source YAML file so pressing 's' saves directly to repo
        self.yaml_path = os.path.expanduser(
            '~/IMDL_1/src/robot_perception/config/hsv_thresholds.yaml'
        )
        self.config = self._load_yaml()
        self.colors = list(self.config['hsv_ranges'].keys())
        self.current_idx = 0

        # Can be used for both the live realsense camera and a recorded ros2 bag
        self.sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_cb, 10
        )

        cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
        self._create_trackbars()
        self._set_trackbars_for_color(self.colors[self.current_idx])

        self.get_logger().info(
            "HSV Tuner started. Keys: [n] next color | [s] save to YAML | [q] quit"
        )

    # reads the yaml file into a python dictionary so the sliders start at our existing values rather than zero
    def _load_yaml(self):
        with open(self.yaml_path, 'r') as f:
            return yaml.safe_load(f)

    # creates six opencv sliders
    def _create_trackbars(self):
        nothing = lambda x: None
        cv2.createTrackbar('H Min', 'HSV Tuner', 0, 180, nothing)
        cv2.createTrackbar('H Max', 'HSV Tuner', 180, 180, nothing)
        cv2.createTrackbar('S Min', 'HSV Tuner', 90, 255, nothing)
        cv2.createTrackbar('S Max', 'HSV Tuner', 255, 255, nothing)
        cv2.createTrackbar('V Min', 'HSV Tuner', 50, 255, nothing)
        cv2.createTrackbar('V Max', 'HSV Tuner', 255, 255, nothing)

    # moves the six sliders to match the currently selected color whenever we switch colors
    def _set_trackbars_for_color(self, color_name):
        first_range = self.config['hsv_ranges'][color_name][0]
        l_h, l_s, l_v = first_range['lower']
        u_h, u_s, u_v = first_range['upper']
        cv2.setTrackbarPos('H Min', 'HSV Tuner', l_h)
        cv2.setTrackbarPos('H Max', 'HSV Tuner', u_h)
        cv2.setTrackbarPos('S Min', 'HSV Tuner', l_s)
        cv2.setTrackbarPos('S Max', 'HSV Tuner', u_s)
        cv2.setTrackbarPos('V Min', 'HSV Tuner', l_v)
        cv2.setTrackbarPos('V Max', 'HSV Tuner', u_v)

    # ROS2 subscriber callback -> every time a new camera or bag frame arrives, it converts the message into a BGR opencv image and stores it
    def image_cb(self, msg):
        self.latest_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # runs the main visual loop
    def spin_gui(self):
        while rclpy.ok():
            # grabs newest frame without freezing the opencv gui window
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.latest_bgr is None:
                continue

            bgr = self.latest_bgr.copy()
            # applies gaussian blur and converts BGR -> HSV
            blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # reads the six slider positions and builds a binary mask
            h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
            h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
            s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
            s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
            v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
            v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')

            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])

            mask = cv2.inRange(hsv, lower, upper)
            kernel = np.ones((5, 5), np.uint8)
            # morphological opening and closing for noise cleanup
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            result = cv2.bitwise_and(bgr, bgr, mask=mask)
            active_color = self.colors[self.current_idx]

            cv2.putText(
                bgr,
                f"Tuning: {active_color.upper()} | [n] Next  [s] Save  [q] Quit",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2
            )

            # displays original camera feed on left and isolated color mask on right
            combined = np.hstack((bgr, result))
            cv2.imshow('HSV Tuner', combined)

            # listens for keyboard shortcuts: n to switch colors, s to write updated slider values back to hsv_thresholds.yaml, q to quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('n'):
                self.current_idx = (self.current_idx + 1) % len(self.colors)
                next_color = self.colors[self.current_idx]
                self._set_trackbars_for_color(next_color)
                self.get_logger().info(f"Switched to color: {next_color}")
            elif key == ord('s'):
                self.config['hsv_ranges'][active_color][0] = {
                    'lower': [int(h_min), int(s_min), int(v_min)],
                    'upper': [int(h_max), int(s_max), int(v_max)],
                }
                with open(self.yaml_path, 'w') as f:
                    yaml.dump(self.config, f, sort_keys=False)
                self.get_logger().info(f"Saved {active_color} thresholds to {self.yaml_path}")

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HsvTunerNode()
    try:
        node.spin_gui()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()