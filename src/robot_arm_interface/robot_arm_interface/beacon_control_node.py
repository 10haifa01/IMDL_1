#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class BeaconControlNode(Node):
    def __init__(self):
        super().__init__('beacon_control_node')
        self.get_logger().info('180° Light Beacon Control Node Initialized')
        # TODO: Subscribe to light target coordinates / state requests
        # TODO: Clamp pan angle within [-90°, +90°] range

    def set_beacon_angle(self, angle_degrees):
        pass

    def set_power_led(self, state: bool):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = BeaconControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()