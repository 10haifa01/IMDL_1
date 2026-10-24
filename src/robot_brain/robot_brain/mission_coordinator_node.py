#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MissionCoordinatorNode(Node):
    def __init__(self):
        super().__init__('mission_coordinator_node')
        self.get_logger().info('Mission Coordinator State Machine Initialized')
        # State Flow: IDLE -> NAV_TO_OBJECT -> BEACON_LIGHT -> DETECT_COLOR -> PICK -> NAV_TO_DROP -> PLACE -> RETURN

def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()