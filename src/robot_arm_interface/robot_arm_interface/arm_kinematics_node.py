#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ArmKinematicsNode(Node):
    def __init__(self):
        super().__init__('arm_kinematics_node')
        self.get_logger().info('Arm Kinematics Node Initialized')
        # TODO: Subscribe to target pick poses
        # TODO: Publish joint angles to arduino_serial_node

    def compute_inverse_kinematics(self, x, z):
        # 2-DOF IK calculations
        joint1_angle = 0.0
        joint2_angle = 0.0
        return joint1_angle, joint2_angle

def main(args=None):
    rclpy.init(args=args)
    node = ArmKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()