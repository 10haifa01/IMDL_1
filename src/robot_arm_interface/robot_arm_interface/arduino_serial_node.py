#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        self.get_logger().info('Arduino Serial Node Initialized')
        # TODO: Initialize serial port (/dev/ttyACM0 or /dev/ttyUSB0)
        # TODO: Subscriptions for joint_angles and beacon_command
        # TODO: Publisher for arm/beacon telemetry feedback

    def send_serial_command(self, command_str):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()