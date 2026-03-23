#!/usr/bin/env python3
# coding=utf-8

"""
ROS2节点示例：订阅并打印来自 /arm_joint_state 话题的机械臂状态数据。
"""

import rclpy
from rclpy.node import Node
from alicia_duo_leader_driver.msg import ArmJointState
import math

RAD_TO_DEG = 180.0 / math.pi


class ArmStateReaderDemo(Node):
    def __init__(self):
        super().__init__('arm_state_reader_demo')
        self.get_logger().info("Arm State Reader Demo Node Started.")
        self.create_subscription(
            ArmJointState, '/arm_joint_state', self.arm_state_callback, 10)
        self.get_logger().info("Subscribed to /arm_joint_state. Waiting for messages...")

    def arm_state_callback(self, msg):
        self.get_logger().info("--- Received Arm Joint State ---")
        self.get_logger().info(f"  Joint 1: {msg.joint1 * RAD_TO_DEG:.2f} deg ({msg.joint1:.4f} rad)")
        self.get_logger().info(f"  Joint 2: {msg.joint2 * RAD_TO_DEG:.2f} deg ({msg.joint2:.4f} rad)")
        self.get_logger().info(f"  Joint 3: {msg.joint3 * RAD_TO_DEG:.2f} deg ({msg.joint3:.4f} rad)")
        self.get_logger().info(f"  Joint 4: {msg.joint4 * RAD_TO_DEG:.2f} deg ({msg.joint4:.4f} rad)")
        self.get_logger().info(f"  Joint 5: {msg.joint5 * RAD_TO_DEG:.2f} deg ({msg.joint5:.4f} rad)")
        self.get_logger().info(f"  Joint 6: {msg.joint6 * RAD_TO_DEG:.2f} deg ({msg.joint6:.4f} rad)")
        self.get_logger().info(f"  Gripper: {msg.gripper * RAD_TO_DEG:.2f} deg ({msg.gripper:.4f} rad)")
        self.get_logger().info(f"  Button 1: {msg.but1}")
        self.get_logger().info(f"  Button 2: {msg.but2}")
        self.get_logger().info("---------------------------------")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmStateReaderDemo()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()