#!/usr/bin/env python3
# coding=utf-8

"""
舵机状态处理节点
接收舵机和夹爪原始数据，转换为标准弧度单位并发布标准关节状态消息。
"""

import rclpy
from rclpy.node import Node
import math
import time
import numpy as np
from std_msgs.msg import UInt8MultiArray, Float32MultiArray, Int32
from alicia_duo_leader_driver.msg import ArmJointState

# 常量定义
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def u8_array_to_rad(u8_array, logger):
    """将原始字节数据转换为弧度值"""
    try:
        if len(u8_array) != 2:
            logger.warn("数据长度错误：需要2个字节，实际%d个字节" % len(u8_array))
            return 0.0

        hex_value = (u8_array[0] & 0xFF) | ((u8_array[1] & 0xFF) << 8)

        if hex_value < 0 or hex_value > 4095:
            logger.warn("舵机值超出范围: %d (有效范围0-4095)" % hex_value)
            hex_value = max(0, min(hex_value, 4095))

        angle_deg = -180.0 + (hex_value / 2048.0) * 180.0
        return angle_deg * DEG_TO_RAD

    except Exception as e:
        logger.error("字节转换异常: %s" % str(e))
        return 0.0


class ServoStatesNode(Node):
    def __init__(self):
        """初始化舵机状态节点"""
        super().__init__('servo_states_node')

        # 配置参数
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('servo_count', 9)
        self.declare_parameter('rate_limit', 0.001)

        self.debug_mode = self.get_parameter('debug_mode').value
        self.servo_count = self.get_parameter('servo_count').value
        self.rate_limit = self.get_parameter('rate_limit').value

        # 派生配置
        self.servo_id_min = 0
        self.servo_id_max = self.servo_count
        self.joint_count = 6

        # 数据存储
        self.servo_angles = np.zeros(self.servo_count, dtype=np.float32)
        self.gripper_angle_rad = 0.0
        self.but1 = 0
        self.but2 = 0

        # 关节映射
        self.servo_to_joint_map = {
            0: (0, 1.0),
            1: None,
            2: (1, 1.0),
            3: None,
            4: (2, 1.0),
            5: None,
            6: (3, 1.0),
            7: (4, 1.0),
            8: (5, 1.0),
        }

        # 时间戳 - 用于数据节流
        self._last_process_time = 0

        # 创建发布者和订阅者
        self._setup_ros_interface()

        self.get_logger().info("舵机状态节点已初始化 (单位: 弧度)")

    def _setup_ros_interface(self):
        """设置ROS接口"""
        self.joint_state_pub = self.create_publisher(ArmJointState, '/arm_joint_state', 10)
        self.array_pub = self.create_publisher(Float32MultiArray, '/servo_states_main', 10)

        self.servo_sub = self.create_subscription(
            UInt8MultiArray, '/servo_states', self.servo_states_callback, 10)
        self.gripper_sub = self.create_subscription(
            Float32MultiArray, '/gripper_angle', self.gripper_angle_callback, 10)

    def _should_process(self):
        """检查是否应该处理当前数据(节流控制)"""
        current_time = time.time()
        if current_time - self._last_process_time >= self.rate_limit:
            self._last_process_time = current_time
            return True
        return False

    def gripper_angle_callback(self, msg):
        self.gripper_angle_rad = msg.data[0] if len(msg.data) > 0 else 0.0

    def servo_states_callback(self, msg):
        """处理舵机状态数据"""
        if not self._should_process():
            return

        try:
            if len(msg.data) < 3:
                self.get_logger().warn("舵机数据帧过短")
                return

            expected_count = self.servo_count
            actual_count = msg.data[2] / 2

            if actual_count != expected_count:
                self.get_logger().warn(
                    "舵机数量不匹配: 期望%d个, 实际%.1f个" % (expected_count, actual_count))
                return

            for i in range(self.servo_id_min, self.servo_id_max):
                byte_idx = 3 + i * 2
                if byte_idx + 1 >= len(msg.data):
                    self.get_logger().warn("舵机数据越界: 索引%d超出范围" % byte_idx)
                    continue

                self.servo_angles[i] = u8_array_to_rad(
                    msg.data[byte_idx:byte_idx + 2], self.get_logger())

            joint_state = ArmJointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()

            joint_values = [0.0] * self.joint_count

            for servo_idx, mapping in self.servo_to_joint_map.items():
                if mapping is not None:
                    joint_idx, direction = mapping
                    joint_values[joint_idx] = float(self.servo_angles[servo_idx] * direction)

            joint_state.joint1 = joint_values[0]
            joint_state.joint2 = joint_values[1]
            joint_state.joint3 = joint_values[2]
            joint_state.joint4 = joint_values[3]
            joint_state.joint5 = joint_values[4]
            joint_state.joint6 = joint_values[5]
            joint_state.gripper = self.gripper_angle_rad
            joint_state.but1 = self.but1
            joint_state.but2 = self.but2

            self.joint_state_pub.publish(joint_state)

            compat_msg = Float32MultiArray()
            compat_data = joint_values + [self.gripper_angle_rad]
            compat_msg.data = compat_data
            self.array_pub.publish(compat_msg)

            if self.debug_mode:
                degrees = [rad * RAD_TO_DEG for rad in joint_values]
                self.get_logger().debug(
                    "关节角度: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]度, 夹爪: %.2f度" % (
                        degrees[0], degrees[1], degrees[2],
                        degrees[3], degrees[4], degrees[5],
                        self.gripper_angle_rad * RAD_TO_DEG))

        except Exception as e:
            self.get_logger().error("处理舵机状态异常: %s" % str(e))


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    try:
        node = ServoStatesNode()
        node.get_logger().info("舵机状态节点已启动")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("舵机状态节点异常: %s" % str(e))
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()