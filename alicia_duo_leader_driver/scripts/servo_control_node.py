#!/usr/bin/env python3
# coding=utf-8

"""
标准机械臂控制节点
接收7自由度(6关节+夹爪)的弧度控制命令，转换为硬件协议格式并发送。
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np
from std_msgs.msg import UInt8MultiArray, Bool

# 常量定义
RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0

# 协议常量
FRAME_HEADER = 0xAA
FRAME_FOOTER = 0xFF
CMD_SERVO_CONTROL = 0x04
CMD_EXTENDED_CONTROL = 0x06
CMD_GRIPPER_CONTROL = 0x02
CMD_ZERO_CAL = 0x03
CMD_DEMO_CONTROL = 0x13


class ServoControlNode(Node):
    def __init__(self):
        """初始化节点"""
        super().__init__('servo_control_node')

        # 配置参数
        self.declare_parameter('servo_count', 9)
        self.declare_parameter('debug_mode', False)

        self.servo_count = self.get_parameter('servo_count').value
        self.joint_count = 6
        self.debug_mode = self.get_parameter('debug_mode').value

        # 设置ROS通信
        self._setup_communication()

        self.get_logger().info("机械臂控制节点已初始化，使用标准弧度接口")

    def _setup_communication(self):
        """设置订阅者和发布者"""
        self.serial_pub = self.create_publisher(UInt8MultiArray, '/send_serial_data', 10)
        self.zero_calibration = self.create_subscription(
            Bool, '/zero_calibrate', self.zero_calib_callback, 10)

    def calculate_checksum(self, frame):
        """计算校验和"""
        checksum = sum(frame[3:-2]) % 2
        return checksum

    def rad_to_hardware_value(self, angle_rad):
        """将弧度转换为硬件值(0-4095)"""
        angle_deg = angle_rad * RAD_TO_DEG

        if angle_deg < -180.0 or angle_deg > 180.0:
            self.get_logger().warn("角度值超出范围: %.2f度，会被截断" % angle_deg)
            angle_deg = max(-180.0, min(180.0, angle_deg))

        value = int((angle_deg + 180.0) / 360.0 * 4096)
        return max(0, min(4095, value))

    def _print_debug_info(self):
        """打印调试信息"""
        servo_hex = " ".join([f"{b:02X}" for b in self.servo_angle_frame])
        self.get_logger().debug("舵机帧: %s" % servo_hex)

        gripper_hex = " ".join([f"{b:02X}" for b in self.gripper_frame])
        self.get_logger().debug("夹爪帧: %s" % gripper_hex)

    def zero_calib_callback(self, msg):
        """处理零点校准命令"""
        try:
            if self.debug_mode:
                self.get_logger().info("接收到零点校准命令: %s" % str(msg.data))

            if msg.data:
                zero_calib_msg = self.frame_ge(CMD_ZERO_CAL)
                self.get_logger().info("开始零点校准")
                self.serial_pub.publish(zero_calib_msg)

            self.get_logger().info("零点校准完成")

        except Exception as e:
            self.get_logger().error("处理零点校准命令出错: %s" % str(e))

    def frame_ge(self, control_cmd, control_data=0x00, check=True):
        frame_d = [0] * 6
        frame_d[0] = FRAME_HEADER
        frame_d[1] = control_cmd
        frame_d[2] = 0x01
        frame_d[3] = control_data
        if check:
            frame_d[-2] = self.calculate_checksum(frame_d)
        else:
            frame_d[-2] = 0x00
        frame_d[-1] = FRAME_FOOTER

        binary_data = bytearray()
        for byte in frame_d:
            binary_data.append(byte)

        frame_d_msg = UInt8MultiArray()
        frame_d_msg.data = list(binary_data)
        return frame_d_msg


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    try:
        node = ServoControlNode()
        node.get_logger().info("机械臂控制节点已启动")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("机械臂控制节点异常: %s" % str(e))
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()