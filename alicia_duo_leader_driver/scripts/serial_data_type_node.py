#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32MultiArray
from std_msgs.msg import Int32
import time
import math


def print_hex_frame(logger, frame_msg):
    """
        @brief 将整数数组转换为十六进制字符串并打印

        @param frame_msg 整数数组
    """
    hex_output = ""

    for byte_int in frame_msg:
        hex_str = hex(byte_int)[2:].upper().zfill(2)

        if hex_output:
            hex_output += " " + hex_str
        else:
            hex_output += hex_str

    logger.info(hex_output)


class SerialReaderNode(Node):
    DEG_TO_RAD = math.pi / 180.0

    def __init__(self):
        super().__init__('read_serial_type_node')

        # 从参数服务器读取调试模式参数
        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').value
        self.get_logger().info("Debug mode: %s" % ("enabled" if self.debug_mode else "disabled"))

        # 统计数据
        self.frame_count = 0
        self.start_time = time.time()

        # 发布者
        self.pub_2 = self.create_publisher(Float32MultiArray, '/gripper_angle', 10)
        self.pub_4 = self.create_publisher(UInt8MultiArray, '/servo_states', 10)
        self.pub_6 = self.create_publisher(UInt8MultiArray, '/servo_states_6', 10)
        self.pub_EE = self.create_publisher(UInt8MultiArray, '/error_frame_deal', 10)

        # 订阅串口数据话题
        self.sub = self.create_subscription(
            UInt8MultiArray, '/read_serial_data', self.serial_data_callback, 50)

        # 诊断计时器
        if self.debug_mode:
            self.create_timer(30.0, self.report_stats)

    def report_stats(self):
        """报告处理统计信息"""
        duration = time.time() - self.start_time
        if duration > 0:
            rate = self.frame_count / duration
            self.get_logger().info("处理率: %.2f 帧/秒 (总计: %d 帧)" % (rate, self.frame_count))

    def serial_data_callback(self, serial_msg):
        """
            @brief 串口数据回调函数

            @param serial_msg: 串口数据
        """
        self.frame_count += 1

        if len(serial_msg.data) < 2:
            self.get_logger().warn("数据帧过短，无法处理")
            return

        command = serial_msg.data[1]

        if command == 0x02:

            button1 = False
            button2 = False

            frame = serial_msg.data
            if len(serial_msg.data) >= 10:
                button1 = (frame[8] & 0x01) != 0
                button2 = (frame[9] & 0x01) != 0

            if len(frame) < 8:
                self.get_logger().warn("夹爪数据帧长度不足")
                return
            if button1:
                gripper_raw = frame[6] | (frame[7] << 8)
            else:
                gripper_raw = frame[4] | (frame[5] << 8)

            if gripper_raw < 2048 or gripper_raw > 2900:
                gripper_raw = max(2048, min(gripper_raw, 2900))

            gripper_data_msg = Float32MultiArray()

            angle_deg = (gripper_raw - 2048) / 8.52

            gripper_rad = angle_deg * self.DEG_TO_RAD
            gripper_data_msg.data = [gripper_rad, float(serial_msg.data[8]), float(serial_msg.data[9])]
            self.pub_2.publish(gripper_data_msg)
            if self.debug_mode:
                self.get_logger().info("夹爪角度: %d" % gripper_raw)

        elif command == 0x04:
            self.pub_4.publish(serial_msg)
            if self.debug_mode:
                self.get_logger().info("舵机状态数据")
                print_hex_frame(self.get_logger(), serial_msg.data)

        elif command == 0x06:
            self.pub_6.publish(serial_msg)
            if self.debug_mode:
                self.get_logger().info("扩展舵机状态数据")
                print_hex_frame(self.get_logger(), serial_msg.data)

        elif command == 0xEE:
            self.pub_EE.publish(serial_msg)

            if len(serial_msg.data) >= 5:
                error_type = serial_msg.data[3]
                error_param = serial_msg.data[4]
                self.get_logger().warn("错误帧: 类型=0x%02X, 参数=0x%02X" % (error_type, error_param))
            else:
                self.get_logger().warn("0x%02X 有话题，但暂时无接收处理" % command)
        else:
            self.get_logger().warn("暂无该指令id 0x%02X 的功能" % command)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SerialReaderNode()
        node.get_logger().info("read_serial_type_node 已启动")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("节点异常: %s" % str(e))
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()