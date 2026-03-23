#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time


class ArmZeroCalibrator(Node):
    def __init__(self):
        """初始化校准节点"""
        super().__init__('arm_zero_calibration')
        self.calibration_pub = self.create_publisher(Bool, '/zero_calibrate', 10)
        self.get_logger().info("Arm Zero Calibration node initialized")

    def wait_for_publisher(self, timeout=5):
        """等待发布器连接"""
        self.get_logger().info("Waiting for publisher to connect...")
        start_time = time.time()
        while self.calibration_pub.get_subscription_count() == 0:
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout waiting for publisher connection")
                return False
            time.sleep(0.1)
        self.get_logger().info("Publisher connected")
        return True

    def calibrate(self):
        """发送校准命令到机械臂"""
        self.get_logger().info("Sending zero calibration command...")

        if not self.wait_for_publisher():
            self.get_logger().error("Failed to connect to publisher. Calibration aborted.")
            return

        calibrate_msg = Bool(data=True)
        self.get_logger().info("Publishing zero calibration command: %s" % str(calibrate_msg.data))
        self.calibration_pub.publish(calibrate_msg)

        time.sleep(2)
        self.get_logger().info("Zero calibration completed successfully")


def main(args=None):
    rclpy.init(args=args)
    try:
        calibrator = ArmZeroCalibrator()
        calibrator.calibrate()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()