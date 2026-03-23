#ifndef SERIAL_SERVER_NODE_H
#define SERIAL_SERVER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <serial.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

class SerialServerNode : public rclcpp::Node {
public:
    SerialServerNode();
    ~SerialServerNode();

private:
    // ROS相关
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_serial_data_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_send_data_;

    // 添加调试模式标志
    bool debug_mode_;

    // 串口相关
    serial::Serial serial_port_;
    std::mutex serial_mutex_;
    std::thread read_thread_;
    std::atomic<bool> is_running_;
    std::string port_name_;

    // 配置参数
    int baudrate_;
    int timeout_ms_;

    // 重连定时器
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    int failure_counter_;

    // 添加缺少的函数声明
    void shutdownCallback();

    bool connectSerial();
    void sendSerialDataCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void readFrameThread();
    std::string findSerialPort();
    bool serialDataCheck(const std::vector<uint8_t>& data);
    uint8_t sumElements(const std::vector<uint8_t>& data);
    void printHexFrame(const std::vector<uint8_t>& data, int type);
};

#endif // SERIAL_SERVER_NODE_H