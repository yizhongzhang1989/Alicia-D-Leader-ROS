#include "serial_server_node/serial_server_node.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <unistd.h>

SerialServerNode::SerialServerNode()
    : Node("serial_server_node"),
      debug_mode_(false),
      is_running_(false),
      baudrate_(921600),
      timeout_ms_(1000),
      failure_counter_(0)
{
    // 声明并读取参数
    this->declare_parameter<bool>("debug_mode", false);
    this->declare_parameter<std::string>("port", "");
    this->declare_parameter<int>("baudrate", 921600);

    this->get_parameter("debug_mode", debug_mode_);
    this->get_parameter("port", port_name_);
    this->get_parameter("baudrate", baudrate_);

    RCLCPP_INFO(this->get_logger(), "Debug mode: %s", debug_mode_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "Port name from param: %s", port_name_.empty() ? "not specified" : port_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate_);

    // 初始化ROS发布者和订阅者
    pub_serial_data_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/read_serial_data", 10);
    sub_send_data_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/send_serial_data", 10,
        std::bind(&SerialServerNode::sendSerialDataCallback, this, std::placeholders::_1));

    // 连接串口
    connectSerial();

    RCLCPP_INFO(this->get_logger(), "serial_server_node is running");
}

// 析构函数
SerialServerNode::~SerialServerNode() {
    shutdownCallback();
}

// 关闭回调
void SerialServerNode::shutdownCallback() {
    is_running_ = false;

    if (reconnect_timer_) {
        reconnect_timer_->cancel();
    }

    if (read_thread_.joinable()) {
        read_thread_.join();
    }

    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_port_.isOpen()) {
        serial_port_.close();
        RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }
}

// 查找可用的串口
std::string SerialServerNode::findSerialPort() {
    static auto last_log_time = std::chrono::steady_clock::time_point{};
    auto now = std::chrono::steady_clock::now();
    bool should_log = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5;

    // 获取串口列表
    std::vector<serial::PortInfo> ports;
    try {
        ports = serial::list_ports();
    } catch (const std::exception& e) {
        if (should_log) {
            RCLCPP_ERROR(this->get_logger(), "列出端口时异常: %s", e.what());
            last_log_time = now;
        }
        return "";
    }

    if (!ports.empty() && should_log) {
        std::stringstream ss;
        ss << "找到 " << ports.size() << " 个串口设备:";
        for (const auto& port : ports) {
            ss << " " << port.port;
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        last_log_time = now;
    }

    if (ports.empty()) {
        return "";
    }

    // 首先尝试使用指定的端口
    if (!port_name_.empty()) {
        std::string full_port_path = "/dev/" + port_name_;
        for (const auto& port : ports) {
            if (port.port == full_port_path) {
                if (access(port.port.c_str(), R_OK | W_OK) == 0) {
                    if (should_log) {
                        RCLCPP_INFO(this->get_logger(), "使用launch指定的端口: %s", port.port.c_str());
                    }
                    return port.port;
                }
            }
        }
        if (should_log) {
            RCLCPP_WARN(this->get_logger(), "指定的端口 %s 不可用，将搜索其他ttyUSB设备", full_port_path.c_str());
        }
    }

    // 尝试所有ttyUSB设备
    for (const auto& port : ports) {
        if (port.port.find("ttyUSB") != std::string::npos) {
            if (access(port.port.c_str(), R_OK | W_OK) == 0) {
                if (should_log) {
                    RCLCPP_INFO(this->get_logger(), "Found available ttyUSB device: %s", port.port.c_str());
                }
                return port.port;
            }
        }
    }

    if (should_log) {
        RCLCPP_WARN(this->get_logger(), "未找到可用的ttyUSB设备");
    }
    return "";
}

// 连接串口
bool SerialServerNode::connectSerial() {
    try {
        std::string port = findSerialPort();

        if (port.empty()) {
            failure_counter_++;
            int wait_seconds = std::min(10, (failure_counter_ / 10) + 1);

            reconnect_timer_ = this->create_wall_timer(
                std::chrono::seconds(wait_seconds),
                [this]() {
                    reconnect_timer_->cancel();
                    connectSerial();
                });
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "正在连接端口: %s", port.c_str());

        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }

        serial_port_.setPort(port);
        serial_port_.setBaudrate(baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
        serial_port_.setTimeout(timeout);

        for (int attempt = 1; attempt <= 3; attempt++) {
            try {
                serial_port_.open();
                if (serial_port_.isOpen()) {
                    RCLCPP_INFO(this->get_logger(), "串口连接成功");
                    failure_counter_ = 0;

                    is_running_ = true;
                    if (read_thread_.joinable()) {
                        read_thread_.join();
                    }
                    read_thread_ = std::thread(&SerialServerNode::readFrameThread, this);
                    return true;
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "连接尝试 #%d 失败: %s", attempt, e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(800));
            }
        }

        failure_counter_++;
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                reconnect_timer_->cancel();
                connectSerial();
            });
        return false;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "连接串口异常: %s", e.what());
        failure_counter_++;
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                reconnect_timer_->cancel();
                connectSerial();
            });
        return false;
    }
}

// 发送数据回调
void SerialServerNode::sendSerialDataCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    std::vector<uint8_t> data(msg->data.begin(), msg->data.end());

    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (!serial_port_.isOpen()) {
            RCLCPP_WARN(this->get_logger(), "Serial port is not open, attempting to reconnect");
            return;
        }

        size_t bytes_written = serial_port_.write(data);
        if (bytes_written != data.size()) {
            RCLCPP_WARN(this->get_logger(), "Only wrote %zu of %zu bytes", bytes_written, data.size());
        }

        printHexFrame(data, 0);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while sending data: %s", e.what());
    }
}

// 读取数据线程
void SerialServerNode::readFrameThread() {
    std::vector<uint8_t> frame_buffer;
    bool wait_for_start = true;
    size_t total_bytes_read = 0;

    RCLCPP_INFO(this->get_logger(), "Read thread started");

    while (is_running_ && rclcpp::ok()) {
        try {
            if (!serial_port_.isOpen()) {
                RCLCPP_WARN(this->get_logger(), "串口未打开，等待重连");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            // 直接阻塞读取1字节（使用已设置的超时），而非轮询available()
            uint8_t byte = 0;
            size_t bytes_read = 0;

            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if (serial_port_.isOpen()) {
                    try {
                        bytes_read = serial_port_.read(&byte, 1);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "读取串口数据异常: %s", e.what());
                        serial_port_.close();
                        wait_for_start = true;
                        frame_buffer.clear();
                        continue;
                    }
                }
            }

            if (bytes_read == 0) {
                // 超时，无数据
                continue;
            }

            total_bytes_read++;
            if (total_bytes_read == 1) {
                RCLCPP_INFO(this->get_logger(), "First byte received from serial: 0x%02X", byte);
            }
            if (total_bytes_read % 10000 == 0) {
                RCLCPP_INFO(this->get_logger(), "Total bytes read from serial: %zu", total_bytes_read);
            }

            // 帧解析逻辑
            if (wait_for_start) {
                if (byte == 0xAA) {
                    frame_buffer.clear();
                    frame_buffer.push_back(byte);
                    wait_for_start = false;
                }
            } else {
                frame_buffer.push_back(byte);

                if (byte == 0xFF && frame_buffer.size() >= 3) {
                    uint8_t expected_length = frame_buffer[2] + 5;

                    if (frame_buffer.size() == expected_length) {
                        if (serialDataCheck(frame_buffer)) {
                            auto msg = std_msgs::msg::UInt8MultiArray();
                            std_msgs::msg::MultiArrayDimension dim;
                            dim.size = frame_buffer.size();
                            dim.stride = 1;
                            msg.layout.dim.push_back(dim);
                            msg.data = frame_buffer;

                            printHexFrame(frame_buffer, 1);
                            pub_serial_data_->publish(msg);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Frame checksum validation failed");
                        }
                        wait_for_start = true;
                    } else if (expected_length > 64 || frame_buffer.size() > 64) {
                        RCLCPP_WARN(this->get_logger(), "Frame too long, discarding");
                        wait_for_start = true;
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in read thread: %s", e.what());
            wait_for_start = true;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

// 检查数据的校验和
bool SerialServerNode::serialDataCheck(const std::vector<uint8_t>& data) {
    if (data.size() < 4) {
        return false;
    }

    uint8_t calculated_check = sumElements(data) % 2;
    uint8_t received_check = data[data.size() - 2];

    return (calculated_check == received_check);
}

// 计算数据的校验和
uint8_t SerialServerNode::sumElements(const std::vector<uint8_t>& data) {
    if (data.size() < 4) {
        RCLCPP_ERROR(this->get_logger(), "Data array too small for checksum calculation");
        return 0;
    }

    uint32_t sum = 0;
    for (size_t i = 3; i < data.size() - 2; ++i) {
        sum += data[i];
    }

    return sum % 2;
}

// 打印十六进制数据
void SerialServerNode::printHexFrame(const std::vector<uint8_t>& data, int type) {
    if (!debug_mode_) {
        return;
    }

    std::stringstream ss;
    if (type == 0) {
        ss << "发送数据: ";
    } else if (type == 1) {
        ss << "数据接收: ";
    } else {
        ss << "接收数据的一部分: ";
    }

    for (const auto& byte : data) {
        ss << std::uppercase << std::setfill('0') << std::setw(2)
           << std::hex << static_cast<int>(byte) << " ";
    }

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}