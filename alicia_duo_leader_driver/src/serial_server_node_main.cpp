#include "serial_server_node/serial_server_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}