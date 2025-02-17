#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Hello from single camera node");
    rclcpp::shutdown();
    return 0;
}
