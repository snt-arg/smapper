// #include <iostream>
// #include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "argus_camera_ros/camera_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<ArgusCameraNode> camera_node = std::make_shared<ArgusCameraNode>();

    rclcpp::spin(camera_node);

    std::cout << "Reached" << std::endl;

    rclcpp::shutdown();
    return 0;
}
