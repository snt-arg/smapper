#include <cstdio>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "camera_lidar_fusion/camera_lidar_fusion.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraLidarFusion>());
    rclcpp::shutdown();
    return 0;
}
