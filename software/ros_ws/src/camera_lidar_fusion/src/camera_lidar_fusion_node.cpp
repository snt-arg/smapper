#include <cstdio>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "camera_lidar_fusion/camera_lidar_fusion.hpp"
#include "camera_lidar_fusion/sensors_synchronizer.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executors;

    std::shared_ptr<CameraLidarFusion> node = std::make_shared<CameraLidarFusion>();

    std::vector<string> camera_names{"/camera/front_left",
                                     "/camera/front_right",
                                     "/camera/side_left",
                                     "/camera/side_right"};

    auto synchronizer =
        std::make_shared<SensorsSynchronizer>(0.05, "/ouster/points", camera_names);

    executors.add_node(node);
    executors.add_node(synchronizer);
    executors.spin();
    rclcpp::shutdown();
    return 0;
}
