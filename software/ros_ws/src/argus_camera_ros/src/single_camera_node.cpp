#include <unistd.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "argus_camera_ros/multi_camera.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Hello from single camera node");

    MultiCamera::Config config{
        .device_ids = {0, 1, 2, 3},
        .image_width = 640,
        .image_height = 480,
        .framerate = 20,
        .buffer_size = 8,
    };

    MultiCamera cameras(config);
    if (cameras.init()) cameras.start_capture();

    cameras.wait();
    cameras.stop_capture();

    rclcpp::shutdown();
    return 0;
}
