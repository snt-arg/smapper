#include "argus_camera_ros/camera_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include "argus_camera_ros/multi_camera.hpp"

ArgusCameraNode::ArgusCameraNode() : Node("argus_camera_node") {
    RCLCPP_INFO(this->get_logger(), "Hello From CameraNode");

    MultiCamera::Config config{
        .device_ids = {0, 1, 2, 3},
        .image_width = 640,
        .image_height = 480,
        .framerate = 20,
        .buffer_size = 8,
    };

    cameras = std::make_unique<MultiCamera>(config);

    if (cameras->init()) cameras->start_capture();

    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1 / config.framerate),
                                     std::bind(&ArgusCameraNode::pub_callback, this));
}

ArgusCameraNode::~ArgusCameraNode() {}

void ArgusCameraNode::pub_callback() {
    std::vector<cv::Mat> frames;
    cameras->get_frames(frames);

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();

    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(header, "bgr8", frames[0]).toImageMsg();

    img_pub_->publish(*msg);
}
