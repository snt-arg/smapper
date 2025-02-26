#include "argus_camera_ros/camera_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>

static const MultiCamera::Config DEFAULT_CONFIG{
    .device_ids = {0, 1, 2, 3},
    .sensor_mode = 2,
    .image_width = 640,
    .image_height = 480,
    .framerate = 20,
    .buffer_size = 4,
};

static const std::vector<std::string> DEFAULT_NAMES{"front_left",
                                                    "front_right",
                                                    "right",
                                                    "left"};

static const std::vector<std::string> DEFAULT_FRAME_IDS{"front_left_camera",
                                                        "front_right_camera",
                                                        "right_camera",
                                                        "left_camera"};

static const std::vector<std::string> DEFAULT_URLS{
    "package://argus_camera_ros/config/front_left_info.yaml",
    "package://argus_camera_ros/config/front_right_info.yaml",
    "package://argus_camera_ros/config/right_info.yaml",
    "package://argus_camera_ros/config/left_info.yaml"};

ArgusCameraNode::ArgusCameraNode() : Node("argus_camera_node"), cameras_config_() {
    declare_parameters_();
    read_parameters_();

    cameras_ = std::make_unique<MultiCamera>(cameras_config_);

    if (cameras_->init()) cameras_->start_capture();

    init_pubs_();
}

ArgusCameraNode::~ArgusCameraNode() {}

void ArgusCameraNode::init_pubs_() {
    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1 / cameras_config_.framerate),
        std::bind(&ArgusCameraNode::pub_callback_, this));

    for (int i = 0; i < camera_names_.size(); i++) {
        // INFO: We could perhaps change the queue size, or use a different QoS
        std::string topic = "/camera/" + camera_names_[i] + "/image_raw";
        img_pubs_.push_back(this->create_publisher<sensor_msgs::msg::Image>(topic, 10));

        camera_info_pubs_.push_back(
            this->create_publisher<sensor_msgs::msg::CameraInfo>(
                "/camera/" + camera_names_[i] + "/camera_info", 10));

        // camera_info_manager_ =
        // std::make_unique<camera_info_manager::CameraInfoManager>(this,
        // camera_names_[i], camera_urls_[i]);

        camera_info_.push_back(std::make_unique<camera_info_manager::CameraInfoManager>(
            this, camera_names_[i], camera_urls_[i]));
    }
}

void ArgusCameraNode::declare_parameters_() {
    this->declare_parameter("devices", DEFAULT_CONFIG.device_ids);
    this->declare_parameter("image_width", DEFAULT_CONFIG.image_width);
    this->declare_parameter("image_height", DEFAULT_CONFIG.image_height);
    this->declare_parameter("framerate", DEFAULT_CONFIG.framerate);
    this->declare_parameter("sensor_mode", DEFAULT_CONFIG.sensor_mode);
    this->declare_parameter("buffer_size", DEFAULT_CONFIG.buffer_size);

    this->declare_parameter("camera_names", DEFAULT_NAMES);
    this->declare_parameter("frame_ids", DEFAULT_FRAME_IDS);
    this->declare_parameter("camera_urls", DEFAULT_URLS);
}

void ArgusCameraNode::read_parameters_() {
    this->get_parameter("devices", cameras_config_.device_ids);
    this->get_parameter("image_width", cameras_config_.image_width);
    this->get_parameter("image_height", cameras_config_.image_height);
    this->get_parameter("framerate", cameras_config_.framerate);
    this->get_parameter("sensor_mode", cameras_config_.sensor_mode);
    this->get_parameter("buffer_size", cameras_config_.buffer_size);

    this->get_parameter("camera_names", camera_names_);
    this->get_parameter("frame_ids", frame_ids_);
    this->get_parameter("camera_urls", camera_urls_);
}

void ArgusCameraNode::pub_callback_() {
    std::vector<cv::Mat> frames;
    if (!cameras_->get_frames(frames)) {
        RCLCPP_WARN(this->get_logger(), "No Frames to publish");
        return;
    }
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();

    for (int i = 0; i < frames.size(); i++) {
        header.frame_id = frame_ids_[i];
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(header, "bgr8", frames[i]).toImageMsg();
        img_pubs_[i]->publish(*msg);

        sensor_msgs::msg::CameraInfo camera_info = camera_info_[i]->getCameraInfo();
        camera_info.header = header;
        camera_info_pubs_[i]->publish(camera_info);
    }
}
