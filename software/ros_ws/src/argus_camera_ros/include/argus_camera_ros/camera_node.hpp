#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "argus_camera_ros/multi_camera.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

class ArgusCameraNode : public rclcpp::Node {
   public:
    ArgusCameraNode();
    ~ArgusCameraNode();

   private:
    void pub_callback_();
    void init_pubs_();
    void declare_parameters_();
    void read_parameters_();
    void build_camera_msg_(std::vector<sensor_msgs::msg::Image::SharedPtr> &msgs,
                           std::vector<cv::Mat> &camera_frames);

    std::vector<std::string> camera_names_;
    std::vector<std::string> frame_ids_;
    std::vector<std::string> camera_urls_;
    MultiCamera::Config cameras_config_;

    std::unique_ptr<MultiCamera> cameras_;

    std::vector<std::unique_ptr<camera_info_manager::CameraInfoManager>> camera_info_;

    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> img_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>
        camera_info_pubs_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};

#endif  // !CAMERA_NODE_HPP
