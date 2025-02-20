#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "argus_camera_ros/multi_camera.hpp"

class ArgusCameraNode : public rclcpp::Node {
   public:
    ArgusCameraNode();
    ~ArgusCameraNode();

   private:
    void pub_callback();
    std::unique_ptr<MultiCamera> cameras;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // !CAMERA_NODE_HPP
