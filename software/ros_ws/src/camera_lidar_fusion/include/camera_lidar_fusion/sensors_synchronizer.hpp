#ifndef SENSORS_SYNCHRONIZER_HPP
#define SENSORS_SYNCHRONIZER_HPP

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <smapper_msgs/msg/detail/synced_data__struct.hpp>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "smapper_msgs/msg/synced_data.hpp"

using namespace std;

class SensorsSynchronizer : public rclcpp::Node {
   public:
    SensorsSynchronizer(double age_penalty,
                        string pcl_topic_,
                        vector<string> camera_names);

   private:
    void init_subs();
    void init_pubs();
    void init_timers();

    void sync_callback();

    void camera_image_callback(size_t idx,
                               const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    void pcl_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    sensor_msgs::msg::PointCloud2::ConstSharedPtr find_matching_lidar(
        const double& stamp);

    bool find_synced_cameras(
        std::vector<sensor_msgs::msg::Image::ConstSharedPtr>& synced_images,
        double& synced_stamp);

    void cleanup_buffers(const double& stamp);

    void publish_synced_data(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
        const std::vector<sensor_msgs::msg::Image::ConstSharedPtr>& camera_msgs);

   private:
    double age_penalty_;
    double sync_tolerance_;
    rclcpp::CallbackGroup::SharedPtr camera_group_;
    rclcpp::CallbackGroup::SharedPtr pcl_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;

    // Cameras
    vector<string> camera_names_;
    vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> cam_image_subs_;
    vector<deque<sensor_msgs::msg::Image::ConstSharedPtr>> cam_msgs_buffer_;

    // PointCloud
    string pcl_topic_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> lidar_msgs_buffer_;

    rclcpp::Publisher<smapper_msgs::msg::SyncedData>::SharedPtr synced_pub_;

    rclcpp::TimerBase::SharedPtr sync_timer_;

    mutex buffer_mutex_;
};

#endif  // !SENSORS_SYNCHRONIZER_HPP
