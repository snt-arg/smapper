#ifndef CAMERA_LIDAR_FUSION
#define CAMERA_LIDAR_FUSION

#include <deque>
#include <memory>
#include <mutex>
#include <rclcpp/subscription.hpp>
#include <smapper_msgs/msg/synced_data.hpp>

#include "camera_lidar_fusion/camera_handler.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CameraLidarFusion : public rclcpp::Node {
   public:
    CameraLidarFusion();

   private:
    void tf_timer_callback_();
    bool get_tf_(std::shared_ptr<CameraHandler> camera_handler,
                 geometry_msgs::msg::TransformStamped &out_tf_msg);
    Eigen::Matrix4d transform_to_matrix_(
        const geometry_msgs::msg::TransformStamped &transform_msg);

    void publish_process_pcl_();

    void synced_callback_(const smapper_msgs::msg::SyncedData::ConstSharedPtr msg);

    void camera_info_callback_(int idx,
                               const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);
    void sort_pointcloud_by_distance(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    std::vector<int> check_point_within_image_(
        std::shared_ptr<CameraHandler> camera_handler,
        const Eigen::Vector4d point4d);

    bool depth_cloud_filtering_(
        const Eigen::Vector4d lidar_point,
        const int cloud_colored_index,
        const std::vector<int> cam_image_point,
        std::vector<std::vector<float>> &cam_depth_buffer,
        std::vector<std::vector<int>> &cam_point_index_buffer,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_colored);

    pcl::PointXYZRGB color_cloud_(std::shared_ptr<CameraHandler> camera_handler,
                                  const cv::Vec3b color);

    void publish_color_cloud(rclcpp::Time stamp,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored);

   private:
    std::vector<std::deque<std::pair<rclcpp::Time, cv_bridge::CvImagePtr>>>
        camera_buffers_;
    std::deque<std::pair<rclcpp::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr>>
        lidar_buffer_;
    std::mutex camera_buf_mutex_, lidar_buf_mutex_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>
        camera_info_subs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;

    std::vector<std::shared_ptr<CameraHandler>> camera_handlers_;

    // Timers
    rclcpp::TimerBase::SharedPtr tf_timer_, lidar_timer_;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Subs
    rclcpp::Subscription<smapper_msgs::msg::SyncedData>::SharedPtr synced_sub;

    std::string pcl_frame_id_;
};

#endif  // !CAMERA_LIDAR_FUSION
