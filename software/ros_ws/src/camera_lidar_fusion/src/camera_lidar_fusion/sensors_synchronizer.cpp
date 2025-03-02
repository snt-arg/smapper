#include "camera_lidar_fusion/sensors_synchronizer.hpp"

#include <chrono>
#include <complex>
#include <functional>
#include <iterator>
#include <mutex>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <smapper_msgs/msg/detail/synced_data__struct.hpp>

// TODO: change constuctor to instead read from parameters
SensorsSynchronizer::SensorsSynchronizer(double age_penalty,
                                         string pcl_topic,
                                         vector<string> camera_names)
    : Node("synchronizer"),
      age_penalty_(age_penalty),
      sync_tolerance_(0.05),
      camera_names_(camera_names),
      cam_msgs_buffer_(camera_names.size()),
      pcl_topic_(pcl_topic) {
    pcl_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    camera_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    init_subs();
    init_pubs();
    init_timers();
}

void SensorsSynchronizer::init_subs() {
    rclcpp::QoS qos = rclcpp::QoS(100);

    auto pcl_options = rclcpp::SubscriptionOptions();
    pcl_options.callback_group = pcl_group_;

    auto camera_options = rclcpp::SubscriptionOptions();
    camera_options.callback_group = camera_group_;

    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ouster/points",
        qos,
        std::bind(&SensorsSynchronizer::pcl_callback, this, placeholders::_1),
        pcl_options);

    for (size_t i = 0; i < camera_names_.size(); i++) {
        string image_topic_name = camera_names_[i] + "/image_raw";
        auto callback = [this, i](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            this->camera_image_callback(i, msg);
        };

        // Create the subscription
        auto sub = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_name,  // e.g., "/camera0/image", "/camera1/image"
            qos,
            callback,
            camera_options);

        cam_image_subs_.push_back(sub);
    }
}

void SensorsSynchronizer::init_pubs() {
    rclcpp::QoS qos = rclcpp::QoS(100);
    auto pcl_options = rclcpp::PublisherOptions();

    // TODO: Make use of parameters
    synced_pub_ = this->create_publisher<smapper_msgs::msg::SyncedData>(
        "synced/data", qos, pcl_options);
}

void SensorsSynchronizer::init_timers() {
    // TODO: change hz to parameter
    sync_timer_ =
        this->create_wall_timer(chrono::milliseconds(1 / 10),
                                std::bind(&SensorsSynchronizer::sync_callback, this));
}

void SensorsSynchronizer::pcl_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    lidar_msgs_buffer_.push_back(msg);
    RCLCPP_INFO(this->get_logger(), "Received PCL Cloud");
}

void SensorsSynchronizer::camera_image_callback(
    size_t idx,
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    string image_topic_name = camera_names_[idx] + "/image_raw";

    if (idx <= 0 && idx >= cam_msgs_buffer_.size()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Received invalid index [%zu] on camera_image_callback. Valid "
                     "range is 0-%zu",
                     idx,
                     cam_msgs_buffer_.size());
        return;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    cam_msgs_buffer_[idx].push_back(msg);
}

void SensorsSynchronizer::sync_callback() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // Step 1: Find synchronized camera images
    std::vector<sensor_msgs::msg::Image::ConstSharedPtr> synced_images;
    double synced_stamp;

    if (!find_synced_cameras(synced_images, synced_stamp)) {
        RCLCPP_ERROR(this->get_logger(), "Could not find synced cameras");
        return;
    }

    // Step 2: Find matching lidar message
    auto lidar_match = find_matching_lidar(synced_stamp);
    if (!lidar_match) {
        RCLCPP_ERROR(this->get_logger(), "Could not find mathching synced lidar");
        return;
    }

    // Step 3: Publish synchronized data
    publish_synced_data(lidar_match, synced_images);

    // Step 4: Cleanup old messages
    cleanup_buffers(synced_stamp);
}

bool SensorsSynchronizer::find_synced_cameras(
    std::vector<sensor_msgs::msg::Image::ConstSharedPtr>& synced_images,
    double& synced_stamp) {
    // Implementation for camera synchronization
    synced_images.clear();

    // Check all cameras have data
    for (const auto& buffer : cam_msgs_buffer_) {
        if (buffer.empty()) return false;
    }

    // Find common time window
    while (true) {
        // Get front elements
        std::vector<rclcpp::Time> stamps;
        for (const auto& buffer : cam_msgs_buffer_) {
            stamps.push_back(buffer.front()->header.stamp);
        }

        // Find time bounds
        auto minmax = std::minmax_element(stamps.begin(), stamps.end());
        double diff = (*minmax.second - *minmax.first).seconds();

        if (diff <= sync_tolerance_) {
            // Calculate average stamp
            synced_stamp =
                (minmax.first->nanoseconds() + minmax.second->nanoseconds()) * 0.5;
            for (size_t i = 0; i < cam_msgs_buffer_.size(); ++i) {
                synced_images.push_back(cam_msgs_buffer_[i].front());
            }
            return true;
        } else {
            // Remove oldest message from earliest camera
            auto earliest = std::min_element(stamps.begin(), stamps.end());
            size_t idx = std::distance(stamps.begin(), earliest);
            cam_msgs_buffer_[idx].pop_front();

            // Check if any buffer became empty
            if (cam_msgs_buffer_[idx].empty()) return false;
        }
    }
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr SensorsSynchronizer::find_matching_lidar(
    const double& stamp) {
    // Find best lidar match
    double best_diff = age_penalty_;
    auto best_match = lidar_msgs_buffer_.end();

    for (auto it = lidar_msgs_buffer_.begin(); it != lidar_msgs_buffer_.end(); ++it) {
        double diff = fabs(((*it)->header.stamp.nanosec - stamp)) / 1e9;
        if (diff < best_diff) {
            best_diff = diff;
            best_match = it;
        }
    }

    if (best_match != lidar_msgs_buffer_.end()) {
        return *best_match;
    }
    return nullptr;
}

void SensorsSynchronizer::cleanup_buffers(const double& stamp) {
    // Cleanup camera buffers
    for (auto& buffer : cam_msgs_buffer_) {
        while (!buffer.empty() &&
               (stamp - buffer.front()->header.stamp.nanosec) > age_penalty_) {
            buffer.pop_front();
        }
    }

    // Cleanup lidar buffer
    while (!lidar_msgs_buffer_.empty() &&
           (stamp - lidar_msgs_buffer_.front()->header.stamp.nanosec) > age_penalty_) {
        lidar_msgs_buffer_.pop_front();
    }
}

void SensorsSynchronizer::publish_synced_data(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
    const std::vector<sensor_msgs::msg::Image::ConstSharedPtr>& camera_msgs) {
    // Create the custom message
    auto synced_msg = smapper_msgs::msg::SyncedData();

    // Copy LiDAR data
    synced_msg.cloud = *lidar_msg;

    // Copy camera data
    synced_msg.images.resize(camera_msgs.size());
    for (size_t i = 0; i < camera_msgs.size(); ++i) {
        synced_msg.images[i] = *camera_msgs[i];
    }

    // Publish the message
    synced_pub_->publish(synced_msg);
}
