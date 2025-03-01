#include "camera_lidar_fusion/camera_lidar_fusion.hpp"

#include <chrono>
#include <functional>
#include <rclcpp/logging.hpp>

#include "pcl_conversions/pcl_conversions.h"

using std::placeholders::_1;
using std::placeholders::_2;

CameraLidarFusion::CameraLidarFusion()
    : Node("camera_lidar_fusion"), pcl_frame_id_("os_lidar") {
    rclcpp::QoS qos = rclcpp::QoS(100);

    camera_handler_ = std::make_shared<CameraHandler>();

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/front_right/camera_info",
        qos,
        std::bind(&CameraLidarFusion::camera_info_callback_, this, _1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    camera_sub_.subscribe(
        this, "/camera/front_right/image_raw", qos.get_rmw_qos_profile());
    lidar_sub_.subscribe(this, "/ouster/points", qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                        sensor_msgs::msg::Image>>>(
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                        sensor_msgs::msg::Image>(100),
        lidar_sub_,
        camera_sub_);

    sync_->setAgePenalty(0.00005);
    sync_->registerCallback(
        std::bind(&CameraLidarFusion::synced_callback_, this, _1, _2));

    lidar_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud/colored", qos);

    tf_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&CameraLidarFusion::tf_timer_callback_, this));
    lidar_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1 / 10),
        std::bind(&CameraLidarFusion::publish_process_pcl_, this));

    RCLCPP_INFO(this->get_logger(), "camera_lidar_fusion has been initilized");
}

void CameraLidarFusion::tf_timer_callback_() {
    geometry_msgs::msg::TransformStamped lidar_camera_tf_msg;
    bool got_camera_tf = get_tf_(lidar_camera_tf_msg);

    if (!camera_handler_->transform_initialized() && got_camera_tf) {
        RCLCPP_INFO(this->get_logger(),
                    "Trying to obtain transform between camera and lidar");
        Eigen::Matrix4d T_lidar_camera = transform_to_matrix_(lidar_camera_tf_msg);
        camera_handler_->set_lidar_to_camera_matrix(T_lidar_camera);
        camera_handler_->set_lidar_to_camera_inv_matrix(T_lidar_camera.inverse());
        camera_handler_->set_lidar_to_camera_projection_matrix();
        RCLCPP_INFO(this->get_logger(), "Initiating camera info");
    }
}

void CameraLidarFusion::camera_info_callback_(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
    if (!camera_handler_->got_camera_info()) {
        RCLCPP_INFO(this->get_logger(), "Received Camera Info Message.");
        camera_handler_->set_camera_info(msg);
    }
}

bool CameraLidarFusion::get_tf_(geometry_msgs::msg::TransformStamped &out_tf_msg) {
    try {
        if (!camera_handler_->transform_initialized()) {
            out_tf_msg =
                tf_buffer_->lookupTransform(camera_handler_->get_camera_frame_id(),
                                            pcl_frame_id_,
                                            tf2::TimePointZero);
            return true;
        }
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Could not transform %s to %s: %s",
                    pcl_frame_id_.c_str(),
                    camera_handler_->get_camera_frame_id().c_str(),
                    ex.what());
        return false;
    }

    return false;
}

Eigen::Matrix4d CameraLidarFusion::transform_to_matrix_(
    const geometry_msgs::msg::TransformStamped &transform_msg) {
    Eigen::Vector3d translation(transform_msg.transform.translation.x,
                                transform_msg.transform.translation.y,
                                transform_msg.transform.translation.z);

    Eigen::Quaterniond rotation(transform_msg.transform.rotation.w,
                                transform_msg.transform.rotation.x,
                                transform_msg.transform.rotation.y,
                                transform_msg.transform.rotation.z);

    Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();

    transform_matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    transform_matrix.block<3, 1>(0, 3) = translation;

    return transform_matrix;
}

void CameraLidarFusion::synced_callback_(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &image_msg) {
    // PCL

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pcl_msg, *cloud);

    lidar_buffer_.emplace_back(pcl_msg->header.stamp, cloud);

    // Camera

    if (!camera_handler_->got_image_encoding()) {
        camera_handler_->set_image_encoding(image_msg->encoding);
    }

    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(image_msg, camera_handler_->get_image_encoding());
    camera_buffer_.emplace_back(image_msg->header.stamp, cv_ptr);
}

void CameraLidarFusion::publish_process_pcl_() {
    if (lidar_buffer_.empty() || camera_buffer_.empty()) {
        // RCLCPP_INFO(this->get_logger(), "No Pointcloud/Image to be processed");
        return;
    }

    if (lidar_buffer_.size() != camera_buffer_.size()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Lidar and Camera buffer do not match in size");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Pointcloud/Image to be processed");

    auto cloud = lidar_buffer_.front().second;
    auto cloud_stamp = lidar_buffer_.front().first;
    auto cv_image = camera_buffer_.front().second->image;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
        new pcl::PointCloud<pcl::PointXYZRGB>());

    // create depth and pointindex buffer
    std::vector<std::vector<float>> depth_buffer(
        cv_image.rows,
        std::vector<float>(cv_image.cols, std::numeric_limits<float>::max()));

    std::vector<std::vector<int>> point_index_buffer(
        cv_image.rows, std::vector<int>(cv_image.cols, -1));

    sort_pointcloud_by_distance(cloud);

    int cloud_colored_index = 0;
    // iterate through the points and color them
    for (auto &point : cloud->points) {
        if (!camera_handler_->transform_initialized()) break;

        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) continue;

        Eigen::Vector4d point4d(point.x, point.y, point.z, 1);
        std::vector<int> image_point = check_point_within_image_(point4d);

        pcl::PointXYZRGB point_colored;
        if (image_point.empty()) {
            point_colored.r = point_colored.g = point_colored.b = 0;
            // RCLCPP_INFO(this->get_logger(), "Could not get any Image Point");
        } else if (!image_point.empty() && !cv_image.empty()) {
            cv::Vec3b color =
                cv_image.at<cv::Vec3b>(cv::Point(image_point[0], image_point[1]));
            bool is_point_occluded = depth_cloud_filtering_(point4d,
                                                            cloud_colored_index,
                                                            image_point,
                                                            depth_buffer,
                                                            point_index_buffer,
                                                            cloud_colored);
            if (!is_point_occluded)
                point_colored = color_cloud_(color);
            else
                point_colored.r = point_colored.g = point_colored.b = 0;
        }
        point_colored.x = point.x;
        point_colored.y = point.y;
        point_colored.z = point.z;
        cloud_colored->points.push_back(point_colored);
        cloud_colored_index = cloud_colored->points.size();
    }

    publish_color_cloud(cloud_stamp, cloud_colored);

    lidar_buffer_.pop_front();
    camera_buffer_.pop_front();
}

void CameraLidarFusion::sort_pointcloud_by_distance(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    std::vector<std::pair<float, pcl::PointXYZI>> distance_points;

    // Calculate the distance of each point from the origin
    for (const auto &point : cloud->points) {
        float distance =
            std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        distance_points.push_back(std::make_pair(distance, point));
    }

    // Sort the vector based on the distance
    std::sort(
        distance_points.begin(),
        distance_points.end(),
        [](const std::pair<float, pcl::PointXYZI> &a,
           const std::pair<float, pcl::PointXYZI> &b) { return a.first < b.first; });

    // Rearrange the cloud points according to the sorted distances
    for (size_t i = 0; i < distance_points.size(); ++i) {
        cloud->points[i] = distance_points[i].second;
    }
}

std::vector<int> CameraLidarFusion::check_point_within_image_(
    const Eigen::Vector4d point4d) {
    std::vector<int> image_point;
    Eigen::Vector3d point3d_transformed_camera =
        camera_handler_->get_lidar_to_camera_projection_matrix() * point4d;

    Eigen::Vector2d point2d_transformed_camera =
        Eigen::Vector2d(point3d_transformed_camera[0] / point3d_transformed_camera[2],
                        point3d_transformed_camera[1] / point3d_transformed_camera[2]);

    int x = static_cast<int>(std::round(point2d_transformed_camera[0]));
    int y = static_cast<int>(std::round(point2d_transformed_camera[1]));

    if (x < 0 || x >= camera_handler_->get_image_width() || y < 0 ||
        y >= camera_handler_->get_image_height() || point3d_transformed_camera[2] < 0) {
        return image_point;
    } else {
        image_point.push_back(x);
        image_point.push_back(y);
        return image_point;
    }
}

bool CameraLidarFusion::depth_cloud_filtering_(
    const Eigen::Vector4d lidar_point,
    const int cloud_colored_index,
    const std::vector<int> cam_image_point,
    std::vector<std::vector<float>> &cam_depth_buffer,
    std::vector<std::vector<int>> &cam_point_index_buffer,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_colored) {
    bool is_occluded = false;

    double point_dist = lidar_point.norm();

    int x = cam_image_point[0];
    int y = cam_image_point[1];

    if (point_dist < cam_depth_buffer[y][x]) {
        if (cam_point_index_buffer[y][x] != -1) {
            int old_index = cam_point_index_buffer[y][x];
            cloud_colored->points[old_index].r = cloud_colored->points[old_index].g =
                cloud_colored->points[old_index].b = 0;
        }
        cam_depth_buffer[y][x] = point_dist;
        cam_point_index_buffer[y][x] = cloud_colored_index;
    } else {
        is_occluded = true;
    }

    return is_occluded;
}

pcl::PointXYZRGB CameraLidarFusion::color_cloud_(const cv::Vec3b color) {
    pcl::PointXYZRGB point_colored;
    if (camera_handler_->get_image_encoding() == "rgb8") {
        point_colored.r = color[0];
        point_colored.g = color[1];
        point_colored.b = color[2];
    } else if (camera_handler_->get_image_encoding() == "bgr8") {
        point_colored.r = color[2];
        point_colored.g = color[1];
        point_colored.b = color[0];
    } else {
        point_colored.r = color[2];
        point_colored.g = color[1];
        point_colored.b = color[0];
    }
    return point_colored;
}

void CameraLidarFusion::publish_color_cloud(
    rclcpp::Time stamp,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored) {
    sensor_msgs::msg::PointCloud2 color_cloud_msg;
    pcl::toROSMsg(*cloud_colored, color_cloud_msg);
    color_cloud_msg.header.stamp = stamp;
    color_cloud_msg.header.frame_id = pcl_frame_id_;
    lidar_pub_->publish(color_cloud_msg);
}
