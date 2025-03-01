#ifndef CAMERA_HANDLER_HPP
#define CAMERA_HANDLER_HPP

#include <Eigen/Eigen>
#include <sensor_msgs/msg/camera_info.hpp>

class CameraHandler {
   public:
    CameraHandler();
    ~CameraHandler();

    Eigen::Vector2d project_lidar_to_camera(Eigen::Vector3d point_3d);
    Eigen::Vector2d pinhole_distort(const Eigen::Vector2d& pt);
    bool transform_initialized();
    bool got_camera_info();
    bool got_image_encoding();

    // Setters
   public:
    void set_camera_frame_id(std::string frame_id);
    void set_image_encoding(std::string encoding);
    void set_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void set_lidar_to_camera_matrix(Eigen::Matrix4d T_lidar_cam);
    void set_lidar_to_camera_inv_matrix(Eigen::Matrix4d T_lidar_cam_inv);
    void set_lidar_to_camera_projection_matrix();

    // Getters
   public:
    double get_image_height();
    double get_image_width();
    std::string get_camera_frame_id();
    std::string get_image_encoding();
    Eigen::Matrix4d get_lidar_to_camera_matrix() { return lidar_to_camera_matrix_; }
    Eigen::Matrix4d get_lidar_to_camera_inv_matrix();
    Eigen::Matrix<double, 3, 4> get_lidar_to_camera_projection_matrix();

   private:
    int image_width_, image_height_;
    Eigen::Matrix3d camera_matrix_;
    Eigen::Matrix<double, 1, 5> distortion_matrix_;
    Eigen::Matrix<double, 3, 4> projection_matrix_;
    Eigen::Matrix<double, 3, 4> lidar_to_camera_projection_matrix_;

    Eigen::Matrix4d lidar_to_camera_matrix_, lidar_to_camera_inv_matrix_;

    bool is_transform_initialized_ = false;
    bool camera_info_received_ = false;
    std::string camera_frame_id_;
    std::string image_encoding_;
};

#endif  // CAMERA_HANDLER_HPP
