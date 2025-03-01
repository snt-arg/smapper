#include "camera_lidar_fusion/camera_handler.hpp"

CameraHandler::CameraHandler()
    : image_width_(0), image_height_(0), camera_frame_id_(""), image_encoding_("") {}
CameraHandler::~CameraHandler() {}

Eigen::Vector2d CameraHandler::project_lidar_to_camera(Eigen::Vector3d point_3d) {
    const auto pt_2d = (point_3d.template head<2>() / point_3d.z()).eval();
    const auto pt_d = pinhole_distort(pt_2d);

    const auto& fx = camera_matrix_(0, 0);
    const auto& fy = camera_matrix_(1, 1);
    const auto& cx = camera_matrix_(0, 2);
    const auto& cy = camera_matrix_(1, 2);

    return {fx * pt_d(0) + cx, fy * pt_d(1) + cy};
}

Eigen::Vector2d CameraHandler::pinhole_distort(const Eigen::Vector2d& pt) {
    const auto& k1 = distortion_matrix_(0, 0);
    const auto& k2 = distortion_matrix_(0, 1);
    const auto& k3 = distortion_matrix_(0, 3);

    const auto& p1 = distortion_matrix_(0, 4);
    const auto& p2 = distortion_matrix_(0, 2);

    const auto x2 = pt.x() * pt.x();
    const auto y2 = pt.y() * pt.y();

    const auto r2 = x2 + y2;
    const auto r4 = r2 * r2;
    const auto r6 = r2 * r4;

    const auto r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const auto t_coeff1 = 2.0 * pt.x() * pt.y();
    const auto t_coeff2 = r2 + 2.0 * x2;
    const auto t_coeff3 = r2 + 2.0 * y2;

    const auto x = r_coeff * pt.x() + p1 * t_coeff1 + p2 * t_coeff2;
    const auto y = r_coeff * pt.y() + p1 * t_coeff3 + p2 * t_coeff1;

    return Eigen::Vector2d(x, y);
}

bool CameraHandler::transform_initialized() { return is_transform_initialized_; }

bool CameraHandler::got_camera_info() { return camera_info_received_; }

bool CameraHandler::got_image_encoding() { return image_encoding_ != ""; }

// Setters

void CameraHandler::set_camera_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    image_height_ = msg->height;
    image_width_ = msg->width;

    camera_matrix_(0, 0) = msg->k[0];
    camera_matrix_(0, 2) = msg->k[2];
    camera_matrix_(1, 1) = msg->k[4];
    camera_matrix_(1, 2) = msg->k[5];
    camera_matrix_(2, 2) = 1;

    distortion_matrix_(0, 0) = msg->d[0];
    distortion_matrix_(0, 1) = msg->d[1];
    distortion_matrix_(0, 2) = msg->d[3];
    distortion_matrix_(0, 3) = msg->d[4];
    distortion_matrix_(0, 4) = msg->d[2];

    projection_matrix_(0, 0) = msg->p[0];
    projection_matrix_(0, 2) = msg->p[2];
    projection_matrix_(1, 1) = msg->p[5];
    projection_matrix_(1, 2) = msg->p[6];
    projection_matrix_(2, 2) = 1;

    camera_info_received_ = true;

    camera_frame_id_ = msg->header.frame_id;
}

void CameraHandler::set_lidar_to_camera_matrix(Eigen::Matrix4d T_lidar_cam) {
    lidar_to_camera_matrix_ = Eigen::Matrix4d::Identity();
    lidar_to_camera_matrix_ = T_lidar_cam;
    is_transform_initialized_ = true;
}

void CameraHandler::set_lidar_to_camera_inv_matrix(Eigen::Matrix4d T_lidar_cam_inv) {
    lidar_to_camera_inv_matrix_ = Eigen::Matrix4d::Identity();
    lidar_to_camera_inv_matrix_ = T_lidar_cam_inv;
}
void CameraHandler::set_lidar_to_camera_projection_matrix() {
    lidar_to_camera_projection_matrix_ =
        projection_matrix_ * lidar_to_camera_matrix_.block<4, 4>(0, 0);
}

void CameraHandler::set_camera_frame_id(std::string frame_id) {
    camera_frame_id_ = frame_id;
}
void CameraHandler::set_image_encoding(std::string encoding) {
    image_encoding_ = encoding;
}

// Getters

Eigen::Matrix<double, 3, 4> CameraHandler::get_lidar_to_camera_projection_matrix() {
    return lidar_to_camera_projection_matrix_;
}
Eigen::Matrix4d CameraHandler::get_lidar_to_camera_inv_matrix() {
    return lidar_to_camera_inv_matrix_;
}

double CameraHandler::get_image_height() { return image_height_; }
double CameraHandler::get_image_width() { return image_width_; }

std::string CameraHandler::get_camera_frame_id() { return camera_frame_id_; }

std::string CameraHandler::get_image_encoding() { return image_encoding_; }
