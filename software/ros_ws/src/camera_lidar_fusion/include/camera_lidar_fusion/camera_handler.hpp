#ifndef CAMERA_HANDLER_HPP
#define CAMERA_HANDLER_HPP

#include <Eigen/Eigen>

class CameraHandler {
   public:
    CameraHandler();
    ~CameraHandler();

   private:
    int image_width, image_height;
    Eigen::Matrix<double, 1, 5> distortion_matrix;
    Eigen::Matrix<double, 3, 4> projection_matrix;
    Eigen::Matrix<double, 3, 4> lidar_to_camera_projection_matrix;

    Eigen::Matrix4d lidar_to_camera_matrix, lidar_to_camera_inv_matrix;

    bool is_transform_initialized = false;
    std::string camera_frame_id;
    std::string image_encoding;
};

#endif  // CAMERA_HANDLER_HPP
