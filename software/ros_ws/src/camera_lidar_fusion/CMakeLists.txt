cmake_minimum_required(VERSION 3.8)
project(camera_lidar_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

SET(DEPENDENCIES 
    rclcpp 
    sensor_msgs
    geometry_msgs 
    cv_bridge 
    tf2_eigen 
    tf2_ros 
    pcl_ros 
    pcl_conversions 
    message_filters
)
SET(LIBS ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})
SET(SOURCES src/camera_lidar_fusion/camera_lidar_fusion.cpp src/camera_lidar_fusion/camera_handler.cpp)

add_executable(camera_lidar_fusion_node src/camera_lidar_fusion_node.cpp ${SOURCES})
ament_target_dependencies(camera_lidar_fusion_node ${DEPENDENCIES})
target_link_libraries(camera_lidar_fusion_node ${LIBS})

install(TARGETS camera_lidar_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
