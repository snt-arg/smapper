cmake_minimum_required(VERSION 3.8)
project(argus_camera_ros)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)


# --------------------------------------
# - Start of Jetson Specific Variables -
# --------------------------------------

SET(TEGRA_ARMABI aarch64-linux-gnu)
SET(JETSON_TEGRA_DIR /usr/lib/${TEGRA_ARMABI})
SET(JETSON_API_DIR /usr/src/jetson_multimedia_api/)
SET(JETSON_CLASS_DIR ${JETSON_API_DIR}/samples/common/classes)
SET(JETSON_UTILS_DIR ${JETSON_API_DIR}/argus/samples/utils)
SET(JETSON_ALGO_CUDA_DIR ${JETSON_API_DIR}/samples/common/algorithm/cuda)
SET(JETSON_ALGO_TRT_DIR ${JETSON_API_DIR}/samples/common/algorithm/trt)

SET(JETSON_HEADERS 
    ${JETSON_API_DIR}/include
    ${JETSON_API_DIR}/include/libjpeg-8b
    /usr/include/${TEGRA_ARMABI}
    /usr/include/libdrm/
    # /usr/include/opencv4/
    /usr/local/cuda/include
    ${JETSON_ALGO_TRT_DIR}
    ${JETSON_ALGO_CUDA_DIR}
    ${JETSON_CLASSS_DIR}
    ${JETSON_UTILS_DIR}
)

SET(JETSON_LIBS_DIR 
  /usr/lib/${TEGRA_ARMABI}/
  /usr/lib/${TEGRA_ARMABI}/nvidia
  /usr/lib/${TEGRA_ARMABI}/tegra/
  /usr/local/cuda/lib64/
)



SET(JETSON_LIBS
  -lnvargus -largussampleutils -lnvbufsurface -lnvargus_socketclient 
  -lpthread -lnvv4l2 -lEGL -lGLESv2 -lX11 -lnvbufsurftransform -lnvjpeg 
  -lnvosd -ldrm -lcuda -lcudart
)

file(GLOB JETSON_SOURCES CONFIGURE_DEPENDS "${JETSON_CLASS_DIR}/*.cpp")


# --------------------------------------
# - End of Jetson Specific Variables -
# --------------------------------------

SET(DEPENDENCIES rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros OpenCV cv_bridge)
SET(LIBS ${JETSON_LIBS})
SET(SOURCES ${JETSON_SOURCES})

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${JETSON_HEADERS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${JETSON_LIBS_DIR}
)


# Executables
add_executable(argus_camera_node src/argus_camera_node.cpp ${SOURCES})
ament_target_dependencies(argus_camera_node ${DEPENDENCIES})
target_link_libraries(argus_camera_node ${LIBS})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS 
  argus_camera_node
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
