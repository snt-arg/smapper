import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

PKG_DIR = get_package_share_directory("smapper_bringup")


def launch_realsense(ld: LaunchDescription) -> None:
    config_file = os.path.join(PKG_DIR, "config", "realsense", "config.yaml")
    preset_file = os.path.join(PKG_DIR, "config", "realsense", "high_accuracy_preset.json")
    realsense_pkg = get_package_share_directory("realsense2_camera")

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_pkg, "launch", "rs_launch.py")
            ),
            launch_arguments={
                "config_file": config_file,
                "json_file_path": preset_file,
                "camera_name": "realsense",
                "rgb_camera.color_profile": "640x480x30"
            }.items(),
        )
    )

def publish_tfs(ld: LaunchDescription) -> None:
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_imu_to_camera",
            arguments=[
                "--x", "0.0772",
                "--y", "0.0175",
                "--z", "0.030",
                "--qx", "0.0",
                "--qy", "0.0",
                "--qz", "0.0",
                "--qw", "1.0",
                "--frame-id", "os_imu",
                "--child-frame-id", "front_right",
            ],
        )
    )

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    launch_realsense(ld)

    return ld

