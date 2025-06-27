import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

PKG_DIR = get_package_share_directory("smapper_bringup")


def launch_ouster_lidar(ld: LaunchDescription) -> None:
    ouster_pkg_dir = get_package_share_directory("ouster_ros")
    params_file = os.path.join(PKG_DIR, "config", "ouster.yaml")

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ouster_pkg_dir, "launch", "driver.launch.py")
            ),
            launch_arguments={
                "params_file": params_file,
                "viz": "false",
            }.items(),
        )
    )


def publish_tfs(ld: LaunchDescription) -> None:
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_base_os",
            arguments=[
                "--x",
                "0.003",
                "--y",
                "-0.0",
                "--z",
                "0.112",
                "--qx",
                "0.0",
                "--qy",
                "0.0",
                "--qz",
                "0.0",
                "--qw",
                "1.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "os_sensor",
            ],
        )
    )


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    launch_ouster_lidar(ld)
    publish_tfs(ld)

    return ld
