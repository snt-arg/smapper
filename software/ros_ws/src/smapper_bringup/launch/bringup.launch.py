import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

PKG_DIR = get_package_share_directory("smapper_bringup")


def launch_argus_cameras(ld: LaunchDescription) -> None:
    params_file = os.path.join(PKG_DIR, "config", "argus_cameras.yaml")
    argus_camera_ros_pkg = get_package_share_directory("argus_camera_ros")

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(argus_camera_ros_pkg, "launch", "argus_camera.launch.py")
            ),
            launch_arguments={
                "params_file": params_file,
            }.items(),
        )
    )


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


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    launch_argus_cameras(ld)
    launch_ouster_lidar(ld)

    return ld
