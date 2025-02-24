import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def create_argus_camera_node(ld: LaunchDescription) -> None:
    params_file = LaunchConfiguration("params_file")

    argus_camera_node = Node(
        package="argus_camera_ros",
        executable="argus_camera_node",
        parameters=[params_file],
        output="screen",
    )

    ld.add_action(argus_camera_node)


def declare_arguments(ld: LaunchDescription, pkg_dir: str) -> None:
    default_param_file = os.path.join(pkg_dir, "config", "params.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_param_file),
        description="Parameters file to be used.",
    )

    ld.add_action(params_file_arg)


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory("argus_camera_ros")

    declare_arguments(ld, pkg_dir)
    create_argus_camera_node(ld)

    return ld
