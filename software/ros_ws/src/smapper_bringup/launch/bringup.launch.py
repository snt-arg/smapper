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


def publish_tfs(ld: LaunchDescription) -> None:
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_base_os",
            arguments=[
                "0.0",
                "-0.027116",
                "0.110",
                "0.0",
                "0.0",
                "0.0",
                "1.0",
                "base_link",
                "os_sensor",
            ],
        )
    )

    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_os_fr_camera",
            arguments=[
                "-0.012168688414324998",
                "-0.05230268675672892",
                "-0.04846000514448317",
                "0.3586529704455553",
                "0.6153675089165976",
                "-0.607803551863725",
                "-0.351092178924853",
                "os_lidar",
                "front_left_camera",
            ],
        )
    )

    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_os_fr_camera",
            arguments=[
                "-0.007861905010314724",
                "0.012601838083764312",
                "-0.04350629916231412",
                "-0.6120399884004516",
                "-0.3606205347019306",
                "0.3508780584930321",
                "0.6101184070475415",
                "os_lidar",
                "front_right_camera",
            ],
        )
    )

    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_os_fr_camera",
            arguments=[
                "0.07974294467004457",
                "0.031088269481674376",
                "-0.0064455203636650015",
                "-0.002113388348180505",
                "0.7245350428347996",
                "-0.6892338777765248",
                "0.0010802866284276606",
                "os_lidar",
                "side_left_camera",
            ],
        )
    )

    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_os_fr_camera",
            arguments=[
                "0.029100216695835537",
                "-0.06844003776442746",
                "-0.03425028075003019",
                "-0.7031074296764369",
                "0.013863898949048182",
                "-0.013130945492662398",
                "0.7108272032710857",
                "os_lidar",
                "side_right_camera",
            ],
        )
    )


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    launch_argus_cameras(ld)
    launch_ouster_lidar(ld)
    publish_tfs(ld)

    return ld
