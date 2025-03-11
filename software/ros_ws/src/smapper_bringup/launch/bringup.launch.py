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
                "0.03318610562506225",
                "-0.015278429497267832",
                "-0.06474282491831904",
                "0.35628801099155066",
                "0.6139333736916183",
                "-0.6088211429763346",
                "-0.3542336541836505",
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
                "0.08462814144045423",
                "0.017340756252757404",
                "-0.06428646007796933",
                "-0.608806696209101",
                "-0.3604359709763084",
                "0.35484366861308814",
                "0.6111679706288038",
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
                "0.05771129303407797",
                "0.027291193242906367",
                "-0.061523287762030456",
                "-0.0024961133903230602",
                "-0.7048276533230966",
                "0.7093740655599994",
                "0.00042853223583295204",
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
                "0.10848973385131369",
                "-0.07588573537121682",
                "-0.08557682440117889",
                "-0.7052468625682459",
                "-0.002056279615153262",
                "-0.0028403720413532987",
                "0.7089531485496462",
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
