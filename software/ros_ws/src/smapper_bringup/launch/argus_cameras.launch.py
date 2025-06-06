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


def publish_tfs(ld: LaunchDescription) -> None:
    # ld.add_action(
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         name="static_transform_publisher_os_fr_camera",
    #         arguments=[
    #             "--x",
    #             "-0.012168688414324998",
    #             "--y",
    #             "-0.05230268675672892",
    #             "--z",
    #             "-0.04846000514448317",
    #             "--qx",
    #             "0.3586529704455553",
    #             "--qy",
    #             "0.6153675089165976",
    #             "--qz",
    #             "-0.607803551863725",
    #             "--qw",
    #             "-0.351092178924853",
    #             "--frame-id",
    #             "os_lidar",
    #             "--child-frame-id",
    #             "front_left_camera",
    #         ],
    #     )
    # )
    #
    # ld.add_action(
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         name="static_transform_publisher_os_fr_camera",
    #         arguments=[
    #             "--x",
    #             "-0.007861905010314724",
    #             "--y",
    #             "0.012601838083764312",
    #             "--z",
    #             "-0.04350629916231412",
    #             "--qx",
    #             "-0.6120399884004516",
    #             "--qy",
    #             "-0.3606205347019306",
    #             "--qz",
    #             "0.3508780584930321",
    #             "--qw",
    #             "0.6101184070475415",
    #             "--frame-id",
    #             "os_lidar",
    #             "--child-frame-id",
    #             "front_right_camera",
    #         ],
    #     )
    # )
    #
    # ld.add_action(
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         name="static_transform_publisher_os_fr_camera",
    #         arguments=[
    #             "--x",
    #             "0.07974294467004457",
    #             "--y",
    #             "0.031088269481674376",
    #             "--z",
    #             "-0.0064455203636650015",
    #             "--qx",
    #             "-0.002113388348180505",
    #             "--qy",
    #             "0.7245350428347996",
    #             "--qz",
    #             "-0.6892338777765248",
    #             "--qw",
    #             "0.0010802866284276606",
    #             "--frame-id",
    #             "os_lidar",
    #             "--child-frame-id",
    #             "side_left_camera",
    #         ],
    #     )
    # )

    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_imu_to_camera",
            arguments=[
                "--x", "0.05734822052548241",
                "--y", "0.05973053014042769",
                "--z", "-0.04733641790040566",
                "--qx", "0.6203218624628579",
                "--qy", "-0.35708040212980113",
                "--qz", "0.35693510550626295",
                "--qw", "-0.60024303729628",
                "--frame-id", "os_imu",
                "--child-frame-id", "front_left",
            ],
        )
    )

    ld.add_action(
    Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_imu_to_camera",
        arguments=[
            "--x", "0.03813065464667919",
            "--y", "-0.047371485526111944",
            "--z", "-0.036135738068488736",
            "--qx", "0.364039840144593",
            "--qy", "-0.6084624915794161",
            "--qz", "0.6125602747186555",
            "--qw", "-0.34931118070435924",
            "--frame-id", "os_imu",
            "--child-frame-id", "front_right",
        ],
    )
)




def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    launch_argus_cameras(ld)
    publish_tfs(ld)

    return ld
