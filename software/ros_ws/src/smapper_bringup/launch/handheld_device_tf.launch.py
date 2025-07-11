from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transforms for handheld device
        
        # base_link to os_sensor
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_os_sensor_publisher',
            arguments=['0.003000', '0.000000', '0.112000', 
                      '0.000000', '0.000000', '0.000000', '1.000000',
                      'base_link', 'os_sensor']
        ),
        
        # os_sensor to os_lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='os_sensor_to_os_lidar_publisher',
            arguments=['0.000000', '0.000000', '0.038195', 
                      '0.000000', '0.000000', '1.000000', '0.000000',
                      'os_sensor', 'os_lidar']
        ),
        
        # os_sensor to os_imu
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='os_sensor_to_os_imu_publisher',
            arguments=['-0.002441', '-0.009725', '0.007533', 
                      '0.000000', '0.000000', '0.000000', '1.000000',
                      'os_sensor', 'os_imu']
        ),
        
        # base_link to realsense_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_realsense_link_publisher',
            arguments=['0.077200', '0.017500', '0.030280', 
                      '0.000000', '0.000000', '0.000000', '1.000000',
                      'base_link', 'realsense_link']
        ),
        
        # base_link to front_left
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_front_left_publisher',
            arguments=['0.060816', '0.063762', '0.084513', 
                      '0.616467', '-0.357533', '0.351321', '-0.607217',
                      'base_link', 'front_left']
        ),
        
        # base_link to front_right
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_front_right_publisher',
            arguments=['0.056637', '-0.063162', '0.084593', 
                      '0.354204', '-0.612500', '0.613581', '-0.350573',
                      'base_link', 'front_right']
        ),
        
        # base_link to side_left
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_side_left_publisher',
            arguments=['-0.060487', '0.087803', '0.080459', 
                      '0.711228', '-0.001269', '-0.005826', '-0.702936',
                      'base_link', 'side_left']
        ),
        
        # base_link to side_right
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_side_right_publisher',
            arguments=['-0.064485', '-0.077166', '0.078651', 
                      '0.001199', '-0.705124', '0.709054', '0.006368',
                      'base_link', 'side_right']
        ),
        
    ])
