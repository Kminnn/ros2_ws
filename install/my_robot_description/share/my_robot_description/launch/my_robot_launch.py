from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to config files (update as needed)
    slam_params = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config',
        'slam_toolbox_params.yaml'
    )

    # Nodes
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen'
    )

    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    slam_toolbox_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params]
            )
        ]
    )

    lidar_odom_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='lidar_odometry',
                executable='lidar_odometry_node',
                name='lidar_odometry_node',
                output='screen'
            )
        ]
    )

    scan_odom_republisher_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='my_robot_bringup',
                executable='scan_odom_republisher',
                name='scan_odom_republisher',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        sllidar_node,
        static_tf_base_to_laser,
        static_tf_odom_to_base,
        slam_toolbox_node,
        lidar_odom_node,
        scan_odom_republisher_node
    ])

