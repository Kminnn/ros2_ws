from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to slam_toolbox parameters
    slam_params = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config',
        'slam_toolbox_params.yaml'
    )

    # Path to RViz config
    rviz_config = os.path.join(
        get_package_share_directory('my_robot_description'),
        'rviz',
        'slam_toolbox_default.rviz'
    )

    return LaunchDescription([
        # Launch slam_toolbox node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),

        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),

        # Static transform publisher from base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # Optional: static transform publisher from odom to base_link if no odom available
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_odom_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
    ])
