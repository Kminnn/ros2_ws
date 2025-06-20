#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments for LiDAR node
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # Paths to configs
    rviz_config_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'rviz',
        'sllidar_ros2.rviz'
    )
    slam_params = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config',
        'slam_toolbox_params.yaml'
    )

    return LaunchDescription([
        # Declare LiDAR launch arguments
        DeclareLaunchArgument('channel_type', default_value='serial'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='true'),
        DeclareLaunchArgument('scan_mode', default_value='Standard'),

        # Launch sllidar_node with parameters from arguments
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            output='screen'
        ),

        # Launch RViz2 with the sllidar config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),

        # Static transform from base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # Static transform from odom to base_link
        #Node(
            #package='tf2_ros',
            #executable='static_transform_publisher',
            #name='static_tf_odom_to_base',
            #arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            #output='screen'
        #),

        # Delay starting slam_toolbox node by 3 seconds to let LiDAR come up
        TimerAction(
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
        ),

        # Delay starting lidar_odometry node by 5 seconds
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='lidar_odometry',
                    executable='lidar_odometry_node',
                    name='lidar_odometry_node',
                    output='screen'
                )
            ]
        ),

        # Delay starting scan_odom_republisher node by 6 seconds
        TimerAction(
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
    ])

