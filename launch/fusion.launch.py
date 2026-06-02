#!/usr/bin/env python3
"""
fusion.launch.py
================
Starts the full camera+LiDAR colorization pipeline:

  /scan  →  scan_to_cloud  →  /lidar/points
                                    │
  /camera/image_raw  ───────────────┤
  /camera/camera_info  ─────────────┴──►  colorize_cloud  →  /colored_cloud

Run AFTER (or alongside) robot_gazebo.launch.py + local_localization.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Step 1: convert 2D LaserScan → 3D PointCloud2
    scan_to_cloud = Node(
        package='my_robot_description',
        executable='scan_to_cloud.py',
        name='scan_to_cloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Step 2: project camera image onto cloud → coloured cloud
    colorize_cloud = Node(
        package='my_robot_description',
        executable='colorize_cloud.py',
        name='colorize_cloud',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'image_topic':  '/camera/image_raw'},
            {'info_topic':   '/camera/camera_info'},
            {'cloud_topic':  '/lidar/points'},
            {'output_topic': '/colored_cloud'},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        scan_to_cloud,
        colorize_cloud,
    ])
