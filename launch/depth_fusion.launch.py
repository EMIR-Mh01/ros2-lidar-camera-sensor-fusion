#!/usr/bin/env python3
"""
depth_fusion.launch.py
======================
Launches the full sensor-fusion pipeline:

  1. scan_to_cloud      – LaserScan  → PointCloud2  (/lidar/points)
  2. colorize_cloud     – Projects RGB camera onto LiDAR cloud
                          publishes  /colored_cloud  (XYZRGB)
  3. depth_colorize_cloud – Colorises the depth camera's own point cloud
                          publishes  /depth_camera/colored_points (XYZRGB)

Run AFTER the robot and localization are already up:
  ros2 launch my_robot_description robot_gazebo.launch.py
  ros2 launch my_robot_description local_localization.launch.py
  ros2 launch my_robot_description depth_fusion.launch.py

RViz topics to add
------------------
  PointCloud2  /colored_cloud               (LiDAR colourised by RGB camera)
  PointCloud2  /depth_camera/colored_points (depth camera cloud, colourised)
  PointCloud2  /depth_camera/points         (raw depth cloud)
  Image        /camera/image_raw
  Image        /depth_camera/image_raw
  Image        /depth_camera/depth/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock from Gazebo',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 1. LaserScan → PointCloud2 ──────────────────────────────────────────
    scan_to_cloud = Node(
        package='my_robot_description',
        executable='scan_to_cloud.py',
        name='scan_to_cloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 2. Colorise LiDAR cloud with RGB camera ──────────────────────────────
    colorize_lidar = Node(
        package='my_robot_description',
        executable='colorize_cloud.py',
        name='colorize_lidar_cloud',
        output='screen',
        parameters=[{
            'use_sim_time':     use_sim_time,
            'image_topic':      '/camera/image_raw',
            'info_topic':       '/camera/camera_info',
            'cloud_topic':      '/lidar/points',
            'output_topic':     '/colored_cloud',
            'tf_timeout_sec':   0.15,
            'apply_distortion': False,   # set True for real HW with distortion
        }],
    )

    # ── 3. Colorise depth camera cloud with its own RGB image ────────────────
    colorize_depth = Node(
        package='my_robot_description',
        executable='depth_colorize_cloud.py',
        name='depth_colorize_cloud',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'image_topic':   '/depth_camera/image_raw',
            'info_topic':    '/depth_camera/camera_info',
            'cloud_topic':   '/depth_camera/points',
            'output_topic':  '/depth_camera/colored_points',
            'slop':          0.05,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        scan_to_cloud,
        colorize_lidar,
        colorize_depth,
    ])
