#!/usr/bin/env python3
"""
perception_pipeline.launch.py  —  v2
======================================
Full perception pipeline: colorize → EKF denoise → segment.

Run order
---------
  Terminal 1:  ros2 launch my_robot_description robot_gazebo.launch.py
  Terminal 2:  ros2 launch my_robot_description local_localization.launch.py
  Terminal 3:  ros2 launch my_robot_description perception_pipeline.launch.py
  Terminal 4:  ros2 launch my_robot_description display.launch.py

Topics
------
  /colored_cloud               raw LiDAR colourised by RGB camera
  /colored_cloud/filtered      EKF-denoised version (smoother, less jitter)
  /depth_camera/colored_points depth camera point cloud
  /segmented_cloud             objects colour-coded by semantic class
  /segment_markers             bounding boxes + class labels in RViz

Segmentation colour key
-----------------------
  Grey   = Ground
  Blue   = Wall
  Yellow = Person
  Orange = Table
  Red    = Car
  Cyan   = Obstacle
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ust_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    ust     = LaunchConfiguration('use_sim_time')

    scan_to_cloud = Node(
        package='my_robot_description',
        executable='scan_to_cloud.py',
        name='scan_to_cloud',
        output='screen',
        parameters=[{'use_sim_time': ust}],
    )

    colorize_lidar = Node(
        package='my_robot_description',
        executable='colorize_cloud.py',
        name='colorize_lidar_cloud',
        output='screen',
        parameters=[{
            'use_sim_time':   ust,
            'image_topic':    '/camera/image_raw',
            'info_topic':     '/camera/camera_info',
            'cloud_topic':    '/lidar/points',
            'output_topic':   '/colored_cloud',
            'voxel_size':     0.05,
            'sor_k':          10,
            'sor_std_ratio':  1.5,
            'min_range_m':    0.30,
            'min_depth_m':    0.05,
        }],
    )

    colorize_depth = Node(
        package='my_robot_description',
        executable='depth_colorize_cloud.py',
        name='depth_colorize_cloud',
        output='screen',
        parameters=[{
            'use_sim_time':  ust,
            'image_topic':   '/depth_camera/image_raw',
            'info_topic':    '/depth_camera/camera_info',
            'cloud_topic':   '/depth_camera/points',
            'output_topic':  '/depth_camera/colored_points',
            'slop':          0.05,
        }],
    )

    ekf_cloud = Node(
        package='my_robot_description',
        executable='ekf_cloud.py',
        name='ekf_cloud',
        output='screen',
        parameters=[{
            'use_sim_time':  ust,
            'input_topic':   '/colored_cloud',
            'output_topic':  '/colored_cloud/filtered',
            'voxel_size':    0.10,
            'q_noise':       0.002,
            'r_noise':       0.08,
            'max_age':       5,       # flush in 5 scans → handles robot motion
            'min_points':    2,
            'color_alpha':   0.40,
        }],
    )

    segment_cloud = Node(
        package='my_robot_description',
        executable='segment_cloud.py',
        name='segment_cloud',
        output='screen',
        parameters=[{
            'use_sim_time':       ust,
            'input_topic':        '/colored_cloud/filtered',
            'output_topic':       '/segmented_cloud',
            'marker_topic':       '/segment_markers',
            'cluster_tolerance':  0.20,   # 2D XY gap between objects
            'min_cluster_size':   5,
            'max_cluster_size':   2000,
            'remove_ground':      True,
            'ground_z_min':      -0.25,
            'ground_z_max':       0.10,
        }],
    )

    return LaunchDescription([
        ust_arg,
        scan_to_cloud,
        colorize_lidar,
        colorize_depth,
        ekf_cloud,
        segment_cloud,
    ])
