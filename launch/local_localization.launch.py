#!/usr/bin/env python3
"""
local_localization.launch.py
============================
Full sensor-fusion + Kalman filter pipeline.

Launch graph
------------

  /odom  ──────────────────────────────────┐
    │                                      │  odom0 (vx, vyaw)
    ▼                                      │
  odom_noisy.py ──► /odom_noisy            │
    │                                      ▼
    ▼                          ┌─────── ekf_filter_node ──► /odometry/filtered
  odom_kalman.py ──► /odom_kalman ──► odom1 (vx, vyaw)    ──► TF: odom → base_footprint
                                      │
  /imu/out                             │  imu0 (yaw, vyaw, ax, ay)
    ▼                                  │
  imu_republisher.py ──► /imu/data ───┘

  kalman.py ──► /kalman/odom   (position KF for comparison / visualisation)
             ──► /kalman/marker

Usage
-----
  ros2 launch my_robot_description local_localization.launch.py
  ros2 launch my_robot_description local_localization.launch.py use_sim_time:=false

RViz extras
-----------
  Add Odometry on /odometry/filtered   ← the authoritative EKF output
  Add Odometry on /kalman/odom         ← position-level KF (comparison)
  Add Marker  on /kalman/marker        ← orange sphere = KF position estimate
  Add Odometry on /odom_noisy          ← raw noisy input
  Add Odometry on /odom_kalman         ← velocity-smoothed input to EKF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Arguments ──────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='true',
        description='Launch Python helpers (imu_republisher, odom_noisy, kalman nodes)',
    )
    use_python = LaunchConfiguration('use_python')

    # ── Config path ────────────────────────────────────────────────────────────
    pkg_share = get_package_share_directory('my_robot_description')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # ── Static TF: EKF internal IMU frame ─────────────────────────────────────
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu_ekf',
        arguments=[
            '0', '0', '0.103',
            '0', '0', '0', '1',
            'base_footprint_ekf', 'imu_link_ekf',
        ],
    )

    static_tf_ekf_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_ekf_to_base',
        arguments=[
            '0', '0', '0',
            '0', '0', '0', '1',
            'base_footprint_ekf', 'base_footprint',
        ],
    )

    # ── robot_localization EKF ─────────────────────────────────────────────────
    # Fuses: odom0=/odom  +  odom1=/odom_kalman  +  imu0=/imu/data
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # ── IMU republisher: /imu/out → /imu/data ────────────────────────────────
    imu_republisher = Node(
        package='my_robot_description',
        executable='imu_republisher.py',
        name='imu_republisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_python),
    )

    # ── Noisy odometry (simulates encoder noise for testing) ──────────────────
    odom_noisy = Node(
        package='my_robot_description',
        executable='odom_noisy.py',
        name='odom_noisy',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_python),
    )

    # ── Kalman velocity filter: /odom_noisy → /odom_kalman ───────────────────
    # Smooths [vx, wz] before they reach the EKF (odom1 input).
    # Q and R can be tuned here without editing source code.
    odom_kalman = Node(
        package='my_robot_description',
        executable='odom_kalman.py',
        name='odom_kalman',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'q_vx': 0.01,   # process noise — smaller = smoother
            'q_wz': 0.01,
            'r_vx': 0.10,   # measurement noise — larger = more filtering
            'r_wz': 0.20,
            'publish_tf': True,
        }],
        condition=IfCondition(use_python),
    )

    # ── Position-level Kalman on clean odom: /odom → /kalman/odom ────────────
    # Demonstrates the KF equations on position+velocity state.
    # Publishes /kalman/odom (Odometry) and /kalman/marker (RViz sphere).
    kalman_pos = Node(
        package='my_robot_description',
        executable='kalman.py',
        name='kalman_position_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'q_pos': 0.001,   # tight process noise → smooth trajectory
            'q_vel': 0.01,
            'r_pos': 0.05,    # moderate measurement noise
            'r_vel': 0.10,
            'publish_tf': True,
        }],
        condition=IfCondition(use_python),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_python_arg,
        # TF
        static_tf_imu,
        static_tf_ekf_to_base,
        # Core EKF
        ekf_node,
        # Helpers
        imu_republisher,
        odom_noisy,
        # Kalman filters (both now active)
        odom_kalman,
        kalman_pos,
    ])
