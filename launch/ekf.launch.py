#!/usr/bin/env python3
"""
ekf.launch.py
=============
Minimal EKF-only launch (no imu_republisher, no odom_noisy).
Useful for hardware runs where sensors publish directly to the correct topics.

For Gazebo / development use local_localization.launch.py instead.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # FIX: was a hardcoded absolute path (/home/ubuntu/...) — now portable
    pkg_share = get_package_share_directory("my_robot_description")
    ekf_config = os.path.join(pkg_share, "config", "ekf.yaml")

    # FIX: consistent base frame (base_footprint_ekf → base_footprint)
    static_tf_ekf_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_ekf_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "1",
                   "base_footprint_ekf", "base_footprint"]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_tf_ekf_to_base,
        ekf_node,
    ])
