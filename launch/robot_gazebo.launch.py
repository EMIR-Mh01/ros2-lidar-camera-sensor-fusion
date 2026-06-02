from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_description = get_package_share_directory('my_robot_description')

    xacro_file = os.path.join(
        pkg_description,
        'urdf',
        'robot.urdf.xacro'
    )

    world_file = os.path.join(
        pkg_description,
        'worlds',
        'world_test.world'
    )

    rviz_config_file = os.path.join(
        pkg_description,
        'rviz',
        'robot.rviz'
    )

    # FIX HERE
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        )
    }

    sim_time_param = {'use_sim_time': True}

    return LaunchDescription([

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ),
            launch_arguments={
                'world': world_file
            }.items()
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[sim_time_param]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                sim_time_param
            ]
        ),

        # Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'my_robot'
            ],
            output='screen',
            parameters=[sim_time_param]
        ),

        # RViz
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_file],
                    parameters=[sim_time_param]
                )
            ]
        )

    ])