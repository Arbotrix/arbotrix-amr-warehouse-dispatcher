"""Gazebo Classic + robot_state_publisher + robot spawn."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    sim_share = get_package_share_directory('arbotrix_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')

    world = os.path.join(sim_share, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(sim_share, 'urdf', 'arbotrix_bot.urdf.xacro')

    gui = LaunchConfiguration('gazebo_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': 'false',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
        ],
    )

    # Give gzserver time to advertise /spawn_entity before the spawn request.
    spawn_robot = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_arbotrix_bot',
                output='screen',
                arguments=[
                    '-entity', 'arbotrix_bot',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '-3.5',
                    '-z', '0.02',
                    '-Y', '1.57079632679',
                ],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gazebo_gui', default_value='false',
            description='Start the Gazebo GUI. Keep false on the 4 GB VM unless needed.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo /clock.',
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
