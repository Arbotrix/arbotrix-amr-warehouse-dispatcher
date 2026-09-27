"""One-command AMR system launch: simulation + Nav2 + bridge + dispatcher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_share = get_package_share_directory('arbotrix_sim')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    use_rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'warehouse.launch.py')
        ),
        launch_arguments={
            'gazebo_gui': gazebo_gui,
            'rviz': use_rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    particle_bridge = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='warehouse_dispatcher',
                executable='particlecloud_bridge',
                name='particlecloud_posearray_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    # Dispatcher contains its own Nav2-action + full-TF readiness gate, so this
    # delay is only to reduce noisy startup logs; it does not replace readiness checks.
    dispatcher_node = Node(
        package='warehouse_dispatcher',
        executable='dispatcher',
        name='warehouse_dispatcher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'goal_timeout_sec': 180.0},
            {'goal_retries': 2},
            {'pickup_wait_sec': 2.0},
            {'delivery_wait_sec': 2.0},
            {'readiness_timeout_sec': 90.0},
        ],
    )
    dispatcher = TimerAction(period=12.0, actions=[dispatcher_node])

    # When the dispatcher exits after success/failure, stop Gazebo/Nav2/RViz
    # cleanly instead of leaving orphaned processes behind.
    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=dispatcher_node,
            on_exit=[EmitEvent(event=Shutdown(reason='Warehouse mission finished'))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        base_stack,
        particle_bridge,
        dispatcher,
        shutdown_when_done,
    ])
