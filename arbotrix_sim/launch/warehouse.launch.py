"""Full localization/navigation launch for the warehouse AMR."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_share = get_package_share_directory('arbotrix_sim')
    nav2_share = get_package_share_directory('nav2_bringup')

    simulation_launch = os.path.join(sim_share, 'launch', 'simulation.launch.py')
    map_yaml = os.path.join(sim_share, 'maps', 'warehouse_map.yaml')
    nav2_params = os.path.join(sim_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(sim_share, 'rviz', 'arbotrix_base.rviz')

    gazebo_gui = LaunchConfiguration('gazebo_gui')
    use_rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            'gazebo_gui': gazebo_gui,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Humble bringup_launch starts map_server, AMCL, lifecycle managers,
    # planner, controller, BT navigator, behavior server and velocity smoother.
    nav2 = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'slam': 'False',
                    'map': map_yaml,
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params,
                    'autostart': 'true',
                    'use_composition': 'True',
                    'use_respawn': 'False',
                }.items(),
            )
        ],
    )

    rviz = TimerAction(
        period=10.0,
        actions=[
            Node(
                condition=IfCondition(use_rviz),
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        simulation,
        nav2,
        rviz,
    ])
