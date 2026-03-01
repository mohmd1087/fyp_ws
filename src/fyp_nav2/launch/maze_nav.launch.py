#!/usr/bin/env python3
"""
Navigation launch file for maze world with imported map
Uses AMCL for localization (not SLAM)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    nav2_pkg = get_package_share_directory('fyp_nav2')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # Map file (imported from autonomous_tb3_ws)
    map_file = os.path.join(nav2_pkg, 'maps', 'maze.yaml')

    # Nav2 params
    nav2_params = os.path.join(nav2_pkg, 'params', 'nav2_params.yaml')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup Nav2 stack'
    )

    # Nav2 bringup launch (includes AMCL, planners, controllers, etc.)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': autostart,
        }.items()
    )

    # RViz for visualization
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        nav2_bringup,
        rviz,
    ])