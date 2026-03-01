#!/usr/bin/env python3
"""
Full system bringup launch file for real robot navigation
Launches all components with proper sequencing

Usage:
  SLAM mode (create map):
    ros2 launch fyp_bringup bringup_nav.launch.py mode:=slam

  Navigation mode (use existing map):
    ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav map:=/path/to/map.yaml

Arguments:
  mode: 'slam' for mapping, 'nav' for navigation (default: nav)
  map: path to map YAML file (required for nav mode)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    bringup_pkg = get_package_share_directory('fyp_bringup')
    nav2_pkg = get_package_share_directory('fyp_nav2')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mode = LaunchConfiguration('mode', default='nav')
    map_yaml = LaunchConfiguration('map')

    # Default map path
    default_map = os.path.join(nav2_pkg, 'maps', 'maze.yaml')

    # ========================================
    # 1. Hardware Bringup
    # ========================================
    # Robot state publisher, hardware drivers (placeholders)
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'hardware_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ========================================
    # 2. Camera Launch (delayed 2 seconds)
    # ========================================
    # Orbbec Astra Pro + depth to scan
    camera_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'camera.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items()
            )
        ]
    )

    # ========================================
    # 3. EKF Node (delayed 3 seconds)
    # ========================================
    # Fuses wheel odometry + IMU
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf_real.yaml')
    ekf_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config,
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('odometry/filtered', '/odometry/filtered'),
                ]
            )
        ]
    )

    # ========================================
    # 4a. SLAM Mode (delayed 5 seconds)
    # ========================================
    # Launch SLAM Toolbox for mapping
    slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'slam_real.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'slam'"])
                )
            )
        ]
    )

    # ========================================
    # 4b. Navigation Mode (delayed 5 seconds)
    # ========================================
    # Launch localization (AMCL) and navigation stack

    # Localization launch
    localization_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'localization.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'nav'"])
                )
            )
        ]
    )

    # Navigation launch (delayed further for localization to start)
    navigation_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'nav'"])
                )
            )
        ]
    )

    # ========================================
    # 5. RViz (delayed 6 seconds)
    # ========================================
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2.rviz')
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                # Only launch RViz in nav mode (SLAM launch already includes RViz)
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'nav'"])
                )
            )
        ]
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (false for real robot)'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='nav',
            description='Operating mode: slam (mapping) or nav (navigation)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map YAML file (for nav mode)'
        ),

        # Launch components
        hardware_bringup,
        camera_launch,
        ekf_node,
        slam_launch,
        localization_launch,
        navigation_launch,
        rviz_node,
    ])
