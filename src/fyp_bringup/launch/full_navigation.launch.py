#!/usr/bin/env python3
"""
Full navigation launch file — single command to start everything.

Launches:
  1. Hardware bringup (robot_state_publisher, joint_state_publisher, micro-ROS agent)
  2. EKF (robot_localization — fuses wheel odometry + IMU)
  3. Localization (map_server + AMCL)
  4. Navigation (planner, controller, smoother, BT navigator, behaviors)
  5. RViz (optional, enabled by default)

Usage:
  ros2 launch fyp_bringup full_navigation.launch.py
  ros2 launch fyp_bringup full_navigation.launch.py map:=/path/to/map.yaml
  ros2 launch fyp_bringup full_navigation.launch.py rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    bringup_pkg = get_package_share_directory('fyp_bringup')
    nav2_pkg = get_package_share_directory('fyp_nav2')

    # Default paths
    default_map = os.path.join(nav2_pkg, 'maps', 'my_map.yaml')
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf_real.yaml')
    default_rviz = os.path.join(nav2_pkg, 'rviz', 'nav2.rviz')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map')
    launch_rviz = LaunchConfiguration('rviz')

    # ========================================
    # 1. Hardware Bringup
    # ========================================
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'hardware_bringup.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ========================================
    # 2. EKF (robot_localization)
    # ========================================
    # Delayed 2s to let micro-ROS agent connect first
    ekf = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            )
        ]
    )

    # ========================================
    # 3. Localization (map_server + AMCL)
    # ========================================
    # Delayed 3s to let EKF start publishing odom->base_footprint
    localization = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'localization.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml,
                }.items()
            )
        ]
    )

    # ========================================
    # 4. Navigation (full Nav2 stack)
    # ========================================
    # Delayed 5s to let localization establish map->odom->base_footprint chain
    navigation = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # ========================================
    # 5. RViz (optional)
    # ========================================
    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', default_rviz] if os.path.exists(default_rviz) else [],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(launch_rviz),
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map YAML file'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz (true/false)'
        ),
        hardware,
        ekf,
        localization,
        navigation,
        rviz,
    ])
