#!/usr/bin/env python3
"""
Navigation launch file for full Nav2 stack
Launches all navigation components: planner, controller, behaviors, etc.

Usage:
ros2 launch fyp_bringup navigation.launch.py

Prerequisites:
- Localization must be running (AMCL with map)
- Hardware must be running (odometry, sensors)
- Camera must be running (for obstacle detection)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    nav2_pkg = get_package_share_directory('fyp_nav2')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')

    # Default params file
    default_params = os.path.join(nav2_pkg, 'params', 'nav2_params_real.yaml')

    # ========================================
    # Controller Server
    # ========================================
    # Executes the local trajectory (follows path from planner)
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ]
    )

    # ========================================
    # Planner Server
    # ========================================
    # Computes global path from current position to goal
    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ========================================
    # Behavior Server
    # ========================================
    # Executes recovery behaviors (spin, backup, wait)
    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ========================================
    # BT Navigator
    # ========================================
    # Behavior tree navigator - orchestrates navigation
    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ========================================
    # Smoother Server
    # ========================================
    # Smooths global path from planner (called by BT SmoothPath node)
    smoother_server = LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ========================================
    # Waypoint Follower
    # ========================================
    # Follows a series of waypoints
    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ========================================
    # Velocity Smoother
    # ========================================
    # Smooths velocity commands for better motion
    velocity_smoother = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel'),
        ]
    )

    # ========================================
    # Lifecycle Manager
    # ========================================
    # Manages lifecycle of all Nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
            'bond_timeout': 4.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to Nav2 params file'
        ),
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])