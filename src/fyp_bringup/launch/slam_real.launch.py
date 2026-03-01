#!/usr/bin/env python3
"""
SLAM launch file for real robot using SLAM Toolbox
Creates a map from laser scan data (from depth camera)

Usage:
1. Launch this file: ros2 launch fyp_bringup slam_real.launch.py
2. Drive the robot with teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard
3. Save the map: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

Prerequisites:
- Hardware must be running (wheel odometry, IMU)
- Camera must be running (depth to scan conversion)
- EKF must be running (fused odometry)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ========================================
    # SLAM Toolbox - Async Mode
    # ========================================
    # Async mode is recommended for real robots as it doesn't block
    # on scan processing

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # SLAM mode
            'mode': 'mapping',

            # Solver parameters
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': 'None',

            # Frame configuration
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',

            # Map parameters
            'resolution': 0.05,              # 5cm per pixel
            'max_laser_range': 8.0,          # Astra Pro max range
            'minimum_time_interval': 0.5,    # Min time between scans
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,

            # Scan matching parameters
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.3,  # Min distance to trigger scan
            'minimum_travel_heading': 0.3,   # Min rotation to trigger scan (rad)

            # Correlation parameters
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1,

            # Loop closure parameters
            'loop_search_space_dimension': 8.0,
            'loop_search_space_resolution': 0.05,
            'loop_search_space_smear_deviation': 0.03,

            # Scan matcher parameters
            'distance_variance_penalty': 0.5,
            'angle_variance_penalty': 1.0,
            'fine_search_angle_offset': 0.00349,  # ~0.2 degrees
            'coarse_search_angle_offset': 0.349,  # ~20 degrees
            'coarse_angle_resolution': 0.0349,    # ~2 degrees
            'minimum_angle_penalty': 0.9,
            'minimum_distance_penalty': 0.5,

            # Response threshold
            'use_response_expansion': True,

            # Stack size (increase if segfault)
            'stack_size_to_use': 40000000,

            # Map update
            'map_update_interval': 5.0,
            'enable_interactive_mode': True,

            # Debug
            'debug_logging': False,
            'throttle_scans': 1,
        }],
    )

    # ========================================
    # RViz for visualization (optional)
    # ========================================
    bringup_pkg = get_package_share_directory('fyp_bringup')
    nav2_pkg = get_package_share_directory('fyp_nav2')

    # Try to find rviz config
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = ''

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if rviz_config else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        slam_toolbox,
        rviz,
    ])
