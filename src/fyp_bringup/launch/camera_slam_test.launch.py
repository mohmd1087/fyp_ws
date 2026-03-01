#!/usr/bin/env python3
"""
Simplified SLAM test launch for camera-only testing
Uses static transforms instead of EKF (no wheel odometry/IMU required)

Usage:
  source ~/ros2_ws/install/setup.bash
  source ~/fyp_ws/install/setup.bash
  ros2 launch fyp_bringup camera_slam_test.launch.py

This is for TESTING ONLY - the robot won't move in the map without odometry.
Use teleop to see if scan data appears correctly.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    description_pkg = get_package_share_directory('fyp_description')
    bringup_pkg = get_package_share_directory('fyp_bringup')
    nav2_pkg = get_package_share_directory('fyp_nav2')

    # Robot URDF
    urdf_file = os.path.join(description_pkg, 'urdf', 'new_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Find astra camera
    try:
        astra_pkg = get_package_share_directory('astra_camera')
        astra_launch = os.path.join(astra_pkg, 'launch', 'astra.launch.xml')
        astra_available = os.path.exists(astra_launch)
    except Exception:
        astra_available = False
        astra_launch = ''

    # ========================================
    # 1. Robot State Publisher
    # ========================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    # ========================================
    # 2. Joint State Publisher (for wheel joints)
    # ========================================
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # ========================================
    # 3. Static Transform: map -> odom (identity)
    # ========================================
    # For testing without localization
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # ========================================
    # 4. Static Transform: odom -> base_footprint (identity)
    # ========================================
    # For testing without odometry - robot stays at origin
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    # ========================================
    # 5. Astra Camera
    # ========================================
    launch_actions = [
        robot_state_publisher,
        joint_state_publisher,
        map_to_odom,
        odom_to_base,
    ]

    if astra_available:
        astra_camera = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(astra_launch),
            launch_arguments={
                'camera_name': 'camera',
                'depth_width': '640',
                'depth_height': '480',
                'depth_fps': '30',
                'enable_depth': 'true',
                'enable_color': 'true',
                'enable_ir': 'false',
                'enable_point_cloud': 'false',
                'publish_tf': 'false',  # We use URDF frames
            }.items()
        )
        launch_actions.append(astra_camera)

    # ========================================
    # 6. Depth to LaserScan
    # ========================================
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        output='screen',
        parameters=[{
            'output_frame': 'scan_frame',
            'range_min': 0.6,
            'range_max': 8.0,
            'scan_height': 10,
            'scan_time': 0.033,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan'),
        ]
    )
    launch_actions.append(depth_to_scan)

    # ========================================
    # 7. SLAM Toolbox (delayed start)
    # ========================================
    slam_toolbox = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'base_frame': 'base_footprint',
                    'scan_topic': '/scan',
                    'resolution': 0.05,
                    'max_laser_range': 8.0,
                    'minimum_time_interval': 0.5,
                    'transform_timeout': 0.5,
                    'tf_buffer_duration': 30.0,
                    'stack_size_to_use': 40000000,
                    'use_scan_matching': True,
                    'use_scan_barycenter': True,
                    'minimum_travel_distance': 0.0,  # Accept all scans (no movement)
                    'minimum_travel_heading': 0.0,
                    'mode': 'mapping',
                }],
            )
        ]
    )
    launch_actions.append(slam_toolbox)

    # ========================================
    # 8. RViz
    # ========================================
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2.rviz')
    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                output='screen',
            )
        ]
    )
    launch_actions.append(rviz)

    return LaunchDescription(launch_actions)