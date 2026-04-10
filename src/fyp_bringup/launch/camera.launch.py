#!/usr/bin/env python3
"""
Camera launch file for Orbbec Astra Pro RGB-D camera.

Behaviour is controlled by the LOCALIZATION_MODE environment variable:

  LOCALIZATION_MODE=amcl (default)
    - Publishes /scan via depthimage_to_laserscan (for AMCL + costmap)
    - depth_sync_node fixes timestamp mismatch
    - Point cloud disabled

  LOCALIZATION_MODE=rtabmap
    - Enables /camera/depth/points (PointCloud2) for Nav2 costmap
    - No fake laser scan (RTAB-Map uses raw RGB+depth directly)
    - depth_sync_node and depthimage_to_laserscan NOT launched

Topics Published by astra_camera:
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/depth/points (PointCloud2) — rtabmap mode only
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Switch behaviour based on env var (resolved at launch time, not runtime)
    localization_mode = os.environ.get('LOCALIZATION_MODE', 'amcl').lower()
    use_rtabmap = (localization_mode == 'rtabmap')

    # Find astra_camera package
    try:
        astra_pkg = get_package_share_directory('astra_camera')
        astra_launch = os.path.join(astra_pkg, 'launch', 'astra_pro.launch.xml')
        astra_available = os.path.exists(astra_launch)
    except Exception:
        astra_available = False
        astra_launch = ''

    launch_actions = []

    # ========================================
    # Orbbec Astra Pro Camera
    # ========================================
    if astra_available:
        astra_camera = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(astra_launch),
            launch_arguments={
                'camera_name': 'camera',
                'depth_width': '640',
                'depth_height': '480',
                'depth_fps': '30',
                'enable_depth': 'true',
                'color_width': '640',
                'color_height': '480',
                'color_fps': '30',
                'enable_color': 'true',
                'enable_ir': 'false',
                # Enable point cloud in rtabmap mode (needed for costmap)
                'enable_point_cloud': 'true' if use_rtabmap else 'false',
                # Sync depth+color timestamps (needed for both modes)
                'color_depth_synchronization': 'true',
                # Use URDF frames, not camera driver TF
                'publish_tf': 'false',
            }.items()
        )
        launch_actions.append(astra_camera)
    else:
        print("WARNING: astra_camera package not found!")
        print("Source ros2_ws first: source ~/ros2_ws/install/setup.bash")

    # ========================================
    # AMCL mode: fake laser scan pipeline
    # ========================================
    if not use_rtabmap:
        # Depth sync node: fixes ~400ms timestamp mismatch between depth image and camera_info
        depth_sync = Node(
            package='fyp_bringup',
            executable='depth_sync_node.py',
            name='depth_sync_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
        launch_actions.append(depth_sync)

        depth_to_scan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'output_frame': 'scan_frame',
                'range_min': 0.6,
                'range_max': 8.0,
                'scan_height': 60,
                'scan_time': 0.033,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('depth', '/camera/depth/image_synced'),
                ('depth_camera_info', '/camera/depth/camera_info_synced'),
                ('scan', '/scan'),
            ]
        )
        launch_actions.append(depth_to_scan)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
    ] + launch_actions)