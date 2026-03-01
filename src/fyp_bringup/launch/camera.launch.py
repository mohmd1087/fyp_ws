#!/usr/bin/env python3
"""
Camera launch file for Orbbec Astra Pro RGB-D camera
Includes depthimage_to_laserscan for Nav2/SLAM compatibility

Orbbec Astra Pro Specs:
- Depth: 640x480 @ 30fps
- Range: 0.6m - 8.0m
- FOV: 60 degrees horizontal

Topics Published by astra_camera:
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)
- /camera/color/image_raw (sensor_msgs/Image)

Topics Published by depthimage_to_laserscan:
- /scan (sensor_msgs/LaserScan)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Find astra_camera package (in ros2_ws)
    # You may need to source ros2_ws before launching
    try:
        astra_pkg = get_package_share_directory('astra_camera')
        astra_launch = os.path.join(astra_pkg, 'launch', 'astra_pro.launch.xml')
        astra_available = os.path.exists(astra_launch)
    except Exception:
        astra_available = False
        astra_launch = ''

    # ========================================
    # Orbbec Astra Pro Camera
    # ========================================
    launch_actions = []

    if astra_available:
        # Include the astra camera launch file
        astra_camera = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(astra_launch),
            launch_arguments={
                'camera_name': 'camera',
                # Depth settings
                'depth_width': '640',
                'depth_height': '480',
                'depth_fps': '30',
                'enable_depth': 'true',
                # Color settings
                'color_width': '640',
                'color_height': '480',
                'color_fps': '30',
                'enable_color': 'true',
                # Disable IR to save bandwidth
                'enable_ir': 'false',
                # Disable point cloud (we use depth image instead)
                'enable_point_cloud': 'false',
                # IMPORTANT: Disable camera TF publishing - we use URDF frames
                'publish_tf': 'false',
            }.items()
        )
        launch_actions.append(astra_camera)
    else:
        # Fallback message if astra_camera not found
        print("WARNING: astra_camera package not found!")
        print("Make sure to source your ros2_ws: source ~/ros2_ws/install/setup.bash")
        print("Then source fyp_ws: source ~/fyp_ws/install/setup.bash")

    # ========================================
    # Depth Image to LaserScan Conversion
    # ========================================
    # Converts depth image to laser scan for Nav2/SLAM
    # Extracts a horizontal slice from the depth image

    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Output frame for laser scan
            # Using scan_frame which is aligned with base_footprint
            'output_frame': 'scan_frame',
            # Range limits (Astra Pro: 0.6m - 8.0m)
            'range_min': 0.6,
            'range_max': 8.0,
            # Scan parameters
            'scan_height': 10,      # Pixels to use for scan (center band)
            'scan_time': 0.033,     # 30 Hz
            # Infinity handling
            'inf_epsilon': 1.0,
        }],
        remappings=[
            # Astra camera publishes to /camera/depth/image_raw
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
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