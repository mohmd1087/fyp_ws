#!/usr/bin/env python3
"""
Handheld RTAB-Map mapping launch file.

Hold the Orbbec Astra Pro and walk around the restaurant to build a visual map.
No robot hardware needed — just the camera connected via USB.

Prerequisites:
  source ~/ros2_ws/install/setup.bash
  source ~/fyp_ws/install/setup.bash

Usage:
  ros2 launch fyp_bringup rtabmap_handheld_mapping.launch.py

Walk slowly around the entire area. When done press Ctrl+C.
Map is saved to ~/.ros/rtabmap.db automatically.

After mapping, navigate with:
  LOCALIZATION_MODE=rtabmap ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Base frame for rtabmap — camera_link, so rgbd_odometry publishes odom→camera_link
# (the static TFs map camera_link to the optical frames that image headers reference)
CAMERA_FRAME = 'camera_link'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    launch_actions = []

    # ========================================
    # 1. Orbbec Astra Pro Camera
    # ========================================
    try:
        astra_pkg = get_package_share_directory('astra_camera')
        astra_launch = os.path.join(astra_pkg, 'launch', 'astra_pro.launch.xml')
        astra_available = os.path.exists(astra_launch)
    except Exception:
        astra_available = False
        astra_launch = ''

    if astra_available:
        astra_camera = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(astra_launch),
            launch_arguments={
                'camera_name': 'camera',
                'depth_width': '320',
                'depth_height': '240',
                'depth_fps': '15',
                'enable_depth': 'true',
                'color_width': '320',
                'color_height': '240',
                'color_fps': '15',
                'enable_color': 'true',
                'enable_ir': 'false',
                'enable_point_cloud': 'false',
                'color_depth_synchronization': 'false',
                # Let camera publish its full TF tree (optical frames)
                'publish_tf': 'true',
            }.items()
        )
        launch_actions.append(astra_camera)
    else:
        print('WARNING: astra_camera package not found!')
        print('Run: source ~/ros2_ws/install/setup.bash')

    # ========================================
    # 2. Static TF: map → odom
    # ========================================
    # The Orbbec driver publishes camera_link → camera_*_optical_frame itself.
    # rgbd_odometry will publish odom → camera_link.
    # We just need map → odom to close the chain.
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    launch_actions.append(tf_map_odom)

    # ========================================
    # 3a. RGBD Odometry node (visual odometry)
    # ========================================
    # Computes odom → camera_link TF from RGB+Depth stream.
    # This is a SEPARATE node from rtabmap slam — required when no wheel odom.
    rgbd_odometry_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rtabmap_odom',
                executable='rgbd_odometry',
                name='rgbd_odometry',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'frame_id': CAMERA_FRAME,
                    'odom_frame_id': 'odom',
                    'publish_tf': True,
                    'wait_for_transform': 0.2,
                    'approx_sync': True,
                    'approx_sync_max_interval': 0.1,
                    'queue_size': 10,
                    'Reg/Force3DoF': 'true',
                    'Vis/MinInliers': '10',
                    'Vis/EstimationType': '1',
                    'Odom/Strategy': '0',
                    'Odom/ResetCountdown': '1',
                }],
                remappings=[
                    ('rgb/image',       '/camera/color/image_raw'),
                    ('rgb/camera_info', '/camera/color/camera_info'),
                    ('depth/image',     '/camera/depth/image_raw'),
                ],
            )
        ]
    )
    launch_actions.append(rgbd_odometry_node)

    # ========================================
    # 3b. RTAB-Map SLAM node (mapping mode)
    # ========================================
    rtabmap_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'frame_id': CAMERA_FRAME,
                    'map_frame_id': 'map',
                    'odom_frame_id': 'odom',
                    # Use odom from rgbd_odometry node — subscribe to the pre-synced
                    # RGBDImage published by rgbd_odometry to avoid sync issues.
                    'subscribe_odom_info': True,
                    'subscribe_rgbd': True,
                    'subscribe_depth': False,
                    'subscribe_rgb': False,
                    'subscribe_scan': False,
                    # Build new map
                    'database_path': os.path.expanduser('~/.ros/rtabmap.db'),
                    'Mem/IncrementalMemory': 'true',
                    'Reg/Force3DoF': 'true',
                    'Vis/MinInliers': '10',
                    'Kp/MaxFeatures': '500',
                    'RGBD/AngularUpdate': '0.01',
                    'RGBD/LinearUpdate': '0.01',
                    'approx_sync': True,
                    'approx_sync_max_interval': 0.1,
                    'queue_size': 10,
                    'publish_tf': True,
                    'wait_for_transform': 0.2,
                    # 2D occupancy grid (for Nav2) from depth cloud
                    'Grid/FromDepth': 'true',
                    'Grid/CellSize': '0.05',
                    'Grid/RangeMax': '4.0',
                    'Grid/MaxGroundHeight': '0.1',
                    'Grid/MaxObstacleHeight': '1.5',
                    'Grid/NormalsSegmentation': 'true',
                    'Grid/3D': 'false',
                    # Publish the map topics
                    'map_always_update': True,
                    'map_empty_ray_tracing': True,
                }],
                remappings=[
                    # Use the RGBDImage already synced and timestamped by rgbd_odometry
                    ('rgbd_image', '/odom_rgbd_image'),
                    ('odom',       '/odom'),
                ],
                arguments=['--delete_db_on_start'],
            )
        ]
    )
    launch_actions.append(rtabmap_node)

    # ========================================
    # 3. RTAB-Map Visualization
    # ========================================
    rtabmap_viz = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                name='rtabmap_viz',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'frame_id': CAMERA_FRAME,
                    'subscribe_depth': True,
                    'subscribe_rgb': True,
                    'subscribe_odom': True,
                    'approx_sync': True,
                    'queue_size': 10,
                }],
                remappings=[
                    ('rgb/image',       '/camera/color/image_raw'),
                    ('rgb/camera_info', '/camera/color/camera_info'),
                    ('depth/image',     '/camera/depth/image_raw'),
                    ('odom',            '/odom'),
                ],
            )
        ]
    )
    launch_actions.append(rtabmap_viz)

    # ========================================
    # 4. RViz — shows the 2D occupancy grid being built
    # ========================================
    bringup_pkg = get_package_share_directory('fyp_bringup')
    rviz_config = os.path.join(bringup_pkg, 'rviz', 'rtabmap_mapping.rviz')
    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_mapping',
                output='screen',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )
    launch_actions.append(rviz_node)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
    ] + launch_actions)
