#!/usr/bin/env python3
"""
RTAB-Map RGB-D localization/mapping launch file.

Two modes controlled by RTABMAP_MAPPING env var:
  RTABMAP_MAPPING=true  → build a new map (drive/walk around the environment)
  RTABMAP_MAPPING=false (default) → localize against existing map (~/.ros/rtabmap.db)

Usage (launched from bringup_nav.launch.py via LOCALIZATION_MODE=rtabmap):
  Mapping:    LOCALIZATION_MODE=rtabmap RTABMAP_MAPPING=true  ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav
  Navigation: LOCALIZATION_MODE=rtabmap                       ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

For handheld mapping (no robot needed):
  ros2 launch fyp_bringup rtabmap_handheld_mapping.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    mapping_mode = os.environ.get('RTABMAP_MAPPING', 'false').lower() == 'true'
    mode_str = 'MAPPING (new map)' if mapping_mode else 'LOCALIZATION (existing map)'
    print(f'[rtabmap.launch.py] Starting in {mode_str} mode')

    # ========================================
    # Visual Odometry node (rgbd_odometry)
    # ========================================
    # Computes camera-based odometry from RGB+Depth stream.
    # Published to /vo/odometry — fused with wheel odom in EKF.
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'vo_odom',   # separate frame — EKF fuses it, not TF
            'publish_tf': False,           # EKF owns odom→base_footprint TF
            'wait_for_transform': 0.2,
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
            'queue_size': 10,
            'Reg/Force3DoF': 'true',
            'Vis/MinInliers': '10',
            'Odom/Strategy': '0',
            'Odom/ResetCountdown': '1',
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('odom',            '/vo/odometry'),
        ],
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Frames
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            # Sensor setup — RGB-D, no laser scan
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            # Map database
            'database_path': os.path.expanduser('~/fyp_ws/src/fyp_bringup/maps/rtab_final_demo.db'),
            # Mapping vs localization
            'Mem/IncrementalMemory': 'true' if mapping_mode else 'false',
            'Mem/InitWMWithAllNodes': 'false' if mapping_mode else 'true',
            # 2D robot — lock roll/pitch to prevent 3D drift
            'Reg/Force3DoF': 'true',
            # Visual odometry update thresholds (update on small movements)
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            'RGBD/OptimizeFromGraphEnd': 'false',
            # Feature detection tuned for Astra Pro indoor
            'Vis/MinInliers': '12',
            'Kp/MaxFeatures': '400',
            # Publish map→odom TF so Nav2 can use it
            'publish_tf': True,
            'tf_delay': 0.05,
            'tf_tolerance': 0.1,
            # Sync RGB and depth (Astra Pro needs approx sync)
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'queue_size': 10,
            # 2D occupancy grid from depth (for Nav2 /map topic)
            'Grid/FromDepth': 'true',
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '4.0',
            'Grid/MaxGroundHeight': '0.1',
            'Grid/MaxObstacleHeight': '1.5',
            'Grid/NormalsSegmentation': 'true',
            'Grid/3D': 'false',
            'Grid/RayTracing': 'true',
            'Grid/MinClusterSize': '10',
            'map_always_update': True,       # bool — ros2 param, not rtabmap string param
            'map_empty_ray_tracing': True,   # bool — ros2 param, not rtabmap string param
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('odom',            '/odometry/filtered'),
        ],
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
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
            ('odom',            '/odometry/filtered'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        rgbd_odometry_node,
        rtabmap_node,
        rtabmap_viz_node,
    ])
