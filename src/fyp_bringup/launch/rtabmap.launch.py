#!/usr/bin/env python3
"""
RTAB-Map RGB-D localization/mapping launch file.

Env vars:
  RTABMAP_MAPPING=true          build a new map (drive/walk around the environment)
  RTABMAP_MAPPING=false (default)  localize against existing map
  RTABMAP_MAP=<name> (default: rtab_final_demo)
                                 map filename under src/fyp_bringup/maps/<name>.db

Usage (launched from bringup_nav.launch.py via LOCALIZATION_MODE=rtabmap):
  Build a new map named "finalest_demo":
    LOCALIZATION_MODE=rtabmap RTABMAP_MAPPING=true RTABMAP_MAP=finalest_demo \
      ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

  Navigate using that map:
    LOCALIZATION_MODE=rtabmap RTABMAP_MAP=finalest_demo \
      ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

  Navigate using the original map (default):
    LOCALIZATION_MODE=rtabmap ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

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

    # Map name is configurable so you can keep multiple maps side-by-side.
    # Stored under src/fyp_bringup/maps/<RTABMAP_MAP>.db
    map_name = os.environ.get('RTABMAP_MAP', 'rtab_final_demo')
    db_path = os.path.expanduser(
        f'~/fyp_ws/src/fyp_bringup/maps/{map_name}.db'
    )
    print(f'[rtabmap.launch.py] Starting in {mode_str} mode — db: {db_path}')

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
            'odom_frame_id': 'vo_odom',
            'publish_tf': False,
            'wait_for_transform': 0.2,
            'approx_sync': True,
            # Tightened: was 0.05 → log showed RGB/depth drift up to 0.038s being
            # accepted. 0.02 rejects spurious bad sync pairs that pollute VO graph.
            'approx_sync_max_interval': 0.02,
            # Was 10 → reduced so old frames get dropped instead of queued. Long
            # queue caused publish delay to climb to 180ms (2-3× the actual
            # processing time), which lagged the EKF correction behind wheel odom
            # and contributed to motion jerks during shim/DWB transitions.
            'queue_size': 2,
            'Reg/Force3DoF': 'true',
            'Vis/MinInliers': '10',
            'Odom/Strategy': '0',
            'Odom/ResetCountdown': '1',
            # Skip frames during fast motion — prevents bad feature matches when
            # the camera rotates quickly during shim spin or DWB heading change
            'Odom/ImageDecimation': '2',
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
            'database_path': db_path,
            # Mapping vs localization
            'Mem/IncrementalMemory': 'true' if mapping_mode else 'false',
            'Mem/InitWMWithAllNodes': 'false' if mapping_mode else 'true',
            # 2D robot — lock roll/pitch to prevent 3D drift
            'Reg/Force3DoF': 'true',
            # Visual odometry update thresholds. Raised from 0.01 → only accept
            # new keyframes after meaningful motion. Prevents the graph from
            # ingesting bad frames captured during fast transitions (shim → DWB
            # handoff, RotateToGoal phase) where motion blur and RGB/depth
            # de-sync produce low-quality matches and pollute localization.
            'RGBD/AngularUpdate': '0.05',     # ~3°
            'RGBD/LinearUpdate': '0.03',      # 3 cm
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
