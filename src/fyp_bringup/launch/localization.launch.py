#!/usr/bin/env python3
"""
Localization launch file using AMCL with saved map
Uses Adaptive Monte Carlo Localization for robot pose estimation

Usage:
ros2 launch fyp_bringup localization.launch.py map:=/path/to/map.yaml

Prerequisites:
- Map file must exist (created with SLAM)
- Hardware must be running (wheel odometry, IMU)
- Camera must be running (depth to scan conversion)
- EKF must be running (fused odometry)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    nav2_pkg = get_package_share_directory('fyp_nav2')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart', default='true')

    # Default map path
    default_map = os.path.join(nav2_pkg, 'maps', 'maze.yaml')

    # ========================================
    # Map Server
    # ========================================
    # Loads the pre-built map from YAML file
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
            'topic_name': 'map',
            'frame_id': 'map',
        }]
    )

    # ========================================
    # AMCL - Adaptive Monte Carlo Localization
    # ========================================
    # Tuned for skid-steer robot with depth camera
    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # Robot configuration
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',

            # Transform tolerance
            'transform_tolerance': 1.0,

            # Particle filter parameters
            'min_particles': 500,
            'max_particles': 2000,
            'pf_err': 0.05,
            'pf_z': 0.99,

            # Initial pose (can be set via RViz or topic)
            'set_initial_pose': False,
            'initial_pose': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'yaw': 0.0,
            },

            # Motion model parameters (tuned for skid-steer)
            # Higher values = more noise = less trust in odometry
            'alpha1': 0.2,   # Rotation from rotation
            'alpha2': 0.2,   # Rotation from translation
            'alpha3': 0.2,   # Translation from translation
            'alpha4': 0.2,   # Translation from rotation
            'alpha5': 0.2,   # Additional translation noise

            # Laser model parameters
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 8.0,      # Astra Pro max range
            'laser_min_range': 0.6,      # Astra Pro min range
            'max_beams': 60,             # Number of beams to use

            # Update parameters
            'update_min_d': 0.1,         # Min distance to trigger update
            'update_min_a': 0.1,         # Min rotation to trigger update (rad)

            # Resample parameters
            'resample_interval': 1,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,

            # Map parameters
            'do_beamskip': False,
            'beam_skip_distance': 0.5,
            'beam_skip_threshold': 0.3,
            'beam_skip_error_threshold': 0.9,

            # Likelihood field parameters
            'z_hit': 0.5,
            'z_short': 0.05,
            'z_max': 0.05,
            'z_rand': 0.5,
            'sigma_hit': 0.2,
            'lambda_short': 0.1,

            # TF broadcast
            'tf_broadcast': True,

            # First map only mode (don't update on map changes)
            'first_map_only': False,

            # Always reset filter on initial pose
            'always_reset_initial_pose': False,

            # Scan topic
            'scan_topic': '/scan',
        }]
    )

    # ========================================
    # Lifecycle Manager
    # ========================================
    # Manages lifecycle of map_server and amcl
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
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
            'map',
            default_value=default_map,
            description='Full path to map YAML file'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),
        map_server,
        amcl,
        lifecycle_manager,
    ])
