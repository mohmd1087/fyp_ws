#!/usr/bin/env python3
"""
Gazebo simulation + SLAM launch.
Opens the Gazebo world, spawns the robot, and runs SLAM Toolbox so you
can drive the robot and build a map visible live in RViz.

Startup sequence:
  t=0s   Gazebo (map_world_gz.sdf) + Robot State Publisher
  t=3s   Spawn robot at (-3.0, 2.0)
  t=4s   ROS-Gazebo bridges (clock, odom, TF, IMU, camera, cmd_vel, joint_states)
  t=6s   depth → laser scan
  t=7s   EKF  (/wheel/odometry + /imu/data → /odometry/filtered)
  t=9s   SLAM Toolbox (async mapping, publishes /map live)
  t=11s  RViz

Drive the robot:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save the map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/fyp_ws/src/fyp_bringup/maps/my_map
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('fyp_bringup')
    description_pkg = get_package_share_directory('fyp_description')
    nav2_pkg = get_package_share_directory('fyp_nav2')

    world_file  = os.path.join(bringup_pkg, 'worlds', 'map_world_gz.sdf')
    sim_urdf    = os.path.join(description_pkg, 'urdf', 'new_robot_sim.urdf')
    ekf_config  = os.path.join(bringup_pkg, 'config', 'ekf_real.yaml')
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2.rviz')

    with open(sim_urdf, 'r') as f:
        robot_description = f.read()

    # ========================================
    # t=0s  Gazebo
    # ========================================
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(bringup_pkg, 'worlds')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # ========================================
    # t=0s  Robot State Publisher
    # ========================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ========================================
    # t=3s  Spawn robot in free corridor
    # ========================================
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'fyp_robot',
                    '-topic', '/robot_description',
                    '-x', '-3.0',
                    '-y', '2.0',
                    '-z', '0.1',
                    '-Y', '0.0',
                ],
                output='screen',
            )
        ]
    )

    # ========================================
    # t=4s  ROS-Gazebo bridges
    # ========================================
    bridges = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_bridges',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/wheel/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                ],
                output='screen',
            )
        ]
    )

    # ========================================
    # t=6s  Depth image to laser scan
    # ========================================
    depth_to_scan = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='depthimage_to_laserscan',
                executable='depthimage_to_laserscan_node',
                name='depth_to_scan',
                remappings=[
                    ('depth', '/camera/depth'),
                    ('depth_camera_info', '/camera/camera_info'),
                    ('scan', '/scan'),
                ],
                parameters=[{
                    'output_frame': 'scan_frame',
                    'range_min': 0.3,
                    'range_max': 6.0,
                    'scan_height': 30,
                    'scan_time': 0.033,
                    'use_sim_time': True,
                }],
                output='screen',
            )
        ]
    )

    # ========================================
    # t=7s  EKF
    # ========================================
    ekf_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config,
                    {'use_sim_time': True},
                ],
            )
        ]
    )

    # ========================================
    # t=9s  SLAM Toolbox (async mapping)
    # Publishes /map live as robot explores
    # ========================================
    slam_toolbox = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'mode': 'mapping',

                    # Solver
                    'solver_plugin': 'solver_plugins::CeresSolver',
                    'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                    'ceres_preconditioner': 'SCHUR_JACOBI',
                    'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
                    'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
                    'ceres_loss_function': 'None',

                    # Frames
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'base_frame': 'base_footprint',
                    'scan_topic': '/scan',

                    # Map
                    'resolution': 0.05,
                    'max_laser_range': 6.0,
                    'minimum_time_interval': 0.5,
                    'transform_timeout': 0.5,
                    'tf_buffer_duration': 30.0,
                    'map_update_interval': 2.0,

                    # Scan matching
                    'use_scan_matching': True,
                    'use_scan_barycenter': True,
                    'minimum_travel_distance': 0.2,
                    'minimum_travel_heading': 0.2,

                    # Correlation
                    'correlation_search_space_dimension': 0.5,
                    'correlation_search_space_resolution': 0.01,
                    'correlation_search_space_smear_deviation': 0.1,

                    # Loop closure
                    'loop_search_space_dimension': 8.0,
                    'loop_search_space_resolution': 0.05,
                    'loop_search_space_smear_deviation': 0.03,

                    # Scan matcher
                    'distance_variance_penalty': 0.5,
                    'angle_variance_penalty': 1.0,
                    'fine_search_angle_offset': 0.00349,
                    'coarse_search_angle_offset': 0.349,
                    'coarse_angle_resolution': 0.0349,
                    'minimum_angle_penalty': 0.9,
                    'minimum_distance_penalty': 0.5,
                    'use_response_expansion': True,

                    'stack_size_to_use': 40000000,
                    'enable_interactive_mode': True,
                    'debug_logging': False,
                    'throttle_scans': 1,
                }],
            )
        ]
    )

    # ========================================
    # t=11s  RViz
    # ========================================
    rviz = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridges,
        depth_to_scan,
        ekf_node,
        slam_toolbox,
        rviz,
    ])
