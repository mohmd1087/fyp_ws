#!/usr/bin/env python3
"""
Gazebo simulation + Nav2 navigation launch using my_map world.

The Gazebo world (map_world_gz.sdf) exactly matches the Nav2 map (my_map.yaml):
  - Same walls and obstacles
  - Same coordinate frame (origin -5,-5)
So RViz and Gazebo show the same environment.

Topic flow (mirrors real robot):
  Gazebo DiffDrive  →  /wheel/odometry  +  /tf (odom→base_footprint)
  Gazebo IMU        →  /imu/data
  Gazebo depth cam  →  /camera/depth  +  /camera/camera_info
  depthimage_to_laserscan  →  /scan
  EKF (ekf_real.yaml)      →  /odometry/filtered
  map_server + AMCL        →  /map  +  map→odom TF
  Nav2               reads /odometry/filtered + /map, sends /cmd_vel

Robot spawns at (1.0, 1.5) in Gazebo (free space in the map).
Set the same initial pose in RViz (2D Pose Estimate) if AMCL needs help.

Usage:
  ros2 launch fyp_bringup sim_nav.launch.py                   # D* Lite (default)
  ros2 launch fyp_bringup sim_nav.launch.py planner:=navfn   # NavfnPlanner / A*
  ros2 launch fyp_bringup sim_nav.launch.py planner:=dstar   # D* Lite (explicit)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _conditional_planner_launch(context, *args, **kwargs):
    """Return the D* Lite node launch action only when planner:=dstar."""
    planner = LaunchConfiguration('planner').perform(context)

    if planner.lower() != 'dstar':
        return []

    dstar_pkg = get_package_share_directory('fyp_dstar_lite')

    return [
        TimerAction(
            period=11.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(dstar_pkg, 'launch', 'dstar_planner.launch.py')
                    ]),
                    launch_arguments={'use_sim_time': 'true'}.items(),
                )
            ],
        )
    ]


def generate_launch_description():
    bringup_pkg = get_package_share_directory('fyp_bringup')
    description_pkg = get_package_share_directory('fyp_description')
    nav2_pkg = get_package_share_directory('fyp_nav2')

    world_file  = os.path.join(bringup_pkg, 'worlds', 'map_world_gz.sdf')
    sim_urdf    = os.path.join(description_pkg, 'urdf', 'new_robot_sim.urdf')
    ekf_config  = os.path.join(bringup_pkg, 'config', 'ekf_real.yaml')
    nav2_params = os.path.join(nav2_pkg, 'params', 'nav2_params_sim.yaml')
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2.rviz')
    map_file    = os.path.join(bringup_pkg, 'maps', 'my_map.yaml')

    with open(sim_urdf, 'r') as f:
        robot_description = f.read()

    # ========================================
    # t=0s  Gazebo with map_world_gz.sdf
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
    # t=3s  Spawn robot
    # Inside saved map bounds (origin 0.14,-9.79, 21.3m x 12.05m)
    # (1.0, 1.5) is in the top free corridor, well inside the map
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
                    '-x', '1.0',
                    '-y', '1.5',
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
    # t=7s  EKF — reuses ekf_real.yaml
    # Fuses /wheel/odometry (vx) + /imu/data (vyaw)
    # Publishes /odometry/filtered + odom→base_footprint TF
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
    # t=8s  Localization: map_server + AMCL
    # Reuses localization.launch.py with sim_time + my_map.yaml
    # AMCL publishes map→odom TF so Nav2 knows where the robot is on the map
    # Initial pose matches spawn position (1.0, 1.5)
    # ========================================
    localization = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(bringup_pkg, 'launch', 'localization.launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'map': map_file,
                }.items()
            )
        ]
    )

    # ========================================
    # t=11s  Nav2
    # planner arg forwarded so navigation.launch.py knows which planner to start
    # ========================================
    navigation = TimerAction(
        period=11.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params,
                    'planner': LaunchConfiguration('planner'),
                }.items()
            )
        ]
    )

    # ========================================
    # t=13s  RViz
    # ========================================
    rviz = TimerAction(
        period=13.0,
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
        DeclareLaunchArgument(
            'planner',
            default_value='dstar',
            description='Global planner: "dstar" (D* Lite) or "navfn" (NavfnPlanner/A*)',
        ),
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridges,
        depth_to_scan,
        ekf_node,
        localization,
        navigation,
        # Conditionally launch D* Lite node (only when planner:=dstar)
        OpaqueFunction(function=_conditional_planner_launch),
        rviz,
    ])
