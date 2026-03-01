#!/usr/bin/env python3
"""
Launch file to spawn custom robot in imported maze world
Uses existing map for navigation (no SLAM)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    bringup_pkg = get_package_share_directory('fyp_bringup')
    description_pkg = get_package_share_directory('fyp_description')

    # Set Gazebo model path to find maze model
    gazebo_model_path = os.path.join(bringup_pkg, 'worlds')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + gazebo_model_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gazebo_model_path

    # Maze world file
    maze_world = os.path.join(bringup_pkg, 'worlds', 'maze_world.sdf')

    # Robot URDF
    urdf_file = os.path.join(description_pkg, 'urdf', 'new_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1) Gazebo with maze world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r {maze_world}'
        }.items()
    )

    # 2) Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 3) Joint State Broadcaster (publish /joint_states)
    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
            )
        ]
    )

    # 4) Spawn robot in Gazebo at maze center (within map bounds)
    # Map origin is [-8.57, -9.22], so [0, 0] should be valid
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'fyp_robot',
                    '-topic', '/robot_description',
                    '-x', '1.0',   # Slightly offset to ensure in free space
                    '-y', '1.0',
                    '-z', '0.1',
                    '-Y', '0.0'
                ],
                output='screen',
            )
        ]
    )

    # 5) Diff Drive Controller
    diff_drive_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diff_drive_controller",
                    "--controller-manager", "/controller_manager",
                    "--param-file", os.path.join(bringup_pkg, 'config', 'controllers.yaml'),
                    "--controller-manager-timeout", "120",
                    "--switch-timeout", "60",
                ],
                output="screen",
            )
        ],
    )

    # 6) Bridge: depth camera + IMU from Gazebo
    depth_bridge = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                ],
                output="screen",
            )
        ]
    )

    # 7) Convert depth image to laser scan
    depth_to_scan = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="depthimage_to_laserscan",
                executable="depthimage_to_laserscan_node",
                name="depth_to_scan",
                remappings=[
                    ("depth", "/camera/depth"),
                    ("depth_camera_info", "/camera/camera_info"),
                    ("scan", "/scan"),
                ],
                parameters=[{
                    "output_frame": "scan_frame",
                    "range_min": 0.2,
                    "range_max": 6.0,
                    "scan_height": 30,
                    "scan_time": 0.033,
                    "angle_min": -1.57,
                    "angle_max": 1.57,
                    "angle_increment": 0.0087,
                    "use_sim_time": True,
                }],
                output="screen",
            )
        ]
    )

    # 8) Cmd_vel relay
    cmd_vel_relay = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='cmd_vel_relay',
                arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel'],
                output='screen',
            )
        ]
    )

    # 9) Odometry relay (diff_drive publishes to /diff_drive_controller/odom)
    odom_relay = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='odom_relay',
                arguments=['/diff_drive_controller/odom', '/odom'],
                output='screen',
            )
        ]
    )

    # 10) EKF for odometry fusion (currently disabled for testing)
    # ekf_node = TimerAction(
    #     period=14.0,
    #     actions=[
    #         Node(
    #             package='robot_localization',
    #             executable='ekf_node',
    #             name='ekf_filter_node',
    #             output='screen',
    #             parameters=[
    #                 os.path.join(bringup_pkg, 'config', 'ekf.yaml'),
    #                 {'use_sim_time': True}
    #             ],
    #         )
    #     ]
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        diff_drive_spawner,
        depth_bridge,
        depth_to_scan,
        cmd_vel_relay,
        odom_relay,
        # ekf_node,  # Disabled for testing
    ])