import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_pkg = get_package_share_directory('fyp_bringup')
    desc_pkg    = get_package_share_directory('fyp_description')

    # Set Gazebo plugin paths so it can find gz_ros2_control
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    # Launch argument for world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_pkg, 'worlds', 'map_world_gz.sdf'),
        description='Path to the Gazebo world file'
    )
    world = LaunchConfiguration('world')
    urdf  = os.path.join(desc_pkg, 'urdf', 'new_robot.urdf')

    robot_description = open(urdf).read()

    # 1) Launch Gazebo Sim
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # 2) Publish robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
    )

    # 3) Delay spawn until Gazebo and robot_description are ready
    # Spawn in the vertical corridor opening (x=16, y=1.5)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'my_robot',
                    '-x', '16.0',
                    '-y', '1.5',
                    '-z', '0.05',
                    '-Y', '3.1416'
                ],
            ),
        ]
    )

    # 4) Delay clock bridge after spawn
    clock_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                parameters=[{'use_sim_time': True}],
                output="screen",
            )
        ]
    )

    # 5) Delay controller spawners (wait for gz_ros2_control plugin to initialize)
    joint_state_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "120",
                    "--switch-timeout", "60",
                ],
                output="screen",
            )
        ],
    )

    diff_drive_spawner = TimerAction(
        period=12.0,
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

    # 6) Bridge depth camera and IMU topics from Gazebo to ROS 2
    depth_bridge = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                ],
                parameters=[{'use_sim_time': True}],
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
                    "output_frame": "scan_frame",  # Aligned with base_footprint for correct obstacle projection
                    "range_min": 0.2,
                    "range_max": 6.0,
                    "scan_height": 30,  # Increased from 10 for better scan quality
                    "scan_time": 0.033,  # Match 30Hz update rate
                    "angle_min": -1.57,  # -90 degrees
                    "angle_max": 1.57,   # +90 degrees
                    "angle_increment": 0.0087,  # ~0.5 degree resolution
                    "use_sim_time": True,
                }],
                output="screen",
            )
        ]
    )

    # 8) Cmd_vel relay: forward /cmd_vel to /diff_drive_controller/cmd_vel
    # Used by both Nav2 and teleop_twist_keyboard
    cmd_vel_relay = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='cmd_vel_relay',
                arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'],
                parameters=[{'use_sim_time': True}],
                output="screen",
            )
        ]
    )

    # 9) Odom relay: republish /diff_drive_controller/odom to /odom
    # Nav2/RViz need odometry on /odom; diff_drive_controller publishes on ~/odom
    # (i.e., /diff_drive_controller/odom), so we relay it to the standard /odom topic.
    odom_relay = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='odom_relay',
                arguments=['/diff_drive_controller/odom', '/odom'],
                parameters=[{'use_sim_time': True}],
                output="screen",
            )
        ]
    )

    # 10) Pose sync - DISABLED: conflicts with AMCL localization
    # pose_sync = Node(
    #     package='fyp_bringup',
    #     executable='pose_sync.py',
    #     name='pose_sync',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'robot_name': 'my_robot',
    #         'world_name': 'default'
    #     }],
    #     output='screen',
    # )

    # 11) Robot localization EKF - fuses wheel odom + IMU for smooth odometry
    # Publishes odom->base_footprint TF (diff_drive_controller TF disabled)
    ekf_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    os.path.join(bringup_pkg, 'config', 'ekf.yaml'),
                    {'use_sim_time': True}
                ],
            )
        ]
    )

    return LaunchDescription([
        gz_plugin_path,  # Set plugin path before launching Gazebo
        world_arg,
        gz,
        rsp,
        spawn_robot,
        clock_bridge,
        joint_state_spawner,
        diff_drive_spawner,
        depth_bridge,
        depth_to_scan,
        cmd_vel_relay,
        odom_relay,
        # ekf_node,      # DISABLED FOR TESTING: Check if raw odom has the bug
        # pose_sync,  # DISABLED: conflicts with AMCL localization
    ])
