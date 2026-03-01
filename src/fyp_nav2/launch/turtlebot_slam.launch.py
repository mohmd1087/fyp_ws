import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    bringup_pkg = get_package_share_directory('fyp_bringup')
    nav2_pkg = get_package_share_directory('fyp_nav2')

    # Set TurtleBot3 model (burger is smallest)
    turtlebot_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Set Gazebo resource path to find TurtleBot3 models
    turtlebot3_gazebo_models = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models'
    )
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        turtlebot3_gazebo_models
    )

    # World file
    world_file = os.path.join(bringup_pkg, 'worlds', 'map_world_gz.sdf')

    # Get TurtleBot3 SDF model path
    turtlebot3_sdf = os.path.join(
        turtlebot3_gazebo_models,
        'turtlebot3_burger',
        'model.sdf'
    )

    # Launch Gazebo with your world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    # Load TurtleBot3 URDF for robot_state_publisher
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf'
    )
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Robot state publisher - publishes TF and /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Spawn TurtleBot3 using SDF file (delayed to ensure Gazebo is ready)
    spawn_turtlebot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'turtlebot3_burger',
                    '-file', turtlebot3_sdf,
                    '-x', '16.0',
                    '-y', '1.5',
                    '-z', '0.01',
                    '-Y', '3.1416'
                ],
                output='screen',
            )
        ]
    )

    # Bridge for TurtleBot topics - essential for robot operation
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # SLAM Toolbox
    slam_params = {
        'use_sim_time': True,
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_footprint',
        'scan_topic': '/scan',
        'mode': 'mapping',
        'resolution': 0.05,
        'max_laser_range': 3.5,
        'minimum_time_interval': 0.2,
        'transform_timeout': 0.5,
        'tf_buffer_duration': 30.0,
        'stack_size_to_use': 40000000,
        'use_scan_matching': True,
        'use_scan_barycenter': True,
        'do_loop_closing': True,
        'loop_match_minimum_chain_size': 2,
        'loop_match_maximum_variance_coarse': 3.0,
        'loop_match_minimum_response_coarse': 0.3,
        'loop_match_minimum_response_fine': 0.4,
        'loop_search_maximum_distance': 3.0,
    }

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        turtlebot_model,
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_turtlebot,
        bridge,
        slam_toolbox,
        rviz,
    ])