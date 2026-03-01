import os

from pexpect import spawn
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    bringup_pkg = get_package_share_directory('fyp_bringup')
    desc_pkg = get_package_share_directory('fyp_description')

    world = os.path.join(bringup_pkg, 'worlds', 'empty.sdf')
    urdf  = os.path.join(desc_pkg, 'urdf', 'new_robot.urdf')

    robot_description = open(urdf).read()

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world}'}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    clock_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    output="screen",
    )


    # Spawn entity from robot_description topic
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'robot', '-topic', 'robot_description']
    )

    joint_state_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60",
        "--switch-timeout", "60",],
    output="screen",
)

    diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_drive_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60",
        "--switch-timeout", "60",],
    output="screen",
)

    return LaunchDescription([
    gz, rsp, spawn,
    TimerAction(period=5.0, actions=[joint_state_spawner]),
    TimerAction(period=7.0, actions=[diff_drive_spawner]),
])
