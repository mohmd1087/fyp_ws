import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Unified launch file that starts Gazebo simulation and Nav2 together
    with proper clock synchronization
    """

    bringup_pkg = get_package_share_directory('fyp_bringup')

    # 1) Launch simulation (Gazebo + robot + controllers)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'sim_with_cmd.launch.py')
        )
    )

    # 2) Delay Nav2 launch to ensure clock and all sensors are ready
    nav2_launch = TimerAction(
        period=15.0,  # Wait 15s for full sim startup
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('fyp_nav2'),
                        'launch',
                        'nav2_map.launch.py'
                    ])
                )
            )
        ]
    )

    return LaunchDescription([
        sim_launch,
        nav2_launch
    ])
