#!/usr/bin/env python3
"""
Navigation launch file for full Nav2 stack.
Launches all navigation components: planner, controller, behaviors, etc.

Usage:
  ros2 launch fyp_bringup navigation.launch.py                   # D* Lite (default)
  ros2 launch fyp_bringup navigation.launch.py planner:=navfn   # NavfnPlanner (A*)
  ros2 launch fyp_bringup navigation.launch.py planner:=dstar   # D* Lite (explicit)

Prerequisites:
- Localization must be running (AMCL with map)
- Hardware must be running (odometry, sensors)
- Camera must be running (for obstacle detection)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


def _create_nav_nodes(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    # ========================================
    # Controller Server
    # ========================================
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
        ],
    )

    # ========================================
    # Behavior Server
    # ========================================
    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ========================================
    # BT Navigator
    # ========================================
    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ========================================
    # Waypoint Follower
    # ========================================
    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ========================================
    # Velocity Smoother
    # ========================================
    velocity_smoother = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel'),  # publish directly to robot — obstacle_gate removed
        ],
    )

    # ========================================
    # Planner Server (always launched)
    # ========================================
    # In D* Lite mode, planner_server still provides the global_costmap
    # (/global_costmap/costmap) that D* Lite subscribes to.
    # The ComputePathToPose action is handled by dstar_planner_node instead.
    # In navfn mode, planner_server also handles ComputePathToPose directly.
    # When planner:=dstar, remap planner_server's ComputePathToPose action
    # so it doesn't shadow dstar_planner_node on the real action name.
    planner_arg = context.launch_configurations.get('planner', 'dstar')
    planner_remaps = []
    if planner_arg == 'dstar':
        planner_remaps = [
            ('compute_path_to_pose', 'compute_path_to_pose_navfn'),
            ('compute_path_through_poses', 'compute_path_through_poses_navfn'),
        ]
    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=planner_remaps,
    )
    smoother_server = LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_node_names = [
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # ========================================
    # Lifecycle Manager
    # ========================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_node_names,
            'bond_timeout': 4.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0,
        }],
    )

    return [
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ]


def generate_launch_description():
    nav2_pkg = get_package_share_directory('fyp_nav2')
    default_params = os.path.join(nav2_pkg, 'params', 'nav2_params_real.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'planner',
            default_value='dstar',
            description='Global planner to use: "dstar" (D* Lite) or "navfn" (NavfnPlanner/A*)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to Nav2 params file',
        ),
        OpaqueFunction(function=_create_nav_nodes),
    ])
