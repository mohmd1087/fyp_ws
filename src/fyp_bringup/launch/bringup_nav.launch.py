#!/usr/bin/env python3
"""
Full system bringup launch file for real robot navigation
Launches all components with proper sequencing

Usage:
  SLAM mode (create map):
    ros2 launch fyp_bringup bringup_nav.launch.py mode:=slam

  Navigation mode — AMCL (default):
    ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

  Navigation mode — RTAB-Map (visual localization, no fake scan):
    LOCALIZATION_MODE=rtabmap ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

  RTAB-Map first-time map building:
    LOCALIZATION_MODE=rtabmap RTABMAP_MAPPING=true ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

Arguments:
  mode: 'slam' for mapping, 'nav' for navigation (default: nav)
  map: path to map YAML file (required for nav mode)

Environment Variables:
  LOCALIZATION_MODE: 'amcl' (default) or 'rtabmap'
  RTABMAP_MAPPING:   'true' to build new map (default: false = localize only)
  RTABMAP_MAP:       map filename under src/fyp_bringup/maps/<name>.db
                     (default: rtab_final_demo). Lets you keep multiple maps
                     side-by-side and pick one per launch.
  CONTROLLER:        'rpp' (default) or 'dwb' — local planner selection for rtabmap mode.
                     dwb enables true dynamic obstacle avoidance via trajectory sampling.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    bringup_pkg = get_package_share_directory('fyp_bringup')
    nav2_pkg = get_package_share_directory('fyp_nav2')

    # Localization mode from environment variable (resolved at launch time)
    localization_mode = os.environ.get('LOCALIZATION_MODE', 'amcl').lower()
    use_rtabmap = (localization_mode == 'rtabmap')

    # Controller (local planner) selection — only affects rtabmap param file today
    controller = os.environ.get('CONTROLLER', 'rpp').lower()
    if controller not in ('rpp', 'dwb'):
        raise RuntimeError(
            f"CONTROLLER must be 'rpp' or 'dwb', got '{controller}'"
        )
    use_dwb = (controller == 'dwb')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mode = LaunchConfiguration('mode', default='nav')
    map_yaml = LaunchConfiguration('map')
    planner = LaunchConfiguration('planner', default='dstar')

    # Default map path
    default_map = os.path.join(bringup_pkg, 'maps', 'my_map.yaml')

    # ========================================
    # 1. Hardware Bringup
    # ========================================
    # Robot state publisher, hardware drivers (placeholders)
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'hardware_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ========================================
    # 2. Camera Launch (delayed 2 seconds)
    # ========================================
    # Orbbec Astra Pro + depth to scan
    camera_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'camera.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items()
            )
        ]
    )

    # ========================================
    # 3. EKF Node (delayed 3 seconds)
    # ========================================
    # Fuses wheel odometry + IMU
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf_real.yaml')
    ekf_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config,
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('odometry/filtered', '/odometry/filtered'),
                ]
            )
        ]
    )

    # ========================================
    # 4a. SLAM Mode (delayed 5 seconds)
    # ========================================
    # Launch SLAM Toolbox for mapping
    slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'slam_real.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'slam'"])
                )
            )
        ]
    )

    # ========================================
    # 4b. Navigation Mode (delayed 5 seconds)
    # ========================================
    # Launch localization and navigation stack
    # Localization choice depends on LOCALIZATION_MODE env var:
    #   amcl     → AMCL + map server (default)
    #   rtabmap  → RTAB-Map RGB-D visual localization

    if use_rtabmap:
        # RTAB-Map: visual localization/mapping using RGB-D camera
        # Runs in both 'nav' mode (localization) and 'mapping' mode (map building)
        localization_launch = TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_pkg, 'launch', 'rtabmap.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                    }.items(),
                    condition=IfCondition(
                        PythonExpression(["'", mode, "' in ['nav', 'mapping']"])
                    )
                )
            ]
        )
    else:
        # AMCL: traditional scan-based localization
        localization_launch = TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_pkg, 'launch', 'localization.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'map': map_yaml,
                    }.items(),
                    condition=IfCondition(
                        PythonExpression(["'", mode, "' == 'nav'"])
                    )
                )
            ]
        )

    # Nav2 params file depends on localization mode and controller selection.
    # rtabmap mode supports both RPP (default) and DWB via CONTROLLER env var.
    if use_rtabmap:
        nav2_params_name = 'nav2_params_rtabmap_dwb.yaml' if use_dwb else 'nav2_params_rtabmap.yaml'
    else:
        nav2_params_name = 'nav2_params_real.yaml'
    nav2_params = os.path.join(nav2_pkg, 'params', nav2_params_name)

    # Navigation launch (delayed further for localization to start)
    navigation_launch = TimerAction(
        period=18.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'planner': planner,
                    'params_file': nav2_params,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'nav'"])
                )
            )
        ]
    )

    # ========================================
    # 4c. D* Lite planner node (nav mode only, delayed 9 seconds)
    # ========================================
    # Only launched when planner:=dstar (default).
    # When planner:=navfn this section is skipped and Nav2's own planner_server runs.
    dstar_pkg_dir = get_package_share_directory('fyp_dstar_lite')
    dstar_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(dstar_pkg_dir, 'launch', 'dstar_planner.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", planner, "' == 'dstar' and '", mode, "' == 'nav'"])
                )
            )
        ]
    )

    # ========================================
    # 5. RViz (delayed 6 seconds)
    # ========================================
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2.rviz')
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                # Only launch RViz in nav mode (SLAM launch already includes RViz)
                condition=IfCondition(
                    PythonExpression(["'", mode, "' in ['nav', 'mapping']"])
                )
            )
        ]
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (false for real robot)'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='nav',
            description='Operating mode: nav (navigation), mapping (rtabmap map building, no Nav2), slam (slam_toolbox)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map YAML file (for nav mode)'
        ),
        DeclareLaunchArgument(
            'planner',
            default_value='dstar',
            description='Global planner: "dstar" (D* Lite) or "navfn" (NavfnPlanner/A*)'
        ),

        # Launch components
        hardware_bringup,
        camera_launch,
        ekf_node,
        slam_launch,
        localization_launch,
        navigation_launch,
        dstar_launch,
        rviz_node,
    ])
