import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_pkg = get_package_share_directory('fyp_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # SLAM Toolbox node - configured for better drift correction
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'mode': 'mapping',
            # Mapping parameters
            'resolution': 0.05,
            'max_laser_range': 5.5,  # Slightly reduced for better accuracy
            'minimum_time_interval': 0.1,  # More frequent updates for better tracking
            'transform_timeout': 0.5,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            # Scan matching - trust scans MORE than odometry (due to wheel axis config)
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.1,  # Update more frequently
            'minimum_travel_heading': 0.1,   # More sensitive to rotation
            'scan_buffer_size': 20,
            'scan_buffer_maximum_scan_distance': 10.0,
            'link_match_minimum_response_fine': 0.1,
            'link_scan_maximum_distance': 1.5,
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1,
            # Loop closure
            'do_loop_closing': True,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_coarse': 0.35,
            'loop_match_minimum_response_fine': 0.45,
            'loop_search_maximum_distance': 3.0,
            # Ceres solver settings for better optimization
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': 'None',
        }],
    )

    # RViz for visualization
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'slam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox,
        rviz,
    ])