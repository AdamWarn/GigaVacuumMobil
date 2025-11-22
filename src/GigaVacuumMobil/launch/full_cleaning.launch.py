#!/usr/bin/env python3
"""Bring up Cartographer SLAM, Nav2, map maintenance, and the cleaning planner."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('GigaVacuumMobil')
    cleaning_share = get_package_share_directory('cleaning_planner')

    default_map_dir = str(Path.home() / 'GigaVacuumMobil_maps')
    default_map_yaml = os.path.join(default_map_dir, 'current_map.yaml')

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_slam', default_value='true'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('filtered_map_topic', default_value='/stable_map'),
        DeclareLaunchArgument('map_yaml', default_value=default_map_yaml),
        DeclareLaunchArgument('map_save_directory', default_value=default_map_dir),
        DeclareLaunchArgument('map_save_period', default_value='60.0'),
        DeclareLaunchArgument('map_keep_history', default_value='true'),
        DeclareLaunchArgument('publish_filtered_map', default_value='true'),
        DeclareLaunchArgument('wall_persistence_observations', default_value='12'),
        DeclareLaunchArgument('obstacle_dilation_radius', default_value='0.08'),
        DeclareLaunchArgument('workspace_bounds', default_value=''),
        DeclareLaunchArgument('wall_gap_fill_enabled', default_value='true'),
        DeclareLaunchArgument('wall_gap_fill_max_width', default_value='0.5'),
        DeclareLaunchArgument('free_decay_step', default_value='1'),
        DeclareLaunchArgument('poses_per_goal', default_value='8'),
        DeclareLaunchArgument('row_spacing', default_value='0.4'),
        DeclareLaunchArgument('safety_margin', default_value='0.05'),
        DeclareLaunchArgument('boundary_laps', default_value='1'),
        DeclareLaunchArgument('inflation_size', default_value='3'),
        DeclareLaunchArgument('orientation_mode', default_value='heading'),
        DeclareLaunchArgument('map_update_min_interval', default_value='10.0'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('exploration_enabled', default_value='true'),
        DeclareLaunchArgument('scan_topic_raw', default_value='/scan'),
        DeclareLaunchArgument('scan_topic_synced', default_value='/scan_synced'),
        DeclareLaunchArgument('scan_sync_max_skew', default_value='5.0'),
        DeclareLaunchArgument('odom_source_topic', default_value='/odom'),
        DeclareLaunchArgument('odom_pose_topic', default_value='/odom_pose'),
        DeclareLaunchArgument('exploration_edge_band', default_value='0.3'),
        DeclareLaunchArgument('exploration_min_edge_fraction', default_value='0.7'),
        DeclareLaunchArgument('exploration_max_unknown_ratio', default_value='0.18'),
        DeclareLaunchArgument('exploration_frontier_clearance', default_value='0.25'),
        DeclareLaunchArgument('exploration_goal_timeout', default_value='120.0'),
        DeclareLaunchArgument('exploration_behavior_tree', default_value=''),
        DeclareLaunchArgument('coverage_enable_topic', default_value='/coverage_enable'),
        DeclareLaunchArgument('pattern_auto_mode', default_value='false'),
        DeclareLaunchArgument('pattern_auto_start_delay', default_value='5.0'),
        DeclareLaunchArgument('pattern_sequence', default_value='["exploration","systematic_cleaning"]'),
        DeclareLaunchArgument('pattern_command_topic', default_value='/pattern_manager/command'),
        DeclareLaunchArgument('pattern_status_topic', default_value='/pattern_manager/status'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_slam = LaunchConfiguration('enable_slam')
    map_topic = LaunchConfiguration('map_topic')
    filtered_map_topic = LaunchConfiguration('filtered_map_topic')
    map_yaml = LaunchConfiguration('map_yaml')
    scan_topic_raw = LaunchConfiguration('scan_topic_raw')
    scan_topic_synced = LaunchConfiguration('scan_topic_synced')
    odom_source_topic = LaunchConfiguration('odom_source_topic')
    odom_pose_topic = LaunchConfiguration('odom_pose_topic')
    map_save_dir = LaunchConfiguration('map_save_directory')
    map_save_period = LaunchConfiguration('map_save_period')
    map_keep_history = LaunchConfiguration('map_keep_history')
    poses_per_goal = LaunchConfiguration('poses_per_goal')
    workspace_bounds = LaunchConfiguration('workspace_bounds')
    wall_gap_fill_enabled = LaunchConfiguration('wall_gap_fill_enabled')
    wall_gap_fill_max_width = LaunchConfiguration('wall_gap_fill_max_width')
    free_decay_step = LaunchConfiguration('free_decay_step')
    coverage_enable_topic = LaunchConfiguration('coverage_enable_topic')
    exploration_enabled = LaunchConfiguration('exploration_enabled')

    odom_pose_bridge = Node(
        package='GigaVacuumMobil',
        executable='odom_to_pose.py',
        name='odom_pose_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': odom_source_topic,
            'output_topic': odom_pose_topic,
        }]
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam.launch.py')),
        condition=IfCondition(enable_slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': LaunchConfiguration('launch_rviz'),
            'scan_topic': scan_topic_synced,
        }.items()
    )

    map_updater = Node(
        package='map_maintenance',
        executable='map_updater_node',
        name='map_updater',
        output='screen',
        parameters=[{
            'map_topic': map_topic,
            'save_directory': map_save_dir,
            'base_map_name': 'live_map',
            'symlink_name': 'current_map.yaml',
            'save_period': map_save_period,
            'keep_history': map_keep_history,
            'filtered_map_topic': filtered_map_topic,
            'publish_filtered_map': LaunchConfiguration('publish_filtered_map'),
            'wall_persistence_observations': LaunchConfiguration('wall_persistence_observations'),
            'obstacle_dilation_radius': LaunchConfiguration('obstacle_dilation_radius'),
            'workspace_bounds': workspace_bounds,
            'wall_gap_fill_enabled': wall_gap_fill_enabled,
            'wall_gap_fill_max_width': wall_gap_fill_max_width,
            'free_decay_step': free_decay_step,
        }]
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation_simple.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': LaunchConfiguration('launch_rviz'),
            'use_slam': enable_slam,
            'map_yaml': map_yaml,
            'static_map_topic': filtered_map_topic,
        }.items()
    )

    scan_sync = Node(
        package='GigaVacuumMobil',
        executable='scan_timestamp_sync.py',
        name='scan_timestamp_sync',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': scan_topic_raw,
            'output_topic': scan_topic_synced,
            'max_skew_sec': LaunchConfiguration('scan_sync_max_skew'),
        }]
    )

    coverage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cleaning_share, 'launch', 'coverage_pipeline.launch.py')),
        launch_arguments={
            'map_file': map_yaml,
            'map_topic': filtered_map_topic,
            'use_sim_time': use_sim_time,
            'poses_per_goal': poses_per_goal,
            'row_spacing': LaunchConfiguration('row_spacing'),
            'safety_margin': LaunchConfiguration('safety_margin'),
            'boundary_laps': LaunchConfiguration('boundary_laps'),
            'inflation_size': LaunchConfiguration('inflation_size'),
            'orientation_mode': LaunchConfiguration('orientation_mode'),
            'map_update_min_interval': LaunchConfiguration('map_update_min_interval'),
            'auto_start': 'false',
            'enable_topic': coverage_enable_topic,
        }.items()
    )

    patterns_node = Node(
        package='patterns',
        executable='pattern_manager',
        name='pattern_manager',
        output='screen',
        condition=IfCondition(exploration_enabled),
        parameters=[{
            'auto_mode': LaunchConfiguration('pattern_auto_mode'),
            'auto_start_delay': LaunchConfiguration('pattern_auto_start_delay'),
            'pattern_sequence': LaunchConfiguration('pattern_sequence'),
            'command_topic': LaunchConfiguration('pattern_command_topic'),
            'status_topic': LaunchConfiguration('pattern_status_topic'),
            'exploration.map_topic': filtered_map_topic,
            'exploration.pose_topic': odom_pose_topic,
            'exploration.navigate_action': 'navigate_to_pose',
            'exploration.map_ready_topic': '/map_ready',
            'exploration.edge_band_width': LaunchConfiguration('exploration_edge_band'),
            'exploration.min_edge_fraction': LaunchConfiguration('exploration_min_edge_fraction'),
            'exploration.max_unknown_ratio': LaunchConfiguration('exploration_max_unknown_ratio'),
            'exploration.fallback_unknown_ratio': 0.3,
            'exploration.frontier_clearance': LaunchConfiguration('exploration_frontier_clearance'),
            'exploration.frontier_retry_limit': 5,
            'exploration.goal_timeout': LaunchConfiguration('exploration_goal_timeout'),
            'exploration.analysis_period': 3.0,
            'exploration.occupied_threshold': 0.6,
            'exploration.free_threshold': 0.3,
            'exploration.max_goal_distance': 10.0,
            'exploration.behavior_tree': LaunchConfiguration('exploration_behavior_tree'),
            'cleaning.start_service': '/coverage_executor/start_coverage',
            'cleaning.enable_topic': coverage_enable_topic,
            'cleaning.auto_finish': True,
        }]
    )

    delayed_nav = TimerAction(period=10.0, actions=[nav_launch])
    delayed_coverage = TimerAction(period=20.0, actions=[coverage_launch])
    delayed_patterns = TimerAction(period=15.0, actions=[patterns_node])

    ld = LaunchDescription(declared_arguments)
    ld.add_action(GroupAction([odom_pose_bridge, scan_sync, slam_launch, map_updater, delayed_nav, delayed_patterns, delayed_coverage]))
    return ld
