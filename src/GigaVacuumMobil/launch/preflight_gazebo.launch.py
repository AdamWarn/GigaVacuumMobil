"""Gazebo pre-flight wrapper that brings up simulation, mapping, and monitoring."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_rosbag_action(context, *_args, **_kwargs):
    record = LaunchConfiguration('record_bag').perform(context).strip().lower()
    if record not in ('true', '1', 'yes', 'on'):
        return []

    bag_prefix_raw = LaunchConfiguration('bag_output').perform(context).strip()
    topics_raw = LaunchConfiguration('bag_topics').perform(context)

    topics = [
        topic if topic.startswith('/') else f'/{topic}'
        for topic in (part.strip() for part in topics_raw.split(','))
        if topic
    ]
    if not topics:
        return []

    bag_prefix = Path(bag_prefix_raw).expanduser()
    if bag_prefix.parent:
        bag_prefix.parent.mkdir(parents=True, exist_ok=True)

    cmd = ['ros2', 'bag', 'record', '-o', str(bag_prefix), *topics]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    pkg_share = get_package_share_directory('GigaVacuumMobil')

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='vacuum_world'),
        DeclareLaunchArgument('gazebo_use_rviz', default_value='false'),
        DeclareLaunchArgument('nav_rviz', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument(
            'bag_output',
            default_value=str(Path.home() / 'rosbags' / 'gvm_preflight'),
            description='Directory/prefix passed to ros2 bag record -o',
        ),
        DeclareLaunchArgument(
            'bag_topics',
            default_value='/scan_synced,/odom,/map,/stable_map,/tf,/tf_static,/pattern_manager/status,/mapping_health/ready',
            description='Comma-separated topics to record when record_bag=true',
        ),
        DeclareLaunchArgument('pattern_auto_mode', default_value='false'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(pkg_share) / 'launch' / 'gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': LaunchConfiguration('gazebo_use_rviz'),
            'world': world,
        }.items(),
    )

    full_cleaning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(pkg_share) / 'launch' / 'full_cleaning.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_slam': 'true',
            'launch_rviz': LaunchConfiguration('nav_rviz'),
            'pattern_auto_mode': LaunchConfiguration('pattern_auto_mode'),
            'exploration_enabled': 'true',
        }.items(),
    )

    health_monitor = Node(
        package='GigaVacuumMobil',
        executable='mapping_health_monitor.py',
        name='mapping_health_monitor',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'scan_topic': '/scan_synced',
                'odom_topic': '/odom',
                'map_topic': '/map',
                'stable_map_topic': '/stable_map',
                'trajectory_topic': '/trajectory_node_list',
            }
        ],
    )

    rosbag_action = OpaqueFunction(function=_make_rosbag_action)

    ld = LaunchDescription(declared_arguments)
    ld.add_action(gazebo_launch)
    ld.add_action(full_cleaning_launch)
    ld.add_action(health_monitor)
    ld.add_action(rosbag_action)
    return ld
