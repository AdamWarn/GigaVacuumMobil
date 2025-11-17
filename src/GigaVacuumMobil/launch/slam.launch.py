"""
Launch file for SLAM with Cartographer

This launch file:
1. Launches Cartographer for 2D SLAM mapping using wheel odometry
2. Optionally starts RViz with SLAM visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('GigaVacuumMobil')
    
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz for visualization',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Map resolution in meters',
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Configuration files
    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare('GigaVacuumMobil'),
        'config'
    ])
    
    cartographer_config_basename = 'cartographer.lua'

    # Cartographer node for SLAM
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),  # Use fixed odometry with correct frame_ids
        ]
    )

    # Cartographer occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': LaunchConfiguration('resolution')},
        ],
    )

    # RViz configuration for SLAM
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('GigaVacuumMobil'),
        'rviz',
        'slam.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Create the launch description
    ld = LaunchDescription(declared_arguments)
    
    # Add nodes
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)
    
    return ld
