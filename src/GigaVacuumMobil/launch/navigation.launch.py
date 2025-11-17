"""
Launch file for autonomous navigation with Nav2

This launch file:
1. Loads a saved map
2. Starts AMCL for localization
3. Launches Nav2 navigation stack
4. Optionally starts RViz with navigation visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('GigaVacuumMobil')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
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
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'map.yaml'),
            description='Full path to map yaml file',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Full path to Nav2 parameters file',
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
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack',
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')

    # Nav2 bringup launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    # RViz configuration for navigation
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
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
    
    # Add actions
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)
    
    return ld
