#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('GigaVacuumMobil').find('GigaVacuumMobil')
    
    # Launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Automatically start lifecycle nodes'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Whether to start RViz'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_slam',
            default_value='False',
            description='Skip static map server/AMCL and rely on live SLAM map if true'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('GigaVacuumMobil'),
                'maps',
                'current_map.yaml'
            ]),
            description='Saved map to load when not running SLAM'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'static_map_topic',
            default_value='/map',
            description='Costmap static layer topic (set to filtered map if available)'
        )
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    map_yaml = LaunchConfiguration('map_yaml')
    static_map_topic = LaunchConfiguration('static_map_topic')
    
    # Paths
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'global_costmap.global_costmap.static_layer.map_topic': static_map_topic,
        },
        convert_types=True,
    )
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(use_slam)
    )
    
    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_slam)
    )
    
    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )
    
    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )
    
    # Behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )
    
    # Waypoint follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )
    
    # Velocity smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )
    
    # Lifecycle manager for localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']}
        ],
        condition=UnlessCondition(use_slam)
    )
    
    # Lifecycle manager for navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]}
        ]
    )
    
    # RViz (delayed)
    rviz_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_rviz),
            )
        ]
    )
    
    # Group all nav2 nodes with delay
    nav2_nodes = TimerAction(
        period=3.0,
        actions=[
            map_server,
            amcl,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager_localization,
            lifecycle_manager_navigation,
        ]
    )
    
    # Create launch description
    ld = LaunchDescription(declared_arguments)
    ld.add_action(nav2_nodes)
    ld.add_action(rviz_node)
    
    return ld
