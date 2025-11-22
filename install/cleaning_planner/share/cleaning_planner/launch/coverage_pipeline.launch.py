from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_map_path = os.path.join(
        get_package_share_directory('GigaVacuumMobil'),
        'maps',
        'my_map.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map_path),
        DeclareLaunchArgument('row_spacing', default_value='0.5'),
        DeclareLaunchArgument('safety_margin', default_value='0.15'),
        DeclareLaunchArgument('boundary_laps', default_value='1'),
        DeclareLaunchArgument('inflation_size', default_value='5'),
        DeclareLaunchArgument('orientation_mode', default_value='heading'),
        DeclareLaunchArgument('min_passage_width', default_value='0.35'),
        DeclareLaunchArgument('poses_per_goal', default_value='1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_topic', default_value=''),
        DeclareLaunchArgument('auto_replan_on_map_update', default_value='true'),
        DeclareLaunchArgument('map_update_min_interval', default_value='5.0'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('enable_topic', default_value=''),

        Node(
            package='cleaning_planner',
            executable='coverage_planner_node',
            name='coverage_planner',
            output='screen',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
                'row_spacing': LaunchConfiguration('row_spacing'),
                'safety_margin': LaunchConfiguration('safety_margin'),
                'boundary_laps': LaunchConfiguration('boundary_laps'),
                'inflation_size': LaunchConfiguration('inflation_size'),
                'orientation_mode': LaunchConfiguration('orientation_mode'),
                'min_passage_width': LaunchConfiguration('min_passage_width'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_topic': LaunchConfiguration('map_topic'),
                'auto_replan_on_map_update': LaunchConfiguration('auto_replan_on_map_update'),
                'map_update_min_interval': LaunchConfiguration('map_update_min_interval'),
            }]
        ),

        Node(
            package='cleaning_planner',
            executable='coverage_executor_node',
            name='coverage_executor',
            output='screen',
            parameters=[{
                'poses_per_goal': LaunchConfiguration('poses_per_goal'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'auto_start': LaunchConfiguration('auto_start'),
                'enable_topic': LaunchConfiguration('enable_topic'),
            }]
        )
    ])
