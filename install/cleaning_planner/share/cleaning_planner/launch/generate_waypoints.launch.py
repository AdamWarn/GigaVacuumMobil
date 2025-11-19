from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Default map file
    default_map_path = os.path.join(
        get_package_share_directory('GigaVacuumMobil'),
        'maps',
        'my_map.yaml' 
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map_path,
            description='Full path to the map YAML file to use for waypoint generation.'
        ),
        DeclareLaunchArgument(
            'row_spacing',
            default_value='0.5',
            description='Spacing between cleaning rows in meters.'
        ),
        DeclareLaunchArgument(
            'inflation_size',
            default_value='5',
            description='Number of pixels to inflate obstacles by for safety margin.'
        ),
        DeclareLaunchArgument(
            'boundary_laps',
            default_value='1',
            description='Number of laps to perform around the boundary.'
        ),
        DeclareLaunchArgument(
            'output_file',
            default_value='waypoints.yaml',
            description='Name of the output file for the generated waypoints.'
        ),

        Node(
            package='cleaning_planner',
            executable='waypoint_generator',
            name='waypoint_generator',
            output='screen',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
                'row_spacing': LaunchConfiguration('row_spacing'),
                'inflation_size': LaunchConfiguration('inflation_size'),
                'boundary_laps': LaunchConfiguration('boundary_laps'),
                'output_file': LaunchConfiguration('output_file')
            }],
            # The script now uses argparse, so we pass arguments directly
            arguments=[
                '--map-file', LaunchConfiguration('map_file'),
                '--row-spacing', LaunchConfiguration('row_spacing'),
                '--inflation-size', LaunchConfiguration('inflation_size'),
                '--boundary-laps', LaunchConfiguration('boundary_laps'),
                '--output-file', LaunchConfiguration('output_file')
            ]
        )
    ])
