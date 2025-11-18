"""
Launch Gazebo simulation for GigaVacuumMobil robot.

This launch file:
1. Starts Gazebo with the vacuum_world
2. Spawns the robot model from URDF
3. Bridges ROS and Gazebo topics
4. Starts controllers
5. Optionally launches RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('GigaVacuumMobil')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz for visualization (disable if using navigation launch)',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='vacuum_world',
            description='Name of the Gazebo world file (without .sdf extension)',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        )
    )

    # Initialize Arguments
    use_rviz = LaunchConfiguration('use_rviz')
    world_name = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Set Gazebo resource path to find our models
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'worlds')
    )

    # World file path
    world_file = PathJoinSubstitution([
        FindPackageShare('GigaVacuumMobil'),
        'worlds',
        [world_name, '.sdf']
    ])

    # Get controller configuration
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('GigaVacuumMobil'), 'config', 'controllers.yaml']
    )

    # Get URDF via xacro with use_sim=true for Gazebo
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('GigaVacuumMobil'), 'urdf', 'GigaVacuumMobile.urdf.xacro']
            ),
            ' ',
            'use_sim:=true',
            ' ',
            'controllers_file:=',
            robot_controllers,
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Gazebo Sim launch with controller parameters
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world_file, ' -r'],  # -r flag starts paused, allowing plugins to load
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'GigaVacuumMobil',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen',
    )

    # Robot State Publisher Node
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
    )

    # ROS-Gazebo Bridge
    bridge_params = PathJoinSubstitution([
        FindPackageShare('GigaVacuumMobil'),
        'config',
        'gz_bridge.yaml'
    ])
    
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', ['config_file:=', bridge_params],
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    # Alternative: Manual bridge for critical topics
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # Note: Controller manager is already started by gz_ros2_control plugin in Gazebo
    # No need to start a separate ros2_control_node for simulation

    # Topic relay from /cmd_vel to /diff_drive_controller/cmd_vel
    cmd_vel_relay = Node(
        package='GigaVacuumMobil',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Odometry frame fixer - republishes odom with correct frame_ids
    odom_frame_fixer = Node(
        package='GigaVacuumMobil',
        executable='odom_frame_fixer.py',
        name='odom_frame_fixer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Static transform for lidar frame to fix RViz warnings
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'GigaVacuumMobil/base_link/lidar_sensor'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', 
                   '--controller-manager-timeout', '30'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Diff Drive Controller Spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz Node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('GigaVacuumMobil'), 'config', 'robot.rviz']
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
        condition=IfCondition(use_rviz),
    )

    # Delay controller spawners after robot spawn with extra time for Gazebo to initialize
    delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=3.0,  # Wait 3 seconds after spawn for Gazebo to settle
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    delay_diff_drive_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=2.0,  # Wait 2 seconds between controllers
                    actions=[diff_drive_controller_spawner],
                )
            ],
        )
    )

    # Create the launch description
    nodes = [
        set_gazebo_resource_path,
        *declared_arguments,
        gazebo,
        robot_state_pub_node,
        spawn_robot,
        clock_bridge,
        cmd_vel_relay,
        odom_frame_fixer,
        static_tf_lidar,
        # gz_bridge,  # Uncomment if you want to use the YAML-based bridge
        # Note: control_node removed - Gazebo's gz_ros2_control handles controllers
        delay_joint_state_broadcaster_after_spawn,
        delay_diff_drive_controller_after_joint_state,
        rviz_node,
    ]

    return LaunchDescription(nodes)
