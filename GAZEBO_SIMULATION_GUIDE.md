# GigaVacuumMobil Gazebo Simulation Guide

## Overview

This guide explains how to run the GigaVacuumMobil robot in Gazebo simulation. The simulation is fully interoperable with the actual hardware - you can develop and test your code in simulation and deploy it to the real robot without changes.

## Architecture

The setup uses:
- **Gazebo Sim** (formerly Ignition Gazebo) for physics simulation
- **ros2_control** with `gz_ros2_control` plugin for controller interface
- **ros_gz_bridge** for ROS-Gazebo topic bridging
- The same URDF/xacro file for both simulation and real hardware

## Prerequisites

### Install Gazebo
```bash
# For ROS 2 Jazzy, install Gazebo Harmonic
sudo apt-get update
sudo apt-get install ros-jazzy-ros-gz

# For ROS 2 Humble, install Gazebo Fortress
# sudo apt-get install ros-humble-ros-gz
```

### Install Dependencies
```bash
cd /home/adam-warn/GigaVacuumMobil
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Key packages installed:
- `ros-jazzy-ros-gz-sim` - Gazebo integration
- `ros-jazzy-ros-gz-bridge` - ROS-Gazebo communication bridge
- `ros-jazzy-gz-ros2-control` - ros2_control integration for Gazebo
- `ros-jazzy-xacro` - URDF macro processing

## Building the Package

```bash
cd /home/adam-warn/GigaVacuumMobil
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

### Launch Gazebo Simulation

This command starts everything needed for simulation:

```bash
ros2 launch GigaVacuumMobil gazebo.launch.py
```

This will:
1. Start Gazebo with the `vacuum_world.sdf` world
2. Spawn the robot model
3. Start ros2_control with simulated hardware
4. Launch the ROS-Gazebo bridge for clock and sensor data
5. Start the diff_drive_controller and joint_state_broadcaster
6. Open RViz for visualization

### Launch Options

**Without RViz:**
```bash
ros2 launch GigaVacuumMobil gazebo.launch.py use_rviz:=false
```

**With a different world:**
```bash
ros2 launch GigaVacuumMobil gazebo.launch.py world:=my_custom_world
```

### Controlling the Robot

Once the simulation is running, you can control the robot using standard ROS 2 tools:

**Using keyboard teleop:**
```bash
sudo apt-get install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

**Publishing velocity commands directly:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}" --once
```

**Stop the robot:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Monitoring the Simulation

### Check Active Controllers
```bash
ros2 control list_controllers
```

Expected output:
```
diff_drive_controller[diff_drive_controller/DiffDriveController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### Monitor Topics
```bash
# List all topics
ros2 topic list

# Watch odometry
ros2 topic echo /odom

# Watch laser scan
ros2 topic echo /scan

# Watch joint states
ros2 topic echo /joint_states
```

### View TF Tree
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing the transform tree
```

## Switching Between Simulation and Hardware

The beauty of this setup is that you can seamlessly switch between simulation and real hardware:

### For Simulation (Gazebo):
```bash
ros2 launch GigaVacuumMobil gazebo.launch.py
```

### For Real Hardware:
```bash
ros2 launch GigaVacuumMobil robot.launch.py use_sim:=false
```

The same controllers, same URDF, and same code work for both!

## How It Works

### 1. URDF Configuration
The `GigaVacuumMobile.urdf.xacro` file uses the `use_sim` argument to switch between:
- **Simulation**: Uses `gz_ros2_control/GazeboSimSystem` plugin
- **Hardware**: Uses `GigaVacuumMobil/DiffDriveHardware` plugin

```xml
<xacro:if value="$(arg use_sim)">
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</xacro:if>
<xacro:unless value="$(arg use_sim)">
    <plugin>GigaVacuumMobil/DiffDriveHardware</plugin>
    <!-- Hardware-specific parameters -->
</xacro:unless>
```

### 2. Gazebo Plugins
The URDF includes Gazebo-specific elements:

- **Lidar sensor**: Simulates the RPLIDAR C1 with realistic parameters
- **Wheel friction**: Configured for realistic motion
- **ros2_control integration**: Connects Gazebo physics to ROS controllers

### 3. ROS-Gazebo Bridge
The bridge (`gz_bridge.yaml`) synchronizes:
- `/clock` - Simulation time
- `/scan` - Lidar data
- `/odom` - Odometry (optional)
- `/tf` - Transforms

## Troubleshooting

### Gazebo doesn't start
```bash
# Check Gazebo installation
gz sim --version

# Try launching Gazebo alone
gz sim vacuum_world.sdf
```

### Controllers not loading
```bash
# Check controller manager
ros2 control list_hardware_interfaces

# Manually spawn controllers
ros2 run controller_manager spawner diff_drive_controller
ros2 run controller_manager spawner joint_state_broadcaster
```

### Clock not synchronized
Ensure the clock bridge is running:
```bash
ros2 topic echo /clock
```

If not receiving clock messages, manually start:
```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Robot not visible in Gazebo
Check the robot spawning:
```bash
# List Gazebo models
gz model -l

# Manually spawn if needed
ros2 run ros_gz_sim create -name GigaVacuumMobil -topic robot_description
```

### RViz shows no robot model
1. Ensure `use_sim_time` is set to true for all nodes
2. Check that robot_state_publisher is running:
   ```bash
   ros2 node list | grep robot_state
   ```
3. Verify the robot_description parameter:
   ```bash
   ros2 param get /robot_state_publisher robot_description
   ```

## Next Steps

1. **Add Navigation**: Integrate Nav2 for autonomous navigation
2. **Custom Worlds**: Create complex environments in `worlds/` directory
3. **Sensor Simulation**: Add cameras, IMUs, or other sensors
4. **Multi-robot**: Spawn multiple robots for swarm testing
5. **Record Data**: Use `ros2 bag` to record simulation data for analysis

## Development Workflow

Recommended workflow for developing with simulation:

1. **Develop in simulation**: Test algorithms, tune parameters
2. **Validate behaviors**: Ensure robot behaves as expected
3. **Test edge cases**: Simulate scenarios hard to reproduce in real life
4. **Deploy to hardware**: Switch to real robot with same code
5. **Fine-tune**: Adjust parameters based on real-world performance

## Files Created

- `worlds/vacuum_world.sdf` - Gazebo world with obstacles
- `config/gz_bridge.yaml` - ROS-Gazebo topic bridge configuration
- `launch/gazebo.launch.py` - Complete simulation launch file
- `urdf/GigaVacuumMobile.urdf.xacro` - Enhanced with Gazebo plugins

## Resources

- [Gazebo Documentation](https://gazebosim.org/docs)
- [ros_gz Documentation](https://github.com/gazebosim/ros_gz)
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- [ROS 2 Control](https://control.ros.org/)
