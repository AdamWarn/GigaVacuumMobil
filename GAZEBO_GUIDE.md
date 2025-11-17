# GigaVacuumMobil - Gazebo Simulation Guide

## Overview

Your robot now has **full Gazebo support** with:
- ✅ **Realistic physics** simulation
- ✅ **LIDAR sensor** (360° laser scanner)
- ✅ **Differential drive dynamics**
- ✅ **Collision detection**
- ✅ **Test world** with obstacles for SLAM testing
- ✅ **ros2_control integration**

## Quick Start

### Launch Gazebo Simulation

```bash
# Simple launch (recommended)
./launch_gazebo.sh

# Or manually:
source ~/GigaVacuumMobil/install/setup.bash
ros2 launch GigaVacuumMobil gazebo.launch.py

# Quick test (launches and shows status)
./test_gazebo.sh
```

**Note:** First launch takes 10-20 seconds. Be patient!

This will open:
1. **Gazebo Harmonic** - 3D simulation with physics
2. **RViz** - Robot state visualization with LIDAR data

### Control the Robot

```bash
# In a new terminal
source ~/GigaVacuumMobil/install/setup.bash

# Drive forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -r 10

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" -r 10

# Or use keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### View LIDAR Data

```bash
# Echo scan data
ros2 topic echo /scan

# Check scan rate
ros2 topic hz /scan

# View in RViz - the LaserScan display should show red points
```

## What's Different from Basic Simulation?

| Feature | Basic Sim | Gazebo Sim |
|---------|-----------|------------|
| Physics | ❌ None | ✅ Full rigid body dynamics |
| LIDAR | ❌ No sensor | ✅ 360° laser scanner |
| Collisions | ❌ No | ✅ Yes, robot bounces off walls |
| Visualization | RViz only | Gazebo 3D + RViz |
| World/Obstacles | ❌ None | ✅ Walls, boxes, cylinders |
| Wheel slip | ❌ No | ✅ Realistic friction |
| SLAM Ready | ❌ No | ✅ Yes, LIDAR + odometry |

## Architecture

### URDF Changes

The Gazebo URDF (`GigaVacuumMobile_gazebo.urdf.xacro`) includes:

1. **Proper Inertias** - Calculated for all links
2. **Collision Meshes** - For physics simulation
3. **Friction Parameters** - Wheels have high friction (μ=1.0), caster has low (μ=0.1)
4. **LIDAR Sensor Plugin** - GPU-accelerated ray tracing
5. **Gazebo ros2_control** - Uses `GazeboSimSystem` instead of custom hardware interface

### Control Flow in Gazebo

```
┌─────────────────────────────────────────────────┐
│              Gazebo Harmonic                    │
│  ┌──────────────────────────────────────┐       │
│  │  Physics Engine                      │       │
│  │  - Simulates robot dynamics          │       │
│  │  - Wheel friction & contact          │       │
│  │  - Collisions with obstacles         │       │
│  └────────────┬─────────────────────────┘       │
│               │                                  │
│  ┌────────────▼─────────────────────────┐       │
│  │  LIDAR Sensor (gpu_lidar)            │       │
│  │  - 360° scan at 10 Hz                │       │
│  │  - Range: 0.12m to 12m               │       │
│  │  - Publishes to /scan                │       │
│  └────────────┬─────────────────────────┘       │
│               │                                  │
│  ┌────────────▼─────────────────────────┐       │
│  │  GazeboSimSystem (ros2_control)      │       │
│  │  - Reads wheel positions/velocities  │       │
│  │  - Writes wheel velocity commands    │       │
│  └────────────┬─────────────────────────┘       │
└───────────────┼──────────────────────────────────┘
                │
                ↓
┌─────────────────────────────────────────────────┐
│           ROS 2 (Jazzy)                         │
│                                                 │
│  diff_drive_controller                          │
│  ↓                                              │
│  Odometry (/odom)                               │
│  TF (odom → base_link)                          │
└─────────────────────────────────────────────────┘
```

### Topics Available in Gazebo

```bash
# Robot control
/cmd_vel                    # Input: Twist commands

# Sensor data
/scan                       # Output: LaserScan from LIDAR
/joint_states               # Output: Wheel positions/velocities

# Navigation
/odom                       # Output: Odometry from wheel encoders
/tf                         # Output: Transform tree

# Simulation
/clock                      # Simulation time
```

## Testing SLAM

Now that you have LIDAR + odometry, you can test SLAM!

### Install SLAM Toolbox

```bash
sudo apt install ros-jazzy-slam-toolbox
```

### Run SLAM

```bash
# Terminal 1: Start Gazebo
./launch_gazebo.sh

# Terminal 2: Start SLAM
source ~/GigaVacuumMobil/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Drive the robot around
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Drive slowly** in circles and the map will build in RViz!

### Save Your Map

```bash
# After mapping, save it
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

## Customizing the World

### Create Your Own World

Edit `src/GigaVacuumMobil/worlds/test_world.sdf` or create a new one:

```xml
<!-- Add a wall -->
<model name="my_wall">
    <static>true</static>
    <pose>5 0 0.5 0 0 0</pose>
    <link name="link">
        <collision name="collision">
            <geometry>
                <box><size>0.2 4 1</size></box>
            </geometry>
        </collision>
        <visual name="visual">
            <geometry>
                <box><size>0.2 4 1</size></box>
            </geometry>
        </visual>
    </link>
</model>
```

### Use Different Worlds

```bash
# Use your custom world
ros2 launch GigaVacuumMobil gazebo.launch.py world:=my_world.sdf

# Use empty world
ros2 launch GigaVacuumMobil gazebo.launch.py world:=empty.sdf
```

## Tuning the Robot

### Adjust LIDAR Parameters

Edit `src/GigaVacuumMobil/urdf/GigaVacuumMobile_gazebo.urdf.xacro`:

```xml
<sensor name="lidar" type="gpu_lidar">
    <update_rate>10</update_rate>  <!-- Change scan rate -->
    <lidar>
        <scan>
            <horizontal>
                <samples>360</samples>  <!-- Points per scan -->
                <min_angle>0.0</min_angle>
                <max_angle>6.28</max_angle>  <!-- Full 360° -->
            </horizontal>
        </scan>
        <range>
            <min>0.12</min>  <!-- Minimum range -->
            <max>12.0</max>  <!-- Maximum range -->
        </range>
    </lidar>
</sensor>
```

### Adjust Wheel Friction

```xml
<gazebo reference="left_wheel_link">
    <mu1>1.0</mu1>  <!-- Static friction -->
    <mu2>1.0</mu2>  <!-- Dynamic friction -->
</gazebo>
```

Higher values = less wheel slip = more accurate odometry

## Comparison: All 3 Modes

### 1. Basic Simulation (`use_sim:=true`)
```bash
ros2 launch GigaVacuumMobil robot.launch.py use_sim:=true
```
- **Use for:** Quick controller testing
- **No:** Physics, sensors, visualization
- **Speed:** Fastest

### 2. Gazebo Simulation
```bash
ros2 launch GigaVacuumMobil gazebo.launch.py
```
- **Use for:** Full testing, SLAM, navigation development
- **Has:** Physics, LIDAR, 3D world, obstacles
- **Speed:** Realistic

### 3. Real Hardware (`use_sim:=false`)
```bash
ros2 launch GigaVacuumMobil robot.launch.py use_sim:=false
```
- **Use for:** Final testing with actual robot
- **Requires:** Arduino, motors, encoders, RPLIDAR connected

## Troubleshooting

### Gazebo takes forever to start

First launch downloads models and can take 20-30 seconds. Subsequent launches are faster (5-10 seconds).

### "gz sim gui: symbol lookup error"

This is a harmless warning about the GUI. The simulation still works fine. Ignore it.

### World file not found error

Make sure you've rebuilt after adding world files:
```bash
cd ~/GigaVacuumMobil
colcon build --packages-select GigaVacuumMobil
source install/setup.bash
```

### Robot falls through floor

The caster wheel provides stability. If robot tips, adjust caster position in URDF.

### LIDAR shows no data

```bash
# Check topic
ros2 topic list | grep scan

# Check if bridge is running
ros2 node list | grep bridge
```

### Robot doesn't move

```bash
# Check controllers
ros2 control list_controllers

# Both should be "active":
# - diff_drive_controller
# - joint_state_broadcaster
```

## Next Steps

1. ✅ **Test basic movement** in Gazebo
2. ✅ **Verify LIDAR** data in RViz
3. ✅ **Try SLAM** with slam_toolbox
4. ✅ **Add Nav2** for autonomous navigation
5. ✅ **Create custom worlds** for specific tests
6. ✅ **Tune PID gains** if needed (in controllers.yaml)

---

**You now have a complete simulation environment for developing autonomous navigation!** 🚀🤖
