# Autonomous Navigation & Cleaning Guide

This guide covers autonomous navigation and systematic cleaning patterns for the GigaVacuumMobil robot.

## Overview

The autonomous navigation system provides:
- **Nav2-based navigation** for path planning and obstacle avoidance
- **Coverage path planning** with boundary laps and systematic coverage
- **Cleaning mission controller** for automated cleaning operations
- **Multi-area cleaning** capability for complex environments

## Prerequisites

Make sure you have completed the SLAM mapping process (see SLAM_GUIDE.md) and have a saved map.

## System Components

### 1. Navigation Stack (Nav2)
- **Global Planner**: NavFn for optimal path finding
- **Local Planner**: DWB for dynamic obstacle avoidance
- **Localization**: AMCL for robot pose estimation
- **Map Server**: Serves the saved map to navigation stack

### 2. Coverage Planner (`coverage_planner.py`)
Generates systematic cleaning paths:
- **Boundary laps**: Perimeter coverage with configurable number of laps
- **Boustrophedon pattern**: Back-and-forth pattern for area coverage
- **Adaptive to map shape**: Uses contour detection and binary erosion

### 3. Cleaning Controller (`cleaning_controller.py`)
Coordinates cleaning missions:
- Loads saved maps
- Generates coverage paths
- Executes navigation using Nav2 actions
- Reports progress and completion

## Quick Start

### Step 1: Create a Map (if not already done)

```bash
# Terminal 1: Launch Gazebo simulation
source install/setup.bash
ros2 launch GigaVacuumMobil gazebo.launch.py

# Terminal 2: Start SLAM
source install/setup.bash
ros2 launch GigaVacuumMobil slam.launch.py

# Terminal 3: Drive around to build the map
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Save the map when complete
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/GigaVacuumMobil/src/GigaVacuumMobil/maps/my_map
```

### Step 2: Run Autonomous Navigation

**IMPORTANT: Do not run Gazebo with RViz when using navigation - it will cause timing conflicts!**

```bash
# Terminal 1: Launch Gazebo WITHOUT RViz (wait for it to fully load)
source install/setup.bash
ros2 launch GigaVacuumMobil gazebo.launch.py
```

**Wait for Gazebo to fully load and stabilize (~10 seconds) before proceeding!**

```bash
# Terminal 2: Start navigation stack (this will open its own RViz)
source install/setup.bash
ros2 launch GigaVacuumMobil navigation.launch.py map:=src/GigaVacuumMobil/maps/my_map.yaml
```

**In RViz:**
1. Wait for the map to appear (may take a few seconds, be patient)
2. If map doesn't appear after 10 seconds, toggle the "Map" display off and on once in the Displays panel
3. Click "2D Pose Estimate" button
4. Click on the map where the robot is (center at 0,0) and drag to set orientation
5. Wait for AMCL to localize (particle cloud should converge around the robot)

### Step 3: Start Cleaning Mission

```bash
# Terminal 3: Run cleaning controller
source install/setup.bash
ros2 run GigaVacuumMobil cleaning_controller.py \
    --map-file src/GigaVacuumMobil/maps/my_map.yaml \
    --boundary-laps 2 \
    --row-spacing 0.3
```

The robot will:
1. Load the map
2. Generate boundary coverage path (2 laps around perimeter)
3. Generate systematic coverage path (back-and-forth pattern)
4. Execute the cleaning mission
5. Report progress and completion

## Advanced Usage

### Manual Navigation Goals

Instead of running the cleaning controller, you can send manual navigation goals:

```bash
# Send a single navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 1.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

### Customize Coverage Parameters

Edit the coverage planner parameters when running:

```bash
ros2 run GigaVacuumMobil cleaning_controller.py \
    --map-file src/GigaVacuumMobil/maps/my_map.yaml \
    --boundary-laps 3 \              # Number of perimeter laps (default: 2)
    --row-spacing 0.25 \              # Distance between coverage rows in meters (default: 0.3)
    --inflation-size 5                # Map erosion iterations (default: 5)
```

### Multi-Area Cleaning

For cleaning multiple disconnected areas, run separate missions for each map:

```bash
# Clean area 1
ros2 run GigaVacuumMobil cleaning_controller.py \
    --map-file src/GigaVacuumMobil/maps/area1_map.yaml

# Then clean area 2
ros2 run GigaVacuumMobil cleaning_controller.py \
    --map-file src/GigaVacuumMobil/maps/area2_map.yaml
```

### Tuning Navigation Performance

Edit `src/GigaVacuumMobil/config/nav2_params.yaml` to adjust navigation behavior:

**Speed Settings:**
```yaml
DWB:
  max_vel_x: 0.26        # Maximum forward velocity (m/s)
  min_vel_x: -0.26       # Maximum backward velocity (m/s)
  max_vel_theta: 1.0     # Maximum rotation velocity (rad/s)
```

**Obstacle Avoidance:**
```yaml
local_costmap:
  inflation_layer:
    inflation_radius: 0.55     # How far to inflate obstacles (m)
    cost_scaling_factor: 3.0   # How aggressively to avoid obstacles
```

**Path Planning:**
```yaml
controller_server:
  FollowPath:
    max_vel_x: 0.26
    min_vel_x: -0.26
    xy_goal_tolerance: 0.1     # Position accuracy (m)
    yaw_goal_tolerance: 0.2    # Orientation accuracy (rad)
```

After editing, rebuild and restart navigation:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch GigaVacuumMobil navigation.launch.py map:=src/GigaVacuumMobil/maps/my_map.yaml
```

## Monitoring and Debugging

### View Navigation Status

```bash
# Check if navigation is active
ros2 topic echo /navigate_through_poses/_action/status

# Monitor current goal
ros2 topic echo /goal_pose

# Check robot position
ros2 topic echo /amcl_pose
```

### Visualize Coverage Path

The coverage planner publishes the generated path for visualization:

```bash
# View the planned path
ros2 topic echo /coverage_path
```

In RViz, add a "Path" display and set topic to `/coverage_path`.

### Common Issues

**Problem: Robot doesn't localize**
- Solution: Set a good initial pose estimate with "2D Pose Estimate" in RViz
- Make sure the map matches the environment

**Problem: Navigation fails or robot gets stuck**
- Solution: Tune `inflation_radius` in nav2_params.yaml
- Reduce `max_vel_x` for more cautious navigation
- Check that obstacles are visible in costmap

**Problem: Coverage path doesn't cover entire area**
- Solution: Reduce `--row-spacing` for denser coverage
- Increase `--inflation-size` if missing tight corners
- Verify map quality (clean, well-defined boundaries)

**Problem: Cleaning controller reports no free space**
- Solution: Check that map has sufficient free (white) space
- Reduce `--inflation-size` if too much area is eroded
- Verify map was saved correctly

## ROS 2 Topics and Services

### Published Topics
- `/coverage_path` (nav_msgs/Path): Generated cleaning path
- `/plan` (nav_msgs/Path): Current navigation plan
- `/local_costmap/costmap` (nav_msgs/OccupancyGrid): Local obstacle map
- `/global_costmap/costmap` (nav_msgs/OccupancyGrid): Global planning map

### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): Lidar data for obstacle detection
- `/odom` (nav_msgs/Odometry): Wheel odometry for localization

### Action Servers
- `/navigate_to_pose`: Navigate to a single goal
- `/navigate_through_poses`: Navigate through multiple waypoints

## Integration with Real Hardware

When deploying on real hardware:

1. **Update the map**: Create a new map of the real environment
2. **Tune parameters**: Real sensors may need different nav2_params.yaml settings
3. **Test incrementally**: Start with simple navigation goals before full cleaning
4. **Monitor battery**: Add battery monitoring for autonomous missions
5. **Safety**: Implement emergency stop and obstacle detection safeguards

The navigation stack will work identically - just launch without `use_sim:=true`:

```bash
ros2 launch GigaVacuumMobil navigation.launch.py map:=maps/real_environment.yaml
```

## Next Steps

1. Create maps of your environments
2. Test navigation with manual goals
3. Run cleaning missions in simulation
4. Tune coverage parameters for your needs
5. Deploy on real hardware

For mapping instructions, see SLAM_GUIDE.md.
For Gazebo simulation, see GAZEBO_GUIDE.md.
