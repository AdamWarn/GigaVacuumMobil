# SLAM and Mapping Guide

This guide explains how to use Cartographer for 2D SLAM mapping with the GigaVacuumMobil robot.

## Overview

The SLAM system uses:
- **Cartographer**: Creates 2D maps using lidar scans and wheel odometry from the diff_drive_controller

## Quick Start

### 1. Install Dependencies

Install Cartographer:

```bash
sudo apt install ros-jazzy-cartographer-ros
```

### 2. Run SLAM in Simulation

Launch Gazebo with the robot:
```bash
ros2 launch GigaVacuumMobil gazebo.launch.py
```

In a new terminal, launch SLAM (Cartographer publishes `/map` and TF):
```bash
source install/setup.bash
ros2 launch GigaVacuumMobil slam.launch.py use_rviz:=false
```

### 3. Control the Robot

Use keyboard teleop to drive the robot and build the map:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Press the play button in Gazebo, then drive around to build the map!

### 4. Run SLAM on Real Robot

```bash
# Launch the real robot
ros2 launch GigaVacuumMobil robot.launch.py

# In another terminal, launch SLAM (with use_sim_time:=false)
ros2 launch GigaVacuumMobil slam.launch.py use_sim_time:=false
```

## Saving Maps

### Save the map created by Cartographer

You can still save manually:

```bash
# Finish the trajectory (if running pure SLAM)
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# Save the map once
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

But for day-to-day runs *let the map updater do it for you* (see "Continuous Mapping" below).

## Launch File Arguments

The `slam.launch.py` accepts these arguments:

- `use_sim_time` (default: true) - Use simulation time or real time
- `use_rviz` (default: true) - Launch RViz for visualization
- `resolution` (default: 0.05) - Map resolution in meters

Example:
```bash
ros2 launch GigaVacuumMobil slam.launch.py use_sim_time:=false resolution:=0.03
```

## Configuration Files

### Cartographer Configuration
Located at: `config/cartographer.lua`

Key parameters (tailored for crisp walls):
- `min_range`: 0.12 m (matches RPLIDAR C1 specs)
- `max_range`: 10.0 m (drops far returns that tended to carve holes)
- `resolution`: 0.04 m grid size
- `missing_data_ray_length`: 1.8 m so unseen space stays unknown rather than instantly cleared
- `range_data_inserter.hit_probability/miss_probability`: 0.75 / 0.35 so hits accumulate quickly and freespace takes sustained evidence
- `use_odometry`: true (uses wheel odometry from diff_drive_controller)

## Topics

### Published Topics
- `/map` - Occupancy grid map from Cartographer
- `/odom` - Odometry from diff_drive_controller (wheel encoders)
- `/tf` - Transform tree (map → odom → base_link)

### Subscribed Topics
- `/scan` - Laser scan from lidar
- `/odom` - Wheel odometry from diff_drive_controller

## Troubleshooting

### Cartographer Not Starting
**Problem**: Cartographer node fails to start

**Solutions**:
1. Check if Cartographer is properly installed:
   ```bash
   ros2 pkg list | grep cartographer
   ```
2. Verify lidar is publishing on `/scan`:
   ```bash
   ros2 topic echo /scan
   ```
3. Check wheel odometry is publishing:
   ```bash
   ros2 topic echo /odom
   ```

### No Map Being Created
**Problem**: Cartographer runs but no map appears

**Solutions**:
1. Check if laser scans are being received:
   ```bash
   ros2 topic hz /scan
   ```
2. Verify wheel odometry is publishing:
   ```bash
   ros2 topic echo /odom
   ```
3. Ensure robot is moving - maps need motion to build
4. Check TF tree is complete:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Poor Map Quality
**Problem**: Map is noisy or misaligned

**Solutions**:
1. Drive slower - fast turns can cause drift
2. Adjust Cartographer parameters in `config/cartographer.lua`:
   - Raise `range_data_inserter.hit_probability` or lower `miss_probability` if walls erode too fast
   - Decrease `resolution` (currently 0.04 m) for finer detail, but keep it aligned with Nav2 costmap resolution
   - Reduce `missing_data_ray_length` or `max_range` if distant measurements keep punching holes through walls
3. Ensure good lidar visibility - avoid reflective surfaces

### TF Errors
**Problem**: "Could not find transform" errors

**Solutions**:
1. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```
2. Verify frames in config match your setup:
   - `map_frame`: "map"
   - `tracking_frame`: "base_link"
   - `odom_frame`: "odom"

## Advanced Usage

### Localization Only (No Mapping)

To use an existing map for localization:

1. Load the map:
   ```bash
   ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml
   ```

2. Launch Cartographer in localization mode (modify cartographer.lua):
   ```lua
   TRAJECTORY_BUILDER.pure_localization_trimmer = {
     max_submaps_to_keep = 3,
   }
   ```

### Tuning for Better Performance

**For faster mapping**:
- Increase `submaps.num_range_data` in cartographer.lua
- Decrease `pose_publish_period_sec`

**For better accuracy**:
- Decrease grid `resolution`
- Increase `optimize_every_n_nodes`
- Use `real_time_correlative_scan_matcher`

## Continuous Mapping + Automatic Saves

The `map_maintenance` package keeps `/map` fresh and saves snapshots automatically:

```bash
source install/setup.bash
ros2 run map_maintenance map_updater_node \
   --ros-args -p map_topic:=/map \
            -p save_directory:=~/GigaVacuumMobil_maps \
            -p save_period:=30.0
```

- Every 30 seconds it writes `~/GigaVacuumMobil_maps/live_map_YYYYmmdd_HHMMSS.(pgm|yaml)`.
- It also refreshes `~/GigaVacuumMobil_maps/current_map.yaml`, which the navigation stack and coverage planner can reload on the next boot.

## Unified SLAM → Navigation → Cleaning

Run the entire pipeline (Cartographer + map updater + Nav2 + coverage planner/executor) in one command:

```bash
source install/setup.bash
ros2 launch GigaVacuumMobil full_cleaning.launch.py \
   map_save_directory:=~/GigaVacuumMobil_maps \
   map_yaml:=~/GigaVacuumMobil_maps/current_map.yaml \
   poses_per_goal:=8
```

What this launch file does:
- Starts Cartographer (if `enable_slam:=true`) to keep `/map` and TF live.
- Runs `map_updater_node` so the latest occupancy grid is persisted to disk.
- Brings up Nav2 in SLAM mode (no AMCL or static map server required) so the robot can navigate immediately.
- Starts the cleaning planner/executor, which now watches `/map` for updates and continuously feeds bite-sized NavigateThroughPoses goals until coverage is complete.

If a saved map already exists, Nav2 and the planner will preload it while they wait for fresh SLAM data. If no map exists yet, they simply wait for the first `/map` message before planning.

## Next Steps

- See `NAVIGATION_GUIDE.md` for navigation tuning and the new full-stack workflow.
- Use the coverage planner launch arguments (`row_spacing`, `boundary_laps`, `poses_per_goal`) to match the environment.

## References

- [Cartographer Documentation](https://google-cartographer-ros.readthedocs.io/)
- [ROS 2 Nav2](https://navigation.ros.org/)
- [Cartographer Tuning Guide](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html)
