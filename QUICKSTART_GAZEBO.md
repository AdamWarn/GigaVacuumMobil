# 🚀 Quick Start - Gazebo Simulation

## TL;DR - Get Running in 30 Seconds

```bash
# 1. Source the workspace
cd /home/adam-warn/GigaVacuumMobil
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 2. Launch the simulation
ros2 launch GigaVacuumMobil gazebo.launch.py

# 3. In a new terminal, control the robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Note: Commands are published to /cmd_vel which is automatically relayed to the controller
```

## What You'll See

- **Gazebo window**: 3D simulation with your robot and obstacles
- **RViz window**: Robot model visualization with sensor data
- **Terminal**: Status messages from controllers

## Control Keys (teleop_twist_keyboard)

```
   i    - Move forward
   ,    - Move backward
   j    - Turn left
   l    - Turn right
   k    - Stop
   q/z  - Increase/decrease speed
```

## Quick Commands

```bash
# View the quick command menu
./gazebo_commands.sh

# Run a specific command (e.g., command #1 - launch simulation)
./gazebo_commands.sh 1

# Test robot movement
./test_robot_movement.sh

# Check if setup is correct
./test_gazebo_setup.sh
```

## Switching Between Simulation and Real Robot

### Simulation Mode (Development)
```bash
ros2 launch GigaVacuumMobil gazebo.launch.py
```

### Hardware Mode (Deployment)
```bash
ros2 launch GigaVacuumMobil robot.launch.py use_sim:=false
```

**Same code, same controllers, different hardware!**

## What's Included

✅ Gazebo world with test environment  
✅ Full robot simulation with differential drive  
✅ Simulated RPLIDAR C1 sensor  
✅ ROS 2 control integration  
✅ RViz visualization  
✅ Hardware interoperability  

## Files You Created

- `worlds/vacuum_world.sdf` - Simulation environment
- `config/gz_bridge.yaml` - ROS-Gazebo bridge config
- `launch/gazebo.launch.py` - Main simulation launch file
- Enhanced `urdf/GigaVacuumMobile.urdf.xacro` - With Gazebo plugins

## Need Help?

- **Full guide**: `GAZEBO_SIMULATION_GUIDE.md`
- **Setup summary**: `GAZEBO_SETUP_COMPLETE.md`
- **Quick commands**: `./gazebo_commands.sh`
- **Verify setup**: `./test_gazebo_setup.sh`

## Common Issues

**Gazebo not starting?**
```bash
# Check version
gz sim --version

# Should show: Gazebo Sim, version 8.x
```

**Controllers not loading?**
```bash
ros2 control list_controllers

# Should show diff_drive_controller and joint_state_broadcaster as active
```

**Robot not moving?**
```bash
# Check if cmd_vel topic is receiving messages
ros2 topic echo /cmd_vel

# Check odometry is being published
ros2 topic echo /odom
```

## Next Steps

1. ✅ **Verify setup** - Run `./test_gazebo_setup.sh`
2. 🚀 **Launch simulation** - `ros2 launch GigaVacuumMobil gazebo.launch.py`
3. 🎮 **Control robot** - Test movement with teleop
4. 📊 **Monitor sensors** - Check `/scan` topic for lidar data
5. 🗺️ **Add navigation** - Integrate Nav2 for autonomous navigation

## Pro Tips

- Use `use_rviz:=false` to launch without RViz (faster)
- Press **Ctrl+T** in Gazebo to toggle transparency view
- Right-click in Gazebo to add/remove plugins
- Use `ros2 topic hz /scan` to check sensor update rate
- Record data with `ros2 bag record -a` for later analysis

---

**You're ready to develop without hardware! 🎉**

Start the simulation and begin testing your algorithms in a safe, virtual environment.
