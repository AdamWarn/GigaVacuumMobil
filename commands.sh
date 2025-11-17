#!/bin/bash
# GigaVacuumMobil - Quick Commands Reference
# Save this file and run: chmod +x commands.sh

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}GigaVacuumMobil - Quick Commands${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Build commands
echo -e "${GREEN}BUILD COMMANDS:${NC}"
echo "colcon build --packages-select GigaVacuumMobil"
echo "source install/setup.bash"
echo ""

# Launch commands
echo -e "${GREEN}LAUNCH COMMANDS:${NC}"
echo "# Simulation mode:"
echo "ros2 launch GigaVacuumMobil robot.launch.py use_sim:=true"
echo ""
echo "# Real hardware mode:"
echo "ros2 launch GigaVacuumMobil robot.launch.py use_sim:=false"
echo ""
echo "# Without RViz:"
echo "ros2 launch GigaVacuumMobil robot.launch.py use_rviz:=false"
echo ""

# Testing commands
echo -e "${GREEN}TESTING COMMANDS:${NC}"
echo "# List all topics:"
echo "ros2 topic list"
echo ""
echo "# Echo odometry:"
echo "ros2 topic echo /odom"
echo ""
echo "# Echo joint states (encoder data):"
echo "ros2 topic echo /joint_states"
echo ""
echo "# Send velocity command (forward):"
echo "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2}}\" -r 10"
echo ""
echo "# Send velocity command (rotate):"
echo "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{angular: {z: 0.5}}\" -r 10"
echo ""
echo "# Stop robot:"
echo "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\" -1"
echo ""

# Controller commands
echo -e "${GREEN}CONTROLLER COMMANDS:${NC}"
echo "# List controllers:"
echo "ros2 control list_controllers"
echo ""
echo "# List hardware components:"
echo "ros2 control list_hardware_components"
echo ""
echo "# View controller state:"
echo "ros2 control list_hardware_interfaces"
echo ""

# Serial/Hardware commands
echo -e "${GREEN}HARDWARE DEBUGGING:${NC}"
echo "# Find Arduino port:"
echo "ls /dev/tty* | grep -E 'USB|ACM'"
echo ""
echo "# Test serial connection (Ctrl+A, K, Y to exit):"
echo "screen /dev/ttyUSB0 57600"
echo ""
echo "# Check permissions:"
echo "groups"
echo "ls -l /dev/ttyUSB0"
echo ""
echo "# Add user to dialout group (requires logout):"
echo "sudo usermod -a -G dialout \$USER"
echo ""

# TF commands
echo -e "${GREEN}TF/TRANSFORM COMMANDS:${NC}"
echo "# View TF tree:"
echo "ros2 run tf2_tools view_frames"
echo "# (Creates frames.pdf)"
echo ""
echo "# Echo specific transform:"
echo "ros2 run tf2_ros tf2_echo odom base_link"
echo ""

# Keyboard teleop
echo -e "${GREEN}KEYBOARD TELEOP:${NC}"
echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""

# Useful checks
echo -e "${GREEN}SYSTEM CHECKS:${NC}"
echo "# Check ROS 2 version:"
echo "echo \$ROS_DISTRO"
echo ""
echo "# Check workspace:"
echo "echo \$ROS_WORKSPACE"
echo ""
echo "# Verify package installed:"
echo "ros2 pkg list | grep GigaVacuumMobil"
echo ""

echo -e "${BLUE}========================================${NC}"
