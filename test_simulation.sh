#!/bin/bash
# Test script for GigaVacuumMobil simulation

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}GigaVacuumMobil - Simulation Test${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Source environment
source ~/GigaVacuumMobil/install/setup.bash

echo -e "${GREEN}Starting robot in simulation mode...${NC}"
ros2 launch GigaVacuumMobil robot.launch.py use_sim:=true use_rviz:=false > /tmp/robot_test.log 2>&1 &
LAUNCH_PID=$!

# Wait for startup
echo "Waiting for controllers to initialize..."
sleep 5

# Check controllers
echo ""
echo -e "${BLUE}Checking controllers:${NC}"
ros2 control list_controllers

# Check topics
echo ""
echo -e "${BLUE}Checking topics:${NC}"
ros2 topic list | grep -E "(cmd_vel|odom|joint_states)"

# Send test command
echo ""
echo -e "${GREEN}Sending forward command (0.2 m/s for 2 seconds)...${NC}"
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once

# Wait and check odometry
sleep 2
echo ""
echo -e "${BLUE}Odometry sample:${NC}"
timeout 2 ros2 topic echo /diff_drive_controller/odom --once | head -20

# Clean up
echo ""
echo -e "${GREEN}Stopping robot...${NC}"
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✓ Test complete!${NC}"
echo -e "${BLUE}========================================${NC}"
