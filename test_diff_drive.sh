#!/bin/bash

# Simple test script for Gazebo differential drive
echo "========================================="
echo "Testing Gazebo Differential Drive"
echo "========================================="

cd /home/adam-warn/GigaVacuumMobil
source install/setup.bash

echo ""
echo "Starting Gazebo simulation (without LIDAR for now)..."
echo "Wait ~10 seconds, then check controller status in another terminal"
echo ""
echo "In another terminal, run:"
echo "  source install/setup.bash"
echo "  ros2 control list_controllers"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}' -r 10"
echo ""
echo "Press Ctrl+C to stop"
echo ""

ros2 launch GigaVacuumMobil gazebo.launch.py use_rviz:=false
