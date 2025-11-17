#!/bin/bash
# Test robot movement in simulation

echo "Testing GigaVacuumMobil movement..."
echo ""

# Source the workspace
source /opt/ros/jazzy/setup.bash
source /home/adam-warn/GigaVacuumMobil/install/setup.bash

echo "1. Checking if controllers are active..."
ros2 control list_controllers 2>/dev/null || echo "  (Controllers not yet loaded - is Gazebo running?)"

echo ""
echo "2. Checking available velocity topics..."
ros2 topic list 2>/dev/null | grep cmd_vel || echo "  (No cmd_vel topics found)"

echo ""
echo "3. Checking if relay node is running..."
ros2 node list 2>/dev/null | grep cmd_vel_relay && echo "  ✓ Relay is running" || echo "  ✗ Relay not running"

echo ""
echo "4. Testing topic echo on /diff_drive_controller/cmd_vel for 2 seconds..."
echo "   (Publishing a test command to /cmd_vel...)"
timeout 2 ros2 topic echo /diff_drive_controller/cmd_vel &
ECHO_PID=$!
sleep 0.5
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once
wait $ECHO_PID 2>/dev/null

echo ""
echo "5. Publishing movement command..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -1

echo ""
echo "6. Stopping robot..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

echo ""
echo "✓ Test complete!"
echo ""
echo "If you saw messages on /diff_drive_controller/cmd_vel, the relay is working."
echo "If the robot didn't move, check:"
echo "  - Is Gazebo simulation running?"
echo "  - Are controllers loaded? (ros2 control list_controllers)"
echo "  - Is the robot on the ground in Gazebo?"
echo ""
echo "To control manually, run:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
