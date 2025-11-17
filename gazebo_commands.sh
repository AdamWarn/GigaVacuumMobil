#!/bin/bash
# Quick commands for GigaVacuumMobil Gazebo simulation

echo "=== GigaVacuumMobil Gazebo Simulation Commands ==="
echo ""

# Source the workspace
source /home/adam-warn/GigaVacuumMobil/install/setup.bash

echo "Available commands:"
echo ""
echo "1. Launch full simulation (Gazebo + RViz + Controllers)"
echo "   ros2 launch GigaVacuumMobil gazebo.launch.py"
echo ""
echo "2. Launch simulation without RViz"
echo "   ros2 launch GigaVacuumMobil gazebo.launch.py use_rviz:=false"
echo ""
echo "3. Control robot with keyboard"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "4. Move robot forward"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2}}\" --once"
echo ""
echo "5. Rotate robot"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{angular: {z: 0.5}}\" --once"
echo ""
echo "6. Stop robot"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\" --once"
echo ""
echo "7. Check controllers"
echo "   ros2 control list_controllers"
echo ""
echo "8. Monitor odometry"
echo "   ros2 topic echo /odom"
echo ""
echo "9. Monitor lidar scan"
echo "   ros2 topic echo /scan"
echo ""
echo "10. View all topics"
echo "    ros2 topic list"
echo ""

# If an argument is provided, execute that command number
if [ $# -eq 1 ]; then
    case $1 in
        1)
            ros2 launch GigaVacuumMobil gazebo.launch.py
            ;;
        2)
            ros2 launch GigaVacuumMobil gazebo.launch.py use_rviz:=false
            ;;
        3)
            ros2 run teleop_twist_keyboard teleop_twist_keyboard
            ;;
        4)
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once
            ;;
        5)
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once
            ;;
        6)
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
            ;;
        7)
            ros2 control list_controllers
            ;;
        8)
            ros2 topic echo /odom
            ;;
        9)
            ros2 topic echo /scan
            ;;
        10)
            ros2 topic list
            ;;
        *)
            echo "Invalid command number. Use 1-10."
            ;;
    esac
fi
