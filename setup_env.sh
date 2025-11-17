#!/bin/bash
# GigaVacuumMobil - Environment Setup Script
# Source this file in every new terminal before working with the robot:
# source ~/GigaVacuumMobil/setup_env.sh

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}GigaVacuumMobil - Environment Setup${NC}"
echo -e "${BLUE}========================================${NC}"

# Source ROS 2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✓ ROS 2 Jazzy sourced${NC}"
else
    echo -e "${YELLOW}! ROS 2 Jazzy not found${NC}"
    return 1
fi

# Source workspace overlay
if [ -f ~/GigaVacuumMobil/install/setup.bash ]; then
    source ~/GigaVacuumMobil/install/setup.bash
    echo -e "${GREEN}✓ GigaVacuumMobil workspace sourced${NC}"
else
    echo -e "${YELLOW}! Workspace not built yet. Run:${NC}"
    echo -e "  cd ~/GigaVacuumMobil && colcon build --packages-select GigaVacuumMobil"
    return 1
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Ready to use GigaVacuumMobil!${NC}"
echo -e ""
echo -e "Quick commands:"
echo -e "  ${BLUE}Simulation:${NC} ros2 launch GigaVacuumMobil robot.launch.py use_sim:=true"
echo -e "  ${BLUE}Real Robot:${NC} ros2 launch GigaVacuumMobil robot.launch.py use_sim:=false"
echo -e "  ${BLUE}Help:${NC}       ~/GigaVacuumMobil/commands.sh"
echo -e "${BLUE}========================================${NC}"
