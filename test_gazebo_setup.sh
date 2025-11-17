#!/bin/bash
# Test script to verify Gazebo simulation setup

echo "=========================================="
echo "GigaVacuumMobil Gazebo Setup Verification"
echo "=========================================="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Track overall status
all_passed=true

# Function to check command existence
check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1 found"
        return 0
    else
        echo -e "${RED}✗${NC} $1 not found"
        all_passed=false
        return 1
    fi
}

# Function to check ROS package
check_ros_package() {
    if ros2 pkg prefix $1 &> /dev/null 2>&1; then
        echo -e "${GREEN}✓${NC} ROS package $1 installed"
        return 0
    else
        echo -e "${RED}✗${NC} ROS package $1 not installed"
        echo -e "   ${YELLOW}Install with: sudo apt-get install ros-$ROS_DISTRO-${1//_/-}${NC}"
        all_passed=false
        return 1
    fi
}

# Function to check file existence
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} File exists: $(basename $1)"
        return 0
    else
        echo -e "${RED}✗${NC} File missing: $1"
        all_passed=false
        return 1
    fi
}

# Function to check directory
check_directory() {
    if [ -d "$1" ]; then
        echo -e "${GREEN}✓${NC} Directory exists: $(basename $1)"
        return 0
    else
        echo -e "${RED}✗${NC} Directory missing: $1"
        all_passed=false
        return 1
    fi
}

echo "1. Checking ROS 2 Environment"
echo "------------------------------"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗${NC} ROS_DISTRO not set. Please source ROS 2 setup file."
    echo -e "   ${YELLOW}Run: source /opt/ros/<distro>/setup.bash${NC}"
    all_passed=false
else
    echo -e "${GREEN}✓${NC} ROS_DISTRO: $ROS_DISTRO"
fi
echo ""

echo "2. Checking Gazebo Installation"
echo "--------------------------------"
check_command "gz"
if command -v gz &> /dev/null; then
    gz_version=$(gz sim --version 2>&1 | head -n 1)
    echo -e "   Version: $gz_version"
fi
echo ""

echo "3. Checking Required ROS Packages"
echo "----------------------------------"
check_ros_package "ros_gz_sim"
check_ros_package "ros_gz_bridge"
check_ros_package "gz_ros2_control"
check_ros_package "controller_manager"
check_ros_package "diff_drive_controller"
check_ros_package "joint_state_broadcaster"
check_ros_package "robot_state_publisher"
check_ros_package "xacro"
echo ""

echo "4. Checking Workspace Files"
echo "----------------------------"
WORKSPACE_ROOT="/home/adam-warn/GigaVacuumMobil"

check_directory "$WORKSPACE_ROOT/src/GigaVacuumMobil"
check_directory "$WORKSPACE_ROOT/src/GigaVacuumMobil/worlds"
check_directory "$WORKSPACE_ROOT/src/GigaVacuumMobil/config"
check_directory "$WORKSPACE_ROOT/src/GigaVacuumMobil/launch"
check_directory "$WORKSPACE_ROOT/src/GigaVacuumMobil/urdf"
echo ""

echo "5. Checking Created Files"
echo "-------------------------"
check_file "$WORKSPACE_ROOT/src/GigaVacuumMobil/worlds/vacuum_world.sdf"
check_file "$WORKSPACE_ROOT/src/GigaVacuumMobil/config/gz_bridge.yaml"
check_file "$WORKSPACE_ROOT/src/GigaVacuumMobil/launch/gazebo.launch.py"
check_file "$WORKSPACE_ROOT/src/GigaVacuumMobil/urdf/GigaVacuumMobile.urdf.xacro"
echo ""

echo "6. Checking Workspace Build"
echo "----------------------------"
if [ -d "$WORKSPACE_ROOT/install/GigaVacuumMobil" ]; then
    echo -e "${GREEN}✓${NC} Workspace has been built"
    
    # Check if built files are installed
    if [ -d "$WORKSPACE_ROOT/install/GigaVacuumMobil/share/GigaVacuumMobil/worlds" ]; then
        echo -e "${GREEN}✓${NC} Worlds directory installed"
    else
        echo -e "${YELLOW}⚠${NC} Worlds directory not installed - rebuild needed"
        all_passed=false
    fi
else
    echo -e "${RED}✗${NC} Workspace not built"
    echo -e "   ${YELLOW}Build with: colcon build --symlink-install${NC}"
    all_passed=false
fi
echo ""

echo "=========================================="
if [ "$all_passed" = true ]; then
    echo -e "${GREEN}All checks passed! ✓${NC}"
    echo ""
    echo "You're ready to run the simulation!"
    echo "Run: ros2 launch GigaVacuumMobil gazebo.launch.py"
else
    echo -e "${RED}Some checks failed ✗${NC}"
    echo ""
    echo "Please resolve the issues above before running the simulation."
    echo ""
    echo "Common fixes:"
    echo "1. Install missing packages: rosdep install --from-paths src --ignore-src -r -y"
    echo "2. Build workspace: colcon build --symlink-install"
    echo "3. Source workspace: source install/setup.bash"
fi
echo "=========================================="
