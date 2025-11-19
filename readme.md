# GigaVacuumMobil - ROS 2 Differential Drive Robot

A ROS 2 Jazzy package for controlling a differential drive robot with `ros2_control` and Nav2. This package supports both simulation and real hardware deployment.

**Author:** AdamWarn  
**Date Created:** 2025-11-17  
**ROS 2 Distribution:** Jazzy

---

## Table of Contents

1. [Hardware Overview](#hardware-overview)
2. [Software Architecture](#software-architecture)
3. [Information Flow Diagram](#information-flow-diagram)
4. [Node Communication Details](#node-communication-details)
5. [Installation](#installation)
6. [Configuration](#configuration)
7. [Running the Robot](#running-the-robot)
8. [Troubleshooting](#troubleshooting)
9. [Tuning Guide](#tuning-guide)

## Additional Guides

- **[GAZEBO_GUIDE.md](GAZEBO_GUIDE.md)** - Simulation setup and testing
- **[SLAM_GUIDE.md](SLAM_GUIDE.md)** - Mapping with Cartographer
- **[NAVIGATION_GUIDE.md](NAVIGATION_GUIDE.md)** - Autonomous navigation and systematic cleaning

---

## Hardware Overview

### Physical Components

- **Brain:** Raspberry Pi 5 (running ROS 2 Jazzy)
- **Motors:** 2x DC motors with encoders
- **Motor Controller:** Motor driver board (receives PWM signals)
- **Microcontroller:** Arduino (connected via USB to RPi)
  - Reads encoder signals
  - Sends PWM values to motor driver
- **Sensor:** RPLIDAR C1 (connected via USB to RPi)

### Key Design Decision

**PID control is handled by ROS (not Arduino)** to allow:
- Better integration with the ROS ecosystem
- Real-time tuning without reflashing Arduino
- Access to sophisticated control algorithms
- Centralized odometry calculations

---

## Software Architecture

### Core Principles

1. **Arduino = Simple Hardware Abstraction Layer**
   - Read raw encoder ticks
   - Write PWM values
   - No complex logic

2. **ROS = Intelligence**
   - PID control loops
   - Odometry calculation
   - Navigation and path planning

3. **Dual Mode Support**
   - Real hardware mode (communicates with Arduino)
   - Simulation mode (no hardware required)

---

## Information Flow Diagram

This diagram shows how data flows through the entire system during normal operation:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Raspberry Pi 5 (ROS 2 Jazzy)                    │
│                                                                         │
│  ┌──────────────┐                                                      │
│  │   Nav2 Stack │                                                      │
│  │              │                                                      │
│  │ - Planner    │                                                      │
│  │ - Controller │                                                      │
│  │ - Costmap    │                                                      │
│  └───────┬──────┘                                                      │
│          │                                                             │
│          │ Publishes: /cmd_vel (geometry_msgs/Twist)                  │
│          │ "Move forward at 0.5 m/s"                                  │
│          ↓                                                             │
│  ┌───────────────────────────────────────────────────────┐            │
│  │         diff_drive_controller                         │            │
│  │         (ros2_controllers)                            │            │
│  │                                                       │            │
│  │  • Inverse Kinematics: Twist → Wheel Velocities      │            │
│  │  • Forward Kinematics: Wheel Movement → Odometry     │            │
│  └─────────────┬─────────────────────────┬───────────────┘            │
│                │                         │                            │
│                │ Writes to Command       │ Reads from State           │
│                │ Interfaces:             │ Interfaces:                │
│                │ - left_wheel: 3.2 rad/s │ - left_wheel: 3.1 rad/s    │
│                │ - right_wheel: 3.2 rad/s│ - right_wheel: 3.15 rad/s  │
│                │                         │                            │
│                │                         │ Publishes:                 │
│                │                         │ • /odom (nav_msgs/Odometry)│
│                │                         │ • /tf (odom→base_link)     │
│                ↓                         ↑                            │
│  ┌──────────────────────────────────────────────────────┐             │
│  │                                                      │             │
│  │       ros2_control Hardware Interface                │             │
│  │       (Your Custom C++ Code)                         │             │
│  │                                                      │             │
│  │  ┌────────────────────────────────────────────────┐  │             │
│  │  │ READ CYCLE (50 Hz)                            │  │             │
│  │  │  1. Request encoder data from Arduino         │  │             │
│  │  │  2. Receive: "e12345,12389\n"                 │  │             │
│  │  │  3. Convert ticks → radians                   │  │             │
│  │  │  4. Calculate velocity from delta             │  │             │
│  │  │  5. Update State Interfaces ───────────────┐  │  │             │
│  │  └────────────────────────────────────────────┼──┘  │             │
│  │                                                │     │             │
│  │  ┌────────────────────────────────────────────┼───┐ │             │
│  │  │ WRITE CYCLE (50 Hz)                        │   │ │             │
│  │  │  1. Read Command Interfaces  ←─────────────┘   │ │             │
│  │  │     Target: 3.2 rad/s                          │ │             │
│  │  │  2. Read current velocity from State           │ │             │
│  │  │     Current: 3.1 rad/s                         │ │             │
│  │  │  3. PID Calculation:                           │ │             │
│  │  │     error = 3.2 - 3.1 = 0.1                    │ │             │
│  │  │     effort = Kp*error + Ki*∫error + Kd*Δerror │ │             │
│  │  │  4. Convert effort → PWM (-255 to 255)        │ │             │
│  │  │  5. Send to Arduino: "p180,182\n"             │ │             │
│  │  └────────────────────────────────────────────────┘ │             │
│  └───────────────────────┬──────────────────────────────┘             │
│                          │                ↑                           │
│                          │ USB Serial     │ USB Serial                │
│                          │ TX: "p180,182" │ RX: "e12345,12389"        │
└──────────────────────────┼────────────────┼───────────────────────────┘
                           ↓                ↑
┌──────────────────────────────────────────────────────────────────────┐
│                           Arduino                                    │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  Main Loop (runs continuously)                                 │ │
│  │                                                                 │ │
│  │  1. Read encoder pins → count ticks                            │ │
│  │     left_ticks = 12345, right_ticks = 12389                    │ │
│  │                                                                 │ │
│  │  2. Listen for serial commands:                                │ │
│  │     - 'e\n' → respond with "e{left},{right}\n"                 │ │
│  │     - 'p{L},{R}\n' → set PWM values                            │ │
│  │                                                                 │ │
│  │  3. Write PWM to motor driver pins                             │ │
│  │     analogWrite(LEFT_PWM, 180)                                 │ │
│  │     analogWrite(RIGHT_PWM, 182)                                │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                           │                                          │
└───────────────────────────┼──────────────────────────────────────────┘
                            ↓
              ┌─────────────────────────────┐
              │     Motor Driver Board      │
              │  (receives PWM signals)     │
              └──────────┬──────────────────┘
                         ↓
              ┌─────────────────────────────┐
              │    Left Motor + Encoder     │
              │    Right Motor + Encoder    │
              └─────────────────────────────┘


PARALLEL DATA FLOW - LIDAR:

┌─────────────────────────────────────────────┐
│  Raspberry Pi 5                             │
│                                             │
│  ┌──────────────────────────┐               │
│  │   rplidar_ros2 node      │               │
│  │                          │               │
│  │  Publishes:              │               │
│  │  /scan (LaserScan)       │               │
│  └────────────┬─────────────┘               │
│               │                             │
│               └──────────────┐              │
│                              ↓              │
│                    ┌──────────────────────┐ │
│                    │   Nav2 Stack         │ │
│                    │ - Uses /scan for     │ │
│                    │   obstacle detection │ │
│                    └──────────────────────┘ │
└─────────────────────────────────────────────┘
                     ↑
                     │ USB
                     │
            ┌────────┴─────────┐
            │  RPLIDAR C1      │
            └──────────────────┘


ADDITIONAL PUBLISHERS:

┌─────────────────────────────────────────────┐
│  joint_state_broadcaster                    │
│  • Reads State Interfaces                  │
│  • Publishes: /joint_states                │
│    (wheel positions for visualization)      │
└───────────────┬─────────────────────────────┘
                ↓
┌─────────────────────────────────────────────┐
│  robot_state_publisher                      │
│  • Reads: /joint_states + URDF             │
│  • Publishes: /tf (all robot transforms)   │
│    base_link→left_wheel, base_link→lidar   │
└─────────────────────────────────────────────┘
```

---

## Node Communication Details

### 1. **Arduino Firmware**
- **Language:** C/C++ (Arduino)
- **Cycle Rate:** As fast as possible (typically >1kHz)
- **Inputs:** 
  - Encoder A/B signals (interrupt-driven)
  - Serial commands from ROS
- **Outputs:**
  - PWM signals to motor driver
  - Encoder tick counts via serial
- **Serial Protocol:**
  - **Request encoders:** `e\n`
  - **Response:** `e{left_ticks},{right_ticks}\n`
  - **Set PWM:** `p{left_pwm},{right_pwm}\n` (PWM range: -255 to 255)

### 2. **Hardware Interface (`diffdrive_hardware.cpp`)**
- **Type:** `hardware_interface::SystemInterface`
- **Cycle Rate:** 50 Hz (configurable in `controllers.yaml`)
- **Key Methods:**
  - `read()`: Fetches encoder data, updates state interfaces
  - `write()`: Computes PID, sends PWM commands
- **State Interfaces (Exported):**
  - `left_wheel_joint/position` (radians)
  - `left_wheel_joint/velocity` (rad/s)
  - `right_wheel_joint/position` (radians)
  - `right_wheel_joint/velocity` (rad/s)
- **Command Interfaces (Exported):**
  - `left_wheel_joint/velocity` (rad/s target)
  - `right_wheel_joint/velocity` (rad/s target)

### 3. **diff_drive_controller**
- **Type:** Pre-built ROS 2 controller
- **Cycle Rate:** 50 Hz
- **Subscribes:** `/cmd_vel` (Twist messages from Nav2/teleop)
- **Publishes:**
  - `/odom` (Odometry estimate)
  - `/tf` (`odom` → `base_link` transform)
- **Interacts With:** Hardware interface via command/state interfaces

### 4. **joint_state_broadcaster**
- **Type:** Pre-built ROS 2 controller
- **Publishes:** `/joint_states` (for RViz visualization)

### 5. **robot_state_publisher**
- **Type:** Standard ROS 2 node
- **Subscribes:** `/joint_states`
- **Reads:** URDF file
- **Publishes:** `/tf` (all fixed and non-fixed transforms)

### 6. **Nav2 Stack**
- **Subscribes:**
  - `/scan` (from RPLIDAR)
  - `/odom` (from diff_drive_controller)
  - `/tf` (from robot_state_publisher)
- **Publishes:** `/cmd_vel` (velocity commands)

### 7. **rplidar_ros2**
- **Type:** Third-party driver node
- **Publishes:** `/scan` (LaserScan messages)

---

## Installation

### Prerequisites

```bash
# Install ROS 2 Jazzy (if not already installed)
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# Install dependencies
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-control-toolbox \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-rviz2
```

### Build the Package

```bash
cd ~/ros2_ws/src
# (Clone or create GigaVacuumMobil package here)

cd ~/ros2_ws
colcon build --packages-select GigaVacuumMobil
source install/setup.bash
```

---

## Configuration

### 1. Update URDF with Your Robot Dimensions

Edit `urdf/GigaVacuumMobile.urdf.xacro`:

```xml
<xacro:property name="wheel_radius" value="0.033" /> <!-- MEASURE THIS! -->
<xacro:property name="wheel_separation" value="0.17" /> <!-- MEASURE THIS! -->
<xacro:property name="enc_ticks_per_rev" value="2400" /> <!-- CHECK DATASHEET! -->
```

### 2. Update Controller Configuration

Edit `config/controllers.yaml` to match your URDF:

```yaml
wheel_separation: 0.17  # Must match URDF!
wheel_radius: 0.033     # Must match URDF!
```

### 3. Configure Serial Port

In `urdf/GigaVacuumMobile.urdf.xacro`:

```xml
<param name="device">/dev/ttyUSB0</param> <!-- Or /dev/ttyACM0 -->
<param name="baud_rate">57600</param>
```

Find your Arduino's port:
```bash
ls /dev/tty*
# Look for /dev/ttyUSB0 or /dev/ttyACM0
```

### 4. Set Permissions (One-Time Setup)

```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

---

## Running the Robot

### Simulation Mode (Basic - No Physics)

```bash
# Terminal 1: Launch robot with fake hardware
ros2 launch GigaVacuumMobil robot.launch.py use_sim:=true

# Terminal 2: Visualize in RViz
rviz2

# Terminal 3: Test with keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Gazebo Simulation (Full Physics + LIDAR)

For complete simulation with physics, LIDAR sensor, and obstacles:

```bash
# Launch Gazebo (includes RViz)
./launch_gazebo.sh

# Or manually:
ros2 launch GigaVacuumMobil gazebo.launch.py

# Control with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**See [GAZEBO_GUIDE.md](GAZEBO_GUIDE.md) for complete Gazebo documentation including SLAM setup!**

### Real Hardware Mode

```bash
# 1. Upload Arduino firmware (see arduino/encoder_motor_driver.ino)

# 2. Verify serial connection
ls -l /dev/ttyUSB0  # Should show your user has access

# 3. Launch robot
ros2 launch GigaVacuumMobil robot.launch.py use_sim:=false

# 4. Check that encoders are being read
ros2 topic echo /joint_states

# 5. Test movement
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Troubleshooting

### Problem: "Failed to open serial port"

**Symptoms:**
```
[ERROR] [diffdrive_hardware]: Failed to open serial port /dev/ttyUSB0
```

**Solutions:**
1. Check port exists: `ls /dev/ttyUSB*`
2. Check permissions: `ls -l /dev/ttyUSB0`
3. Add user to dialout group: `sudo usermod -a -G dialout $USER` (requires logout)
4. Try different port: Change `device` parameter in URDF

---

### Problem: Robot spins in circles / doesn't drive straight

**Symptoms:** When commanding forward motion, robot curves or spins.

**Causes & Solutions:**

1. **Encoder direction mismatch**
   - One encoder might be counting backwards
   - **Fix:** In Arduino code, swap encoder A/B pins for one motor OR negate one encoder count

2. **Motor wiring reversed**
   - One motor spins opposite direction
   - **Fix:** Swap motor wires OR negate PWM in Arduino for one motor

3. **Incorrect wheel separation**
   - Value in URDF doesn't match reality
   - **Fix:** Physically measure distance between wheel centers, update URDF

4. **PID not tuned**
   - Motors spinning at different speeds
   - **Fix:** See [Tuning Guide](#tuning-guide)

---

### Problem: Encoders not updating

**Symptoms:** `/joint_states` shows zero velocity, or position doesn't change.

**Solutions:**

1. **Check Arduino serial output:**
   ```bash
   # Disconnect from ROS, then:
   screen /dev/ttyUSB0 57600
   # Type 'e' and press Enter
   # Should see: e12345,67890
   ```

2. **Check encoder wiring:** Encoder signals reaching Arduino?

3. **Check `enc_ticks_per_rev` parameter:** Must match your encoder specs

---

### Problem: Motors don't respond to commands

**Symptoms:** `/cmd_vel` published but robot doesn't move.

**Debug Steps:**

1. **Check command is reaching controller:**
   ```bash
   ros2 topic echo /cmd_vel
   ```

2. **Check controller is active:**
   ```bash
   ros2 control list_controllers
   # Should show diff_drive_controller as "active"
   ```

3. **Check hardware interface is writing:**
   - Add RCLCPP_INFO in `write()` method to print PWM values

4. **Test Arduino directly:**
   ```bash
   screen /dev/ttyUSB0 57600
   # Type: p100,100
   # Motors should spin
   ```

---

### Problem: High-frequency oscillations / jittery movement

**Symptoms:** Robot vibrates or oscillates around target velocity.

**Cause:** PID gains too high (especially D gain).

**Fix:** Lower PID gains (see [Tuning Guide](#tuning-guide))

---

### Problem: Slow response / sluggish movement

**Symptoms:** Robot takes a long time to reach target velocity.

**Cause:** PID gains too low.

**Fix:** Increase P gain incrementally.

---

## Tuning Guide

### PID Tuning Procedure

The PID controller is defined in `src/diffdrive_hardware.cpp`:

```cpp
pid_gains.p_gain_ = 10.0;  // Start here
pid_gains.i_gain_ = 0.1;
pid_gains.d_gain_ = 0.5;
```

**Step-by-step tuning:**

1. **Set I and D to zero**
   ```cpp
   pid_gains.p_gain_ = 1.0;
   pid_gains.i_gain_ = 0.0;
   pid_gains.d_gain_ = 0.0;
   ```

2. **Increase P until response is fast but slightly oscillatory**
   - Rebuild and test after each change
   - Typical range: 5.0 - 20.0

3. **Add I gain to eliminate steady-state error**
   - Start with 0.01, increase slowly
   - Typical range: 0.01 - 1.0

4. **Add D gain to reduce overshoot**
   - Start with 0.1
   - Typical range: 0.1 - 2.0

5. **Test on real robot:**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -1
   # Observe response, adjust gains
   ```

### Making PID Tunable Without Recompiling (Advanced)

You can expose PID gains as ROS parameters - see the ROS 2 `control_toolbox` documentation.

---

## Next Steps

- [ ] Upload and test Arduino firmware
- [ ] Measure and update robot dimensions in URDF
- [ ] Tune PID gains
- [ ] Install and configure RPLIDAR driver
- [ ] Set up Nav2 configuration
- [ ] Create map of environment
- [ ] Test autonomous navigation

---

## Useful Commands

```bash
# List all controllers
ros2 control list_controllers

# Check controller manager status
ros2 control list_hardware_components

# View transforms
ros2 run tf2_tools view_frames

# Echo odometry
ros2 topic echo /odom

# Echo joint states
ros2 topic echo /joint_states

# Manually send velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Systematic Waypoint Generator

Use the `cleaning_planner` package to create Nav2-compatible coverage routes from any saved map:

```bash
source ~/GigaVacuumMobil/setup_env.sh
ros2 run cleaning_planner waypoint_generator \
   --map-file src/GigaVacuumMobil/maps/my_map.yaml \
   --row-spacing 0.5 \
   --boundary-laps 1 \
   --orientation-mode identity \
   --safety-margin 0.1 \
   --output-file waypoints.yaml
```

- `--orientation-mode identity` keeps quaternions neutral for the Nav2 waypoint follower.
- `--safety-margin` (meters) shrinks the free space before planning; increase it if RViz shows waypoints grazing walls.
- Generated YAML matches the Nav2 waypoint plugin format, so you can load it directly in RViz or feed it to the waypoint follower node.

---

## File Structure Reference

```
GigaVacuumMobil/
├── CMakeLists.txt
├── package.xml
├── README.md (this file)
├── GigaVacuumMobil.xml (plugin export)
├── arduino/
│   └── encoder_motor_driver.ino
├── config/
│   └── controllers.yaml
├── include/
│   └── GigaVacuumMobil/
│       └── diffdrive_hardware.hpp
├── launch/
│   └── robot.launch.py
├── src/
│   └── diffdrive_hardware.cpp
└── urdf/
    └── GigaVacuumMobile.urdf.xacro
```

---

**Last Updated:** 2025-11-17 by AdamWarn