# GigaVacuumMobil - Next Steps Guide

## ✅ Project Status

Your ROS 2 Jazzy differential drive robot package is now **structurally complete**! All the core files have been created:

### Core C++ Code
- ✅ Hardware interface header (`include/diffdrive_hardware.hpp`)
- ✅ Hardware interface implementation (`src/diffdrive_hardware.cpp`)
- ✅ Plugin export file (`GigaVacuumMobil.xml`)

### Configuration Files
- ✅ Build configuration (`CMakeLists.txt`, `package.xml`)
- ✅ Robot description (`urdf/GigaVacuumMobile.urdf.xacro`)
- ✅ Controller config (`config/controllers.yaml`)
- ✅ RViz config (`config/robot.rviz`)
- ✅ Launch file (`launch/robot.launch.py`)

### Microcontroller Code
- ✅ Arduino firmware (`arduino/encoder_motor_driver.ino`)

---

## 🚀 What to Do Next

### Step 1: Build the ROS 2 Package

```bash
cd ~/ros2_ws  # Or wherever your workspace is
colcon build --packages-select GigaVacuumMobil
source install/setup.bash
```

**Expected output:** Build should complete successfully with no errors.

**If you get errors:**
- Check that all ROS 2 dependencies are installed (see README.md)
- Make sure you're using ROS 2 Jazzy
- Verify your workspace is properly set up

---

### Step 2: Test in Simulation Mode

Before connecting any hardware, test the entire system in simulation:

```bash
# Terminal 1: Launch robot in simulation mode
ros2 launch GigaVacuumMobil robot.launch.py use_sim:=true

# Terminal 2: Check that topics are publishing
ros2 topic list
# You should see /cmd_vel, /odom, /joint_states, /tf, etc.

# Terminal 3: Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -r 10

# Terminal 4: Monitor odometry
ros2 topic echo /odom
```

**What to look for:**
- RViz should open and show your robot model
- Robot should move in RViz when you publish `/cmd_vel`
- `/odom` topic should show changing position values
- No error messages in the controller manager

---

### Step 3: Upload Arduino Firmware

1. **Connect your Arduino to your development computer** (not the Raspberry Pi yet)

2. **Open the Arduino IDE** and load the firmware:
   ```
   ~/ros2_ws/src/GigaVacuumMobil/arduino/encoder_motor_driver.ino
   ```

3. **CRITICAL: Update pin definitions** in the Arduino code to match your wiring:
   ```cpp
   // Update these lines at the top of the .ino file:
   #define LEFT_ENCODER_A 2
   #define LEFT_ENCODER_B 4
   #define RIGHT_ENCODER_A 3
   #define RIGHT_ENCODER_B 5
   #define LEFT_MOTOR_PWM 6
   #define LEFT_MOTOR_DIR 7
   #define RIGHT_MOTOR_PWM 9
   #define RIGHT_MOTOR_DIR 8
   ```

4. **Upload to Arduino** (Ctrl+U or Sketch → Upload)

5. **Test the firmware** using Arduino Serial Monitor (Ctrl+Shift+M):
   - Set baud rate to **57600**
   - Type `e` and press Enter → Should get response like `e0,0`
   - Type `p100,100` → Motors should spin (if connected)
   - Type `e` again → Encoder counts should be changing

---

### Step 4: Measure Your Robot Dimensions

These measurements are **CRITICAL** for accurate odometry and control:

1. **Wheel Radius** (`wheel_radius`)
   - Measure the diameter of your wheel
   - Divide by 2
   - Convert to meters
   - Example: 66mm diameter → 0.033m radius

2. **Wheel Separation** (`wheel_separation`)
   - Measure the distance between the centers of the two wheels
   - Convert to meters
   - Example: 170mm → 0.17m

3. **Encoder Ticks Per Revolution** (`enc_ticks_per_rev`)
   - Check your encoder datasheet
   - Common values: 400, 600, 2400
   - If unsure, you can test:
     ```bash
     # In Arduino Serial Monitor, type: r (reset encoders)
     # Manually rotate wheel exactly 1 full revolution
     # Type: e (read encoders)
     # The number you see is your ticks per revolution
     ```

4. **Update the values in two places:**

   **File 1: `urdf/GigaVacuumMobile.urdf.xacro`**
   ```xml
   <xacro:property name="wheel_radius" value="0.033" />
   <xacro:property name="wheel_separation" value="0.17" />
   <param name="enc_ticks_per_rev">2400</param>
   ```

   **File 2: `config/controllers.yaml`**
   ```yaml
   wheel_separation: 0.17
   wheel_radius: 0.033
   ```

---

### Step 5: Connect and Test Hardware

1. **Wire everything up:**
   - Arduino → Raspberry Pi (USB)
   - Arduino → Motor driver board (PWM/DIR pins)
   - Motor driver → Motors (power)
   - Encoders → Arduino (signal pins)
   - Power supply → Motor driver

2. **On the Raspberry Pi, find the Arduino's serial port:**
   ```bash
   ls /dev/tty*
   # Look for /dev/ttyUSB0 or /dev/ttyACM0
   ```

3. **Update the URDF if needed:**
   ```xml
   <param name="device">/dev/ttyUSB0</param>  <!-- Or /dev/ttyACM0 -->
   ```

4. **Set serial port permissions:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```

5. **Test hardware connection:**
   ```bash
   # Test that you can communicate with Arduino
   screen /dev/ttyUSB0 57600
   # Type 'e' → should see encoder data
   # Ctrl+A, then K, then Y to exit screen
   ```

---

### Step 6: Run with Real Hardware

```bash
# Terminal 1: Launch with real hardware
ros2 launch GigaVacuumMobil robot.launch.py use_sim:=false

# If successful, you should see:
# [INFO] [controller_manager]: Loaded controller 'diff_drive_controller'
# [INFO] [DiffDriveHardware]: Serial port /dev/ttyUSB0 opened successfully

# Terminal 2: Check encoder data is being read
ros2 topic echo /joint_states

# Terminal 3: Test motor control (MAKE SURE ROBOT IS SAFE TO MOVE!)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -1
```

**SAFETY NOTE:** 
- Start with very low velocities (0.1 m/s)
- Have the robot on blocks or be ready to unplug power
- Verify it moves in the expected direction

---

### Step 7: Tune PID Gains

If the robot moves but doesn't respond well to commands:

1. **Open `src/diffdrive_hardware.cpp`**

2. **Find the PID initialization** (around line 68):
   ```cpp
   control_toolbox::Pid::Gains pid_gains;
   pid_gains.p_gain_ = 10.0;   // ← Tune this
   pid_gains.i_gain_ = 0.1;    // ← And this
   pid_gains.d_gain_ = 0.5;    // ← And this
   ```

3. **Follow the tuning procedure in README.md:**
   - Start with just P gain
   - Add I gain for steady-state accuracy
   - Add D gain to reduce oscillations

4. **Rebuild after each change:**
   ```bash
   colcon build --packages-select GigaVacuumMobil
   source install/setup.bash
   ```

---

## 🔍 Common Issues and Solutions

### Issue: "Failed to open serial port"
```
[ERROR] [DiffDriveHardware]: Failed to open serial port /dev/ttyUSB0
```

**Solutions:**
1. Check port exists: `ls /dev/ttyUSB*`
2. Check permissions: `groups` (should include "dialout")
3. Verify Arduino is connected and powered
4. Try different port in URDF (/dev/ttyACM0)

---

### Issue: Robot spins instead of going straight

**Likely causes:**
1. **Encoder direction:** One encoder counting backwards
   - **Fix:** In Arduino code, swap A/B pins for one encoder
   
2. **Motor direction:** One motor spinning backwards
   - **Fix:** In Arduino code, invert the DIR signal for one motor
   
3. **Incorrect wheel_separation:** Measurement is wrong
   - **Fix:** Re-measure and update both URDF and controllers.yaml

---

### Issue: Motors don't respond

**Debug steps:**
1. Test Arduino directly: `screen /dev/ttyUSB0 57600`, type `p100,100`
2. Check motor wiring to driver board
3. Verify power supply to motors
4. Add debug output in `write()` function to see PWM values

---

### Issue: Encoders always read zero

**Debug steps:**
1. Test Arduino directly: `screen /dev/ttyUSB0 57600`, type `e`
2. Check encoder wiring
3. Manually spin wheels, see if counts change
4. Verify `enc_ticks_per_rev` is correct

---

## 📝 Optional Enhancements

Once basic robot control is working:

### 1. Install RPLIDAR Driver

```bash
sudo apt install ros-jazzy-rplidar-ros

# Add to your launch file:
rplidar_node = Node(
    package='rplidar_ros',
    executable='rplidar_composition',
    parameters=[{'serial_port': '/dev/ttyUSB1', 'frame_id': 'lidar_link'}]
)
```

### 2. Set Up Nav2

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Create a map using SLAM:
ros2 launch nav2_bringup slam_launch.py

# Save the map:
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# Run autonomous navigation:
ros2 launch nav2_bringup navigation_launch.py map:=~/my_map.yaml
```

### 3. Add Keyboard Teleop

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard

# Run it:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🎯 Success Criteria

You'll know everything is working when:

- ✅ Simulation runs without errors
- ✅ Robot responds to `/cmd_vel` commands
- ✅ `/odom` publishes accurate odometry
- ✅ Robot drives straight when commanded straight
- ✅ Encoder values increase when wheels turn
- ✅ PID control keeps velocity stable

---

## 📚 Additional Resources

- [ROS 2 Control Documentation](https://control.ros.org/)
- [Differential Drive Robot Tutorial](https://articulatedrobotics.xyz/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [RPLIDAR ROS 2 Package](https://github.com/Slamtec/rplidar_ros)

---

**Good luck with your robot! 🤖**

If you encounter issues not covered here, check the main README.md or consult the ROS 2 community forums.
