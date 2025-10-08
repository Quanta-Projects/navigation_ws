# 🤖 SMRR Robot Hardware Deployment Checklist

## ✅ Complete Step-by-Step Guide to Run on Real Hardware

### **Step 1: Upload Arduino Firmware** 🔌

1. **Open Arduino IDE**
   ```bash
   arduino
   ```

2. **Load the firmware file:**
   - Open: `src/smrr_base_controller/firmware/firmware.ino`
   - This will automatically open related files (`motor_control.ino`)

3. **Configure Arduino IDE:**
   - **Board**: Select your Arduino board (e.g., Arduino Mega, Arduino Uno)
   - **Port**: Select the USB port (e.g., `/dev/ttyUSB0`, `/dev/ttyACM0`)
   - Go to: `Tools > Board` and `Tools > Port`

4. **Verify pin connections match your hardware:**
   Check these definitions in `firmware.ino`:
   ```cpp
   #define L298N_enA 9   // Motor A PWM
   #define L298N_enB 11  // Motor B PWM
   #define L298N_in1 12  // Motor A Direction 1
   #define L298N_in2 13  // Motor A Direction 2
   #define L298N_in3 7   // Motor B Direction 1
   #define L298N_in4 8   // Motor B Direction 2
   
   #define right_encoder_phaseA 3  // Right encoder (interrupt pin)
   #define right_encoder_phaseB 5
   #define left_encoder_phaseA 2   // Left encoder (interrupt pin)
   #define left_encoder_phaseB 4
   ```

5. **Upload to Arduino:**
   - Click "Upload" button (or Ctrl+U)
   - Wait for "Done uploading" message

6. **Verify upload:**
   - Open Serial Monitor (Ctrl+Shift+M)
   - Set baud rate to **57600**
   - You should see encoder feedback messages

---

### **Step 2: Set Up Serial Port Permissions** 🔐

1. **Find the Arduino port:**
   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```
   
   You'll see something like:
   ```
   crw-rw---- 1 root dialout 188, 0 Oct  7 10:30 /dev/ttyUSB0
   ```

2. **Add yourself to dialout group (one-time setup):**
   ```bash
   sudo usermod -a -G dialout $USER
   ```

3. **Apply group changes (choose one):**
   - **Option A**: Logout and login again
   - **Option B**: Reboot
   - **Option C**: Temporary fix for current session:
     ```bash
     sudo chmod 666 /dev/ttyUSB0
     ```

4. **Verify permissions:**
   ```bash
   groups
   # Should show 'dialout' in the list
   ```

---

### **Step 3: Update Port Configuration** ⚙️

1. **Check which port Arduino is connected to:**
   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```

2. **Update the port in your URDF:**
   
   Edit: `src/smrr_description/urdf/smrr_ros2_control.urdf.xacro`
   
   Find this section:
   ```xml
   <xacro:unless value="$(arg is_sim)">
       <hardware>
           <plugin>smrr_base_controller/BaseController</plugin>
           <param name="port">/dev/ttyUSB0</param>  <!-- Update this if needed -->
       </hardware>
   </xacro:unless>
   ```
   
   Change `/dev/ttyUSB0` to match your actual port (e.g., `/dev/ttyACM0`)

---

### **Step 4: Build the Workspace** 🔨

```bash
cd ~/navigation_ws
colcon build
source install/setup.bash
```

**Important:** Always source after building!

---

### **Step 5: Launch Hardware Mode** 🚀

```bash
ros2 launch smrr_description simple_hardware.launch.py
```

**What this does:**
- ✅ Loads robot description with `is_sim:=false`
- ✅ Starts `ros2_control_node` with hardware interface
- ✅ Connects to Arduino via serial port
- ✅ Spawns all controllers
- ✅ Ready to receive commands!

---

## 📊 Verification Steps

### **Check 1: Hardware Interface Loaded**
```bash
ros2 control list_hardware_interfaces
```

**Expected output:**
```
command interfaces:
  left_wheel_joint/velocity [available] [claimed]
  right_wheel_joint/velocity [available] [claimed]
  shoulder_r_joint/position [available] [claimed]
  ...

state interfaces:
  left_wheel_joint/position
  left_wheel_joint/velocity
  right_wheel_joint/position
  right_wheel_joint/velocity
  ...
```

### **Check 2: Controllers Active**
```bash
ros2 control list_controllers
```

**Expected output:**
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
diff_drive_controller[diff_drive_controller/DiffDriveController] active
arm_controller[position_controllers/JointGroupPositionController] active
```

### **Check 3: Joint States Publishing**
```bash
ros2 topic echo /joint_states --once
```

**Expected:** You should see current encoder positions

### **Check 4: Odometry Publishing**
```bash
ros2 topic echo /diff_drive_controller/odom --once
```

**Expected:** You should see odometry data

---

## 🎮 Testing Robot Movement

### **Test 1: Send Velocity Command**
```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}" \
  --once
```

**Expected:** Robot should move forward slowly

### **Test 2: Keyboard Teleoperation**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel
```

**Controls:**
- `i` = forward
- `,` = backward
- `j` = rotate left
- `l` = rotate right
- `k` = stop

### **Test 3: Monitor Encoders in Real-Time**
```bash
# Terminal 1: Drive the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 2: Watch encoder counts
ros2 topic echo /joint_states
```

---

## 🐛 Troubleshooting

### ❌ **Error: "Permission denied" on serial port**

**Solution:**
```bash
sudo chmod 666 /dev/ttyUSB0
```
Or add yourself to dialout group and reboot.

---

### ❌ **Error: "Hardware component 'RobotSystem' failed to initialize"**

**Possible causes:**
1. **Wrong serial port:** Check port in URDF matches actual port
   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```

2. **Arduino not responding:** 
   - Open Arduino Serial Monitor (57600 baud)
   - Check if you see encoder feedback
   - Re-upload firmware if needed

3. **Hardware interface not built:**
   ```bash
   colcon build --packages-select smrr_base_controller
   source install/setup.bash
   ```

---

### ❌ **Error: "Controller manager not responding"**

**Solution:**
```bash
# Kill any existing controller manager
pkill -9 ros2_control_node

# Restart launch
ros2 launch smrr_description simple_hardware.launch.py
```

---

### ❌ **Robot not moving / No encoder feedback**

**Check:**
1. **Motor driver powered?** Verify L298N has power supply connected
2. **Encoder wiring:** Check encoder pins match firmware definitions
3. **Serial communication:** Open Arduino Serial Monitor and send test command manually

**Test Arduino manually:**
```bash
# Install screen if needed
sudo apt install screen

# Connect to Arduino
screen /dev/ttyUSB0 57600

# Send test command (right wheel forward at 1.0 rad/s)
r+01.00

# Send test command (left wheel forward at 1.0 rad/s)
l+01.00

# Stop both wheels
r+00.00
l+00.00

# Exit screen: Ctrl+A then K then Y
```

---

### ❌ **"use_sim_time" warnings**

**Check all nodes have use_sim_time:=false:**
```bash
ros2 param get /robot_state_publisher use_sim_time
ros2 param get /controller_manager use_sim_time
```

Both should return: `Boolean value is: False`

---

## 📋 Quick Reference: Complete Workflow

```bash
# 1. Upload firmware to Arduino (Arduino IDE)
#    File: src/smrr_base_controller/firmware/firmware.ino

# 2. Set permissions (one-time)
sudo usermod -a -G dialout $USER
# Then logout/login

# 3. Build workspace
cd ~/navigation_ws
colcon build
source install/setup.bash

# 4. Launch hardware
ros2 launch smrr_description simple_hardware.launch.py

# 5. Test movement (new terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel
```

---

## ⚡ That's It!

**Yes, you're correct! The process is:**

1. ✅ Upload `firmware.ino` to Arduino
2. ✅ Connect Arduino to PC (check port with `ls /dev/ttyUSB*`)
3. ✅ Ensure port in URDF matches actual port
4. ✅ Build workspace
5. ✅ Run `ros2 launch smrr_description simple_hardware.launch.py`

**And the robot should work!** 🎉

---

## 📞 Need Help?

If something doesn't work:
1. Check the troubleshooting section above
2. Verify each verification step
3. Test Arduino communication manually with `screen`
4. Check hardware connections (motors, encoders, power)
