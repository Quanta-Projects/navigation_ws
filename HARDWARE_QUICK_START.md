# Hardware Deployment - Simple Visual Guide

```
┌─────────────────────────────────────────────────────────────────┐
│                    HARDWARE DEPLOYMENT FLOW                      │
└─────────────────────────────────────────────────────────────────┘

STEP 1: Arduino Firmware
┌────────────────────────────────────┐
│  Open Arduino IDE                   │
│  ↓                                  │
│  Load: firmware/firmware.ino        │
│  ↓                                  │
│  Select: Board & Port               │
│  ↓                                  │
│  Click: Upload                      │
│  ↓                                  │
│  ✓ Done uploading                   │
└────────────────────────────────────┘
         ↓
         ↓
STEP 2: Connect Hardware
┌────────────────────────────────────┐
│  Arduino → USB → Computer           │
│                                     │
│  Check port:                        │
│  $ ls /dev/ttyUSB*                  │
│    /dev/ttyUSB0  ← Your port       │
└────────────────────────────────────┘
         ↓
         ↓
STEP 3: Permissions (One-time)
┌────────────────────────────────────┐
│  $ sudo usermod -a -G dialout $USER │
│  $ logout/login or reboot           │
│                                     │
│  OR quick fix:                      │
│  $ sudo chmod 666 /dev/ttyUSB0      │
└────────────────────────────────────┘
         ↓
         ↓
STEP 4: Build Workspace
┌────────────────────────────────────┐
│  $ cd ~/navigation_ws               │
│  $ colcon build                     │
│  $ source install/setup.bash        │
└────────────────────────────────────┘
         ↓
         ↓
STEP 5: Launch!
┌────────────────────────────────────┐
│  $ ros2 launch smrr_description \   │
│      simple_hardware.launch.py      │
│                                     │
│  ✓ Robot ready!                     │
└────────────────────────────────────┘
         ↓
         ↓
STEP 6: Test Movement
┌────────────────────────────────────┐
│  $ ros2 run teleop_twist_keyboard \ │
│      teleop_twist_keyboard          │
│                                     │
│  Use: i/j/k/l keys to drive         │
└────────────────────────────────────┘
```

---

## What Happens When You Launch?

```
simple_hardware.launch.py
    │
    ├─→ robot_state_publisher
    │   └─→ Publishes TF transforms
    │
    ├─→ ros2_control_node
    │   ├─→ Loads: smrr_base_controller/BaseController
    │   ├─→ Opens: /dev/ttyUSB0 (serial port)
    │   └─→ Reads: arm_controller.yaml
    │
    └─→ controller.launch.py (is_sim:=False)
        ├─→ Spawns: joint_state_broadcaster
        ├─→ Spawns: arm_controller
        ├─→ Spawns: diff_drive_controller
        ├─→ Starts: twist_mux
        └─→ Starts: twist_stamper

Result: Robot listening on /diff_drive_controller/cmd_vel
```

---

## Communication Flow

```
Your Command
    ↓
/cmd_vel topic
    ↓
twist_mux (arbitrates multiple cmd_vel sources)
    ↓
twist_stamper (adds timestamp)
    ↓
/diff_drive_controller/cmd_vel
    ↓
DiffDriveController (converts twist → wheel velocities)
    ↓
ros2_control hardware interface
    ↓
smrr_base_controller/BaseController
    ↓
Serial Port (/dev/ttyUSB0)
    ↓
Arduino (firmware.ino)
    ↓
Motor Driver (L298N)
    ↓
Motors Turn!
    ↓
Encoders measure rotation
    ↓
Feedback flows back up ←←←
```

---

## File Locations Quick Map

```
navigation_ws/
├── src/
│   ├── smrr_base_controller/
│   │   ├── firmware/
│   │   │   └── firmware.ino          ← Upload to Arduino
│   │   └── src/
│   │       └── base_controller.cpp    ← Hardware interface
│   │
│   ├── smrr_description/
│   │   ├── urdf/
│   │   │   └── smrr_ros2_control.urdf.xacro  ← Port config
│   │   └── launch/
│   │       └── simple_hardware.launch.py     ← Run this!
│   │
│   └── smrr_controller/
│       └── config/
│           └── arm_controller.yaml    ← Controller params
│
└── HARDWARE_DEPLOYMENT_CHECKLIST.md  ← Full guide
```

---

## Quick Answer to Your Question

**Q: "So I just have to upload firmware.ino to Arduino, connect to correct port, and run simple_hardware.launch.py?"**

**A: YES! Exactly! ✅**

But don't forget:
1. ✅ Upload `firmware.ino` via Arduino IDE
2. ✅ Connect Arduino via USB
3. ✅ Check port: `ls /dev/ttyUSB*` → should show `/dev/ttyUSB0`
4. ✅ **One-time setup**: `sudo usermod -a -G dialout $USER` then logout/login
5. ✅ Build: `cd ~/navigation_ws && colcon build && source install/setup.bash`
6. ✅ Launch: `ros2 launch smrr_description simple_hardware.launch.py`

That's it! 🎉

---

## The Difference: Simulation vs Hardware

| What? | Simulation | Hardware |
|-------|-----------|----------|
| **Upload firmware?** | ❌ No | ✅ Yes (firmware.ino to Arduino) |
| **Arduino needed?** | ❌ No | ✅ Yes (connected via USB) |
| **Launch command** | `smrr_world_navigation.launch.py` | `simple_hardware.launch.py` |
| **Backend** | Gazebo physics | Real motors & encoders |
| **Serial port** | N/A | /dev/ttyUSB0 |

---

## TL;DR - Absolute Minimum Steps

```bash
# 1. Upload firmware (Arduino IDE)
#    Open: src/smrr_base_controller/firmware/firmware.ino
#    Click: Upload

# 2. One-time permissions
sudo usermod -a -G dialout $USER
# Logout/login

# 3. Build & Launch
cd ~/navigation_ws
colcon build
source install/setup.bash
ros2 launch smrr_description simple_hardware.launch.py

# 4. Drive (new terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Done!** 🚀
