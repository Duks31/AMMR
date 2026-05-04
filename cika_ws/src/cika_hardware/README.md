# cika_hardware

ros2_control hardware interface plugins for the cika AMR.

## Package contents

```
cika_hardware/
├── include/cika_hardware/
│   └── cika_drive_interface.hpp     # Drive base SystemInterface header
├── src/
│   └── cika_drive_interface.cpp     # Drive base implementation
│   └── drive_esp32_firmware.cpp     # ESP32 firmware
├── cika_hardware_plugins.xml        # pluginlib registration
├── CMakeLists.txt
└── package.xml
```

## What this package does

Replaces the `gz_ros2_control/GazeboSimSystem` plugin with a real hardware
backend when `use_sim:=false`.  The `skid_steer_controller` (diff_drive_controller)
and Nav2 remain completely unchanged — only the bottom layer swaps.

```
Nav2
 └─ skid_steer_controller (diff_drive_controller)
     └─ CikaDriveInterface          ← this package
         ├─ write()  →  publishes /cika/drive/wheel_vel_cmd  →  ESP32  →  BTS7960
         └─ read()   ←  subscribes /cika/odom               ←  ESP32  ←  encoders
```

## Build

```bash
cd ~/code/ws/AMMR/cika_ws
cp -r /path/to/cika_hardware src/
colcon build --packages-select cika_hardware
source install/setup.bash
```

## URDF update

Make sure your launch file passes `use_sim` as a xacro argument:
- Sim:  `use_sim:=true`   → loads GazeboSimSystem (existing behaviour)
- Real: `use_sim:=false`  → loads CikaDriveInterface

## ESP32 firmware
Update these values before flashing:
1. `ENCODER_CPR` — confirm once you have the motor datasheet
2. GPIO pin defines (`L_EN`, `L_RPWM`, `L_LPWM`, `R_EN`, `R_RPWM`, `R_LPWM`)
3. Encoder GPIO pins (`ENCODER_LEFT_A`, `ENCODER_RIGHT_A`)
4. Uncomment the ISR functions and `attachInterrupt()` calls once wired

## Testing (no motor wiring needed yet)

### Step 1 — compile the ROS2 side
```bash
colcon build --packages-select cika_hardware
```

### Step 2 — flash ESP32 and start agent
```bash
# Flash via PlatformIO
pio run --target upload

# Start micro-ROS agent on RPi/laptop
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### Step 3 — verify topics exist
```bash
ros2 node list
# should show: /cika/cika_drive_esp32

ros2 topic list
# should show: /cika/odom  and  /cika/drive/wheel_vel_cmd

ros2 topic echo /cika/odom
```

### Step 4 — start real robot bringup
```bash
ros2 launch cika_bringup cika_bringup.launch.py use_sim:=false
```

### Step 5 — send a test command
```bash
ros2 topic pub /cika/drive/wheel_vel_cmd std_msgs/msg/Float32MultiArray \
  "{data: [1.0, 1.0, 1.0, 1.0]}" --once
```