# CIKA - Autonomous Mobile Manipulation Robot (AMMR) for Waste Collection and Sorting

![cika](/static/cika.jpeg)


## Project Structure 
```
amr_ws/src/
├── cika_description/        # URDF, meshes, ros2_control config
├── cika_bringup/            # Top-level launch files
├── cika_navigation/         # Nav2, SLAM, RTAB-Map
└── cika_manipulation/       # MoveIt2, arm planning, gripper
└── cika_perception/         # Object detection, point cloud processing
└── cika_task_manager/       # High-level task execution, state machine, connections to perception and navigation 
```

## Mapping and Navigation

```bash
# SLAM mode — builds and saves the map database
ros2 launch cika_bringup cika_full_sim.launch.py mode:=slam gui:=false

# Navigation mode — loads saved map and runs Nav2
ros2 launch cika_bringup cika_full_sim.launch.py mode:=navigation gui:=false
```

After a mapping run, the database is saved automatically to
`cika_navigation/maps/cika_map.db`. To switch to navigation mode,
simply relaunch with `mode:=navigation` — no manual copying needed.

> **Warning:** launching `mode:=slam` deletes the existing database on start.
> Back up `cika_warehouse.db` before remapping if you want to keep a good map.
```bash
cp src/cika_navigation/maps/cika_warehouse.db \
   src/cika_navigation/maps/cika_warehouse_backup.db
```

## Perception

```bash
# Simulation — YOLOv8 detector + EfficientNet-B0 classifier, simulated OAK-D Lite
ros2 launch cika_perception perception.launch.py backend:=sim use_sim_time:=true

# Hardware — same pipeline, tuned for Raspberry Pi 4B + physical OAK-D Lite
ros2 launch cika_perception perception.launch.py backend:=hardware use_sim_time:=false
```

The perception pipeline runs two nodes in sequence:

- **detector_node** — runs YOLOv8 on the RGB stream, back-projects detections to 3D using the depth image and camera intrinsics, and publishes on `/cika/waste_detections`
- **classifier_node** — crops each detection bounding box from the RGB frame, runs EfficientNet-B0 to verify the `recyclable` / `non_recyclable` label, and republishes enriched detections on `/cika/waste_detections_classified`

A debug image with bounding boxes and labels is published on `/cika/perception/debug_image` whenever a subscriber is present.

> **Note:** `backend:=sim` and `backend:=hardware` load different parameter configs
> (`perception_sim.yaml` / `perception_hardware.yaml`) — inference rate is 5 Hz in
> sim and 2 Hz on hardware to stay within RPi 4B CPU limits.


## Task Manager

```bash
# Launch task manager (simulation)
ros2 launch cika_task_manager cika_task_manager.launch.py use_sim_time:=true

# Launch task manager (hardware)
ros2 launch cika_task_manager cika_task_manager.launch.py use_sim_time:=false
```

The task manager bridges perception, navigation, and manipulation through a five-state machine:

- **IDLE** — listens to `/cika/waste_detections_classified` and scores candidates by combined detector + classifier confidence
- **SELECTING** — picks the highest-scoring detection with a valid 3D position and triggers navigation
- **NAVIGATING** — sends a `NavigateToPose` goal to Nav2, stopping `approach_distance` (0.4 m) short of the object; retries once on failure before returning to IDLE
- **VERIFYING** — confirms the target is still visible over 5 consecutive detection frames (~3 seconds) before committing to a pick
- **PICKING** — sends a `PickAndDispose` action goal to `cika_manipulator`

## Visualization (Foxglove Studio)

Foxglove Studio is installed system-wide via apt and launched standalone — no ROS2 launch file is needed.

```bash
# Launch Foxglove Studio
foxglove-studio
```

Once open, connect to your running ROS2 stack via the **Foxglove WebSocket** bridge:

```bash
# Start the bridge (run this alongside your other nodes)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Then in Foxglove Studio, select **Open Connection → Foxglove WebSocket** and enter:
ws://<_ip_address_>:8765

## Physical Robot 

``` bash
# Teleop only (default)
ros2 launch cika_bringup cika_real.launch.py nav:=false

# Compute launch
ros2 launch cika_bringup cika_compute.launch.py 

# SLAM — build a map
ros2 launch cika_navigation cika_nav.launch.py mode:=slam use_sim_time:=false

# Navigation — autonomous with saved map
ros2 launch cika_navigation cika_nav.launch.py mode:=navigation use_sim_time:=false
```

# Cika Hardware Arm — How to Run

## 1. Upload Firmware to ESP32
- Open `cika_arm_esp32.ino` in Arduino IDE
- Board: `ESP32 Dev Module` | Port: `/dev/ttyUSB1`
- Click Upload, then press EN/RESET on ESP32

## 2. Give Port Permission
```bash
sudo chmod a+rw /dev/ttyUSB1
```

## 3. Build
```bash
cd ~/AMMR/cika_ws
colcon build --packages-select cika_hardware_arm
source install/setup.bash
```

## 4. Launch
```bash
ros2 launch cika_bringup cika_real_arm.launch.py
```
```bash
pip install sudo apt install xvfb
xvfb-run -a ros2 launch cika_manipulator moveit.launch.pi.py

```bash
ros2 run cika_manipulator paper_client 
```

## 5. Verify
```bash
ros2 control list_controllers
```
Expected:
```
joint_state_broadcaster   active
arm_controller            active
```

## 6. Monitor ESP32 (optional)
```bash
screen /dev/ttyUSB1 115200
# Close with: Ctrl+A then K → y
```

---

## If Something Goes Wrong
| Problem | Fix |
|---|---|
| Port busy | Close Arduino Serial Monitor or screen |
| Controllers fail | Unplug and replug ESP32, relaunch |
| Wrong port | Change `ttyUSB1` to `ttyUSB0` in `cika_hardware_arm.cpp` and rebuild |
