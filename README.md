# CIKA - Autonomous Mobile Manipulation Robot for Waste Collection and Sorting

## Prerequisites
- Linux OR Windows 10/11 with WSL2 (Ubuntu 22.04 LTS recommended)
- Docker Desktop with WSL2 backend enabled
- Git

## Getting Started

### 1. Clone the repo
`git clone https://github.com/yourrepo/AMMR.git`

### 2. Build dev environment
`cd AMMR/docker`
`docker compose -f docker-compose.dev.yaml build`

### 3. Start containers
`docker compose -f docker-compose.dev.yaml up -d`

### 4. Enter a container
`docker compose -f docker-compose.dev.yaml exec {service name} bash`

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
- **PICKING** — sends a `PickAndDispose` action goal to `cika_manipulator`; falls back to a timed stub until the manipulator package is ready

Current task state is published on `/cika/task_state` at 1 Hz, useful for RViz overlays and debugging.

> **Note:** the arm pick is currently **stubbed** — `cika_manipulator` is not yet integrated.
> The stub simulates a 3-second pick and returns to IDLE automatically.
> Swap the stub by building `cika_manipulator` with a `pick_and_dispose` action server.

