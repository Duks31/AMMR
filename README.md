# CIKA - Autonomous Mobile Manipulation Robot for Waste Collection and Sorting

## Prerequisites
- Windows 10/11 with WSL2
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