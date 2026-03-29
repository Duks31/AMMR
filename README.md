# AMR-Robot

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

amr_ws/src/
├── cika_description/        # URDF, meshes, ros2_control config
├── cika_bringup/            # Top-level launch files
├── cika_navigation/         # Nav2, SLAM, RTAB-Map
└── cika_manipulation/       # MoveIt2, arm planning, gripper