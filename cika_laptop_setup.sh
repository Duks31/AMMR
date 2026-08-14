#!/bin/bash
set -euo pipefail

# ═══════════════════════════════════════════════════════════
#  CIKA Dev Setup Script — ROS2 Humble
#  Target: x86_64 laptop, Ubuntu 22.04 (Jammy), sim/code-only
#  (no physical sensors — no udev rules needed)
# ENSURE ROS2 HUMBLE IS INSTALLED BEFORE RUNNING THIS SCRIPT
# ═══════════════════════════════════════════════════════════

# ── Pinned versions ────────────────────────────────────────
ROS_DISTRO="humble"
DEPTHAI_VERSION="3.6.1"
NUMPY_VERSION="1.26.4"       # numpy 2.x breaks several ROS2 nodes
OPENCV_VERSION="4.9.0.80"
# ──────────────────────────────────────────────────────────

# ── Step counter ───────────────────────────────────────────
STEP=0
TOTAL=6

step() {
  STEP=$((STEP + 1))
  echo ""
  echo "══════════════════════════════════════════"
  echo "  [${STEP}/${TOTAL}] $1"
  echo "══════════════════════════════════════════"
}

# ── Guards ─────────────────────────────────────────────────
if [[ "$(lsb_release -cs 2>/dev/null)" != "jammy" ]]; then
  echo "ERROR: This script requires Ubuntu 22.04 (Jammy)."
  echo "       Detected: $(lsb_release -cs 2>/dev/null || echo 'unknown')"
  exit 1
fi

if [[ "$EUID" -eq 0 ]]; then
  echo "ERROR: Do not run as root. Run as a normal user with sudo access."
  exit 1
fi

# ── Fast-path: skip if already fully set up ────────────────
if [[ "${FORCE:-0}" != "1" ]] && \
   dpkg -l "ros-${ROS_DISTRO}-ros-base" &>/dev/null && \
   python3 -c "import depthai" &>/dev/null && \
   [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
  echo ""
  echo "Environment already set up — nothing to do."
  echo "To force a full reinstall: FORCE=1 bash $0"
  exit 0
fi

echo ""
echo "══════════════════════════════════════════"
echo "  CIKA Dev Setup — ROS2 Humble (laptop)"
echo "══════════════════════════════════════════"

# ── Helper: add line to ~/.bashrc only if not already there ─
add_if_missing() {
  grep -qxF "$1" "$HOME/.bashrc" || echo "$1" >> "$HOME/.bashrc"
}

# ══════════════════════════════════════════════════════════
# 1. System update
# ══════════════════════════════════════════════════════════
step "Updating system packages"
sudo apt-get update -q
sudo apt-get upgrade -y -q
sudo apt-get install -y -q \
  curl gnupg2 lsb-release git \
  build-essential cmake python3-pip wget

# ══════════════════════════════════════════════════════════
# 2. Nav2 / SLAM / Perception stack
# ══════════════════════════════════════════════════════════
step "Installing Nav2, RTAB-Map, SLAM & perception stack"
sudo apt-get install -y -q \

  `# ── Gazebo ───────────────` \
  ros-${ROS_DISTRO}-ros2-control \
  `# ── Navigation ──────────────────────────` \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-nav2-msgs \
  `# ── SLAM ────────────────────────────────` \
  ros-${ROS_DISTRO}-rtabmap-ros \
  ros-${ROS_DISTRO}-slam-toolbox \
  `# ── Sensor fusion / IMU ─────────────────` \
  ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-imu-filter-madgwick \
  `# ── Teleop / input ──────────────────────` \
  ros-${ROS_DISTRO}-twist-mux \
  ros-${ROS_DISTRO}-joy \
  ros-${ROS_DISTRO}-teleop-twist-joy \
  `# ── ros2_control ────────────────────────` \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-diff-drive-controller \
  ros-${ROS_DISTRO}-joint-state-broadcaster \
  `# ── Manipulation (MoveIt 2) ─────────────` \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-ros-perception \
  `# ── Camera / vision (OAK-D Lite) ────────` \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-vision-opencv \
  `# ── LiDAR / serial ──────────────────────` \
  ros-${ROS_DISTRO}-laser-filters \
  python3-serial \
  `# ── Debug / visualisation ───────────────` \
  ros-${ROS_DISTRO}-foxglove-bridge

# ══════════════════════════════════════════════════════════
# 3. DepthAI + Python ML deps (library only — no OAK-D attached)
# ══════════════════════════════════════════════════════════
step "Installing DepthAI ${DEPTHAI_VERSION} + Python ML deps"
sudo apt-get install -y -q libusb-1.0-0-dev

pip3 install \
  "depthai==${DEPTHAI_VERSION}" \
  "numpy==${NUMPY_VERSION}" \
  "opencv-python-headless==${OPENCV_VERSION}"

# ══════════════════════════════════════════════════════════
# 4. rosdep
# ══════════════════════════════════════════════════════════
step "Initialising rosdep"
sudo rosdep init 2>/dev/null || echo "(rosdep already initialised — skipping)"
rosdep update

# ══════════════════════════════════════════════════════════
# 5. Shell environment
# ══════════════════════════════════════════════════════════
step "Configuring ~/.bashrc"
add_if_missing "source /opt/ros/${ROS_DISTRO}/setup.bash"
add_if_missing "export ROS_DOMAIN_ID=0"
echo "~/.bashrc updated."

# ══════════════════════════════════════════════════════════
echo ""
echo "══════════════════════════════════════════"
echo "  Setup complete!"
echo ""
echo "  Next steps:"
echo ""
echo "  1. Reload your shell:"
echo "     source ~/.bashrc"
echo ""
echo "  2. Clone the workspace (if not already):"
echo "     git clone https://github.com/Duks31/AMMR ~/AMMR"
echo ""
echo "  3. Install ROS deps from package.xml files:"
echo "     cd ~/AMMR/cika_ws"
echo "     rosdep install --from-paths src --ignore-src -r -y"
echo ""
echo "  4. Build:"
echo "     colcon build"
echo "══════════════════════════════════════════"   