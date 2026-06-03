#!/bin/bash
set -euo pipefail

# ═══════════════════════════════════════════════════════════
#  CIKA Pi Setup Script — ROS2 Humble
#  Target: Raspberry Pi 4B, Ubuntu 22.04 (Jammy)
# ═══════════════════════════════════════════════════════════

# ── Pinned versions ────────────────────────────────────────
ROS_DISTRO="humble"
DEPTHAI_VERSION="3.6.1"
NUMPY_VERSION="1.26.4"       # numpy 2.x breaks several ROS2 nodes
OPENCV_VERSION="4.9.0.80"
# ──────────────────────────────────────────────────────────

# ── Step counter ───────────────────────────────────────────
STEP=0
TOTAL=7

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
echo "  CIKA Pi Setup — ROS2 Humble"
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
# 2. ROS2 Humble
# ══════════════════════════════════════════════════════════
step "Installing ROS2 ${ROS_DISTRO}"

if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
else
  echo "ROS2 GPG key already present — skipping."
fi

if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
else
  echo "ROS2 apt source already present — skipping."
fi

sudo apt-get update -q
sudo apt-get install -y -q \
  ros-${ROS_DISTRO}-ros-base \
  python3-colcon-common-extensions \
  python3-rosdep

# ══════════════════════════════════════════════════════════
# 3. Nav2 / SLAM / Perception stack
# ══════════════════════════════════════════════════════════
step "Installing Nav2, RTAB-Map, SLAM & perception stack"
sudo apt-get install -y -q \
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
# 4. RPLIDAR C1 + ESP32 udev rules
# ══════════════════════════════════════════════════════════
step "Installing RPLIDAR C1 + ESP32 udev rules"

SERIAL_RULES_FILE=/etc/udev/rules.d/99-cika-serial.rules

if [[ ! -f "$SERIAL_RULES_FILE" ]]; then
  sudo tee "$SERIAL_RULES_FILE" > /dev/null << 'EOF'
# RPLIDAR C1 — CP2102N, unique serial
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="8a7648b4aed5ef119414694b49d2c684", MODE:="0666", SYMLINK+="rplidar"

# ESP32 #1 (micro-ROS bridge) — CP2102 clone, serial 0001
# NOTE: if ESP32 #2 also has serial 0001, switch to KERNELS== port-path matching
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", SYMLINK+="esp32"
EOF
  sudo udevadm control --reload-rules && sudo udevadm trigger
  echo "cika serial udev rules installed → /dev/rplidar, /dev/esp32"
else
  echo "cika serial udev rules already present — skipping."
fi

# ══════════════════════════════════════════════════════════
# 5. OAK-D Lite (DepthAI)
# ══════════════════════════════════════════════════════════
step "Installing DepthAI ${DEPTHAI_VERSION} + Python ML deps"
sudo apt-get install -y -q libusb-1.0-0-dev

pip3 install \
  "depthai==${DEPTHAI_VERSION}" \
  "numpy==${NUMPY_VERSION}" \
  "opencv-python-headless==${OPENCV_VERSION}"

# USB udev rule for Myriad X (OAK-D Lite)
MOVIDIUS_RULES_FILE=/etc/udev/rules.d/80-movidius.rules
MOVIDIUS_RULE='SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"'

if ! grep -qF "03e7" "$MOVIDIUS_RULES_FILE" 2>/dev/null; then
  echo "$MOVIDIUS_RULE" | sudo tee "$MOVIDIUS_RULES_FILE" > /dev/null
  sudo udevadm control --reload-rules && sudo udevadm trigger
  echo "OAK-D Lite udev rule installed."
else
  echo "OAK-D Lite udev rule already present — skipping."
fi

# ══════════════════════════════════════════════════════════
# 6. rosdep
# ══════════════════════════════════════════════════════════
step "Initialising rosdep"
sudo rosdep init 2>/dev/null || echo "(rosdep already initialised — skipping)"
rosdep update

# ══════════════════════════════════════════════════════════
# 7. Shell environment
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