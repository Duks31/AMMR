#!/bin/bash
set -e

echo "=========================================="
echo " CIKA Pi Setup Script - ROS2 Humble"
echo "=========================================="

echo "[1/8] Updating system..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release git build-essential cmake python3-pip

echo "[2/8] Installing ROS2 Humble..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions python3-rosdep

echo "[3/8] Installing Nav2, RTAB-Map, SLAM deps..."
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-rtabmap-ros \
  ros-humble-robot-localization \
  ros-humble-imu-filter-madgwick \
  ros-humble-slam-toolbox \
  ros-humble-twist-mux \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  python3-serial

echo "[4/8] Installing RPLIDAR driver..."
sudo apt install -y ros-humble-rplidar-ros

echo "[5/8] Installing depthai..."
sudo apt install -y libusb-1.0-0-dev
pip3 install depthai --break-system-packages
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "[6/8] Installing IMU filter dependencies..."
sudo apt install ros-humble-imu-filter-madgwick

echo "[7/8] Installing Python ML dependencies..."
pip3 install --break-system-packages numpy opencv-python-headless

echo "[8/8] Initialising rosdep..."
sudo rosdep init || true
rosdep update

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

echo ""
echo "=========================================="
echo " Done! Next:"
echo "  git clone https://github.com/Duks31/AMMR ~/AMMR"
echo "  cd ~/AMMR/cika_ws && colcon build"
echo "=========================================="