FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# ─── System dependencies ───────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    git wget curl \
    # # Nav2 + SLAM
    # ros-humble-navigation2 \
    # ros-humble-nav2-bringup \
    # ros-humble-slam-toolbox \
    # # MoveIt2
    # ros-humble-moveit \
    # ros-humble-moveit-ros-planning-interface \
    # ros2_control
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    # # micro-ROS
    # ros-humble-micro-ros-agent \
    # # OAK-D
    # ros-humble-depthai-ros \
    # Robot description
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# ─── Python dependencies ───────────────────────────────────────────
# CPU only PyTorch (Pi has no GPU)
RUN pip3 install --no-cache-dir --default-timeout=1000\
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu

# Everything else
RUN pip3 install \
    depthai \
    blobconverter \
    ultralytics \
    numpy \
    opencv-python-headless \
    Pillow

# ─── Workspace setup ───────────────────────────────────────────────
WORKDIR /amr_ws

COPY amr_ws/src/ src/
COPY amr_ws/config/ config/
COPY amr_ws/models/ models/

# rosdep - skip Gazebo packages (not needed on robot)
RUN rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src \
    --ignore-src -r -y \
    --skip-keys="ros_gz_sim ros_gz_bridge gz_ros2_control ign_ros2_control"

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /amr_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]