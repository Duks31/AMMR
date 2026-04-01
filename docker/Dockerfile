FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    git wget curl \
    # Nav2
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # RTAB-Map (localization only on robot — no Gazebo)
    ros-humble-rtabmap-ros \
    # ros2_control
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    # micro-ROS (ESP32 comms)
    ros-humble-micro-ros-agent \
    # OAK-D
    ros-humble-depthai-ros \
    # Robot description
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    # Joystick teleop
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    # Laser filters
    ros-humble-laser-filters \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies — CPU only PyTorch for RPi
RUN pip3 install --no-cache-dir --default-timeout=1000 \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu

RUN pip3 install --no-cache-dir --default-timeout=1000 numpy Pillow
RUN pip3 install --no-cache-dir --default-timeout=1000 depthai blobconverter
RUN pip3 install --no-cache-dir --default-timeout=1000 opencv-python-headless
RUN pip3 install --no-cache-dir --default-timeout=1000 ultralytics

WORKDIR /cika_ws

COPY cika_ws/src/ src/
COPY cika_ws/config/ config/

RUN rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src \
    --ignore-src -r -y \
    --skip-keys="ros_gz_sim ros_gz_bridge gz_ros2_control ign_ros2_control \
                 ignition-fortress robot_arm_description robot_arm_controller \
                 robot_arm_moveit robot_arm_msgs robot_arm_remote"

RUN . /opt/ros/humble/setup.sh && \
    colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-skip \
        robot_arm_description \
        robot_arm_controller \
        robot_arm_moveit \
        robot_arm_msgs \
        robot_arm_remote

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /cika_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=31" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]