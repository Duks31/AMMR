#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Build workspace if not already built
if [ ! -f /cika_ws/install/setup.bash ]; then
    echo "[entrypoint] Building workspace..."
    cd /cika_ws
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

source /cika_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [ "$USE_SIM_TIME" = "true" ]; then
    export ROS_SIM_TIME=true
fi

exec "$@"