#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Only source workspace if it has been built
if [ -f /cika_ws/install/setup.bash ]; then
    source /cika_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [ "$USE_SIM_TIME" = "true" ]; then
    export ROS_SIM_TIME=true
fi

exec "$@"