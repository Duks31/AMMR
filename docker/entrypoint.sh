#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /amr_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec "$@"