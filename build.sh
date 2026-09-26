#!/usr/bin/env bash
set -eo pipefail
# ROS setup scripts are not nounset-safe on some Jazzy installations.
set +u
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
