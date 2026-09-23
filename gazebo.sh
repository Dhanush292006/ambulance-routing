#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
exec ros2 launch ambulance_bringup ambulance_system.launch.py use_gazebo:=true use_navigation:=true
