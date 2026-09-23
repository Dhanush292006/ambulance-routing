#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
exec ros2 run ambulance_core ambulance_nav_goal "$@"
