#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
export PYTHONPATH="$ROOT/src/ambulance_core:${PYTHONPATH:-}"
if [[ "${1:-}" == "--ros" || "${1:-}" == "--gazebo" ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$ROOT/install/setup.bash"
  ros2 launch ambulance_bringup ambulance_system.launch.py use_gazebo:=$([[ "${1:-}" == "--gazebo" ]] && echo true || echo false)
else
  python3 -m ambulance_core.demo --scenario "${2:-research_demo}" --planner dt-aagr
fi
