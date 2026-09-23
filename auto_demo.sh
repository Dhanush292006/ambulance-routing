#!/usr/bin/env bash
# One-command research demonstration. Ctrl+C stops Gazebo and the dashboard.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
./build.sh
./run.sh --demo
./gazebo.sh &
GAZEBO_PID=$!
cleanup() {
  kill "$GAZEBO_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
sleep 4
"$ROOT/.venv/bin/python" -m streamlit run "$ROOT/src/ambulance_dashboard/dashboard.py"
