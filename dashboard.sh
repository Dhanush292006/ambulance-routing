#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
exec "$ROOT/.venv/bin/python" -m streamlit run "$ROOT/src/ambulance_dashboard/dashboard.py" --server.address 127.0.0.1 --server.port 8501
