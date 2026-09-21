#!/usr/bin/env bash
set -euo pipefail
VENV_DIR="$(cd "$(dirname "$0")" && pwd)/.venv"
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  python3 -m venv "$VENV_DIR"
fi
"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install streamlit pandas plotly matplotlib psutil
echo "Dependencies installed in .venv. Start: .venv/bin/python -m streamlit run src/ambulance_dashboard/dashboard.py"
