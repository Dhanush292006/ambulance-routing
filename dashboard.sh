#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
exec "$ROOT/.venv/bin/python" -m streamlit run "$ROOT/src/ambulance_dashboard/dashboard.py"
