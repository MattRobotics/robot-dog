#!/usr/bin/env bash
# Isolated Python environment of the MuJoCo twin (official PyPI wheels, pinned in
# requirements.txt). Outside the repository and the ESP32 toolchain
# (~/.arduino15, arduino-cli and the system python3 are not touched).
# Usage: setup_venv.sh [venv dir]   (default ~/MATDOG/tools/mujoco-venv)
set -euo pipefail
venv="${1:-$HOME/MATDOG/tools/mujoco-venv}"
python3 -m venv "$venv"
"$venv/bin/python" -m pip install --upgrade pip
"$venv/bin/python" -m pip install -r "$(cd "$(dirname "$0")" && pwd)/requirements.txt"
"$venv/bin/python" -c "import mujoco, numpy; print('mujoco', mujoco.__version__, 'numpy', numpy.__version__)"
