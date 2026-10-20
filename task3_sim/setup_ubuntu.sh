#!/usr/bin/env bash
set -euo pipefail

TASK3_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK3_VENV="$TASK3_DIR/.venv"
# A terminal may already have ROS or another project sourced. Keep this
# script's interpreter isolated; the calling shell is unaffected.
unset PYTHONHOME PYTHONPATH

if command -v python3.10 >/dev/null 2>&1; then
    TASK3_PYTHON="$(command -v python3.10)"
else
    TASK3_PYTHON="$(command -v python3 || true)"
fi
if [[ -z "$TASK3_PYTHON" ]]; then
    echo 'Python 3.10 is required. On Ubuntu 22.04 install python3.10 and python3.10-venv.' >&2
    exit 1
fi
"$TASK3_PYTHON" -c 'import sys; assert (3,10) <= sys.version_info[:2] <= (3,12), "Use Python 3.10-3.12 (Ubuntu 22.04: 3.10)."'
if [[ -e "$TASK3_VENV" && ! -x "$TASK3_VENV/bin/python" ]]; then
    echo "Existing $TASK3_VENV is not a Linux virtual environment. Rename it before setup." >&2
    exit 1
fi
if [[ ! -x "$TASK3_VENV/bin/python" ]]; then
    if ! "$TASK3_PYTHON" -m venv "$TASK3_VENV"; then
        echo 'venv creation failed. On Ubuntu 22.04 install python3.10-venv, then rerun this script.' >&2
        exit 1
    fi
fi
"$TASK3_VENV/bin/python" -m pip install --disable-pip-version-check -r "$TASK3_DIR/requirements.txt"
"$TASK3_VENV/bin/python" -c 'import mujoco, numpy, cv2; from pupil_apriltags import Detector; print("Dependencies ready:", mujoco.__version__, numpy.__version__, cv2.__version__)'
echo "Task 3 environment ready: $TASK3_VENV"
echo 'Run from the project: bash task3_sim/run_task3.sh --headless'
