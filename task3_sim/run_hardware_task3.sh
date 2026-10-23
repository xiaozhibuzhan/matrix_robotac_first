#!/usr/bin/env bash
set -euo pipefail
TASK3_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK3_PYTHON="$TASK3_DIR/.venv_hardware/bin/python"
unset PYTHONHOME PYTHONPATH
[[ -x "$TASK3_PYTHON" ]] || { echo 'First run: bash task3_sim/setup_hardware_ubuntu.sh' >&2; exit 2; }
cd -- "$TASK3_DIR/.."
if [[ $# -eq 0 ]]; then set -- --preflight; fi
exec "$TASK3_PYTHON" -u -B -m task3_sim.hardware "$@"
