#!/usr/bin/env bash
set -euo pipefail
TASK3_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK3_ROOT="$(cd -- "$TASK3_DIR/.." && pwd)"
TASK3_PYTHON="$TASK3_DIR/.venv/bin/python"
unset PYTHONHOME PYTHONPATH
if [[ ! -x "$TASK3_PYTHON" ]]; then
    echo 'First run: bash task3_sim/setup_ubuntu.sh' >&2
    exit 2
fi
"$TASK3_PYTHON" -c 'import platform,sys; assert sys.platform=="linux" and sys.version_info[:2]==(3,10) and platform.machine()=="x86_64", "Official runtime needs Ubuntu x86_64 CPython 3.10"'
for TASK3_TOOL in cmake c++ pkg-config; do
    command -v "$TASK3_TOOL" >/dev/null || { echo "Missing build tool: $TASK3_TOOL (see OFFICIAL_DYNAMICS_030.md)" >&2; exit 2; }
done
if [[ ! -f /usr/include/robot_sdk.pb.h ]]; then
    echo 'Install the supplied official .deb dependencies first; see OFFICIAL_DYNAMICS_030.md.' >&2
    exit 2
fi
cmake -S "$TASK3_DIR/native" -B "$TASK3_DIR/local/native" -DCMAKE_BUILD_TYPE=Release
cmake --build "$TASK3_DIR/local/native" --parallel 2
cd -- "$TASK3_ROOT"
"$TASK3_PYTHON" -m task3_sim.official --preflight
