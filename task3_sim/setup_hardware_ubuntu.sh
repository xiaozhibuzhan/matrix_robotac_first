#!/usr/bin/env bash
set -euo pipefail
TASK3_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK3_ROOT="$(cd -- "$TASK3_DIR/.." && pwd)"
TASK3_VENV="$TASK3_DIR/.venv_hardware"
TASK3_SDK_ROOT="$TASK3_ROOT/deps/zsibot_sdk"
if [[ $# -gt 0 ]]; then
    [[ $# -eq 2 && "$1" == --sdk-root ]] || { echo 'Usage: bash task3_sim/setup_hardware_ubuntu.sh [--sdk-root /absolute/official_sdk]' >&2; exit 2; }
    TASK3_SDK_ROOT="$2"
fi
unset PYTHONHOME PYTHONPATH
TASK3_PYTHON="$(command -v python3.10 || command -v python3 || true)"
[[ -n "$TASK3_PYTHON" ]] || { echo 'Install CPython 3.10 and its venv package first.' >&2; exit 2; }
cd -- "$TASK3_ROOT"
"$TASK3_PYTHON" -B -c 'import sys; from task3_sim.sdk_platform import inspect_sdk; r=inspect_sdk(sys.argv[1]); print(r); sys.exit(0 if r["ready"] else 2)' "$TASK3_SDK_ROOT"
TASK3_ARCH="$("$TASK3_PYTHON" -B -c 'from task3_sim.sdk_platform import normalize_arch; print(normalize_arch())')"
if [[ -e "$TASK3_VENV" && ! -x "$TASK3_VENV/bin/python" ]]; then
    echo "Existing $TASK3_VENV is not a Linux venv; preserve it elsewhere before installing." >&2
    exit 2
fi
if [[ ! -x "$TASK3_VENV/bin/python" ]]; then
    "$TASK3_PYTHON" -m venv "$TASK3_VENV"
fi
# Reject a copied x86 virtualenv on the ARM board before invoking its pip.
"$TASK3_VENV/bin/python" -B -c 'import sys; from task3_sim.sdk_platform import inspect_sdk; r=inspect_sdk(sys.argv[1]); assert r["ready"], r' "$TASK3_SDK_ROOT"
# scikit-build invokes cmake/ninja by name; prefer the pinned venv tools.
export PATH="$TASK3_VENV/bin:$PATH"
if [[ "$TASK3_ARCH" == aarch64 ]]; then
    for TASK3_TOOL in cc c++; do
        command -v "$TASK3_TOOL" >/dev/null || { echo 'AprilTag ARM64 needs a compiler: sudo apt install build-essential python3.10-dev' >&2; exit 2; }
    done
    "$TASK3_VENV/bin/python" -m pip install --disable-pip-version-check -r "$TASK3_DIR/requirements_hardware_build.txt"
    "$TASK3_VENV/bin/python" -m pip install --disable-pip-version-check --only-binary=:all: numpy==1.26.4 opencv-python==4.11.0.86
    # Upstream post11 has no Linux ARM64 wheel. Compile its unmodified source
    # locally with CMake 3.x and two jobs; do not use an x86 native library.
    CMAKE_BUILD_PARALLEL_LEVEL=2 "$TASK3_VENV/bin/python" -m pip install --disable-pip-version-check --no-build-isolation --no-binary=pupil-apriltags pupil-apriltags==1.0.4.post11
else
    "$TASK3_VENV/bin/python" -m pip install --disable-pip-version-check -r "$TASK3_DIR/requirements_hardware.txt"
fi
"$TASK3_VENV/bin/python" -B -m task3_sim.hardware --preflight --sdk-root "$TASK3_SDK_ROOT"
echo 'Task3 hardware environment prepared. No SDK connection, movement or firing was performed.'
