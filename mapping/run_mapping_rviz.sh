#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Scan transforms/PCA use tiny matrices. Large BLAS thread pools contend with
# ROS callbacks instead of speeding them up. Override only for this launcher.
export OPENBLAS_NUM_THREADS="${MAPPING_BLAS_THREADS:-1}"
export OMP_NUM_THREADS="${MAPPING_BLAS_THREADS:-1}"
export MKL_NUM_THREADS="${MAPPING_BLAS_THREADS:-1}"

exec python3 mapping/run_mapping_rviz.py "$@"
