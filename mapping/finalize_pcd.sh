#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export OPENBLAS_NUM_THREADS="${MAPPING_BLAS_THREADS:-1}"
export OMP_NUM_THREADS="${MAPPING_BLAS_THREADS:-1}"
export MKL_NUM_THREADS="${MAPPING_BLAS_THREADS:-1}"

# Keep the caller's working directory so a relative run-directory argument
# still points to the same capture when invoked from outside the project.
exec python3 "$SCRIPT_DIR/finalize_mapping.py" "$@"
