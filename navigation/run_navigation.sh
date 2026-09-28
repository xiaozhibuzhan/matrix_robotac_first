#!/usr/bin/env bash
set -euo pipefail
TASK2_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$TASK2_ROOT"
exec "${TASK2_PYTHON:-/usr/bin/python3}" -u -B -m navigation.launcher "$@"
