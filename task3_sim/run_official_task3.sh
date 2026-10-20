#!/usr/bin/env bash
set -euo pipefail
TASK3_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK3_ROOT="$(cd -- "$TASK3_DIR/.." && pwd)"
TASK3_PYTHON="$TASK3_DIR/.venv/bin/python"
unset PYTHONHOME PYTHONPATH
[[ -x "$TASK3_PYTHON" ]] || { echo 'First run setup_ubuntu.sh and build_official_bridge.sh.' >&2; exit 2; }
cd -- "$TASK3_ROOT"
TASK3_ARGS=("$@")
TASK3_OUTPUT=''
TASK3_INTERACTIVE=0
for ((TASK3_INDEX=0; TASK3_INDEX<${#TASK3_ARGS[@]}; TASK3_INDEX++)); do
    case "${TASK3_ARGS[$TASK3_INDEX]}" in
        --help|-h|--preflight) exec "$TASK3_PYTHON" -m task3_sim.official "$@" ;;
        --show|--viewer) TASK3_INTERACTIVE=1 ;;
        --output)
            (( TASK3_INDEX + 1 < ${#TASK3_ARGS[@]} )) || { echo '--output needs a path' >&2; exit 2; }
            TASK3_INDEX=$((TASK3_INDEX+1))
            TASK3_OUTPUT="${TASK3_ARGS[$TASK3_INDEX]}" ;;
        --output=*) TASK3_OUTPUT="${TASK3_ARGS[$TASK3_INDEX]#--output=}" ;;
    esac
done
if [[ -z "$TASK3_OUTPUT" ]]; then
    TASK3_OUTPUT="$TASK3_DIR/runs/official_$(date +%Y%m%d_%H%M%S)_$$"
    TASK3_ARGS+=(--output "$TASK3_OUTPUT")
fi
if [[ -e "$TASK3_OUTPUT" && ! -d "$TASK3_OUTPUT" ]] || [[ -d "$TASK3_OUTPUT" && -n "$(find "$TASK3_OUTPUT" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    echo "Refusing nonempty output: $TASK3_OUTPUT" >&2
    exit 2
fi
mkdir -p -- "$TASK3_OUTPUT"
TASK3_OUTPUT="$(cd -- "$TASK3_OUTPUT" && pwd)"
if [[ -z "${MUJOCO_GL:-}" ]]; then
    if [[ "$TASK3_INTERACTIVE" -eq 1 ]]; then export MUJOCO_GL=glfw; else export MUJOCO_GL=egl; fi
fi
export PYTHONFAULTHANDLER=1
echo "TASK3_TERMINAL_LOG $TASK3_OUTPUT/terminal.log"
set +e
trap ':' INT
"$TASK3_PYTHON" -u -X faulthandler -m task3_sim.official "${TASK3_ARGS[@]}" 2>&1 | tee -i "$TASK3_OUTPUT/terminal.log"
TASK3_STATUS=${PIPESTATUS[0]}
set -e
printf 'TASK3_PROCESS_EXIT code=%s\n' "$TASK3_STATUS" | tee -i -a "$TASK3_OUTPUT/terminal.log"
exit "$TASK3_STATUS"
