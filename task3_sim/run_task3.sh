#!/usr/bin/env bash
set -euo pipefail

TASK3_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK3_ROOT="$(cd -- "$TASK3_DIR/.." && pwd)"
TASK3_PYTHON="$TASK3_DIR/.venv/bin/python"
unset PYTHONHOME PYTHONPATH
if [[ ! -x "$TASK3_PYTHON" ]]; then
    echo 'Create the isolated environment first: bash task3_sim/setup_ubuntu.sh' >&2
    exit 1
fi
cd -- "$TASK3_ROOT"
TASK3_INTERACTIVE=0
TASK3_HELP=0
TASK3_OUTPUT_SET=0
TASK3_OUTPUT=''
TASK3_ARGS=("$@")
for TASK3_ARG in "$@"; do
    if [[ "$TASK3_ARG" == '--show' || "$TASK3_ARG" == '--viewer' ]]; then
        TASK3_INTERACTIVE=1
    fi
    if [[ "$TASK3_ARG" == '--help' || "$TASK3_ARG" == '-h' ]]; then
        TASK3_HELP=1
    fi
done
for ((TASK3_INDEX=0; TASK3_INDEX<${#TASK3_ARGS[@]}; TASK3_INDEX++)); do
    if [[ "${TASK3_ARGS[$TASK3_INDEX]}" == '--output' ]]; then
        if (( TASK3_INDEX + 1 >= ${#TASK3_ARGS[@]} )); then
            exec "$TASK3_PYTHON" -m task3_sim "$@"
        fi
        TASK3_INDEX=$((TASK3_INDEX + 1))
        TASK3_OUTPUT="${TASK3_ARGS[$TASK3_INDEX]}"
        TASK3_OUTPUT_SET=1
    elif [[ "${TASK3_ARGS[$TASK3_INDEX]}" == --output=* ]]; then
        TASK3_OUTPUT="${TASK3_ARGS[$TASK3_INDEX]#--output=}"
        TASK3_OUTPUT_SET=1
    fi
done
if [[ -z "${MUJOCO_GL:-}" ]]; then
    if [[ "$TASK3_INTERACTIVE" -eq 1 ]]; then
        export MUJOCO_GL=glfw
    else
        export MUJOCO_GL=egl
    fi
fi
if [[ "$TASK3_HELP" -eq 1 ]]; then
    exec "$TASK3_PYTHON" -m task3_sim "$@"
fi
if [[ "$TASK3_OUTPUT_SET" -eq 0 ]]; then
    TASK3_OUTPUT="$TASK3_DIR/runs/run_$(date +%Y%m%d_%H%M%S)_$$"
    TASK3_ARGS+=(--output "$TASK3_OUTPUT")
fi
if [[ -e "$TASK3_OUTPUT" && ! -d "$TASK3_OUTPUT" ]]; then
    echo "Task 3 output must be a new or empty directory: $TASK3_OUTPUT" >&2
    exit 2
fi
if [[ -d "$TASK3_OUTPUT" && -n "$(find "$TASK3_OUTPUT" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    echo "Task 3 refuses to overwrite nonempty output directory: $TASK3_OUTPUT" >&2
    exit 2
fi
mkdir -p -- "$TASK3_OUTPUT"
TASK3_OUTPUT="$(cd -- "$TASK3_OUTPUT" && pwd)"
export PYTHONFAULTHANDLER=1
echo "TASK3_TERMINAL_LOG $TASK3_OUTPUT/terminal.log"
# Capture native stderr and late interpreter-shutdown failures as well as
# Python exceptions. Preserve Python's exit status instead of tee's status.
# A terminal Ctrl+C targets the whole foreground pipeline. Keep tee alive
# until Python has recorded its handled interrupt and native cleanup; an
# interrupted tee closes Python's stdout and turns successful cleanup into
# BrokenPipeError/exit 1. Catch (do not ignore) SIGINT in this shell, so the
# Python child still inherits its normal SIGINT disposition on exec.
set +e
trap ':' INT
"$TASK3_PYTHON" -u -X faulthandler -m task3_sim "${TASK3_ARGS[@]}" 2>&1 | tee -i "$TASK3_OUTPUT/terminal.log"
TASK3_STATUS=${PIPESTATUS[0]}
set -e
printf 'TASK3_PROCESS_EXIT code=%s\n' "$TASK3_STATUS" | tee -i -a "$TASK3_OUTPUT/terminal.log"
exit "$TASK3_STATUS"
