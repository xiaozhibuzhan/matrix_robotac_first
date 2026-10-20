#!/usr/bin/env bash
# Full local Ubuntu acceptance. No automatic package installation or real IP.
set -euo pipefail
TASK3_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK3_ROOT="$(cd -- "$TASK3_DIR/.." && pwd)"
TASK3_PYTHON="$TASK3_DIR/.venv/bin/python"
TASK3_REPORT="${1:-$TASK3_DIR/runs/official_acceptance_$(date +%Y%m%d_%H%M%S)_$$}"
unset PYTHONHOME PYTHONPATH
[[ -x "$TASK3_PYTHON" ]] || { echo 'Run setup_ubuntu.sh first.' >&2; exit 2; }
if [[ -e "$TASK3_REPORT" ]]; then echo 'Acceptance output must be a new directory.' >&2; exit 2; fi
mkdir -p -- "$TASK3_REPORT"
TASK3_REPORT="$(cd -- "$TASK3_REPORT" && pwd)"
cd -- "$TASK3_ROOT"
TASK3_FAILED=0
TASK3_PHASES="$TASK3_REPORT/phases.tsv"
task3_record_phase() {
    printf '%s\t%s\t%s\n' "$1" "$2" "$3" >>"$TASK3_PHASES"
}
echo "TASK3_ACCEPTANCE_PHASE unit_tests log=$TASK3_REPORT/unit_tests.log"
if "$TASK3_PYTHON" -m unittest discover -s task3_sim/tests -v >"$TASK3_REPORT/unit_tests.log" 2>&1; then
    task3_record_phase unit_tests passed 0
    tail -n 6 "$TASK3_REPORT/unit_tests.log"
else
    TASK3_CODE=$?
    task3_record_phase unit_tests failed "$TASK3_CODE"
    TASK3_FAILED=1
    tail -n 100 "$TASK3_REPORT/unit_tests.log"
fi
if [[ "$TASK3_FAILED" -eq 0 ]]; then
    echo "TASK3_ACCEPTANCE_PHASE official_dependency_preflight"
    if "$TASK3_PYTHON" -m task3_sim.official --preflight --output "$TASK3_REPORT/preflight" >"$TASK3_REPORT/preflight.log" 2>&1; then
        task3_record_phase official_dependency_preflight passed 0
    else
        TASK3_CODE=$?
        task3_record_phase official_dependency_preflight failed "$TASK3_CODE"
        TASK3_FAILED=1
        tail -n 60 "$TASK3_REPORT/preflight.log"
    fi
fi
if [[ "$TASK3_FAILED" -eq 0 ]]; then
    "$TASK3_PYTHON" - "$TASK3_REPORT" <<'PY'
import copy, json, pathlib, random, sys
out = pathlib.Path(sys.argv[1])
base = json.loads(pathlib.Path('task3_sim/config/multi_official.json').read_text())
cases = {'native_nominal': copy.deepcopy(base)}
repeat = copy.deepcopy(base)
repeat['simulation']['ammo_count'] = 5
cases['native_repeat_five_shots'] = repeat
swapped = copy.deepcopy(base)
placements = [copy.deepcopy(t['simulation']) for t in swapped['targets']]
for target, index in zip(swapped['targets'], [2, 0, 1]):
    target['simulation'] = placements[index]
swapped['targets'].reverse()
cases['native_identity_order_swap'] = swapped
stress = copy.deepcopy(base)
rng = random.Random(20261019)
stress['simulation']['initial_position_m'][:2] = [rng.uniform(-.04, .04), rng.uniform(-.04, .04)]
stress['simulation']['initial_rpy_rad'][2] = rng.uniform(-.07, .07)
for target in stress['targets']:
    target['simulation']['target_center_world_m'][:2] = [v*1.04 for v in target['simulation']['target_center_world_m'][:2]]
stress['observation'] = {'delay_s': .025, 'jitter_s': .01, 'drop_probability': .1, 'seed': 20261019}
cases['native_start_distance_delay_drop'] = stress
for name, config in cases.items():
    (out/(name+'.json')).write_text(json.dumps(config, indent=2)+'\n')
(out/'declared_cases.json').write_text(json.dumps({'seed':20261019, 'cases':list(cases),
    'ammo_counts':{name:config['simulation']['ammo_count'] for name,config in cases.items()},
    'five_shot_case':'Explicit repeat-visit diagnostic; competition default remains three shots.',
    'scope':'Predeclared official native physics candidates; any failure remains a failure.'}, indent=2)+'\n')
PY
    for TASK3_CASE in native_nominal native_repeat_five_shots native_identity_order_swap native_start_distance_delay_drop; do
        echo "TASK3_OFFICIAL_CASE $TASK3_CASE"
        if bash task3_sim/run_official_task3.sh --config "$TASK3_REPORT/$TASK3_CASE.json" --headless --max-seconds 240 --output "$TASK3_REPORT/$TASK3_CASE"; then
            if "$TASK3_PYTHON" -m task3_sim.validate_official "$TASK3_REPORT/$TASK3_CASE"; then
                task3_record_phase "$TASK3_CASE" passed 0
            else
                TASK3_CODE=$?
                task3_record_phase "$TASK3_CASE" failed "$TASK3_CODE"
                TASK3_FAILED=1
            fi
        else
            TASK3_CODE=$?
            task3_record_phase "$TASK3_CASE" failed "$TASK3_CODE"
            TASK3_FAILED=1
            "$TASK3_PYTHON" -m task3_sim.validate_official "$TASK3_REPORT/$TASK3_CASE" || true
        fi
        if [[ "$TASK3_FAILED" -ne 0 ]]; then break; fi
    done
fi
if [[ "$TASK3_FAILED" -eq 0 ]]; then
    echo "TASK3_ACCEPTANCE_PHASE vision_fixed_plus_20_random"
    if "$TASK3_PYTHON" -m task3_sim.validate_multi --random-cases 20 --output "$TASK3_REPORT/vision" 2>&1 | tee -i "$TASK3_REPORT/vision_terminal.log"; then
        task3_record_phase vision_fixed_plus_20_random passed 0
    else
        TASK3_CODE=$?
        task3_record_phase vision_fixed_plus_20_random failed "$TASK3_CODE"
        TASK3_FAILED=1
    fi
fi
if "$TASK3_PYTHON" - "$TASK3_REPORT" "$TASK3_FAILED" <<'PY'
import json, pathlib, sys
out = pathlib.Path(sys.argv[1])
names = ['native_nominal','native_repeat_five_shots','native_identity_order_swap','native_start_distance_delay_drop']
reports = {}
for name in names:
    path = out/name/'official_validation.json'
    reports[name] = json.loads(path.read_text()) if path.exists() else {'all_passed':False,'not_completed':True}
vision = out/'vision/validation.json'
vision_ok = vision.exists() and json.loads(vision.read_text()).get('all_passed') is True
result = {'all_passed':sys.argv[2]=='0' and vision_ok and all(r.get('all_passed') is True for r in reports.values()),
          'vision_passed':vision_ok, 'native_cases':reports, 'scope':'Ubuntu official physics acceptance; no hardware certification'}
phases = {}
for line in (out/'phases.tsv').read_text().splitlines():
    name, status, exit_code = line.split('\t')
    phases[name] = {'status':status, 'exit_code':int(exit_code)}
expected = ['unit_tests','official_dependency_preflight'] + names + ['vision_fixed_plus_20_random']
result['phases'] = {name:phases.get(name, {'status':'skipped','exit_code':None}) for name in expected}
result['phases']['evidence_summary'] = {'status':'passed' if result['all_passed'] else 'failed',
                                      'exit_code':0 if result['all_passed'] else 1}
expected.append('evidence_summary')
result['first_failed_phase'] = next((name for name in expected if result['phases'][name]['status']=='failed'), None)
result['not_started_native_cases'] = [name for name in names if result['phases'][name]['status']=='skipped']
(out/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
print('TASK3_ACCEPTANCE', 'PASS' if result['all_passed'] else 'FAIL', out/'acceptance.json')
raise SystemExit(0 if result['all_passed'] else 1)
PY
then
    TASK3_FAILED=0
else
    TASK3_FAILED=1
fi
# Export logs and evidence; copied official binaries/models are already on the
# user's machine and need not be sent back. Runtime manifests remain included.
tar -czf "$TASK3_REPORT.tar.gz" \
    --exclude='*/official_runtime/mc' \
    --exclude='*/official_runtime/zsibot_robots' \
    --exclude='*/official_runtime/simulate/build' \
    -C "$(dirname -- "$TASK3_REPORT")" "$(basename -- "$TASK3_REPORT")"
echo "TASK3_RETURN_ARCHIVE $TASK3_REPORT.tar.gz"
exit "$TASK3_FAILED"
