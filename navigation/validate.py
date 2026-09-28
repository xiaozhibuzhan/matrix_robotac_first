"""Run offline checks and save an honest, reproducible development report."""
import argparse
import ast
from datetime import datetime,timezone
import hashlib
import io
import json
from pathlib import Path
import platform
import sys
import time
import unittest
import yaml
from .configuration import ROOT,load_config
from .grid import GridMap


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-protected',action='store_true',help='Compare this development snapshot, not a different Ubuntu installation')
    args=parser.parse_args(); extension=ROOT/'navigation'
    syntax=[]
    for path in extension.rglob('*.py'):
        if 'runs' in path.relative_to(extension).parts: continue
        ast.parse(path.read_text(encoding='utf-8'),filename=str(path),feature_version=(3,10))
        syntax.append(path.relative_to(ROOT).as_posix())
    output=io.StringIO(); suite=unittest.defaultTestLoader.discover(str(extension/'tests'))
    started=time.perf_counter(); result=unittest.TextTestRunner(stream=output,verbosity=2).run(suite)
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'python':sys.version,'platform':platform.platform(),
            'scope':'Offline Python/geometry/state-machine tests; no ROS2 runtime, SDK motion or Ubuntu gait trial',
            'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),'skipped':len(result.skipped),
            'seconds':time.perf_counter()-started,'python310_syntax_files':syntax,'map':GridMap.load(load_config()['map']).summary()}
    if args.check_protected:
        baseline=json.loads((extension/'validation/protected_baseline.json').read_text(encoding='utf-8'))['files']
        changed=[name for name,digest in baseline.items() if not (ROOT/name).is_file() or hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest]
        report['protected_snapshot']={'checked_files':len(baseline),'changed':changed}
        derived=json.loads((Path(load_config()['map']).parent/'report.json').read_text(encoding='utf-8'))
        original_yaml=ROOT/derived['source_yaml'].replace(chr(92),'/')
        original_meta=yaml.safe_load(original_yaml.read_text(encoding='utf-8'))
        sources=(original_yaml,original_yaml.parent/original_meta['image'],ROOT/derived['source_pcd'].replace(chr(92),'/'))
        report['task_one_sources']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()==derived['source_sha256'][p.name] for p in sources}
    report['source_hashes']={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in extension.rglob('*.py') if 'runs' not in p.relative_to(extension).parts}
    destination=extension/'validation'; destination.mkdir(exist_ok=True)
    (destination/'offline_tests.txt').write_text(output.getvalue(),encoding='utf-8')
    (destination/'local_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+chr(10),encoding='utf-8')
    print(output.getvalue()); print(json.dumps({k:v for k,v in report.items() if k not in ('source_hashes','python310_syntax_files')},ensure_ascii=False,indent=2))
    return 0 if (result.wasSuccessful() and not report.get('protected_snapshot',{}).get('changed') and all(report.get('task_one_sources',{}).values())) else 1


if __name__=='__main__': raise SystemExit(main())
