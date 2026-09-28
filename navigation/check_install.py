"""Verify packaged navigation files after extraction, without loading ROS/SDK."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def main():
    manifest=ROOT/'navigation/package_manifest.json'
    if not manifest.is_file():
        print('No package manifest: source checkout. Run navigation.preflight --offline instead.'); return 1
    data=json.loads(manifest.read_text(encoding='utf-8')); changed=[]
    for name,digest in data['files'].items():
        path=(ROOT/name).resolve()
        if (ROOT/'navigation').resolve() not in path.parents: raise ValueError('Manifest path escapes navigation/')
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest: changed.append(name)
    print(json.dumps({'checked':len(data['files']),'missing_or_changed':changed},ensure_ascii=False,indent=2))
    return int(bool(changed))


if __name__=='__main__': raise SystemExit(main())
