"""Create an extension-only ZIP; never package official files or runtime logs."""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import zipfile
from .configuration import ROOT


def main():
    extension=ROOT/'navigation'; folder=extension/'updates'; folder.mkdir(exist_ok=True)
    destination=folder/'task2_point_navigation_20260926.zip'
    files=[]
    for path in sorted(extension.rglob('*')):
        relative=path.relative_to(extension)
        if any(part in ('runs','updates','__pycache__') or part.startswith('.') for part in relative.parts): continue
        if path.is_symlink() or not path.is_file(): continue
        if path.suffix not in ('.py','.sh','.yaml','.rviz','.md','.json','.txt','.pgm'): continue
        if path.name=='package_manifest.json': continue
        data=path.read_bytes()
        if path.suffix in ('.py','.sh','.yaml','.rviz','.md','.json','.txt') and bytes([13]) in data:
            raise ValueError(f'Normalize this extension file to LF before packaging: {path}')
        files.append((path.relative_to(ROOT).as_posix(),data))
    manifest={'created_utc':datetime.now(timezone.utc).isoformat(),'scope':'navigation/ only; no official code or task-one artifacts',
              'files':{name:hashlib.sha256(data).hexdigest() for name,data in files}}
    with zipfile.ZipFile(destination,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as archive:
        for name,data in files:
            info=zipfile.ZipInfo(name); info.compress_type=zipfile.ZIP_DEFLATED
            info.external_attr=(0o755 if name.endswith('.sh') else 0o644)<<16
            archive.writestr(info,data)
        archive.writestr('navigation/package_manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2)+chr(10))
    with zipfile.ZipFile(destination) as archive:
        for name,digest in manifest['files'].items():
            if hashlib.sha256(archive.read(name)).hexdigest()!=digest: raise RuntimeError('Archive verification failed')
        if archive.testzip() is not None: raise RuntimeError('Archive CRC check failed')
    digest=hashlib.sha256(destination.read_bytes()).hexdigest()
    (folder/(destination.name+'.sha256')).write_text(digest+'  '+destination.name+chr(10),encoding='ascii',newline='\n')
    print(json.dumps({'archive':str(destination),'files':len(files)+1,'bytes':destination.stat().st_size,'sha256':digest},ensure_ascii=False,indent=2))


if __name__=='__main__': main()
