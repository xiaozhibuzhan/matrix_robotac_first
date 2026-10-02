"""Launch only this extension's processes, preserving the working simulator."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import yaml
from .configuration import DEFAULT,ROOT
from .preflight import inspect


def stop_child(process):
    if process is None: return
    # Children own separate sessions; never signal official simulator/router.
    for sig,timeout in ((signal.SIGINT,5),(signal.SIGTERM,2),(signal.SIGKILL,2)):
        try: os.killpg(process.pid,sig)
        except ProcessLookupError: pass
        try: process.wait(timeout=timeout); return
        except subprocess.TimeoutExpired: pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT); parser.add_argument('--map',type=Path)
    parser.add_argument('--drive',action='store_true'); parser.add_argument('--stand-up',action='store_true')
    parser.add_argument('--no-rviz',action='store_true'); parser.add_argument('--use-sim-time',action='store_true')
    args=parser.parse_args()
    if args.stand_up and not args.drive: parser.error('--stand-up requires --drive')
    try: cfg,report=inspect(args.config,args.map,drive=args.drive,rviz=not args.no_rviz)
    except (ValueError,OSError,ImportError,RuntimeError) as exc: parser.exit(1,str(exc)+chr(10))
    import fcntl
    lock_path=Path(tempfile.gettempdir())/('robotac_task2_navigation_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock')
    lock=open(lock_path,'a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: parser.exit(1,'Another task2 session owns this ROS domain; stop it first'+chr(10))
    runs=ROOT/'navigation/runs'; runs.mkdir(exist_ok=True)
    run_dir=Path(tempfile.mkdtemp(prefix=datetime.now().strftime('run_%Y%m%d_%H%M%S_'),dir=runs))
    config_path=run_dir/'config.yaml'; config_path.write_text(yaml.safe_dump(cfg,sort_keys=False),encoding='utf-8')
    rviz=yaml.safe_load((ROOT/'navigation/rviz/point_navigation.rviz').read_text(encoding='utf-8'))
    rviz['Visualization Manager']['Global Options']['Fixed Frame']=cfg['frame']
    for tool in rviz['Visualization Manager']['Tools']:
        if tool['Class'].endswith('/SetInitialPose'): tool['Topic']['Value']=cfg['initialpose_topic']
        if tool['Class'].endswith('/PublishPoint'): tool['Topic']['Value']=cfg['clicked_point_topic']
    rviz_path=run_dir/'point_navigation.rviz'; rviz_path.write_text(yaml.safe_dump(rviz,sort_keys=False),encoding='utf-8')
    command=[sys.executable,'-u','-B','-m','navigation.node','--config',str(config_path),'--run-dir',str(run_dir)]
    if args.drive: command.append('--drive')
    if args.stand_up: command.append('--stand-up')
    if args.use_sim_time: command.append('--use-sim-time')
    map_yaml=Path(cfg['map']); map_meta=yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    report['map_hashes']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (map_yaml,map_yaml.parent/map_meta['image'])}
    report.update({'created_utc':datetime.now(timezone.utc).isoformat(),'pid':os.getpid(),'command':command,
                   'config':cfg,'use_sim_time':args.use_sim_time,'source_hashes':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'navigation').glob('*.py')}})
    (run_dir/'run.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+chr(10),encoding='utf-8')
    print(('DRIVE' if args.drive else 'PREVIEW (no SDK motion)')+' | logs: '+str(run_dir),flush=True)
    print(f"Loaded limits: max_speed={cfg['max_speed']:.2f} m/s, max_yaw_rate={cfg['max_yaw_rate']:.2f} rad/s",flush=True)
    print('Wait for sensors, set 2D Pose Estimate while stopped, then Publish Point. Ctrl+C stops.',flush=True)
    node=viewer=None; logs=[]; code=0
    def interrupted(signum,frame): raise KeyboardInterrupt
    old_term=signal.signal(signal.SIGTERM,interrupted)
    try:
        logs.append(open(run_dir/'node.log','ab',buffering=0))
        node=subprocess.Popen(command,cwd=ROOT,stdout=logs[-1],stderr=logs[-1],start_new_session=True)
        if not args.no_rviz:
            logs.append(open(run_dir/'rviz.log','ab',buffering=0))
            rviz_command=['rviz2','-d',str(rviz_path)]
            if args.use_sim_time: rviz_command+=['--ros-args','-p','use_sim_time:=true']
            viewer=subprocess.Popen(rviz_command,cwd=ROOT,stdout=logs[-1],stderr=logs[-1],start_new_session=True)
        while True:
            if node.poll() is not None:
                code=node.returncode; print('Navigation exited; inspect node.log and sdk.log',flush=True); break
            if viewer is not None and viewer.poll() is not None:
                code=viewer.returncode; print('RViz closed; stopping navigation.',flush=True); break
            time.sleep(.2)
    except KeyboardInterrupt: print('Stopping this navigation session...',flush=True)
    except OSError as exc: print(str(exc),file=sys.stderr); code=1
    finally:
        signal.signal(signal.SIGTERM,signal.SIG_IGN); previous_int=signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:
            stop_child(node); stop_child(viewer)
            for stream in logs: stream.close()
            report.update({'finished_utc':datetime.now(timezone.utc).isoformat(),'node_exit':node.returncode if node else None,'rviz_exit':viewer.returncode if viewer else None})
            (run_dir/'run.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+chr(10),encoding='utf-8')
        finally:
            signal.signal(signal.SIGINT,previous_int); signal.signal(signal.SIGTERM,old_term); lock.close()
    return code if code>=0 else 1


if __name__=='__main__': raise SystemExit(main())
