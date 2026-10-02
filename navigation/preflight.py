"""Read-only offline/Ubuntu prerequisite checks; never connects to the robot."""
import argparse
import importlib
import json
import os
from pathlib import Path
import platform
import shutil
import sys
from .configuration import DEFAULT,ROOT,load_config
from .grid import GridMap


def inspect(config=DEFAULT,map_override=None,offline=False,drive=False,rviz=True):
    cfg=load_config(config,map_override); grid=GridMap.load(cfg['map'],cfg['robot_radius'],cfg['safety_margin'])
    if not grid.passable.any(): raise ValueError('Map has no footprint-clear cells')
    report={'python':sys.version.split()[0],'platform':platform.platform(),'offline':offline,
            'mode':'drive' if drive else 'preview','map':cfg['map'],'map_summary':grid.summary(),
            'control_settings':{key:cfg[key] for key in ('max_speed','max_yaw_rate','acceleration','goal_timeout','max_tracking_replans','replan_progress_distance')},
            'environment':{key:os.environ.get(key,'<unset>') for key in ('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','SDK_CLIENT_IP')}}
    if not offline:
        if sys.platform!='linux': raise RuntimeError('ROS/SDK launch requires the target Ubuntu; use --offline here')
        for module in ('rclpy','geometry_msgs.msg','nav_msgs.msg','sensor_msgs.msg','std_msgs.msg','std_srvs.srv','visualization_msgs.msg'):
            importlib.import_module(module)
        if rviz and not shutil.which('rviz2'): raise RuntimeError('rviz2 is not on PATH; load the working ROS environment')
        if os.environ.get('ROS_DISTRO')!='humble': raise RuntimeError('Load the existing ROS2 Humble environment first')
        if drive:
            if sys.version_info[:2]!=(3,10): raise RuntimeError('Official SDK requires /usr/bin/python3 (CPython 3.10)')
            arch={'amd64':'x86_64','arm64':'aarch64'}.get(platform.machine().lower(),platform.machine().lower())
            if arch not in ('x86_64','aarch64'): raise RuntimeError('Unsupported SDK CPU architecture')
            sdk=ROOT/'deps/zsibot_sdk/lib/zsl-1'/arch
            required=[sdk/f'libmc_sdk_zsl_1_{arch}.so',sdk/f'mc_sdk_zsl_1_py.cpython-310-{arch}-linux-gnu.so']
            for path in required:
                if not path.is_file(): raise RuntimeError(f'Official Linux SDK file missing: {path}')
                with path.open('rb') as stream: magic=stream.read(4)
                if magic!=bytes([127,69,76,70]): raise RuntimeError(f'Invalid official ELF: {path}')
            report['sdk_files']=[str(path) for path in required]
    return cfg,report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT); parser.add_argument('--map',type=Path)
    parser.add_argument('--offline',action='store_true'); parser.add_argument('--drive',action='store_true')
    parser.add_argument('--no-rviz',action='store_true'); args=parser.parse_args()
    try: _,report=inspect(args.config,args.map,args.offline,args.drive,not args.no_rviz)
    except (ValueError,OSError,ImportError,RuntimeError) as exc: parser.exit(1,str(exc)+chr(10))
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__': main()
