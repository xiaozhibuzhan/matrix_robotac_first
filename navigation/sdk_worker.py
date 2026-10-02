"""Independent POSIX SDK process; watchdog survives parent stalls or exit."""
import argparse
import importlib
import json
import math
import os
import platform
import select
import signal
import sys
import time
from .configuration import ROOT,load_config


class CommandGuard:
    def __init__(self,timeout,max_speed,max_yaw):
        self.timeout=timeout; self.max_speed=max_speed; self.max_yaw=max_yaw
        self.sequence=-1; self.issued=None; self.velocity=(0.,0.); self.tripped=False

    def accept(self,line,now):
        if self.issued is not None and any(self.velocity) and now-self.issued>self.timeout:
            self.tripped=True
        if self.tripped: raise ValueError('Watchdog is latched')
        fields=line.split()
        if len(fields)!=4: raise ValueError('Invalid command packet')
        sequence=int(fields[0]); issued,v,w=map(float,fields[1:])
        if sequence<=self.sequence or not all(math.isfinite(n) for n in (issued,v,w)):
            raise ValueError('Nonfinite or out-of-order command')
        if not 0<=now-issued<=self.timeout: raise ValueError('Expired/future command')
        if not 0<=v<=self.max_speed+1e-8 or abs(w)>self.max_yaw+1e-8: raise ValueError('SDK velocity limit exceeded')
        self.sequence=sequence; self.issued=issued; self.velocity=(v,w)

    def current(self,now):
        if self.issued is None: return (0.,0.)
        if now-self.issued>self.timeout or now<self.issued:
            if any(self.velocity): self.tripped=True
            return (0.,0.)
        return (0.,0.) if self.tripped else self.velocity


def result_ok(value,operation='SDK call'):
    # Some binding versions return void; physical stop is verified by odometry.
    if value is not None and int(value)!=0:
        code=int(value)
        hint='; state transition rejected: move requires standUp first' if code==0x3007 and operation.startswith('move') else ''
        raise RuntimeError(f'{operation}: SDK returned {code} (0x{code:04X}){hint}')


def load_sdk():
    if sys.platform!='linux' or sys.version_info[:2]!=(3,10):
        raise RuntimeError('Bundled SDK requires Linux CPython 3.10; use target Ubuntu /usr/bin/python3')
    arch=platform.machine().lower(); arch={'amd64':'x86_64','arm64':'aarch64'}.get(arch,arch)
    if arch not in ('x86_64','aarch64'): raise RuntimeError(f'Unsupported SDK architecture: {arch}')
    directory=ROOT/'deps/zsibot_sdk/lib/zsl-1'/arch
    if not list(directory.glob('mc_sdk_zsl_1_py.cpython-310-*.so')): raise RuntimeError('Matching official Python SDK is missing')
    sys.path.insert(0,str(directory))
    import ctypes
    ctypes.CDLL(str(directory/f'libmc_sdk_zsl_1_{arch}.so'),mode=ctypes.RTLD_GLOBAL)
    sdk=importlib.import_module('mc_sdk_zsl_1_py').HighLevel()
    for name in ('initRobot','checkConnect','move','standUp'):
        if not callable(getattr(sdk,name,None)): raise RuntimeError(f'SDK binding missing {name}')
    return sdk


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True); parser.add_argument('--stand-up',action='store_true')
    parser.add_argument('--ready-fd',type=int)
    args=parser.parse_args(); cfg=load_config(args.config)
    sdk=None; exit_code=0; running=[True]
    def stop_signal(signum,frame): running[0]=False
    for sig in (signal.SIGINT,signal.SIGTERM): signal.signal(sig,stop_signal)
    try:
        if os.name!='posix': raise RuntimeError('SDK worker requires POSIX')
        import fcntl
        lock=open(f'/tmp/robotac_task2_sdk_{cfg["client_port"]}.lock','a')
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        sdk=load_sdk(); sdk.initRobot(cfg['client_ip'],cfg['client_port'],cfg['robot_ip'])
        deadline=time.monotonic()+10
        while running[0] and not sdk.checkConnect():
            if time.monotonic()>deadline: raise RuntimeError('SDK connection timeout')
            time.sleep(.05)
        if not running[0]: return 0
        if args.stand_up:
            # Even a zero-speed move changes SDK state; standUp must come first.
            print('SDK connected; requesting standUp before the first move',flush=True)
            result_ok(sdk.standUp(),'standUp'); until=time.monotonic()+4
            while running[0] and time.monotonic()<until: time.sleep(.05)
        if not running[0]: return 0
        # Preserve errors instead of treating a rejected zero command as a stop.
        result_ok(sdk.move(0.,0.,0.),'move(0,0,0) startup')
        if not running[0]: return 0
        guard=CommandGuard(cfg['command_timeout'],cfg['max_speed'],cfg['max_yaw_rate'])
        os.set_blocking(sys.stdin.fileno(),False); buffer=b''
        print('TASK2_READY',flush=True)
        if args.ready_fd is not None:
            os.write(args.ready_fd,b'READY'); os.close(args.ready_fd)
        while running[0]:
            readable,_,_=select.select([sys.stdin.fileno()],[],[],.025)
            if readable:
                chunk=os.read(sys.stdin.fileno(),4096)
                if not chunk: break
                buffer+=chunk
                if len(buffer)>8192: raise ValueError('Command buffer exceeded limit')
                lines=buffer.split(bytes([10])); buffer=lines.pop()
                for line in lines:
                    if line: guard.accept(line.decode('ascii'),time.monotonic())
            v,w=guard.current(time.monotonic())
            if not sdk.checkConnect(): raise RuntimeError('SDK disconnected')
            result_ok(sdk.move(v,0.,w),'move')
            if guard.tripped: raise RuntimeError('Motion watchdog expired and latched; restart navigation')
    except Exception as exc:
        print(json.dumps({'sdk_error':str(exc)}),file=sys.stderr,flush=True); exit_code=2
    finally:
        if sdk is not None:
            for _ in range(15):
                try: sdk.move(0.,0.,0.)
                except Exception: pass
                time.sleep(.025)
    return exit_code


if __name__=='__main__': raise SystemExit(main())
