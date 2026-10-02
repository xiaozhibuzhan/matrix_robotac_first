"""Parent-side pipe transport; no SDK import in the ROS process."""
import os
from pathlib import Path
import select
import subprocess
import sys
import time
from .configuration import ROOT


class SDKBridge:
    def __init__(self,config_path,log_path,stand_up=False):
        if os.name!='posix': raise RuntimeError('SDK bridge is only supported on target Ubuntu')
        self.process=None; self.sequence=0; self.log=open(log_path,'ab',buffering=0)
        read_fd,write_fd=os.pipe()
        command=[sys.executable,'-u','-B','-m','navigation.sdk_worker','--config',str(config_path),'--ready-fd',str(write_fd)]
        if stand_up: command.append('--stand-up')
        try:
            self.process=subprocess.Popen(command,cwd=ROOT,stdin=subprocess.PIPE,stdout=self.log,stderr=self.log,pass_fds=(write_fd,))
            os.close(write_fd); write_fd=None
            deadline=time.monotonic()+20
            while True:
                if self.process.poll() is not None: raise RuntimeError(f'SDK worker exited; inspect {log_path}')
                if time.monotonic()>deadline: raise RuntimeError(f'SDK worker startup timed out; inspect {log_path}')
                ready,_,_=select.select([read_fd],[],[],.1)
                if ready:
                    if os.read(read_fd,64)!=b'READY': raise RuntimeError(f'SDK worker readiness pipe closed; inspect {log_path}')
                    break
            os.set_blocking(self.process.stdin.fileno(),False)
        except BaseException:
            self.close(); raise
        finally:
            os.close(read_fd)
            if write_fd is not None: os.close(write_fd)

    def send(self,v,w):
        if self.process is None or self.process.poll() is not None: raise RuntimeError('SDK worker is no longer running')
        self.sequence+=1
        packet=(f'{self.sequence} {time.monotonic():.9f} {v:.9f} {w:.9f}'+chr(10)).encode('ascii')
        count=os.write(self.process.stdin.fileno(),packet)
        if count!=len(packet): raise RuntimeError('Partial SDK command write')

    def close(self):
        if self.process is not None:
            if self.process.stdin and not self.process.stdin.closed: self.process.stdin.close()
            try: self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try: self.process.wait(timeout=2)
                except subprocess.TimeoutExpired: self.process.kill(); self.process.wait()
        if not self.log.closed: self.log.close()
