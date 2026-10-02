"""ROS-free navigation state machine and live obstacle stop guard."""
from collections import deque
import math
import numpy as np
from .geometry import align_pose,transform_pose,wrap,yaw_of,rotation_matrix,rpy_matrix,normalized_quaternion
from .sdk_limits import MIN_FORWARD_SPEED,MIN_YAW_RATE


class Navigator:
    def __init__(self,grid,config,planner=None):
        self.grid=grid; self.cfg=config; self.history=deque(maxlen=512)
        self.planner=planner; self.plan_request=None; self.plan_pose=None
        self.odom=None; self.odom_stamp=None; self.odom_received=None; self.frames=None
        self.alignment=None; self.pose=None; self.linear_speed=math.inf; self.angular_speed=math.inf
        self.cloud_stamp=None; self.cloud_received=None; self.obstacles=np.empty((0,2))
        self.goal=None; self.path=[]; self.path_index=1; self.goal_started=None
        self.state='WAIT_INITIAL_POSE'; self.reason='Use 2D Pose Estimate while stopped'
        self.command=(0.,0.); self.last_tick=None; self.settle_since=None
        self.ramp_speed=0.
        self.best_remaining=math.inf; self.progress_time=None
        self.turning=True; self.corner_settle=None
        self.tracking_replans=0
        self.tracking_replan_streak=0; self.replan_anchor_remaining=None; self.replan_anchor_pose=None
        self.turn_reference=None; self.best_turn_error=math.inf
        self.sensor_rotation=rpy_matrix(config['sensor_rpy'])

    def cancel(self,reason='Cancelled'):
        self.discard_plan()
        self.goal=None; self.path=[]; self.command=(0.,0.); self.settle_since=None
        self.ramp_speed=0.
        self.state='IDLE' if self.alignment is not None else 'WAIT_INITIAL_POSE'
        self.reason=reason

    def discard_plan(self):
        if self.planner is not None and self.plan_request is not None:
            self.planner.discard(self.plan_request)
        self.plan_request=None; self.plan_pose=None

    def fail(self,reason,lose_localization=False):
        self.cancel(reason)
        if lose_localization:
            self.alignment=None; self.pose=None; self.cloud_received=None; self.cloud_stamp=None
        self.state='WAIT_INITIAL_POSE' if self.alignment is None else 'STOPPED'

    def update_odometry(self,position,quaternion,stamp,now,frames=('world','base_link')):
        xyz=np.asarray(position,dtype=float); q=normalized_quaternion(quaternion)
        if xyz.shape!=(3,) or not np.isfinite(xyz).all() or not math.isfinite(stamp) or stamp<=0:
            raise ValueError('Invalid odometry position/timestamp')
        if not all(isinstance(f,str) and f for f in frames): raise ValueError('Empty odometry frame')
        tilt=math.acos(float(np.clip(rotation_matrix(q)[2,2],-1,1)))
        if tilt>self.cfg['max_tilt']:
            self.fail('Excessive robot tilt; reinitialize after recovery',True)
            raise ValueError('Excessive tilt')
        yaw=yaw_of(q)
        if self.odom_stamp is not None:
            if stamp==self.odom_stamp: return False
            reset=(stamp<self.odom_stamp or tuple(frames)!=self.frames
                   or now-self.odom_received>self.cfg['odom_timeout'])
            dt=stamp-self.odom_stamp
            if not reset and self.odom is not None:
                jump=np.linalg.norm(xyz[:2]-self.odom[0][:2])
                turn=abs(wrap(yaw-yaw_of(self.odom[1])))
                reset=jump>max(.30,5*self.cfg['max_speed']*dt) or turn>max(.5,5*self.cfg['max_yaw_rate']*dt)
            if reset:
                self.fail('Odometry reset/gap/frame change/jump; initial pose is invalid',True)
                self.history.clear()
        self.odom=(xyz,q); self.odom_stamp=stamp; self.odom_received=now; self.frames=tuple(frames)
        self.history.append((stamp,xyz.copy(),q.copy()))
        while len(self.history)>2 and stamp-self.history[0][0]>.8: self.history.popleft()
        older=min(self.history,key=lambda item:abs(item[0]-(stamp-.15)))
        dt=stamp-older[0]
        if dt>=.07:
            self.linear_speed=float(np.linalg.norm(xyz[:2]-older[1][:2])/dt)
            self.angular_speed=abs(wrap(yaw-yaw_of(older[2])))/dt
        else: self.linear_speed=self.angular_speed=math.inf
        if self.alignment is not None:
            self.pose=transform_pose(self.alignment,(float(xyz[0]),float(xyz[1]),yaw))
        return True

    def initialize(self,initial,now):
        self.cancel('Initial pose requested'); self.alignment=None; self.pose=None
        if self.odom_received is None or now-self.odom_received>self.cfg['odom_timeout']:
            self.fail('Initial pose rejected: odometry unavailable',True); return False
        if not self.is_stopped():
            self.fail('Initial pose rejected: wait for measured standstill, then repeat',True); return False
        raw=(float(self.odom[0][0]),float(self.odom[0][1]),yaw_of(self.odom[1]))
        self.alignment=align_pose(initial,raw); self.pose=transform_pose(self.alignment,raw)
        self.state='IDLE'; self.reason='Initial pose set; check overlay before clicking'
        return True

    def update_cloud(self,local_points,stamp,now):
        if not math.isfinite(stamp) or stamp<=0: raise ValueError('Invalid cloud timestamp')
        if self.cloud_stamp is not None and stamp<=self.cloud_stamp:
            if stamp<self.cloud_stamp: self.fail('Cloud timestamp reset; reinitialize',True)
            return False
        if not self.history: raise ValueError('Cloud has no corresponding odometry')
        sample=min(self.history,key=lambda item:abs(item[0]-stamp))
        if abs(sample[0]-stamp)>.12: raise ValueError('Cloud/odometry mismatch exceeds 120 ms')
        points=np.asarray(local_points,dtype=float)
        if points.ndim!=2 or points.shape[1]!=3: raise ValueError('Invalid cloud shape')
        valid=np.isfinite(points).all(axis=1)&(np.linalg.norm(points,axis=1)>.10)
        points=points[valid]
        if not len(points): raise ValueError('Empty/invalid cloud cannot certify a clear path')
        body=points@self.sensor_rotation.T+np.array(self.cfg['sensor_translation'])
        world=body@rotation_matrix(sample[2]).T+sample[1]
        z=world[:,2]-self.cfg['ground_z']
        obstacle=(z>=self.cfg['obstacle_min_height'])&(z<=self.cfg['obstacle_max_height'])
        self.obstacles=world[obstacle,:2]
        self.cloud_stamp=stamp; self.cloud_received=now
        return True

    def is_stopped(self):
        return self.linear_speed<=self.cfg['stop_speed'] and self.angular_speed<=self.cfg['stop_yaw_rate']

    def healthy(self,now):
        if self.alignment is None or self.pose is None: return 'Initial pose is not set'
        if self.odom_received is None or now-self.odom_received>self.cfg['odom_timeout']: return 'Odometry is stale'
        if self.cloud_received is None or now-self.cloud_received>self.cfg['cloud_timeout']: return 'LiDAR is stale/invalid'
        return ''

    def set_goal(self,point,now):
        self.cancel('New click replaces previous goal')
        error=self.healthy(now)
        if error: self.fail(error); return False
        try:
            if not self.grid.is_free(self.grid.cell(point)): raise ValueError('Clicked point is not robot-sized known free space')
        except (TypeError,ValueError) as exc:
            self.fail(str(exc)); return False
        self.goal=tuple(map(float,point)); self.goal_started=now
        self.tracking_replans=0
        self.tracking_replan_streak=0; self.replan_anchor_remaining=None; self.replan_anchor_pose=None
        self.turn_reference=None; self.best_turn_error=math.inf
        self.state='STOPPING_FOR_PLAN'; self.reason='Waiting for standstill before planning'
        return True

    @staticmethod
    def arc(v,w,t):
        if abs(w)<1e-8: return (v*t,0.)
        return (v*math.sin(w*t)/w,v*(1-math.cos(w*t))/w)

    def motion_clear(self,v,w):
        measured=self.linear_speed if math.isfinite(self.linear_speed) else 0.
        braking_speed=max(v,measured)
        stopping=braking_speed*self.cfg['reaction_seconds']+braking_speed*braking_speed/(2*self.cfg['acceleration'])+self.cfg['obstacle_buffer']
        target=self.path[self.path_index] if self.path and self.path_index<len(self.path) else self.goal
        # Near a corner, don't extrapolate a straight cruise past the turn;
        # retain the full stopping distance even if it extends past the goal.
        distance=max(stopping,min(math.dist(self.pose[:2],target),self.cfg['local_lookahead'])) if v else 0.
        horizon=distance/max(v,.01) if v else self.cfg['reaction_seconds']
        times=np.linspace(0,horizon,max(2,int(distance/(self.grid.resolution*.4))+1))
        offsets=np.array([self.arc(v,w,float(t)) for t in times])
        raw_yaw=yaw_of(self.odom[1]); c,s=math.cos(raw_yaw),math.sin(raw_yaw)
        local_obstacles=(self.obstacles-self.odom[0][:2])@np.array([[c,-s],[s,c]])
        radius=self.cfg['robot_radius']+self.cfg['safety_margin']
        if len(local_obstacles):
            near=local_obstacles[np.linalg.norm(local_obstacles,axis=1)<=distance+radius+.2]
            for centre in offsets:
                if len(near) and np.any(np.linalg.norm(near-centre,axis=1)<=radius): return False,'Live obstacle: stopped; inspect and choose another goal'
        c,s=math.cos(self.pose[2]),math.sin(self.pose[2])
        projected=offsets@np.array([[c,s],[-s,c]])+np.asarray(self.pose[:2])
        if any(not self.grid.is_free(self.grid.cell(p)) for p in projected): return False,'Predicted motion leaves footprint-clear space'
        return True,''

    def tick(self,now):
        dt=1/self.cfg['control_hz'] if self.last_tick is None else now-self.last_tick
        if dt<0 or (dt>self.cfg['command_timeout'] and any(self.command)):
            self.fail('Control loop stalled; old goal cancelled')
        self.last_tick=now
        if self.goal is None:
            self.ramp_speed=0.; self.command=(0.,0.); return self.command
        error=self.healthy(now)
        if error:
            self.fail(error); return self.command
        if now-self.goal_started>self.cfg['goal_timeout']:
            self.fail('Goal time limit exceeded'); return self.command
        if self.state in ('STOPPING_FOR_PLAN','PLANNING'):
            self.ramp_speed=0.; self.command=(0.,0.)
            if (not self.is_stopped() or (self.plan_pose is not None
                    and math.dist(self.pose[:2],self.plan_pose)>self.cfg['goal_tolerance']*.5)):
                self.discard_plan(); self.state='STOPPING_FOR_PLAN'
                self.reason='Waiting for standstill before planning'
                self.settle_since=None; return self.command
            if self.settle_since is None: self.settle_since=now
            if now-self.settle_since<self.cfg['settle_seconds']: return self.command
            if self.planner is None:
                try: self.path=self.grid.plan(self.pose[:2],self.goal)
                except ValueError as exc: self.fail(str(exc)); return self.command
            else:
                if self.plan_request is None:
                    self.plan_pose=tuple(self.pose[:2])
                    self.plan_request=self.planner.submit(self.plan_pose,self.goal)
                    self.state='PLANNING'; self.reason='Planning while stopped; sensors remain active'
                    return self.command
                result=self.planner.poll(self.plan_request)
                if result is None: return self.command
                self.plan_request=None; self.plan_pose=None
                path,error=result
                if error is not None:
                    self.fail('Path planning failed: '+str(error)); return self.command
                self.path=path
            self.path_index=1; self.state='TRACKING'; self.reason='Following planned path'
            self.turning=True; self.corner_settle=None
            self.replan_anchor_remaining=sum(math.dist(a,b) for a,b in zip(self.path,self.path[1:]))
            self.replan_anchor_pose=tuple(self.pose[:2])
            self.turn_reference=None; self.best_turn_error=math.inf
            self.best_remaining=math.inf; self.progress_time=now; self.settle_since=None
            return self.command
        distance=math.dist(self.pose[:2],self.goal)
        if distance<=self.cfg['goal_tolerance']:
            self.ramp_speed=0.; self.command=(0.,0.); self.state='SETTLING'; self.reason='Zero command; verifying measured standstill'
            if not self.is_stopped(): self.settle_since=None; return self.command
            if self.settle_since is None: self.settle_since=now
            if now-self.settle_since>=self.cfg['settle_seconds']:
                self.goal=None; self.state='ARRIVED'; self.reason='Within tolerance and measured standstill confirmed'
            return self.command
        self.settle_since=None; self.state='TRACKING'
        if (self.path_index<len(self.path)-1
                and math.dist(self.pose[:2],self.path[self.path_index])<min(self.cfg['goal_tolerance']*.5,self.grid.resolution*.6)
                and self.grid.segment_free(self.pose[:2],self.path[self.path_index+1])):
            # Brake and verify actual standstill before changing segment. A
            # commanded instant turn otherwise cuts corners with SDK/gait lag.
            self.ramp_speed=0.; self.command=(0.,0.)
            if not self.is_stopped(): self.corner_settle=None; return self.command
            if self.corner_settle is None: self.corner_settle=now
            if now-self.corner_settle<.2: return self.command
            self.path_index+=1; self.turning=True; self.corner_settle=None
        target=self.path[self.path_index]
        if not self.grid.segment_free(self.pose[:2],target):
            if self.grid.is_free(self.grid.cell(self.pose[:2])) and self.tracking_replan_streak<self.cfg['max_tracking_replans']:
                self.tracking_replans+=1; self.tracking_replan_streak+=1
                self.ramp_speed=0.; self.command=(0.,0.); self.path=[]; self.settle_since=None
                self.state='STOPPING_FOR_PLAN'; self.reason='Tracking deviation: stop before bounded static-map replan'
                return self.command
            self.fail('Tracking error crosses blocked cells; reselect a goal'); return self.command
        remaining=math.dist(self.pose[:2],target)+sum(math.dist(a,b) for a,b in zip(self.path[self.path_index:],self.path[self.path_index+1:]))
        if (self.tracking_replan_streak and self.replan_anchor_remaining is not None
                and self.replan_anchor_remaining-remaining>=self.cfg['replan_progress_distance']
                and math.dist(self.pose[:2],self.replan_anchor_pose)>=self.cfg['replan_progress_distance']*.5):
            # Separate recoveries along a long route are not repeated failure
            # at one location. Total goal time remains anchored to the click.
            self.tracking_replan_streak=0
        if remaining<self.best_remaining-.02: self.best_remaining=remaining; self.progress_time=now
        heading=math.atan2(target[1]-self.pose[1],target[0]-self.pose[0]); error=wrap(heading-self.pose[2])
        if self.turning:
            if self.turn_reference!=tuple(target):
                self.turn_reference=tuple(target); self.best_turn_error=abs(error)
            elif abs(error)<self.best_turn_error-.03:
                self.best_turn_error=abs(error); self.progress_time=now
        if now-self.progress_time>self.cfg['progress_timeout']:
            self.fail('No path progress; stopped'); return self.command
        w=float(np.clip(1.8*error,-self.cfg['max_yaw_rate'],self.cfg['max_yaw_rate']))
        if abs(error)<.015: w=0.
        v=min(self.cfg['max_speed'],.7*math.dist(self.pose[:2],target),math.sqrt(2*self.cfg['acceleration']*max(0,distance-self.cfg['goal_tolerance']*.5)))
        v=0. if abs(error)>.35 else v*math.cos(error)
        if abs(error)>.35 and not self.turning:
            self.turning=True
            self.turn_reference=tuple(target); self.best_turn_error=abs(error)
        if self.turning:
            v=0.
            if abs(error)<.025:
                w=0.
                if self.is_stopped(): self.turning=False
        # The SDK cannot express 0 < vx < 0.05. Accumulate the startup ramp
        # separately from the emitted zero commands, or it never starts.
        # Approaching an intermediate corner still needs a legal crawl speed
        # until the existing measured-position/standstill transition accepts it.
        if v>0:
            desired=min(self.cfg['max_speed'],max(v,MIN_FORWARD_SPEED))
            self.ramp_speed=min(desired,self.ramp_speed+self.cfg['acceleration']*min(max(dt,0),.1))
            v=self.ramp_speed if self.ramp_speed>=MIN_FORWARD_SPEED else 0.
        else:
            self.ramp_speed=0.
        if abs(w)<MIN_YAW_RATE: w=0.
        # Check the actual SDK-domain command, including the minimum crawl.
        clear,reason=self.motion_clear(v,w)
        if not clear and reason.startswith('Predicted') and v>0:
            self.ramp_speed=0.; v=0.; clear,reason=self.motion_clear(v,w)
        if not clear:
            self.fail(reason); return self.command
        self.reason='Aligning next segment' if self.turning else 'Following planned path'
        self.command=(float(v),float(w))
        return self.command

    def snapshot(self):
        remaining=None
        if self.goal is not None and self.pose is not None and self.path_index<len(self.path):
            remaining=(math.dist(self.pose[:2],self.path[self.path_index])+
                       sum(math.dist(a,b) for a,b in zip(self.path[self.path_index:],self.path[self.path_index+1:])))
        return {'state':self.state,'reason':self.reason,'pose':self.pose,'goal':self.goal,
                'tracking_replans':self.tracking_replans,
                'tracking_replan_streak':self.tracking_replan_streak,
                'path_index':self.path_index,'path_points':len(self.path),
                'remaining_path_m':remaining,
                'command':self.command,'linear_speed':self.linear_speed if math.isfinite(self.linear_speed) else None,
                'angular_speed':self.angular_speed if math.isfinite(self.angular_speed) else None}
