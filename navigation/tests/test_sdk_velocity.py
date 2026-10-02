"""Exercise navigation against the documented SDK domain, without loading it."""
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from navigation.configuration import load_config
from navigation.grid import GridMap
from navigation.sdk_worker import CommandGuard
from navigation.tests.test_navigation import Rig


class VelocityDomainSDK:
    """Independent fake of ZSL-1 move() limits from docs/api_zsl-1.md."""
    def __init__(self,config):
        self.config=config
        self.commands=[]

    def move(self,vx,vy,yaw):
        if not all(math.isfinite(x) for x in (vx,vy,yaw)):
            raise AssertionError('Nonfinite command reached SDK')
        if vx!=0 and not .05<=abs(vx)<=3.:
            raise AssertionError(f'SDK would reject vx={vx} with 0x3013')
        if vy!=0 and not .1<=abs(vy)<=1.:
            raise AssertionError(f'SDK would reject vy={vy} with 0x3013')
        if yaw!=0 and not .02<=abs(yaw)<=3.:
            raise AssertionError(f'SDK would reject yaw={yaw} with 0x3013')
        if not 0<=vx<=self.config['max_speed'] or vy!=0:
            raise AssertionError('Navigation exceeded configured forward-only limit')
        if abs(yaw)>self.config['max_yaw_rate']:
            raise AssertionError('Navigation exceeded configured yaw limit')
        self.commands.append((vx,vy,yaw))
        return 0


class SDKDomainRig(Rig):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.sdk=VelocityDomainSDK(self.cfg)
        original_tick=self.nav.tick

        def checked_tick(now):
            command=original_tick(now)
            self.sdk.move(command[0],0.,command[1])
            return command

        # Check every emitted tick, including direct calls outside Rig.step().
        self.nav.tick=checked_tick


class SDKVelocityNavigationTests(unittest.TestCase):
    def advance_until_moving(self,rig,minimum=.05):
        for _ in range(150):
            if rig.step()[0]>=minimum:
                return
        self.fail(f'Navigation did not start: {rig.nav.snapshot()}')

    def assert_restart_ramps_from_zero(self,rig):
        for _ in range(150):
            rig.step()
            if rig.nav.state=='TRACKING':
                break
        else:
            self.fail(f'Navigation never entered TRACKING: {rig.nav.snapshot()}')
        commands=[]
        for _ in range(40):
            commands.append(rig.step())
            if commands[-1][0]>0:
                break
        self.assertGreater(commands[-1][0],0.,rig.nav.snapshot())
        self.assertGreaterEqual(len(commands)-1,4,'Startup ramp emitted motion too soon')
        self.assertLessEqual(commands[-1][0],.05+rig.cfg['acceleration']*.05+1e-9)

    def test_short_goals_arrive_under_real_sdk_domain_and_configured_caps(self):
        for max_speed,lag,goal in ((.2,0.,(.3,0.)),(.05,.25,(.4,.2)),(.12,.25,(1.,.3))):
            with self.subTest(max_speed=max_speed,lag=lag):
                config=load_config(); config['max_speed']=max_speed
                rig=SDKDomainRig(config=config,response_tau=lag)
                result=rig.reach(goal)
                self.assertEqual(result['state'],'ARRIVED',result)
                self.assertLessEqual(math.dist(rig.pose[:2],goal),config['goal_tolerance'])
                self.assertTrue(any(v>0 for v,_,_ in rig.sdk.commands))
                settling=[sample for sample in rig.samples if sample[-1]=='SETTLING']
                self.assertTrue(settling)
                self.assertTrue(all(sample[4:6]==(0.,0.) for sample in settling))
                for _ in range(20):
                    self.assertEqual(rig.step(),(0.,0.))

    def test_intermediate_corner_does_not_stall_in_sdk_deadband(self):
        corner=(.3,0.); goal=(.3,.7)
        for lag in (0.,.25):
            with self.subTest(response_tau=lag):
                rig=SDKDomainRig(response_tau=lag)
                # Supply a real, footprint-clear two-segment route so the test
                # cannot disappear when the open-room planner simplifies it.
                path=[(0.,0.),corner,goal]
                self.assertTrue(all(rig.grid.segment_free(a,b) for a,b in zip(path,path[1:])))
                with patch.object(rig.grid,'plan',return_value=path):
                    result=rig.reach(goal)
                self.assertEqual(result['state'],'ARRIVED',result)
                near_corner=[r for r in rig.samples if .03<math.dist(r[1:3],corner)<.07 and r[4]>0]
                self.assertTrue(near_corner,'Route must traverse the former low-speed stall region')
                self.assertTrue(any(abs(r[4]-.05)<1e-9 for r in near_corner))
                self.assertLessEqual(math.dist(rig.pose[:2],goal),rig.cfg['goal_tolerance'])

    def test_planned_route_around_wall_with_lag_emits_only_sdk_legal_commands(self):
        cells=np.zeros((160,160)); cells[45:105,80]=100
        grid=GridMap(cells,.05,(-4.,-4.,0.))
        start=(-1.5,0.,0.); goal=(1.5,0.)
        self.assertGreater(len(grid.plan(start[:2],goal)),2)
        rig=SDKDomainRig(grid,start,response_tau=.25)
        result=rig.reach(goal)
        self.assertEqual(result['state'],'ARRIVED',result)
        self.assertLessEqual(result['linear_speed'],rig.cfg['stop_speed'])

    def test_cancel_or_replacement_goal_restarts_with_fresh_acceleration(self):
        for cancel_first in (False,True):
            with self.subTest(cancel_first=cancel_first):
                rig=SDKDomainRig(); self.assertTrue(rig.nav.set_goal((2.,0.),rig.t))
                self.advance_until_moving(rig,.18)
                if cancel_first:
                    rig.nav.cancel()
                    for _ in range(10):
                        self.assertEqual(rig.step(),(0.,0.))
                self.assertTrue(rig.nav.set_goal((float(rig.pose[0])+1.,0.),rig.t))
                self.assertEqual(rig.nav.command,(0.,0.))
                self.assert_restart_ramps_from_zero(rig)

    def test_safety_check_sees_legal_output_and_blocked_motion_resets_startup(self):
        rig=SDKDomainRig(); self.assertTrue(rig.nav.set_goal((2.,0.),rig.t))
        original_clear=rig.nav.motion_clear
        inspected=VelocityDomainSDK(rig.cfg)
        blocked=[False]

        def checking_clear(v,w):
            inspected.move(v,0.,w)
            if blocked[0] and v>0:
                return False,'Predicted motion leaves footprint-clear space'
            return original_clear(v,w)

        with patch.object(rig.nav,'motion_clear',side_effect=checking_clear):
            self.advance_until_moving(rig,.18)
            blocked[0]=True
            self.assertEqual(rig.step(),(0.,0.))
            self.assertEqual(rig.nav.state,'TRACKING')
            blocked[0]=False
            # A blocked command must not leave a hidden cruise-speed ramp.
            for _ in range(4):
                self.assertEqual(rig.step(),(0.,0.))
            self.advance_until_moving(rig)
            self.assertLessEqual(rig.nav.command[0],.06+1e-9)
        self.assertTrue(inspected.commands)

    def test_goal_already_in_tolerance_never_receives_minimum_crawl(self):
        rig=SDKDomainRig(response_tau=.25)
        result=rig.reach((.05,0.))
        self.assertEqual(result['state'],'ARRIVED',result)
        self.assertTrue(all(command==(0.,0.,0.) for command in rig.sdk.commands))

    def test_sensor_fault_emits_zero_and_cannot_resume_without_new_goal(self):
        rig=SDKDomainRig(); self.assertTrue(rig.nav.set_goal((2.,0.),rig.t))
        self.advance_until_moving(rig,.18)
        for _ in range(12):
            rig.step(cloud=False)
        self.assertEqual(rig.nav.state,'STOPPED')
        self.assertEqual(rig.nav.command,(0.,0.))
        for _ in range(20):
            self.assertEqual(rig.step(),(0.,0.))
        self.assertIsNone(rig.nav.goal)


class SDKVelocityBoundaryTests(unittest.TestCase):
    def test_worker_rejects_nonzero_sdk_deadbands_before_accepting_packet(self):
        for v,w in ((.01,0.),(.0499,0.),(0.,.01),(0.,-.01),(.05,.0199)):
            with self.subTest(v=v,w=w):
                guard=CommandGuard(.3,.2,.35)
                with self.assertRaisesRegex(ValueError,'SDK'):
                    guard.accept(f'1 10 {v} {w}',10.)
                self.assertEqual(guard.current(10.),(0.,0.))

    def test_worker_preserves_legal_boundaries_and_exact_zero(self):
        guard=CommandGuard(.3,.2,.35)
        for i,(v,w) in enumerate(((.05,.02),(.05,-.02),(0.,0.),(.2,.35)),1):
            guard.accept(f'{i} 10 {v} {w}',10.)
            self.assertEqual(guard.current(10.),(v,w))

    def test_configuration_rejects_maxima_that_cannot_express_sdk_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            config=Path(tmp)/'local.yaml'
            for field,value in (('max_speed',.0499),('max_yaw_rate',.0199)):
                with self.subTest(field=field):
                    config.write_text(f'{field}: {value}\n',encoding='utf-8')
                    with self.assertRaisesRegex(ValueError,'SDK requires'):
                        load_config(config)
            config.write_text('max_speed: 0.05\nmax_yaw_rate: 0.02\n',encoding='utf-8')
            loaded=load_config(config)
            self.assertEqual(loaded['max_speed'],.05)
            self.assertEqual(loaded['max_yaw_rate'],.02)


if __name__=='__main__':
    unittest.main()
