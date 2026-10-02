"""Deterministically block planning while control and sensor callbacks continue."""
import threading
import time
import unittest

from navigation.configuration import load_config
from navigation.planning import AsyncPlanner
from navigation.tests.test_sdk_velocity import SDKDomainRig


class AsyncPlanningTests(unittest.TestCase):
    def setup_blocked(self, **changes):
        cfg=load_config(); cfg.update(changes)
        rig=SDKDomainRig(config=cfg,response_tau=.25)
        entered=threading.Event(); release=threading.Event(); calls=[]

        def plan(start,goal):
            calls.append(goal); entered.set()
            if not release.wait(5): raise RuntimeError('Test did not release planner')
            return rig.grid.plan(start,goal)

        planner=AsyncPlanner(plan); rig.nav.planner=planner
        self.addCleanup(release.set); self.addCleanup(planner.close)
        self.assertTrue(rig.nav.set_goal((2.,0.),rig.t))
        self.submit(rig)
        self.assertTrue(entered.wait(2),'Worker did not begin planning')
        return rig,planner,release,calls

    def submit(self,rig):
        for _ in range(40):
            rig.step()
            if rig.nav.state=='PLANNING': return
        self.fail('No asynchronous planning request: '+str(rig.nav.snapshot()))

    def consume(self,rig):
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            rig.step()
            if rig.nav.state!='PLANNING': return
            # Yield to the worker; correctness never depends on A* wall time.
            time.sleep(.001)
        self.fail('Worker result was not consumed')

    def test_slow_planner_keeps_zero_commands_and_receives_fresh_sensors(self):
        rig,planner,release,calls=self.setup_blocked()
        initial=rig.nav.odom_received
        for _ in range(200):
            self.assertEqual(rig.step(),(0.,0.))
            self.assertEqual(rig.nav.state,'PLANNING')
            self.assertEqual(rig.nav.healthy(rig.t),'')
        self.assertGreater(rig.nav.odom_received-initial,9.)
        release.set(); self.consume(rig)
        self.assertEqual(rig.nav.state,'TRACKING',rig.nav.snapshot())
        self.assertIsNotNone(rig.nav.alignment)

    def test_cancelled_inflight_result_cannot_restore_goal(self):
        rig,planner,release,calls=self.setup_blocked()
        token=rig.nav.plan_request
        rig.nav.cancel(); release.set()
        for _ in range(40): self.assertEqual(rig.step(),(0.,0.))
        self.assertIsNone(planner.poll(token))
        self.assertIsNone(rig.nav.goal); self.assertEqual(rig.nav.path,[])
        self.assertEqual(rig.nav.state,'IDLE')

    def test_replacement_goal_discards_old_result_and_coalesces_waiting_requests(self):
        rig,planner,release,calls=self.setup_blocked()
        old_token=rig.nav.plan_request
        self.assertTrue(rig.nav.set_goal((1.,1.),rig.t)); self.submit(rig)
        intermediate_token=rig.nav.plan_request
        self.assertTrue(rig.nav.set_goal((1.,-1.),rig.t)); self.submit(rig)
        release.set(); self.consume(rig)
        self.assertEqual(rig.nav.state,'TRACKING',rig.nav.snapshot())
        self.assertEqual(rig.nav.path[-1],(1.,-1.))
        self.assertEqual(calls,[(2.,0.),(1.,-1.)])
        self.assertIsNone(planner.poll(old_token)); self.assertIsNone(planner.poll(intermediate_token))

    def test_goal_deadline_still_expires_during_blocked_planning(self):
        rig,planner,release,calls=self.setup_blocked(goal_timeout=2.)
        token=rig.nav.plan_request
        for _ in range(50): rig.step()
        self.assertEqual(rig.nav.state,'STOPPED')
        self.assertIn('time limit',rig.nav.reason)
        release.set()
        self.assertIsNone(planner.poll(token)); self.assertIsNone(rig.nav.goal)

    def test_stale_sensors_cancel_pending_plan(self):
        rig,planner,release,calls=self.setup_blocked()
        token=rig.nav.plan_request
        rig.t+=rig.cfg['odom_timeout']+.01
        self.assertEqual(rig.nav.tick(rig.t),(0.,0.))
        self.assertEqual(rig.nav.state,'STOPPED'); self.assertIn('stale',rig.nav.reason)
        release.set()
        self.assertIsNone(planner.poll(token)); self.assertIsNone(rig.nav.goal)

    def test_motion_during_planning_discards_start_pose_and_waits_to_stop(self):
        rig,planner,release,calls=self.setup_blocked()
        token=rig.nav.plan_request
        rig.pose[0]+=.1; rig.step()
        self.assertEqual(rig.nav.state,'STOPPING_FOR_PLAN')
        self.assertIsNone(rig.nav.plan_request); self.assertIsNone(planner.poll(token))
        self.assertEqual(rig.nav.command,(0.,0.))

    def test_worker_exception_is_reported_as_stopped(self):
        rig=SDKDomainRig()

        def fail(start,goal): raise ValueError('No safe route in test')

        planner=AsyncPlanner(fail); rig.nav.planner=planner; self.addCleanup(planner.close)
        self.assertTrue(rig.nav.set_goal((2.,0.),rig.t)); self.submit(rig); self.consume(rig)
        self.assertEqual(rig.nav.state,'STOPPED'); self.assertIn('No safe route',rig.nav.reason)
        self.assertIsNone(rig.nav.goal); self.assertEqual(rig.nav.command,(0.,0.))

    def test_close_discards_running_result_and_rejects_new_work(self):
        rig,planner,release,calls=self.setup_blocked()
        token=rig.nav.plan_request
        rig.nav.cancel(); planner.close(); release.set()
        with self.assertRaises(RuntimeError): planner.submit((0.,0.),(1.,0.))
        planner._thread.join(timeout=2)
        self.assertFalse(planner._thread.is_alive())
        self.assertIsNone(planner.poll(token)); self.assertIsNone(rig.nav.goal)

    def test_reentering_turn_on_same_segment_refreshes_measured_progress(self):
        cfg=load_config(); cfg['max_yaw_rate']=.02
        rig=SDKDomainRig(config=cfg,response_tau=.25)
        self.assertTrue(rig.nav.set_goal((2.,0.),rig.t))
        for _ in range(100):
            rig.step()
            if not rig.nav.turning and rig.nav.command[0]>.1: break
        self.assertFalse(rig.nav.turning)
        # A measured heading disturbance on the same target, within jump limits.
        rig.pose[2]=.4
        for _ in range(600):
            rig.step()
            self.assertNotEqual(rig.nav.state,'STOPPED',rig.nav.snapshot())
            if not rig.nav.turning and rig.nav.command[0]>.1: break
        else: self.fail('Turn did not finish')
        self.assertLess(abs(rig.pose[2]),.025)
        self.assertEqual(rig.nav.state,'TRACKING')


if __name__=='__main__': unittest.main()
