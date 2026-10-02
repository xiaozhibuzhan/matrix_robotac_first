"""Offline behavioral tests. These do not claim ROS, SDK, or gait validation."""
import json
import math
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace as NS
import unittest
import numpy as np
from navigation.configuration import ROOT,load_config
from navigation.core import Navigator
from navigation.geometry import align_pose,transform_pose
from navigation.grid import GridMap,read_pgm,write_pgm
from navigation.prepare_map import ground_support
from navigation.sdk_worker import CommandGuard
from navigation.sensors import lidar_points


def room():
    cells=np.zeros((160,160),dtype=np.int8)
    cells[[0,-1],:]=100; cells[:,[0,-1]]=100
    return GridMap(cells,.05,(-4.,-4.,0.))


class Rig:
    def __init__(self,grid=None,start=(0.,0.,0.),config=None,response_tau=0.):
        self.cfg=config or load_config(); self.grid=grid or room(); self.nav=Navigator(self.grid,self.cfg)
        self.pose=np.array(start,dtype=float); self.t=10.; self.samples=[]
        self.response_tau=response_tau; self.actual=np.zeros(2)
        for _ in range(7): self.feed(); self.t+=.05
        assert self.nav.initialize(tuple(start),self.t)

    def feed(self,cloud=True):
        x,y,yaw=self.pose
        self.nav.update_odometry((x,y,0.),(0.,0.,math.sin(yaw/2),math.cos(yaw/2)),1000+self.t,self.t)
        if cloud: self.nav.update_cloud([[2.,2.,-.3]],1000+self.t,self.t)

    def step(self,cloud=True,move=True):
        self.t+=.05; self.feed(cloud)
        v,w=self.nav.tick(self.t)
        self.samples.append((self.t,*self.pose,v,w,self.nav.state))
        if move:
            alpha=1. if self.response_tau==0 else 1.-math.exp(-.05/self.response_tau)
            self.actual+=alpha*(np.array([v,w])-self.actual)
            dx,dy=self.nav.arc(*self.actual,.05); c,s=math.cos(self.pose[2]),math.sin(self.pose[2])
            self.pose[:2]+=[c*dx-s*dy,s*dx+c*dy]; self.pose[2]+=self.actual[1]*.05
        return v,w

    def reach(self,goal,limit=4000):
        assert self.nav.set_goal(goal,self.t),self.nav.reason
        for _ in range(limit):
            self.step()
            assert self.grid.is_free(self.grid.cell(self.pose[:2])),self.nav.snapshot()
            if self.nav.state in ('ARRIVED','STOPPED','WAIT_INITIAL_POSE'): break
        return self.nav.snapshot()


class GridTests(unittest.TestCase):
    def test_binary_pgm_preserves_whitespace_pixels(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'x.pgm'; expected=np.array([[10,13,32,9,0,255]],dtype=np.uint8)
            write_pgm(p,expected); np.testing.assert_array_equal(read_pgm(p),expected)

    def test_endpoint_on_grid_line_does_not_walk_past_end(self):
        g=room(); start=(.00047,.00016); end=(1.,0.)
        self.assertTrue(g.segment_free(start,end)); self.assertTrue(g.segment_free(end,start))

    def test_diagonal_corner_cut_is_blocked(self):
        cells=np.zeros((5,5)); cells[1,2]=100
        g=GridMap(cells,1.,(0.,0.,0.),0.,0.)
        # Use raw masks solely to isolate traversal from footprint inflation.
        g.passable=cells==0
        self.assertFalse(g.segment_free((1.5,1.5),(2.5,2.5)))

    def test_pgm_comments_and_ascii(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'x.pgm'; p.write_text('P2'+chr(10)+'# comment'+chr(10)+'2 1'+chr(10)+'255'+chr(10)+'0 254')
            np.testing.assert_array_equal(read_pgm(p),[[0,254]])

    def test_rotated_origin_roundtrip(self):
        g=GridMap(np.zeros((30,30)),.1,(5.,-4.,math.pi/3),.1,.02)
        for cell in [(0,0),(10,20),(29,29)]: self.assertEqual(g.cell(g.world(cell)),cell)

    def test_footprint_excludes_unknown_and_border(self):
        g=room(); self.assertFalse(g.is_free((1,1))); self.assertTrue(g.is_free(g.cell((0.,0.))))
        cells=np.zeros((80,80)); cells[:,40]=-1; g=GridMap(cells,.05,(-2.,-2.,0.))
        with self.assertRaisesRegex(ValueError,'No path'): g.plan((-1.,0.),(1.,0.))

    def test_astar_routes_around_wall_without_corner_cutting(self):
        cells=np.zeros((100,100)); cells[15:80,50]=100
        g=GridMap(cells,.05,(-2.5,-2.5,0.),.2,.05)
        start=(-1.,0.); goal=(1.,0.); path=g.plan(start,goal)
        self.assertGreater(len(path),2); self.assertEqual(path[-1],goal)
        self.assertTrue(all(g.segment_free(a,b) for a,b in zip(path,path[1:])))
        self.assertFalse(g.segment_free(start,goal))

    def test_invalid_goal_not_moved_to_nearest_free_cell(self):
        g=room()
        with self.assertRaises(ValueError): g.plan((0.,0.),(10.,10.))

    def test_ground_interpolation_does_not_bridge_large_gaps(self):
        g=GridMap(np.full((60,60),-1),.05,(-1.5,-1.5,0.))
        xy=np.array([(x,y) for base in (-1.,.8) for x in np.arange(base,base+.21,.1) for y in np.arange(-.2,.21,.1)])
        points=np.column_stack((xy,np.zeros(len(xy))))
        support,_=ground_support(points,g,max_edge=.3)
        self.assertTrue(support.any()); self.assertFalse(support[g.cell((0.,0.))[1],g.cell((0.,0.))[0]])

    def test_collinear_ground_rejected(self):
        with self.assertRaises(ValueError): ground_support([[0,0,0],[1,0,0],[2,0,0]],room())

    def test_real_map_preserves_all_original_obstacles(self):
        cfg=load_config(); derived=GridMap.load(cfg['map']); source=GridMap.load(ROOT/'maps/run_20260911_223510_ujmvehqs/field_map.yaml')
        self.assertTrue(np.all(derived.occupancy[source.occupancy==100]==100))
        self.assertTrue(np.all(derived.occupancy[source.occupancy==0]==0)); self.assertGreater(derived.passable.sum(),40000)


class NavigationTests(unittest.TestCase):
    def test_straight_goal_arrives_and_stays_stopped(self):
        rig=Rig(); result=rig.reach((1.5,0.))
        self.assertEqual(result['state'],'ARRIVED',result)
        self.assertLessEqual(math.dist(result['pose'][:2],(1.5,0.)),rig.cfg['goal_tolerance'])
        self.assertLessEqual(result['linear_speed'],rig.cfg['stop_speed'])
        for _ in range(30): self.assertEqual(rig.step(),(0.,0.))

    def test_turn_before_driving(self):
        rig=Rig(start=(0.,0.,math.pi)); result=rig.reach((1.,0.))
        self.assertEqual(result['state'],'ARRIVED',result)
        moving=[r for r in rig.samples if r[4]>0]
        self.assertTrue(moving); self.assertLess(abs(moving[0][3]),.35)

    def test_braking_lag_must_settle_before_arrival(self):
        rig=Rig(start=(0.,0.,1.),response_tau=.25); result=rig.reach((1.5,.5))
        self.assertEqual(result['state'],'ARRIVED',result)
        self.assertLessEqual(np.linalg.norm(rig.actual),rig.cfg['stop_speed'])
        self.assertLessEqual(math.dist(rig.pose[:2],(1.5,.5)),rig.cfg['goal_tolerance'])

    def test_repeated_goals_and_different_headings(self):
        rig=Rig()
        for goal in ((1.,0.),(.8,.7),(-.3,.8),(-.3,-.5),(0.,0.)):
            with self.subTest(goal=goal): self.assertEqual(rig.reach(goal)['state'],'ARRIVED',rig.nav.snapshot())

    def test_route_around_wall(self):
        cells=np.zeros((160,160)); cells[45:105,80]=100
        for lag in (0.,.25):
            with self.subTest(response_tau=lag):
                rig=Rig(GridMap(cells,.05,(-4.,-4.,0.)),(-1.5,0.,0.),response_tau=lag)
                result=rig.reach((1.5,0.))
                self.assertEqual(result['state'],'ARRIVED',result)
                self.assertLessEqual(rig.nav.tracking_replans,rig.cfg['max_tracking_replans'])

    def test_invalidated_tracking_segment_stops_and_respects_replan_budget(self):
        cells=np.zeros((160,160)); cells[45:105,80]=100
        rig=Rig(GridMap(cells,.05,(-4.,-4.,0.)),(-1.5,0.,0.))
        self.assertTrue(rig.nav.set_goal((1.5,0.),rig.t))
        started=rig.nav.goal_started
        for attempt in range(rig.cfg['max_tracking_replans']+1):
            # A deviation has left the current segment crossing the known wall.
            # Exercise this explicitly instead of requiring every lagged run
            # to deviate (a successful collision-free route need not replan).
            rig.nav.state='TRACKING'
            rig.nav.path=[(-1.5,0.),(1.5,0.)]; rig.nav.path_index=1
            self.assertEqual(rig.step(move=False),(0.,0.))
            if attempt<rig.cfg['max_tracking_replans']:
                self.assertEqual(rig.nav.state,'STOPPING_FOR_PLAN')
                self.assertEqual(rig.nav.goal,(1.5,0.))
                self.assertEqual(rig.nav.goal_started,started)
                self.assertEqual(rig.nav.tracking_replans,attempt+1)
            else:
                self.assertEqual(rig.nav.state,'STOPPED')
                self.assertIsNone(rig.nav.goal)

    def test_real_map_short_goal(self):
        cfg=load_config(); g=GridMap.load(cfg['map']); rig=Rig(g,(-.00291023254,.0058063507,0.))
        candidates=[g.world((x,y)) for y,x in np.argwhere(g.passable) if 1.4<math.dist(g.world((x,y)),rig.pose[:2])<1.6 and g.segment_free(rig.pose[:2],g.world((x,y)))]
        self.assertTrue(candidates); goal=min(candidates,key=lambda p:abs(math.atan2(p[1]-rig.pose[1],p[0]-rig.pose[0])))
        self.assertEqual(rig.reach(goal)['state'],'ARRIVED',rig.nav.snapshot())

    def test_initial_alignment_uses_manual_pose_and_odom(self):
        transform=align_pose((5.,6.,math.pi/2),(1.,2.,0.))
        np.testing.assert_allclose(transform_pose(transform,(2.,2.,0.)),(5.,7.,math.pi/2),atol=1e-10)

    def test_no_initial_pose_rejects_motion(self):
        nav=Navigator(room(),load_config()); self.assertFalse(nav.set_goal((1.,0.),10.)); self.assertEqual(nav.tick(10.),(0.,0.))

    def test_new_goal_and_cancel_send_zero(self):
        rig=Rig(); rig.nav.set_goal((2.,0.),rig.t)
        for _ in range(40): rig.step()
        self.assertGreater(rig.nav.command[0],0)
        self.assertTrue(rig.nav.set_goal((1.,1.),rig.t)); self.assertEqual(rig.nav.command,(0.,0.))
        rig.nav.cancel(); self.assertIsNone(rig.nav.goal); self.assertEqual(rig.step(),(0.,0.))

    def test_cloud_staleness_cancels_and_does_not_resume(self):
        rig=Rig(); rig.nav.set_goal((2.,0.),rig.t)
        for _ in range(40): rig.step()
        for _ in range(12): rig.step(cloud=False)
        self.assertEqual(rig.nav.state,'STOPPED'); self.assertEqual(rig.nav.command,(0.,0.))
        for _ in range(10): rig.step()
        self.assertIsNone(rig.nav.goal)

    def test_duplicate_odometry_does_not_refresh_freshness(self):
        rig=Rig(); old=rig.nav.odom_received
        self.assertFalse(rig.nav.update_odometry((0,0,0),(0,0,0,1),rig.nav.odom_stamp,rig.t+1))
        self.assertEqual(rig.nav.odom_received,old); self.assertIn('Odometry',rig.nav.healthy(rig.t+1))

    def test_odometry_restart_or_frame_change_requires_new_initial_pose(self):
        for stamp,frames in [(1.,('world','base_link')),(1011.,('changed','base_link'))]:
            with self.subTest(stamp=stamp,frames=frames):
                rig=Rig(); rig.nav.update_odometry((0,0,0),(0,0,0,1),stamp,rig.t+.05,frames)
                self.assertIsNone(rig.nav.alignment); self.assertEqual(rig.nav.state,'WAIT_INITIAL_POSE')

    def test_empty_cloud_and_bad_timestamp_rejected(self):
        rig=Rig()
        for points,stamp in [([],rig.nav.odom_stamp+.01),([[1,1,0]],rig.nav.odom_stamp+2)]:
            with self.subTest(points=points):
                with self.assertRaises(ValueError): rig.nav.update_cloud(points,stamp,rig.t)

    def test_tilt_invalidates_localization(self):
        rig=Rig()
        with self.assertRaises(ValueError): rig.nav.update_odometry((0,0,0),(math.sin(.3),0,0,math.cos(.3)),1011.,rig.t)
        self.assertIsNone(rig.nav.alignment)

    def test_live_obstacle_stops_motion(self):
        rig=Rig(); rig.nav.set_goal((2.,0.),rig.t)
        for _ in range(40): rig.step()
        rig.t+=.05; rig.feed(False); rig.nav.update_cloud([[.65,0.,.1]],1000+rig.t,rig.t)
        self.assertEqual(rig.nav.tick(rig.t),(0.,0.)); self.assertEqual(rig.nav.state,'STOPPED')

    def test_loop_stall_latches_old_goal(self):
        rig=Rig(); rig.nav.set_goal((2.,0.),rig.t)
        for _ in range(40): rig.step()
        rig.t+=1; rig.feed(); self.assertEqual(rig.nav.tick(rig.t),(0.,0.)); self.assertIsNone(rig.nav.goal)

    def test_odometry_reconnection_requires_reinitialization_even_when_idle(self):
        rig=Rig(); rig.t+=.5; rig.feed()
        self.assertIsNone(rig.nav.alignment); self.assertFalse(rig.nav.set_goal((1.,0.),rig.t))

    def test_position_tolerance_alone_does_not_claim_arrival(self):
        rig=Rig(); rig.nav.set_goal((.05,0.),rig.t)
        for _ in range(19): rig.step()
        rig.nav.linear_speed=.1
        self.assertEqual(rig.nav.tick(rig.t+.001),(0.,0.)); self.assertNotEqual(rig.nav.state,'ARRIVED')

    def test_no_progress_times_out(self):
        rig=Rig(); rig.nav.set_goal((2.,0.),rig.t)
        for _ in range(360): rig.step(move=False)
        self.assertEqual(rig.nav.state,'STOPPED'); self.assertIn('progress',rig.nav.reason)


class WatchdogTests(unittest.TestCase):
    def test_fresh_packet_cannot_revive_expired_motion(self):
        guard=CommandGuard(.3,.2,.35); guard.accept('1 10 0.1 0',10)
        with self.assertRaises(ValueError): guard.accept('2 10.4 0.1 0',10.4)
        self.assertTrue(guard.tripped); self.assertEqual(guard.current(10.4),(0.,0.))

    def test_expired_moving_command_latches_stop(self):
        guard=CommandGuard(.3,.2,.35); guard.accept('1 10.0 0.2 0.1',10.)
        self.assertEqual(guard.current(10.1),(.2,.1)); self.assertEqual(guard.current(10.31),(0.,0.)); self.assertTrue(guard.tripped)
        with self.assertRaises(ValueError): guard.accept('2 10.32 0.1 0',10.32)

    def test_queued_commands_keep_source_age(self):
        guard=CommandGuard(.3,.2,.35)
        with self.assertRaises(ValueError): guard.accept('1 10.0 0.2 0',10.4)

    def test_invalid_replayed_or_out_of_range_command_rejected(self):
        for line in ['2 10 nan 0','2 10 0.21 0','2 10 -0.1 0','2 10 0 1','2 11 0 0','1 10 .1 0','junk']:
            guard=CommandGuard(.3,.2,.35); guard.accept('1 10 0 0',10)
            with self.subTest(line=line):
                with self.assertRaises(ValueError): guard.accept(line,10)

    def test_idle_expiry_stays_zero_without_motion_trip(self):
        guard=CommandGuard(.3,.2,.35); self.assertEqual(guard.current(10),(0.,0.)); self.assertFalse(guard.tripped)


class SensorTests(unittest.TestCase):
    def test_padded_rows_endianness_and_metadata_removal(self):
        for endian in ('<','>'):
            payload=bytearray(64)
            struct.pack_into(endian+'ffff',payload,0,1.,2.,3.,0.)
            struct.pack_into(endian+'ffff',payload,32,100.,200.,300.,111.)
            msg=NS(width=1,height=2,point_step=26,row_step=32,data=payload,is_bigendian=endian=='>',fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))])
            np.testing.assert_allclose(lidar_points(msg),[[1,2,3]])
            msg.data=payload[:-1]
            with self.assertRaises(ValueError): lidar_points(msg)


if __name__=='__main__': unittest.main()
