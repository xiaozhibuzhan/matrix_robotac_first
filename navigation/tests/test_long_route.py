"""Logged-route regressions with simulated lag; no ROS, SDK or gait validation."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from navigation.configuration import load_config
from navigation.grid import GridMap
from navigation.tests.test_sdk_velocity import SDKDomainRig


# Accepted goals and preceding poses from the 2026-10-02 18:00:42 field run.
# Keep these fixtures here so tests work in an update ZIP without private logs.
LOGGED_ROUTES = (
    ((2.3239095177, -.1842999194, -.0388229733),
     (16.7220039368, 3.6776235104)),
    ((16.6930056454, 3.5986972337, 1.2487612143),
     (13.6852378845, -5.8343863487)),
    ((13.7610095743, -5.8269904203, -2.9414488565),
     (25.5653285980, -8.4376745224)),
)


def cruise_config(speed=.4, yaw=.6):
    cfg = load_config()
    cfg.update(max_speed=speed, max_yaw_rate=yaw, acceleration=.2)
    return cfg


def corridor():
    cells = np.zeros((100, 300), dtype=np.int8)
    cells[[0, -1], :] = 100
    cells[:, [0, -1]] = 100
    return GridMap(cells, .05, (-1., -2.5, 0.))


class LongRouteTests(unittest.TestCase):
    def assert_stable_arrival(self, rig, goal, result):
        self.assertEqual(result['state'], 'ARRIVED', result)
        self.assertLessEqual(math.dist(rig.pose[:2], goal), rig.cfg['goal_tolerance'])
        self.assertLessEqual(result['linear_speed'], rig.cfg['stop_speed'])
        self.assertLessEqual(result['angular_speed'], rig.cfg['stop_yaw_rate'])
        # ARRIVED must be preceded by measured settling, not just a zero command.
        settling = [s for s in rig.samples if s[-1] == 'SETTLING']
        self.assertTrue(settling)
        self.assertTrue(all(s[4:6] == (0., 0.) for s in settling))
        self.assertGreaterEqual(settling[-1][0] - settling[0][0],
                                rig.cfg['settle_seconds'] - .051)
        for _ in range(30):
            self.assertEqual(rig.step(), (0., 0.))
            self.assertLessEqual(math.dist(rig.pose[:2], goal), rig.cfg['goal_tolerance'])

    def test_logged_long_detour_arrives_faster_with_lag_at_higher_speed(self):
        start, goal = LOGGED_ROUTES[2]
        elapsed = {}
        for speed in (.2, .4):
            with self.subTest(speed=speed):
                cfg = cruise_config(speed)
                grid = GridMap.load(cfg['map'], cfg['robot_radius'], cfg['safety_margin'])
                path = grid.plan(start[:2], goal)
                self.assertGreater(sum(math.dist(a, b) for a, b in zip(path, path[1:])), 20.)
                rig = SDKDomainRig(grid, start, config=cfg, response_tau=.25)
                before = rig.t
                result = rig.reach(goal, limit=4000)
                elapsed[speed] = rig.t - before
                self.assert_stable_arrival(rig, goal, result)
                self.assertTrue(any(v >= speed * .95 for v, _, _ in rig.sdk.commands))
        # Compare the same map, planner, turn speed, acceleration and response.
        # Extra turning/settling means doubling cruise speed need not halve time.
        self.assertLess(elapsed[.4], elapsed[.2] * .8, elapsed)

    def test_other_logged_goals_arrive_and_measure_standstill_with_lag(self):
        for start, goal in LOGGED_ROUTES[:2]:
            with self.subTest(goal=goal):
                cfg = cruise_config()
                grid = GridMap.load(cfg['map'], cfg['robot_radius'], cfg['safety_margin'])
                rig = SDKDomainRig(grid, start, config=cfg, response_tau=.25)
                self.assert_stable_arrival(rig, goal, rig.reach(goal))

    def begin_corridor(self, **changes):
        cfg = cruise_config()
        cfg.update(changes)
        rig = SDKDomainRig(corridor(), config=cfg, response_tau=.25)
        self.assertTrue(rig.nav.set_goal((10., 0.), rig.t))
        self.wait_tracking(rig)
        return rig

    def wait_tracking(self, rig):
        for _ in range(160):
            if rig.nav.state == 'TRACKING':
                return
            rig.step()
            self.assertNotIn(rig.nav.state, ('STOPPED', 'WAIT_INITIAL_POSE'), rig.nav.snapshot())
        self.fail(f'Planning did not resume: {rig.nav.snapshot()}')

    def invalidate_current_segment(self, rig):
        # Fault injection represents a tracking displacement that invalidates
        # the active segment, without introducing a real wall into this route.
        with patch.object(rig.grid, 'segment_free', return_value=False):
            self.assertEqual(rig.step(), (0., 0.))

    def test_four_separated_recoveries_do_not_exhaust_lifetime_allowance(self):
        rig = self.begin_corridor()
        started = rig.nav.goal_started
        for recovery in range(1, 5):
            threshold = float(recovery)
            for _ in range(300):
                if rig.pose[0] >= threshold:
                    break
                rig.step()
                self.assertEqual(rig.nav.state, 'TRACKING', rig.nav.snapshot())
            else:
                self.fail(f'No progress to recovery {recovery}: {rig.nav.snapshot()}')
            self.invalidate_current_segment(rig)
            self.assertEqual(rig.nav.state, 'STOPPING_FOR_PLAN', rig.nav.snapshot())
            self.assertEqual(rig.nav.tracking_replans, recovery)
            self.assertLessEqual(rig.nav.tracking_replan_streak, rig.cfg['max_tracking_replans'])
            self.assertEqual(rig.nav.goal_started, started)
            self.wait_tracking(rig)
        # Continue the existing goal; set_goal() would reset the counters.
        for _ in range(1000):
            rig.step()
            if rig.nav.state != 'TRACKING' and rig.nav.state != 'SETTLING':
                break
        self.assert_stable_arrival(rig, (10., 0.), rig.nav.snapshot())
        self.assertEqual(rig.nav.tracking_replans, 4)

    def test_repeated_recoveries_without_translation_still_latch_stop(self):
        rig = self.begin_corridor()
        started = rig.nav.goal_started
        for recovery in range(rig.cfg['max_tracking_replans']):
            self.invalidate_current_segment(rig)
            self.assertEqual(rig.nav.state, 'STOPPING_FOR_PLAN', rig.nav.snapshot())
            self.assertEqual(rig.nav.tracking_replans, recovery + 1)
            self.wait_tracking(rig)
        self.invalidate_current_segment(rig)
        self.assertEqual(rig.nav.state, 'STOPPED', rig.nav.snapshot())
        self.assertIsNone(rig.nav.goal)
        self.assertEqual(rig.nav.goal_started, started)
        for _ in range(25):
            self.assertEqual(rig.step(), (0., 0.))

    def test_slow_turn_can_make_progress_before_translation(self):
        cfg = cruise_config(yaw=.1)
        rig = SDKDomainRig(start=(0., 0., math.pi), config=cfg, response_tau=.25)
        result = rig.reach((1.5, 0.))
        moving = [s for s in rig.samples if s[4] > 0.]
        self.assertTrue(moving, result)
        self.assertGreater(moving[0][0] - rig.samples[0][0], cfg['progress_timeout'])
        self.assert_stable_arrival(rig, (1.5, 0.), result)

    def test_turn_command_without_measured_rotation_still_times_out(self):
        cfg = cruise_config()
        rig = SDKDomainRig(start=(0., 0., math.pi), config=cfg, response_tau=.25)
        self.assertTrue(rig.nav.set_goal((1.5, 0.), rig.t))
        for _ in range(400):
            rig.step(move=False)
            if rig.nav.state == 'STOPPED':
                break
        self.assertEqual(rig.nav.state, 'STOPPED', rig.nav.snapshot())
        self.assertIn('progress', rig.nav.reason)
        self.assertEqual(rig.nav.command, (0., 0.))
        self.assertIsNone(rig.nav.goal)

    def test_replan_does_not_restart_absolute_goal_timeout(self):
        rig = self.begin_corridor(goal_timeout=4.)
        started = rig.nav.goal_started
        while rig.t - started < 1.8:
            rig.step()
        self.invalidate_current_segment(rig)
        self.assertEqual(rig.nav.state, 'STOPPING_FOR_PLAN')
        for _ in range(100):
            rig.step()
            if rig.nav.state == 'STOPPED':
                break
        self.assertEqual(rig.nav.state, 'STOPPED', rig.nav.snapshot())
        self.assertIn('time limit', rig.nav.reason)
        self.assertLessEqual(rig.t - started, 4.1)
        self.assertEqual(rig.nav.command, (0., 0.))

    def test_high_speed_live_obstacle_brakes_and_cannot_auto_resume(self):
        rig = self.begin_corridor()
        for _ in range(150):
            rig.step()
            if rig.actual[0] >= .39:
                break
        self.assertGreaterEqual(rig.actual[0], .39)
        rig.t += .05
        rig.feed(cloud=False)
        obstacle = rig.pose[:2] + np.array((1., 0.))
        rig.nav.update_cloud([[1., 0., .1]], 1000 + rig.t, rig.t)
        self.assertEqual(rig.nav.tick(rig.t), (0., 0.))
        self.assertEqual(rig.nav.state, 'STOPPED', rig.nav.snapshot())
        self.assertIn('Live obstacle', rig.nav.reason)
        for _ in range(50):
            self.assertEqual(rig.step(), (0., 0.))
            self.assertGreater(math.dist(rig.pose[:2], obstacle),
                               rig.cfg['robot_radius'] + rig.cfg['safety_margin'])
        self.assertLessEqual(rig.nav.linear_speed, rig.cfg['stop_speed'])
        self.assertIsNone(rig.nav.goal)


if __name__ == '__main__':
    unittest.main()
