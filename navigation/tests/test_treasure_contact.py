"""Behavioral bounds and collision isolation for explicit treasure contact."""
import math
import unittest
import numpy as np
from navigation.configuration import load_config
from navigation.core import Navigator
from navigation.grid import GridMap
from navigation.sdk_limits import validate_sdk_velocity
from navigation.treasure_contact import TreasureContactController


class ContactRig:
    def __init__(self, yaw=0., settings=None, occupied=True, rotation=0.):
        cells = np.zeros((120, 120), dtype=np.int8)
        if occupied:
            cells[59:61, 74:76] = 100
        self.grid = GridMap(cells, .05, (-3., -3., 0.))
        self.nav = Navigator(self.grid, load_config())
        self.pose = np.array([0., 0., yaw])
        self.now = 10.
        self.nav.alignment = (0., 0., rotation)
        self.nav.state = 'ARRIVED'
        self.contact = TreasureContactController(self.nav, settings)
        self.live = np.array([[.75, 0.]])
        self.feed()

    def feed(self):
        x, y, yaw = self.pose
        a = self.nav.alignment[2] if self.nav.alignment else 0.
        c, s = math.cos(a), math.sin(a)
        raw = np.array([c*x+s*y, -s*x+c*y, 0.])
        self.nav.pose = tuple(self.pose)
        self.nav.odom = (raw, np.array([0., 0., math.sin((yaw-a)/2), math.cos((yaw-a)/2)]))
        self.nav.odom_received = self.nav.cloud_received = self.now
        self.nav.linear_speed = self.nav.angular_speed = 0.
        self.nav.obstacles = self.live @ np.array([[c, -s], [s, c]])

    def start(self):
        return self.contact.start((.75, 0.), .20, self.now)

    def step(self, move=True):
        self.now += .05
        self.feed()
        v, w = self.contact.tick(self.now)
        validate_sdk_velocity(v, w)
        if move:
            dx, dy = self.nav.arc(v, w, .05)
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            self.pose[:2] += [c*dx-s*dy, s*dx+c*dy]
            self.pose[2] += w*.05
        return v, w


class TreasureContactTests(unittest.TestCase):
    def test_isolated_target_reaches_stop_and_requires_confirmation(self):
        rig = ContactRig()
        original = rig.grid.occupancy.copy()
        self.assertTrue(rig.start(), rig.contact.reason)
        samples = []
        for _ in range(600):
            samples.append(rig.step())
            if not rig.contact.active:
                break
        self.assertEqual(rig.contact.state, 'WAIT_CONFIRM', rig.contact.snapshot())
        self.assertLessEqual(math.dist(rig.pose[:2], (.75, 0.)), .251)
        self.assertTrue(any(v > 0 for v, w in samples))
        self.assertTrue(all(v == 0 or .05 <= v <= .08 for v, w in samples))
        self.assertEqual(rig.contact.command, (0., 0.))
        np.testing.assert_array_equal(rig.grid.occupancy, original)
        self.assertIs(rig.nav.grid, rig.grid)
        self.assertNotIn('success', rig.contact.snapshot())

    def test_turns_before_forward_motion(self):
        rig = ContactRig(yaw=.6)
        self.assertTrue(rig.start())
        for _ in range(100):
            v, w = rig.step()
            if v:
                self.assertLess(abs(rig.pose[2]), .04)
                break
        else:
            self.fail(rig.contact.snapshot())

    def test_map_and_live_obstacles_use_map_to_odom_transform(self):
        rig = ContactRig(rotation=math.pi/2)
        self.assertTrue(rig.start(), rig.contact.reason)
        rig.step()
        self.assertGreater(rig.step()[0], 0.)

    def test_unknown_target_and_target_touching_wall_are_rejected(self):
        for kind in ('unknown', 'wall', 'ring'):
            rig = ContactRig()
            if kind == 'unknown':
                rig.grid.occupancy[60, 75] = -1
            elif kind == 'wall':
                rig.grid.occupancy[60, 75:100] = 100
            else:
                rig.grid.occupancy[60, 81] = 100
            self.assertFalse(rig.start(), kind)
            self.assertEqual(rig.contact.command, (0., 0.))

    def test_unrelated_live_obstacle_stops_contact(self):
        rig = ContactRig()
        self.assertTrue(rig.start())
        rig.step()
        rig.live = np.array([[.75, 0.], [.35, .15]])
        self.assertEqual(rig.step(), (0., 0.))
        self.assertEqual(rig.contact.state, 'STOPPED')
        self.assertIn('Live obstacle', rig.contact.reason)

    def test_live_target_connected_across_roi_refused(self):
        rig = ContactRig()
        rig.live = np.array([[.93, 0.], [.99, 0.]])
        rig.feed()
        self.assertFalse(rig.start())
        self.assertIn('join', rig.contact.reason)

    def test_nonblocking_overlap_trigger_can_be_approached_without_false_success(self):
        rig = ContactRig(occupied=False)
        rig.live = np.empty((0, 2)); rig.feed()
        self.assertTrue(rig.start(), rig.contact.reason)
        rig.step()
        self.assertGreater(rig.step()[0], 0.)
        self.assertNotIn('success', rig.contact.snapshot())
        rig.live = np.array([[.35,.15]])
        self.assertEqual(rig.step(), (0.,0.))
        self.assertEqual(rig.contact.state, 'STOPPED')

    def test_moving_or_not_arrived_refused(self):
        for kind in ('moving', 'not_arrived', 'command'):
            rig = ContactRig()
            if kind == 'moving':
                rig.nav.linear_speed = .06
            elif kind == 'not_arrived':
                rig.nav.state = 'TRACKING'
            else:
                rig.nav.command = (.05, 0.)
            self.assertFalse(rig.start(), kind)

    def test_cancellation_is_latched(self):
        rig = ContactRig(); self.assertTrue(rig.start()); rig.step()
        rig.contact.cancel('Operator cancel')
        for _ in range(10):
            self.assertEqual(rig.step(), (0., 0.))
        self.assertEqual(rig.contact.state, 'STOPPED')

    def test_sensor_stale_and_localization_change_stop(self):
        for kind in ('cloud', 'odom', 'alignment', 'goal', 'map'):
            rig = ContactRig(); self.assertTrue(rig.start())
            rig.now += .05; rig.feed()
            if kind == 'cloud':
                rig.nav.cloud_received = rig.now - 1.
            elif kind == 'odom':
                rig.nav.odom_received = rig.now - 1.
            elif kind == 'alignment':
                rig.nav.alignment = (.1, 0., 0.)
            elif kind == 'goal':
                rig.nav.goal = (2., 0.)
            else:
                rig.nav.grid = GridMap(rig.grid.occupancy.copy(), .05, (-3., -3., 0.))
            self.assertEqual(rig.contact.tick(rig.now), (0., 0.), kind)
            self.assertEqual(rig.contact.state, 'STOPPED', kind)

    def test_odometry_jump_and_loop_stall_stop(self):
        for kind in ('jump', 'stall'):
            rig = ContactRig(); self.assertTrue(rig.start())
            if kind == 'jump':
                rig.pose[0] += .2
            else:
                rig.now += .4
            self.assertEqual(rig.step(), (0., 0.))
            self.assertEqual(rig.contact.state, 'STOPPED')

    def test_no_progress_and_time_limits(self):
        for settings, expected in [({}, 'progress'), ({'timeout': .5}, 'time limit')]:
            rig = ContactRig(settings=settings); self.assertTrue(rig.start())
            for _ in range(130):
                rig.step(move=False)
                if not rig.contact.active:
                    break
            self.assertEqual(rig.contact.state, 'STOPPED')
            self.assertIn(expected, rig.contact.reason)

    def test_total_travel_limit_stops_before_any_confirmation(self):
        rig = ContactRig()
        self.assertTrue(rig.start())
        rig.contact.travelled = .801
        self.assertEqual(rig.step(), (0., 0.))
        self.assertIn('travel limit', rig.contact.reason)

    def test_endpoint_does_not_confirm_while_measured_moving(self):
        rig = ContactRig(); self.assertTrue(rig.start())
        # Reach endpoint through individually valid odometry steps.
        for _ in range(25):
            rig.pose[0] += .02
            rig.now += .15; rig.feed()
            rig.nav.linear_speed = .08
            rig.contact.tick(rig.now)
        self.assertEqual(rig.contact.state, 'SETTLING', rig.contact.snapshot())
        for _ in range(8):
            rig.now += .1; rig.feed(); rig.nav.linear_speed = .06
            self.assertEqual(rig.contact.tick(rig.now), (0., 0.))
        self.assertEqual(rig.contact.state, 'SETTLING')
        for _ in range(20):
            rig.step(move=False)
        self.assertEqual(rig.contact.state, 'WAIT_CONFIRM')

    def test_setting_caps_and_far_target_refused(self):
        for settings in ({'speed': .04}, {'speed': .09}, {'max_travel': 1.}, {'timeout': 31.},
                         {'max_step': .1}, {'isolation_gap': .01}):
            with self.assertRaises(ValueError):
                ContactRig(settings=settings)
        rig = ContactRig()
        self.assertFalse(rig.contact.start((1.5, 0.), .2, rig.now))
        self.assertFalse(rig.contact.start((.75, 0.), .31, rig.now))


if __name__ == '__main__':
    unittest.main()

