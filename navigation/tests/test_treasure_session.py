import math
import tempfile
import unittest
import numpy as np
from navigation.grid import GridMap
from navigation.treasure_contact import cleared_target_grid
from navigation.treasure_session import TreasureSession
from navigation.tests.test_navigation import Rig


class SessionRig:
    def __init__(self, folder, mode='drive'):
        cells = np.zeros((160, 160), dtype=np.int8)
        self.centers = [(1.2, 0.), (2.5, 1.5)]
        for x, y in self.centers:
            cx, cy = int((x+4)/.05), int((y+4)/.05)
            cells[cy-1:cy+1, cx-1:cx+1] = 100
        self.rig = Rig(grid=GridMap(cells, .05, (-4., -4., 0.)), response_tau=.15)
        self.session = TreasureSession(self.rig.nav, folder, map_hash='fixture',
            mode=mode, monotonic=lambda: self.rig.t, wall_clock=lambda: 10000+self.rig.t)
        self.counter = 0
        self.feed()

    def feed(self):
        self.rig.feed()
        self.rig.nav.obstacles = np.array(self.centers, dtype=float).reshape((-1, 2))

    def command(self, action, **kwargs):
        self.counter += 1
        return self.session.handle(dict(request_id=str(self.counter), action=action, **kwargs))

    def step(self):
        self.rig.t += .05
        self.feed()
        v, w = self.session.tick()
        self.rig.actual += (1.-math.exp(-.05/.15))*(np.array([v,w])-self.rig.actual)
        dx, dy = self.rig.nav.arc(*self.rig.actual, .05)
        c, s = math.cos(self.rig.pose[2]), math.sin(self.rig.pose[2])
        self.rig.pose[:2] += [c*dx-s*dy, s*dx+c*dy]
        self.rig.pose[2] += self.rig.actual[1]*.05
        return v, w

    def until(self, predicate, limit=3000):
        for _ in range(limit):
            self.step()
            if predicate():
                return
        raise AssertionError(self.session.snapshot())

    def approach(self, box_id, center, approach):
        result = self.command('select', box_id=box_id, radius=.2)
        assert result['success'], result
        self.session.click(center)
        self.session.click(approach)
        self.until(lambda: self.session.mission.current_attempt['state'] in ('READY_CONTACT','FAILED'))
        assert self.session.mission.current_attempt['state'] == 'READY_CONTACT', self.session.snapshot()


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.r = SessionRig(self.temp.name)
        self.assertTrue(self.r.command('new-round')['success'])

    def test_two_boxes_contact_confirm_and_next_route_without_mutating_source(self):
        original = self.r.rig.grid.occupancy.copy()
        for index, (center, approach) in enumerate([((1.2,0.),(.45,0.)), ((2.5,1.5),(1.75,1.5))]):
            self.r.approach('box'+str(index), center, approach)
            self.assertEqual(self.r.session.mission.snapshot()['confirmed_count'], index)
            response = self.r.command('contact')
            self.assertTrue(response['success'], response)
            self.r.until(lambda: self.r.session.mission.current_attempt['state'] in ('WAIT_CONFIRM','FAILED'))
            self.assertEqual(self.r.session.mission.current_attempt['state'], 'WAIT_CONFIRM', self.r.session.snapshot())
            self.r.centers.remove(center)
            self.assertTrue(self.r.command('confirm', disappeared=True)['success'])
            self.assertEqual(self.r.session.mission.snapshot()['confirmed_count'], index)
            self.r.until(lambda: self.r.session.mission.current_attempt['state']=='CONFIRMED')
            self.assertEqual(self.r.session.mission.snapshot()['confirmed_count'], index+1)
            self.assertTrue(self.r.rig.nav.grid.is_free(self.r.rig.nav.grid.cell(self.r.rig.pose[:2])))
        np.testing.assert_array_equal(self.r.rig.grid.occupancy, original)
        self.assertEqual(self.r.rig.nav.grid.occupancy.sum(), 0)

    def test_duplicate_transport_request_is_idempotent(self):
        req = dict(request_id='same', action='select', box_id='box1', radius=.2)
        first = self.r.session.handle(req)
        second = self.r.session.handle(req)
        self.assertEqual(first, second)
        altered = self.r.session.handle({**req, 'radius':.25})
        self.assertFalse(altered['success'])
        self.assertEqual(self.r.session.selection['radius'], .2)

    def test_contact_is_explicit_and_confirm_is_not_automatic(self):
        self.r.approach('box1', (1.2,0.), (.45,0.))
        for _ in range(30):
            self.assertEqual(self.r.step(), (0.,0.))
        self.assertEqual(self.r.session.mission.current_attempt['state'], 'READY_CONTACT')
        self.assertFalse(self.r.command('confirm')['success'])
        self.assertEqual(self.r.session.mission.snapshot()['confirmed_count'], 0)

    def test_observation_during_motion_stops_then_confirms_after_measured_settling(self):
        self.r.approach('box1', (1.2,0.), (.45,0.))
        self.assertTrue(self.r.command('contact')['success'])
        self.r.until(lambda: self.r.rig.nav.linear_speed>.04)
        self.r.centers.remove((1.2,0.))
        self.assertTrue(self.r.command('confirm', disappeared=True)['success'])
        self.assertEqual(self.r.session.mission.snapshot()['confirmed_count'], 0)
        self.r.until(lambda: self.r.session.mission.current_attempt['state']=='CONFIRMED')
        self.assertTrue(self.r.rig.nav.is_stopped())

    def test_round_expiry_cancels_contact_and_does_not_resume(self):
        self.r.approach('box1', (1.2,0.), (.45,0.))
        self.r.command('contact')
        self.r.session.mission.base_elapsed = 1800.
        self.assertEqual(self.r.step(), (0.,0.))
        self.assertEqual(self.r.session.mission.snapshot()['state'], 'EXPIRED')
        for _ in range(5):
            self.assertEqual(self.r.step(), (0.,0.))

    def test_reset_restores_original_boxes_and_cancels_old_motion(self):
        self.r.approach('box1', (1.2,0.), (.45,0.))
        self.r.centers.remove((1.2,0.))
        self.r.command('confirm', disappeared=True)
        self.r.until(lambda: self.r.session.mission.current_attempt['state']=='CONFIRMED')
        self.r.command('finish')
        self.assertTrue(self.r.command('new-round')['success'])
        self.assertIs(self.r.rig.nav.grid, self.r.session.base_grid)
        self.assertEqual(self.r.session.mission.snapshot()['confirmed_count'], 0)

    def test_sensor_loss_cancels_pending_confirmation(self):
        self.r.approach('box1', (1.2,0.), (.45,0.))
        self.r.command('confirm', disappeared=True)
        self.r.rig.nav.cloud_received = self.r.rig.t-10
        self.assertEqual(self.r.session.tick(), (0.,0.))
        self.assertFalse(self.r.session.pending_confirmation)
        self.assertEqual(self.r.session.mission.snapshot()['confirmed_count'], 0)

    def test_clearing_one_confirmed_box_keeps_other_obstacles_and_unknown(self):
        grid = self.r.rig.grid
        changed = cleared_target_grid(grid, (1.2,0.), .2)
        self.assertLess(changed.occupancy.sum(), grid.occupancy.sum())
        self.assertGreater(changed.occupancy.sum(), 0)
        cells = grid.occupancy.copy()
        cells[80,105:130] = 100
        wall = GridMap(cells, grid.resolution, grid.origin)
        with self.assertRaises(ValueError):
            cleared_target_grid(wall, (1.2,0.), .2)
