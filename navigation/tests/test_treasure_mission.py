import json
import tempfile
import unittest
from unittest.mock import patch
from navigation.treasure_mission import TreasureMission, MissionError


class Clock:
    def __init__(self):
        self.now = 100.
        self.wall = 10000.

    def advance(self, seconds):
        self.now += seconds
        self.wall += seconds


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.clock = Clock()
        self.mission = self.make()
        self.mission.new_round('round_test')

    def make(self, **kwargs):
        return TreasureMission(self.temp.name, map_hash='test_hash', map_frame='world',
                               monotonic=lambda: self.clock.now, wall_clock=lambda: self.clock.wall, **kwargs)

    def ready(self, name='box1', x=1.):
        self.mission.select(name, (x, 0.), (x-.75, 0.), .2)
        self.mission.begin_approach()
        self.mission.approach_arrived()

    def confirmed(self, name='box1', x=1.):
        self.ready(name, x)
        self.mission.contact_stopped()
        return self.mission.confirm(disappeared=True, stopped=True)

    def test_arrival_never_counts_and_confirm_needs_explicit_stopped_observation(self):
        self.ready()
        self.assertEqual(self.mission.snapshot()['confirmed_count'], 0)
        with self.assertRaises(MissionError):
            self.mission.confirm(disappeared=True, stopped=True)
        self.mission.contact_stopped()
        for kwargs in ({}, {'disappeared': True}, {'stopped': True}):
            with self.assertRaises(MissionError):
                self.mission.confirm(**kwargs)
        self.assertEqual(self.mission.confirm(disappeared=True, stopped=True)['confirmed_count'], 1)

    def test_duplicate_id_and_overlapping_target_cannot_count_twice(self):
        self.confirmed()
        for box, center in [('box1', (5., 0.)), ('new-id', (1.2, 0.))]:
            with self.assertRaises(MissionError):
                self.mission.select(box, center, (0., 0.), .2)
        with self.assertRaises(MissionError):
            self.mission.confirm(disappeared=True, stopped=True)
        self.assertEqual(self.mission.snapshot()['ammo_estimate'], 1)

    def test_failure_retry_keeps_history_without_awarding(self):
        self.ready()
        self.mission.begin_contact()
        self.mission.fail('sensor stale')
        self.mission.retry()
        state = self.mission.snapshot()
        self.assertEqual(len(state['attempts']), 2)
        self.assertEqual(state['confirmed_count'], 0)
        self.assertEqual(state['current_attempt']['state'], 'SELECTED')

    def test_remaining_minutes_floor_and_score_caps(self):
        for index in range(6):
            self.confirmed('box'+str(index), x=1.+index)
        self.clock.advance(1020.)
        state = self.mission.snapshot()
        self.assertEqual(state['score_estimate'], {'box': 20, 'time': 10, 'total': 30})
        self.assertEqual(state['ammo_estimate'], 6)
        self.clock.advance(.01)
        self.assertEqual(self.mission.snapshot()['score_estimate']['time'], 9.6)

    def test_finish_freezes_elapsed_and_disallows_motion_actions(self):
        self.ready()
        self.clock.advance(100.)
        self.mission.finish()
        self.clock.advance(200.)
        self.assertEqual(self.mission.snapshot()['elapsed_seconds'], 100.)
        with self.assertRaises(MissionError):
            self.mission.begin_contact()

    def test_expiry_cancels_unfinished_attempt(self):
        self.ready()
        self.clock.advance(1800.1)
        state = self.mission.tick()
        self.assertEqual(state['state'], 'EXPIRED')
        self.assertEqual(state['current_attempt']['reason'], 'round_expired')
        with self.assertRaises(MissionError):
            self.mission.confirm(disappeared=True, stopped=True)

    def test_restart_accounts_for_downtime_and_never_restores_motion(self):
        self.confirmed()
        self.ready('box2', 3.)
        self.clock.advance(120.)
        resumed = self.make()
        state = resumed.resume('round_test')
        self.assertEqual(state['confirmed_count'], 1)
        self.assertEqual(state['current_attempt']['state'], 'CANCELLED')
        self.assertAlmostEqual(state['elapsed_seconds'], 120.)
        resumed.retry()
        self.assertEqual(resumed.current_attempt['state'], 'SELECTED')

    def test_resume_rejects_clock_rollback_and_map_changes(self):
        self.clock.wall -= 1
        with self.assertRaises(MissionError):
            self.make().resume('round_test')
        self.clock.wall += 1
        other = TreasureMission(self.temp.name, map_hash='different', monotonic=lambda: self.clock.now,
                                wall_clock=lambda: self.clock.wall)
        with self.assertRaises(MissionError):
            other.resume('round_test')

    def test_preview_cannot_claim_official_success(self):
        mission = self.make(mode='preview')
        mission.new_round('preview_round')
        mission.select('box1', (1., 0.), (0., 0.), .2)
        mission.begin_approach(); mission.approach_arrived(); mission.contact_stopped()
        with self.assertRaises(MissionError):
            mission.confirm(disappeared=True, stopped=True)

    def test_bad_saved_record_and_path_traversal_rejected(self):
        with self.assertRaises(MissionError):
            self.make().resume('../round_test')
        from pathlib import Path
        path = Path(self.temp.name)/'round_test.json'
        data = json.loads(path.read_text())
        data['current_index'] = 99
        path.write_text(json.dumps(data))
        with self.assertRaises(MissionError):
            self.make().resume('round_test')

    def test_atomic_write_failure_does_not_count_box_in_memory(self):
        self.ready(); self.mission.contact_stopped()
        with patch('navigation.treasure_mission.os.replace', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                self.mission.confirm(disappeared=True, stopped=True)
        self.assertEqual(self.mission.snapshot()['confirmed_count'], 0)
        self.assertEqual(self.mission.current_attempt['state'], 'WAIT_CONFIRM')
