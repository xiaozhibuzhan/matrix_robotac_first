"""Invalid annotation clicks are recoverable only while healthy and stationary."""
import tempfile
import unittest

from navigation.tests.test_treasure_session import SessionRig
from navigation.treasure_mission import MissionError


class TreasureClickRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.r = SessionRig(self.temp.name)
        self.session = self.r.session
        self.nav = self.r.rig.nav
        self.assertTrue(self.r.command('new-round')['success'])

    def select(self, center=True):
        result = self.r.command('select', box_id='box-1', radius=.2)
        self.assertTrue(result['success'], result)
        if center:
            self.session.click((1.2, 0.))

    def assert_zero(self):
        self.assertEqual(self.nav.command, (0., 0.))
        self.assertEqual(self.session.contact.command, (0., 0.))
        self.assertIsNone(self.nav.goal)

    def test_selection_step_explains_null_attempt_before_both_points(self):
        self.assertIsNone(self.session.snapshot()['selection_step'])
        self.select(center=False)
        self.assertEqual(self.session.snapshot()['selection_step'], 'WAIT_CENTER')
        self.assertIsNone(self.session.mission.current_attempt)
        self.session.click((1.2, 0.))
        self.assertEqual(self.session.snapshot()['selection_step'], 'WAIT_APPROACH')
        self.assertIsNone(self.session.mission.current_attempt)
        self.session.click((.45, 0.))
        self.assertIsNone(self.session.snapshot()['selection_step'])
        self.assertEqual(self.session.mission.current_attempt['state'], 'APPROACHING')

    def test_invalid_plane_keeps_center_then_accepts_correct_approach(self):
        self.select()
        # The ROS callback rejects z before calling session.click().
        self.assertTrue(self.session.reject_click('Clicked point is not on map plane'))
        self.assertEqual(self.session.selection['center'], [1.2, 0.])
        self.assertEqual(self.session.snapshot()['selection_step'], 'WAIT_APPROACH')
        self.assertIn('approach', self.session.last_reason)
        self.assertIsNone(self.session.mission.current_attempt)
        self.assert_zero()
        self.session.click((.45, 0.))
        self.assertEqual(self.session.mission.current_attempt['state'], 'APPROACHING')
        self.r.until(lambda: self.session.mission.current_attempt['state'] in ('READY_CONTACT', 'FAILED'))
        self.assertEqual(self.session.mission.current_attempt['state'], 'READY_CONTACT')

    def test_invalid_plane_before_center_keeps_box_selection(self):
        self.select(center=False)
        self.assertTrue(self.session.reject_click('Clicked point is not on map plane'))
        self.assertEqual(self.session.selection['box_id'], 'box-1')
        self.assertEqual(self.session.snapshot()['selection_step'], 'WAIT_CENTER')
        self.assertIn('center', self.session.last_reason)
        self.assert_zero()
        self.session.click((1.2, 0.))
        self.assertEqual(self.session.snapshot()['selection_step'], 'WAIT_APPROACH')

    def test_invalid_distance_keeps_center_and_can_be_corrected(self):
        self.select()
        with self.assertRaisesRegex(MissionError, '0.55 to 1.00 m') as caught:
            self.session.click((-.2, 0.))
        self.assertTrue(self.session.reject_click(str(caught.exception)))
        self.assertEqual(self.session.selection['center'], [1.2, 0.])
        self.assert_zero()
        self.session.click((.45, 0.))
        self.assertEqual(self.nav.goal, (.45, 0.))

    def test_sensor_loss_cancels_selection_instead_of_preserving_it(self):
        for sensor in ('cloud_received', 'odom_received'):
            with self.subTest(sensor=sensor):
                self.r.feed()
                self.select()
                old_received = getattr(self.nav, sensor)
                setattr(self.nav, sensor, self.r.rig.t-10.)
                self.assertFalse(self.session.reject_click('Invalid point'))
                self.assertIsNone(self.session.selection)
                self.assertIsNone(self.session.snapshot()['selection_step'])
                self.assert_zero()
                setattr(self.nav, sensor, old_received)

    def test_measured_motion_or_nonzero_command_cancels_selection(self):
        for field, value in [('linear_speed', .1), ('angular_speed', .2), ('command', (.05, 0.))]:
            with self.subTest(field=field):
                self.r.feed()
                self.select()
                previous = getattr(self.nav, field)
                setattr(self.nav, field, value)
                self.assertFalse(self.session.reject_click('Invalid point'))
                self.assertIsNone(self.session.selection)
                self.assert_zero()
                setattr(self.nav, field, previous)

    def test_extra_click_during_motion_still_cancels_active_target(self):
        self.select()
        self.session.click((.45, 0.))
        self.r.until(lambda: self.nav.linear_speed > .04)
        with self.assertRaises(MissionError) as caught:
            self.session.click((.6, 0.))
        self.assertFalse(self.session.reject_click(str(caught.exception)))
        self.assertIsNone(self.session.selection)
        self.assertEqual(self.session.mission.current_attempt['state'], 'CANCELLED')
        self.assert_zero()
        self.assertEqual(self.session.tick(), (0., 0.))

    def test_active_goal_prevents_recovery_even_before_motion(self):
        self.select()
        self.assertTrue(self.nav.set_goal((.45, 0.), self.r.rig.t))
        self.assertFalse(self.session.reject_click('Invalid point'))
        self.assertIsNone(self.session.selection)
        self.assert_zero()

    def test_expired_round_cancels_selection(self):
        self.select()
        self.session.mission.base_elapsed = 1800.
        self.assertFalse(self.session.reject_click('Invalid point'))
        self.assertEqual(self.session.mission.snapshot()['state'], 'EXPIRED')
        self.assertIsNone(self.session.selection)
        self.assert_zero()

    def test_ready_contact_attempt_cannot_preserve_selection(self):
        self.r.approach('box-1', (1.2, 0.), (.45, 0.))
        # Even an inconsistent lingering selection must not bypass an active attempt.
        self.session.selection = dict(box_id='box-2', radius=.2, center=None)
        self.assertFalse(self.session.reject_click('Invalid point'))
        self.assertEqual(self.session.mission.current_attempt['state'], 'CANCELLED')
        self.assertIsNone(self.session.selection)
        self.assert_zero()

    def test_pending_confirmation_or_contact_blocks_recovery(self):
        for stage in ('confirmation', 'contact'):
            with self.subTest(stage=stage):
                self.select()
                if stage == 'confirmation':
                    self.session.pending_confirmation = True
                else:
                    self.session.contact.state = 'APPROACHING'
                    self.assertTrue(self.session.contact.active)
                self.assertFalse(self.session.reject_click('Invalid point'))
                self.assertIsNone(self.session.selection)
                self.assertFalse(self.session.pending_confirmation)
                self.assertFalse(self.session.contact.active)
                self.assert_zero()

    def test_final_attempt_allows_next_selection_to_be_corrected(self):
        self.select()
        self.session.click((.45, 0.))
        self.session.interrupt('Operator cancelled previous box')
        self.select()
        self.assertTrue(self.session.reject_click('Invalid point'))
        self.assertEqual(self.session.selection['center'], [1.2, 0.])
        self.assertEqual(self.session.mission.current_attempt['state'], 'CANCELLED')
        self.assert_zero()

    def test_relocalization_interrupt_still_clears_selection_and_goal(self):
        for active_goal in (False, True):
            with self.subTest(active_goal=active_goal):
                self.select()
                if active_goal:
                    self.session.click((.45, 0.))
                # The initial-pose callback always interrupts before initializing.
                self.session.interrupt('Initial pose requested')
                self.assertTrue(self.nav.initialize((0., 0., 0.), self.r.rig.t))
                self.assertIsNone(self.session.selection)
                self.assertIsNone(self.session.snapshot()['selection_step'])
                self.assert_zero()
                if active_goal:
                    self.assertEqual(self.session.mission.current_attempt['state'], 'CANCELLED')

    def test_lost_localization_refuses_recovery(self):
        self.select()
        self.nav.fail('Odometry reset', lose_localization=True)
        self.assertFalse(self.session.reject_click('Invalid point'))
        self.assertIsNone(self.session.selection)
        self.assertEqual(self.nav.state, 'WAIT_INITIAL_POSE')
        self.assert_zero()


if __name__ == '__main__':
    unittest.main()
