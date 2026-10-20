"""Behavioral regressions for aiming geometry and the independent fire gates."""
from dataclasses import replace
import unittest

import numpy as np

from task3_sim.controller import AutoAimController
from task3_sim.geometry import estimate_aim, ray_plane_intersection
from task3_sim.interfaces import Frame, RobotFeedback, RobotMotionCommand
from task3_sim.perception import TagDetection


CAMERA = {"rotation_base_from_camera": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
          "position_base_m": [0, 0, 0]}
LASER = {"origin_base_m": [0, 0, 0], "direction_base": [1, 0, 0]}
TAG = {"id": 0, "bullseye_offset_tag_m": [0, 0.2, 0]}


def detection(now=0.0, **overrides):
    result = TagDetection(0, np.eye(3), np.array([0, -0.2, 3.0]), 100.0, 0,
                          np.zeros((4, 2)), 0.1, now)
    return replace(result, **overrides)


def feedback(now=0.0, **overrides):
    result = RobotFeedback(True, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), now)
    return replace(result, **overrides)


def frame(now=0.0):
    return Frame(np.zeros((8, 8, 3), dtype=np.uint8), now)


class GeometryTests(unittest.TestCase):
    def test_tag_above_bullseye_is_not_an_aim_at_tag_center(self):
        measured = detection(t_camera_from_tag=np.array([0, 0, 3.0]))
        geometry = estimate_aim(measured, CAMERA, LASER, TAG)
        np.testing.assert_allclose(geometry.bullseye_base_m, [3, 0, -0.2])
        self.assertAlmostEqual(geometry.error_norm_m, 0.2)
        self.assertGreater(geometry.pitch_error_rad, 0)
        self.assertAlmostEqual(geometry.yaw_error_rad, 0)

    def test_lateral_visual_error_commands_left_turn(self):
        geometry = estimate_aim(detection(t_camera_from_tag=np.array([-0.3, -0.2, 3.])), CAMERA, LASER, TAG)
        self.assertGreater(geometry.yaw_error_rad, 0)
        self.assertAlmostEqual(geometry.error_norm_m, 0.3)

    def test_plane_intersection_cannot_hit_behind_origin_or_parallel(self):
        self.assertIsNone(ray_plane_intersection([0, 0, 0], [1, 0, 0], [-1, 0, 0], [1, 0, 0]))
        self.assertIsNone(ray_plane_intersection([0, 0, 0], [1, 0, 0], [0, 0, 1], [0, 0, 1]))
        point, distance = ray_plane_intersection([0, 0, 0], [2, 0, 0], [3, 0, 0], [1, 0, 0])
        np.testing.assert_allclose(point, [3, 0, 0])
        self.assertEqual(distance, 3)


class FireSafetyTests(unittest.TestCase):
    def setUp(self):
        self.controller = AutoAimController({"camera": CAMERA, "laser": LASER, "tag": TAG,
                                             "control": {"stable_duration_s": 0.3}})

    def update(self, now, image=None, dets=None, state=None):
        return self.controller.update(image or frame(now), [detection(now)] if dets is None else dets,
                                      state or feedback(now), now)

    def assert_stopped_without_fire(self, result):
        self.assertFalse(result.should_fire)
        self.assertEqual(result.command, RobotMotionCommand())

    def test_firing_requires_fresh_alignment_for_full_dwell(self):
        self.assertFalse(self.update(0).should_fire)
        self.assertFalse(self.update(0.29).should_fire)
        self.assertTrue(self.update(0.31).should_fire)
        self.controller.mark_fired()
        self.assert_stopped_without_fire(self.update(0.7))

    def test_stale_image_resets_dwell_and_stops(self):
        self.update(0)
        self.assert_stopped_without_fire(self.update(0.31, image=frame(0)))
        self.assertFalse(self.update(0.4).should_fire)
        self.assertFalse(self.update(0.6).should_fire)
        self.assertTrue(self.update(0.71).should_fire)

    def test_repeated_and_out_of_order_images_cannot_extend_stable_dwell(self):
        self.assertFalse(self.update(0).should_fire)  # timestamp zero is valid
        repeated = self.update(0.05, image=frame(0), dets=[detection(0)])
        self.assert_stopped_without_fire(repeated)
        self.assertIn("repeated", repeated.reason)
        self.assertFalse(self.update(0.1).should_fire)
        older = self.update(0.15, image=frame(0.05), dets=[detection(0.05)])
        self.assert_stopped_without_fire(older)
        self.assertIn("out of order", older.reason)
        self.assertFalse(self.update(0.2).should_fire)
        self.assertFalse(self.update(0.4).should_fire)
        self.assertTrue(self.update(0.51).should_fire)

    def test_motion_feedback_and_bad_observations_inhibit_fire(self):
        self.update(0)
        cases = (
            {"state": feedback(0.31, gyro_rad_s=np.array([0, 0, 0.05]))},
            {"state": feedback(0.31, velocity_m_s=np.array([0.1, 0, 0]))},
            {"state": feedback(0.31, valid=False)},
            {"state": feedback(0.0)},
            {"state": feedback(0.31, gyro_rad_s=np.array([np.nan, 0, 0]))},
            {"state": feedback(0.31, position_m=np.array([0, np.nan, 0]))},
            {"dets": []},
            {"dets": [detection(0.31, tag_id=2)]},
            {"dets": [detection(0.31, hamming=1)]},
            {"dets": [detection(0.31, decision_margin=10)]},
            {"dets": [detection(0.31, reprojection_error_px=10)]},
            {"dets": [detection(0.1)]},
            {"image": frame(0.32)},
        )
        for case in cases:
            with self.subTest(case=case):
                # Each cause is tested against a fresh otherwise-fire-ready
                # controller, so a preceding failure cannot mask the next one.
                self.setUp()
                self.update(0)
                self.assert_stopped_without_fire(self.update(0.31, **case))


if __name__ == "__main__":
    unittest.main()
