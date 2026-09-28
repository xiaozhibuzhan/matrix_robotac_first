"""Interpolation, timing and motion-gating checks without ROS dependencies."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import FrozenInstanceError
import math
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_buffer import PoseBuffer


def yaw(degrees):
    return Rotation.from_euler("z", degrees, degrees=True).as_quat()


class PoseBufferTests(unittest.TestCase):
    def test_midpoint_interpolates_translation_and_normalizes_quaternion(self):
        buffer = PoseBuffer()
        buffer.append(1.0, (1, 2, 3), (0, 0, 0, 2))
        buffer.append(1.1, (3, 4, 5), (0, 0, 0, 3))
        match, reason = buffer.lookup(1.05, window_before=0, window_after=0)
        self.assertEqual(reason, "ok")
        np.testing.assert_allclose(match.position, (2, 3, 4))
        np.testing.assert_allclose(match.quaternion, (0, 0, 0, 1))
        self.assertAlmostEqual(match.span, 0.1)
        self.assertAlmostEqual(match.linear_speed, math.sqrt(12) / 0.1)
        self.assertAlmostEqual(match.angular_speed, 0)

    def test_yaw_wrap_uses_shortest_rotation(self):
        buffer = PoseBuffer()
        buffer.append(1.0, (0, 0, 0), yaw(179))
        buffer.append(1.1, (0, 0, 0), yaw(-179))
        match, reason = buffer.lookup(1.05, window_before=0, window_after=0)
        self.assertEqual(reason, "ok")
        angle = Rotation.from_quat(match.quaternion).as_euler("xyz", degrees=True)[2]
        self.assertAlmostEqual(abs(angle), 180)
        self.assertAlmostEqual(match.angular_speed, math.radians(2) / 0.1)

    def test_missing_future_sample_unblocks_after_append(self):
        buffer = PoseBuffer()
        for value in range(8):
            buffer.append(1.0 + value * 0.05, (0, 0, 0), yaw(0))
        self.assertEqual(buffer.lookup(1.35)[1], "need_future")
        buffer.append(1.4, (0, 0, 0), yaw(0))
        match, reason = buffer.lookup(1.35)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(match.window_linear_speed, 0)
        self.assertAlmostEqual(match.window_angular_speed, 0)

    def test_full_past_window_is_required(self):
        buffer = PoseBuffer()
        buffer.append(1.0, (0, 0, 0), yaw(0))
        buffer.append(1.1, (0, 0, 0), yaw(0))
        self.assertEqual(buffer.lookup(1.05)[1], "need_past")

    def test_window_gap_rejects_even_when_requested_pose_is_exact(self):
        buffer = PoseBuffer()
        for timestamp in (1.0, 1.05, 1.10, 1.30, 1.35, 1.40):
            buffer.append(timestamp, (0, 0, 0), yaw(0))
        self.assertEqual(buffer.lookup(1.35)[1], "gap")

    def test_bracket_gap_not_nearest_distance_controls_acceptance(self):
        buffer = PoseBuffer()
        buffer.append(1.0, (0, 0, 0), yaw(0))
        buffer.append(1.15, (0, 0, 0), yaw(0))
        self.assertEqual(buffer.lookup(1.075, window_before=0, window_after=0)[1], "gap")

    def test_roundoff_at_window_start_does_not_include_a_previous_gap(self):
        buffer = PoseBuffer()
        for timestamp in (0.07, 0.22, 0.27, 0.32, 0.37, 0.42, 0.47, 0.52, 0.57, 0.62):
            buffer.append(timestamp, (0, 0, 0), yaw(0))
        self.assertLess(0.57 - 0.35, 0.22)
        self.assertEqual(buffer.lookup(0.57)[1], "ok")

    def test_exact_pose_can_use_valid_right_bracket_when_left_is_outside_window(self):
        buffer = PoseBuffer()
        for timestamp in (1.0, 1.2, 1.25):
            buffer.append(timestamp, (timestamp, 0, 0), yaw(0))
        match, reason = buffer.lookup(1.2, window_before=0, window_after=0.05)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(match.span, 0.05)
        self.assertAlmostEqual(match.linear_speed, 1.0)

    def test_zero_window_exact_pose_uses_adjacent_motion_and_still_rejects_large_gaps(self):
        buffer = PoseBuffer()
        for timestamp in (1.0, 1.2, 1.25):
            buffer.append(timestamp, (timestamp, 0, 0), yaw(0))
        match, reason = buffer.lookup(1.2, window_before=0, window_after=0)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(match.window_linear_speed, 1.0)
        self.assertEqual(buffer.lookup(1.0, window_before=0, window_after=0)[1], "gap")

    def test_exact_timestamp_still_needs_two_samples(self):
        buffer = PoseBuffer()
        buffer.append(1.0, (0, 0, 0), yaw(0))
        self.assertEqual(buffer.lookup(1.0, window_before=0, window_after=0)[1], "insufficient_samples")
        buffer.append(1.1, (0.1, 0, 0), yaw(0))
        for timestamp in (1.0, 1.1):
            match, reason = buffer.lookup(timestamp, window_before=0, window_after=0)
            self.assertEqual(reason, "ok")
            self.assertAlmostEqual(match.linear_speed, 1.0)

    def test_recent_rotation_and_after_scan_rotation_are_in_motion_maximum(self):
        for turning_interval in (1, 7):
            with self.subTest(turning_interval=turning_interval):
                buffer = PoseBuffer()
                for index in range(9):
                    buffer.append(1.0 + 0.05 * index, (0, 0, 0), yaw(10 if index > turning_interval else 0))
                match, reason = buffer.lookup(1.35)
                self.assertEqual(reason, "ok")
                self.assertAlmostEqual(match.angular_speed, 0)
                self.assertAlmostEqual(match.window_angular_speed, math.radians(10) / 0.05)

    def test_recent_translation_is_in_motion_window_maximum(self):
        buffer = PoseBuffer()
        for index in range(9):
            buffer.append(1.0 + index * 0.05, (0.1 if index > 1 else 0, 0, 0), yaw(0))
        match, reason = buffer.lookup(1.35)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(match.linear_speed, 0)
        self.assertAlmostEqual(match.window_linear_speed, 2.0)

    def test_quaternion_sign_flip_does_not_count_as_motion(self):
        buffer = PoseBuffer()
        quaternion = yaw(35)
        buffer.append(1.0, (0, 0, 0), quaternion)
        buffer.append(1.1, (0, 0, 0), -quaternion)
        match, reason = buffer.lookup(1.05, window_before=0, window_after=0)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(match.angular_speed, 0)

    def test_duplicate_replaces_and_out_of_order_samples_remain_sorted(self):
        buffer = PoseBuffer()
        buffer.append(1.1, (1, 0, 0), yaw(0))
        buffer.append(1.0, (0, 0, 0), yaw(0))
        buffer.append(1.05, (0.5, 0, 0), yaw(0))
        self.assertTrue(buffer.append(1.05, (0.75, 0, 0), yaw(0)))
        self.assertEqual(buffer.sample_count, 3)
        self.assertEqual(buffer.first_stamp, 1.0)
        self.assertEqual(buffer.latest_stamp, 1.1)
        match, reason = buffer.lookup(1.05, window_before=0, window_after=0)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(match.position[0], 0.75)

    def test_horizon_and_count_are_bounded_and_old_data_cannot_clear_buffer(self):
        buffer = PoseBuffer(horizon=1, max_samples=3)
        for timestamp in (1.0, 1.5, 2.0, 2.1):
            self.assertTrue(buffer.append(timestamp, (0, 0, 0), yaw(0)))
        self.assertEqual(buffer.sample_count, 3)
        self.assertEqual(buffer.first_stamp, 1.5)
        self.assertFalse(buffer.append(0.5, (0, 0, 0), yaw(0)))
        self.assertFalse(buffer.append(1.4, (0, 0, 0), yaw(0)))
        self.assertEqual(buffer.latest_stamp, 2.1)
        self.assertTrue(buffer.append(4.0, (0, 0, 0), yaw(0)))
        self.assertEqual(buffer.sample_count, 1)
        self.assertEqual(buffer.first_stamp, 4.0)

    def test_invalid_stamps_and_poses_are_rejected(self):
        buffer = PoseBuffer()
        for timestamp in (0, -1, math.nan, math.inf, None, "bad"):
            with self.subTest(stamp=timestamp):
                self.assertFalse(buffer.append(timestamp, (0, 0, 0), (0, 0, 0, 1)))
                self.assertEqual(buffer.lookup(timestamp)[1], "invalid_stamp")
        for position, quaternion in (((math.nan, 0, 0), yaw(0)), ((0, 0, 0), (0, 0, 0, 0)),
                                     ((0, 0, 0), (0, math.inf, 0, 1)), ((0, 0), yaw(0))):
            self.assertFalse(buffer.append(1.0, position, quaternion))
        self.assertEqual(buffer.sample_count, 0)

    def test_snapshots_are_immutable_and_clear_removes_history(self):
        buffer = PoseBuffer()
        buffer.append(1.0, (0, 0, 0), yaw(0))
        buffer.append(1.1, (1, 0, 0), yaw(0))
        match, _ = buffer.lookup(1.05, window_before=0, window_after=0)
        with self.assertRaises(FrozenInstanceError):
            match.span = 5
        buffer.clear()
        self.assertEqual(buffer.sample_count, 0)
        self.assertIsNone(buffer.first_stamp)
        self.assertIsNone(buffer.latest_stamp)
        self.assertAlmostEqual(match.position[0], 0.5)

    def test_parallel_appends_keep_sorted_valid_samples(self):
        buffer = PoseBuffer()
        timestamps = [1.0 + index * 0.01 for index in reversed(range(100))]
        with ThreadPoolExecutor(max_workers=4) as executor:
            accepted = list(executor.map(lambda timestamp: buffer.append(timestamp, (timestamp, 0, 0), yaw(0)), timestamps))
        self.assertTrue(all(accepted))
        self.assertEqual(buffer.sample_count, 100)
        match, reason = buffer.lookup(1.5)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(match.position[0], 1.5)
        self.assertAlmostEqual(match.window_linear_speed, 1)

    def test_epoch_timestamps_allow_nominal_max_gap_with_float_roundoff(self):
        buffer = PoseBuffer()
        for index in range(6):
            buffer.append(1788790000.0 + index * 0.1, (0, 0, 0), yaw(0))
        self.assertEqual(buffer.lookup(1788790000.4)[1], "ok")

    def test_invalid_limits_fail_clearly(self):
        for kwargs in ({"horizon": 0}, {"horizon": math.inf}, {"max_samples": 1}, {"max_samples": 2.5}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                PoseBuffer(**kwargs)
        buffer = PoseBuffer()
        for kwargs in ({"max_gap": 0}, {"max_gap": math.inf}, {"window_before": -1}, {"window_after": math.nan}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                buffer.lookup(1.0, **kwargs)


if __name__ == "__main__":
    unittest.main()
