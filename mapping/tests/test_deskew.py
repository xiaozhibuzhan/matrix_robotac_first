"""Independent moving-scene checks for recorded per-point pose alignment."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deskew import Trajectory, deskew_cloud


def cloud(points, times, stamp=10., *, endian="<", height=1, padding=0):
    points = np.asarray(points, dtype=float)
    width, step = len(points) // height, 24
    row_step = width * step + padding
    payload = bytearray(row_step * height)
    dtype = np.dtype({"names": ["x", "y", "z", "timestamp"],
                      "formats": [endian + "f4"] * 3 + [endian + "f8"],
                      "offsets": [0, 4, 8, 16], "itemsize": step})
    records = np.ndarray((height, width), dtype=dtype, buffer=payload, strides=(row_step, step))
    for index, name in enumerate(("x", "y", "z")):
        records[name] = points[:, index].reshape(height, width)
    records["timestamp"] = np.asarray(times).reshape(height, width)
    whole = int(stamp)
    return SimpleNamespace(width=width, height=height, point_step=step, row_step=row_step,
                           fields=[SimpleNamespace(name=name, offset=offset, count=1, datatype=datatype)
                                   for name, offset, datatype in (("x", 0, 7), ("y", 4, 7),
                                                                  ("z", 8, 7), ("timestamp", 16, 8))],
                           data=payload, is_bigendian=endian == ">",
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=whole,
                                                                       nanosec=round((stamp-whole)*1e9))))


class DeskewTests(unittest.TestCase):
    def stationary(self):
        return Trajectory([9.9, 10., 10.1], np.zeros((3, 3)), [[0, 0, 0, 1]] * 3)

    def test_moving_and_turning_sensor_reconstructs_fixed_world_geometry(self):
        pose_times = np.linspace(9.9, 10.1, 21)
        positions = np.column_stack(((pose_times-10.) * 2., np.zeros((21, 2))))
        rotations = Rotation.from_euler("z", (pose_times-10.) * 100., degrees=True)
        trajectory = Trajectory(pose_times, positions, rotations.as_quat())
        query = np.linspace(9.95, 10.05, 51)
        world = np.column_stack((np.full(len(query), 8.), np.linspace(-2, 2, len(query)),
                                 np.full(len(query), 1.)))
        query_positions = np.column_stack(((query-10.) * 2., np.zeros((len(query), 2))))
        local = Slerp(pose_times, rotations)(query).inv().apply(world-query_positions) - [0., 0., .3]
        result = deskew_cloud(cloud(local, query), trajectory, .3, .1, .02)
        self.assertEqual(result.reason, "ok")
        self.assertTrue(result.valid_mask.all())
        np.testing.assert_allclose(result.points, world, atol=5e-7)
        # Applying only the header pose cannot reconstruct this rotating scan.
        single_pose_error = np.linalg.norm(local + [0., 0., .3] - world, axis=1)
        self.assertGreater(single_pose_error.max(), .5)

    def test_no_extrapolation_or_interpolation_across_missing_odometry(self):
        trajectory = Trajectory([9.95, 9.97, 10.03, 10.05], np.zeros((4, 3)), [[0, 0, 0, 1]] * 4)
        times = [9.94, 9.95, 9.96, 10., 10.04, 10.05, 10.06]
        result = deskew_cloud(cloud([[1, 0, 0]] * 7, times), trajectory, .3, .1, .025)
        self.assertEqual(result.reason, "ok")
        self.assertEqual(result.valid_mask.tolist(), [False, True, True, False, True, True, False])
        self.assertTrue(np.isnan(result.points[~result.valid_mask]).all())
        np.testing.assert_allclose(result.points[result.valid_mask], [[1, 0, .3]] * 4)

    def test_big_endian_organized_rows_and_epoch_tolerance(self):
        stamp = 1788888888.
        times = np.array([stamp-.1, stamp-.03, stamp+.04, stamp+.1])
        trajectory = Trajectory([stamp-.11, stamp, stamp+.11], np.zeros((3, 3)), [[0, 0, 0, 1]] * 3)
        points = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]]
        result = deskew_cloud(cloud(points, times, stamp, endian=">", height=2, padding=16),
                              trajectory, .3, .1, .12)
        self.assertEqual(result.reason, "ok")
        self.assertTrue(result.valid_mask.all())
        np.testing.assert_allclose(result.points, np.array(points) + [0, 0, .3])

    def test_relative_second_offsets_are_anchored_to_header(self):
        trajectory = Trajectory([10.0, 10.1, 10.2], np.zeros((3, 3)), [[0, 0, 0, 1]] * 3)
        result = deskew_cloud(cloud([[1, 0, 0]] * 3, [0.0, .05, .1], 10.0),
                              trajectory, .3, .1, .11)
        self.assertEqual(result.reason, "ok")
        self.assertTrue(result.valid_mask.all())
        np.testing.assert_allclose(result.timestamps, [10.0, 10.05, 10.1])

    def test_relative_offsets_can_be_anchored_at_header_end(self):
        trajectory = Trajectory([10.0, 10.05, 10.1], np.zeros((3, 3)), [[0, 0, 0, 1]] * 3)
        result = deskew_cloud(cloud([[1, 0, 0]] * 3, [0.0, .05, .1], 10.1),
                              trajectory, .3, .1, .06)
        self.assertEqual(result.reason, "ok")
        self.assertTrue(result.valid_mask.all())
        np.testing.assert_allclose(result.timestamps, [10.0, 10.05, 10.1])

    def test_integer_microsecond_offsets_are_decoded(self):
        trajectory = Trajectory([10.0, 10.1], np.zeros((2, 3)), [[0, 0, 0, 1]] * 2)
        result = deskew_cloud(cloud([[1, 0, 0]] * 2, [0.0, 50000.0], 10.0),
                              trajectory, .3, .05, .06)
        self.assertEqual(result.reason, "ok")
        np.testing.assert_allclose(result.timestamps, [10.0, 10.05])

    def test_ambiguous_or_invalid_timestamps_require_fallback(self):
        cases = [([10., 10.], "constant_timestamp"),
                 ([9.89, 10.], "timestamp_outside_scan"),
                 ([10., np.nan], "nonfinite_timestamp"),
                 ([10., np.inf], "nonfinite_timestamp")]
        for times, reason in cases:
            with self.subTest(reason=reason, times=times):
                result = deskew_cloud(cloud([[1, 0, 0]] * 2, times), self.stationary(), .3, .1, .11)
                self.assertEqual(result.reason, reason)
                self.assertFalse(result.valid_mask.any())

    def test_missing_wrong_type_and_duplicate_timestamp_fields_require_fallback(self):
        for variant in ("missing", "float32", "duplicate", "outside"):
            message = cloud([[1, 0, 0]] * 2, [9.99, 10.01])
            if variant == "missing":
                message.fields.pop()
            elif variant == "float32":
                message.fields[-1].datatype = 7
            elif variant == "duplicate":
                message.fields.append(message.fields[-1])
            else:
                message.fields[-1].offset = 20
            result = deskew_cloud(message, self.stationary(), .3, .1, .11)
            self.assertEqual(result.reason, "missing_timestamp" if variant == "missing" else "invalid_timestamp_field")
            self.assertFalse(result.valid_mask.any())

    def test_nonfinite_xyz_is_rejected_without_disabling_other_timed_points(self):
        result = deskew_cloud(cloud([[np.nan, 0, 0], [1, 0, 0]], [9.99, 10.01]),
                              self.stationary(), .3, .1, .11)
        self.assertEqual(result.reason, "ok")
        self.assertEqual(result.valid_mask.tolist(), [False, True])
        np.testing.assert_allclose(result.points[1], [1, 0, .3])

    def test_spherical_interpolation_matches_scipy_with_quaternion_sign_changes(self):
        times = np.array([1., 2., 3.])
        rotations = Rotation.from_euler("xyz", [[0, 0, 0], [45, 60, 100], [46, 60.1, 100.2]], degrees=True)
        quaternions = rotations.as_quat()
        quaternions[1] *= -1
        trajectory = Trajectory(times, np.zeros((3, 3)), quaternions)
        query = np.linspace(1., 3., 101)
        _, actual, valid = trajectory.interpolate(query, 1.)
        self.assertTrue(valid.all())
        np.testing.assert_allclose(Rotation.from_quat(actual).as_matrix(),
                                   Slerp(times, rotations)(query).as_matrix(), atol=2e-8)

    def test_rejects_invalid_trajectory(self):
        for times, positions, rotations in (([1, 1], [[0, 0, 0]] * 2, [[0, 0, 0, 1]] * 2),
                                           ([0, 1], [[0, 0, 0]] * 2, [[0, 0, 0, 1]] * 2),
                                           ([1, 2], [[0, 0, 0]] * 2, [[0, 0, 0, 0]] * 2),
                                           ([1, 2], [[0, 0, 0]] * 2, [[1e308] * 4] * 2),
                                           ([1, 2], [[0, np.nan, 0]] * 2, [[0, 0, 0, 1]] * 2)):
            with self.assertRaises(ValueError):
                Trajectory(times, positions, rotations)

    def test_epoch_ulp_does_not_reject_a_nominal_pose_gap(self):
        epoch = 1788888888.
        times = np.array([epoch + .1, epoch + .2])
        self.assertGreater(times[1] - times[0], .1)
        trajectory = Trajectory(times, np.zeros((2, 3)), [[0, 0, 0, 1]] * 2)
        _, _, valid = trajectory.interpolate([epoch + .15], .1)
        self.assertTrue(valid[0])
        _, _, invalid = trajectory.interpolate([epoch + .15], .099)
        self.assertFalse(invalid[0])


if __name__ == "__main__":
    unittest.main()
