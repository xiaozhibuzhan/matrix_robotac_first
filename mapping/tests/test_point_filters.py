"""Geometric regression tests for the ROS-independent point-cloud filter."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from point_filters import filter_xyz


class PointFilterTests(unittest.TestCase):
    def test_neighbor_threshold_matches_direct_distances_after_deduplication(self):
        rng = np.random.default_rng(17)
        points = rng.uniform(-1, 1, (150, 3)).astype(np.float32)
        points = np.vstack((points, points[:10]))
        unique, inverse = np.unique(points, axis=0, return_inverse=True)
        distances = np.linalg.norm(unique[:, None, :].astype(float) - unique[None, :, :], axis=2)
        for radius, neighbors in ((0.2, 1), (0.4, 2), (0.7, 8)):
            with self.subTest(radius=radius, neighbors=neighbors):
                expected = unique[((distances <= radius).sum(axis=1) - 1 >= neighbors)]
                actual, _ = filter_xyz(points, min_range=0, max_range=0,
                                       radius=radius, min_neighbors=neighbors)
                np.testing.assert_array_equal(actual, expected)

    def test_thin_pole_survives_while_isolated_and_extreme_points_are_removed(self):
        pole = np.column_stack((np.full(21, 2.0), np.zeros(21), np.arange(21) * 0.05))
        points = np.vstack((pole, [12, 12, 12], [1000, 0, 0], [0, 0, 0]))
        filtered, stats = filter_xyz(points)
        np.testing.assert_array_equal(filtered, pole.astype(np.float32))
        self.assertEqual(stats, {"input": 24, "nonfinite": 0, "range": 2, "isolated": 1})

    def test_repeated_invalid_coordinate_cannot_support_itself(self):
        points = np.repeat([[2.0, 2.0, 2.0]], 100, axis=0)
        filtered, stats = filter_xyz(points)
        self.assertEqual(filtered.shape, (0, 3))
        self.assertEqual(stats["isolated"], 1)
        self.assertEqual(stats["duplicate_points"], 99)

    def test_duplicates_do_not_turn_one_other_neighbor_into_two(self):
        points = np.array([[2, 0, 0], [2, 0, 0], [2, 0, 0], [2, 0, 0.05]])
        filtered, stats = filter_xyz(points, min_neighbors=2)
        self.assertEqual(len(filtered), 0)
        self.assertEqual(stats["isolated"], 2)
        self.assertEqual(stats["duplicate_points"], 2)

    def test_supported_duplicates_are_collapsed_to_one_measurement(self):
        points = np.array([[2, 0, 0.1], [10, 0, 0], [2, 0, 0], [2, 0, 0.05], [2, 0, 0]])
        filtered, stats = filter_xyz(points)
        np.testing.assert_array_equal(filtered, points[[2, 3, 0]].astype(np.float32))
        self.assertEqual(stats["isolated"], 1)
        self.assertEqual(stats["duplicate_points"], 1)

    def test_range_limits_are_inclusive_and_use_all_three_axes(self):
        points = np.array([[0, 0, 0.5], [0, 0, 2], [0, 0, 0.49], [0, 2.01, 0], [0, -1, 0]])
        filtered, stats = filter_xyz(points, min_range=0.5, max_range=2, radius=0)
        np.testing.assert_array_equal(filtered, points[[4, 0, 1]].astype(np.float32))
        self.assertEqual(stats["range"], 2)

    def test_radius_boundary_is_inclusive_and_self_is_excluded(self):
        points = np.array([[2, 0, 0], [2, 0, 0.25], [2, 0, 0.5]])
        filtered, stats = filter_xyz(points, radius=0.25, min_neighbors=2)
        np.testing.assert_array_equal(filtered, points[[1]].astype(np.float32))
        self.assertEqual(stats["isolated"], 2)

    def test_zero_max_range_and_radius_disable_their_stages(self):
        points = np.array([[0, 0, 0], [1000, 1000, 1000]])
        filtered, stats = filter_xyz(points, min_range=0, max_range=0, radius=0, min_neighbors=0)
        np.testing.assert_array_equal(filtered, points.astype(np.float32))
        self.assertEqual(stats, {"input": 2, "nonfinite": 0, "range": 0, "isolated": 0})

    def test_disabled_upper_range_still_applies_lower_range(self):
        points = np.array([[0, 0, 0], [1000, 0, 0]])
        filtered, stats = filter_xyz(points, min_range=1, max_range=0, radius=0)
        np.testing.assert_array_equal(filtered, points[[1]].astype(np.float32))
        self.assertEqual(stats["range"], 1)

    def test_empty_and_nonfinite_rows_have_consistent_shape_and_counts(self):
        filtered, stats = filter_xyz([])
        self.assertEqual(filtered.shape, (0, 3))
        self.assertEqual(filtered.dtype, np.float32)
        self.assertEqual(stats, {"input": 0, "nonfinite": 0, "range": 0, "isolated": 0})
        points = np.array([[np.nan, 0, 0], [0, np.inf, 0], [0, 0, -np.inf],
                           [1, 0, 0], [1000, 0, 0]])
        filtered, stats = filter_xyz(points, radius=0)
        np.testing.assert_array_equal(filtered, np.array([[1, 0, 0]], dtype=np.float32))
        self.assertEqual(stats, {"input": 5, "nonfinite": 3, "range": 1, "isolated": 0})

    def test_float32_range_square_does_not_overflow(self):
        points = np.array([[1e30, 0, 0], [1, 0, 0]], dtype=np.float32)
        with np.errstate(over="raise", invalid="raise"):
            filtered, stats = filter_xyz(points, radius=0)
        np.testing.assert_array_equal(filtered, points[[1]])
        self.assertEqual(stats["range"], 1)

    def test_input_is_not_modified(self):
        points = np.array([[1, 0, 0], [1, 0, 0.05], [1, 0, 0.1], [1000, 0, 0]], dtype=np.float32)
        original = points.copy()
        filter_xyz(points)
        np.testing.assert_array_equal(points, original)

    def test_invalid_parameters_and_shapes_fail_clearly(self):
        for kwargs in ({"min_range": -1}, {"max_range": -1}, {"min_range": 3, "max_range": 2},
                       {"radius": -1}, {"radius": np.nan}, {"max_range": np.inf},
                       {"min_neighbors": 0}, {"min_neighbors": 1.5}, {"min_neighbors": True}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                filter_xyz([[1, 0, 0]], **kwargs)
        for points in ([1, 0, 0], [[1, 0]], np.zeros((0, 2)), np.zeros((1, 3, 1))):
            with self.subTest(shape=np.asarray(points).shape), self.assertRaises(ValueError):
                filter_xyz(points)


if __name__ == "__main__":
    unittest.main()
