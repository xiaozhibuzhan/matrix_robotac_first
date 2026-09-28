import sys
from pathlib import Path
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ground_surface import GroundSurfaceFilter


class GroundSurfaceTests(unittest.TestCase):
    def test_sparse_horizontal_patch_selects_original_points(self):
        xy = np.array([(x, y) for x in (-.4, -.2, 0, .2, .4)
                       for y in (-.4, -.2, 0, .2, .4)], float)
        points = np.column_stack((xy, np.zeros(len(xy))))
        selected = GroundSurfaceFilter().mask(points)
        self.assertGreaterEqual(selected.sum(), 20)
        np.testing.assert_array_equal(points[selected], points[np.flatnonzero(selected)])

    def test_dense_one_and_three_centimetre_ground_are_not_rejected_by_support_cap(self):
        for spacing in (.01, .03):
            with self.subTest(spacing=spacing):
                points = np.array([(x, y, 0.) for x in np.arange(-.3, .301, spacing)
                                   for y in np.arange(-.3, .301, spacing)])
                self.assertTrue(GroundSurfaceFilter().mask(points).all())

    def test_sparse_thirty_centimetre_ground_retains_supported_interior(self):
        points = np.array([(x * .3, y * .3, 0.) for x in range(7) for y in range(7)])
        selected = GroundSurfaceFilter().mask(points)
        self.assertGreaterEqual(int(selected.sum()), 45)

    def test_dense_scan_rings_do_not_hide_nearby_ground_support(self):
        # The old 32-neighbour query filled every slot from the query's own
        # 5 mm-spaced ring and rejected both rings as one-dimensional lines.
        points = np.array([(x, y, .004) for x in np.arange(-1., 1.001, .005)
                           for y in (-.2, .2)])
        selected = GroundSurfaceFilter().mask(points)
        self.assertTrue(selected.all())

    def test_recent_scan_lines_supply_plane_support_without_creating_points(self):
        filt = GroundSurfaceFilter()
        first = np.array([(x, 0., .006) for x in np.linspace(-.8, .8, 17)])
        second = first + [0., .25, 0.]
        self.assertFalse(filt.mask(first, stamp=10).any())
        self.assertTrue(filt.mask(second, stamp=10.1).all())
        np.testing.assert_array_equal(second[filt.mask(second, stamp=10.2)], second)
        # The output is a mask over this scan, never the old cached row.
        self.assertEqual(filt.mask(second, stamp=10.3).shape, (len(second),))

    def test_temporal_support_expires_and_clear_resets_timestamp(self):
        filt = GroundSurfaceFilter(support_age=1)
        first = np.array([(x, 0., 0.) for x in np.linspace(-.8, .8, 17)])
        second = first + [0., .25, 0.]
        self.assertFalse(filt.mask(first, stamp=10).any())
        self.assertFalse(filt.mask(second, stamp=11.01).any())
        self.assertTrue(filt.mask(first, stamp=11.02).all())
        filt.clear()
        self.assertFalse(filt.mask(first, stamp=1).any())

    def test_duplicate_or_backwards_scans_do_not_add_plane_support(self):
        filt = GroundSurfaceFilter()
        first = np.array([(x, 0., 0.) for x in np.linspace(-.8, .8, 17)])
        second = first + [0., .25, 0.]
        self.assertFalse(filt.mask(first, stamp=10).any())
        self.assertFalse(filt.mask(second, stamp=10).any())
        self.assertFalse(filt.mask(second, stamp=9).any())
        self.assertFalse(filt.mask(first, stamp=10.1).any())

    def test_support_history_has_bounded_frame_count(self):
        filt = GroundSurfaceFilter(support_frames=2)
        first = np.array([(x, 0., 0.) for x in np.linspace(-.8, .8, 17)])
        second = first + [0., .25, 0.]
        self.assertFalse(filt.mask(first, stamp=10).any())
        self.assertTrue(filt.mask(second, stamp=10.1).all())
        self.assertFalse(filt.mask(second, stamp=10.2).any())

    def test_absent_ground_does_not_build_spatial_index(self):
        wall = np.array([(0., y, z) for y in np.arange(20) * .1
                         for z in 1 + np.arange(20) * .1])
        with mock.patch("ground_surface.cKDTree") as tree:
            self.assertFalse(GroundSurfaceFilter().mask(wall, stamp=10).any())
        tree.assert_not_called()

    def test_temporal_wall_measurements_are_not_cropped_into_floor(self):
        filt = GroundSurfaceFilter()
        wall = np.array([(0., y, z) for y in np.linspace(-.5, .5, 12)
                         for z in np.linspace(-.1, .7, 12)])
        for i in range(8):
            self.assertFalse(filt.mask(wall + [0., i * .002, 0.], stamp=10 + i * .1).any())

    def test_local_cross_scan_confirmation_maps_moving_nonoverlapping_voxels(self):
        filt = GroundSurfaceFilter(min_observations=2)
        first = np.array([(x, y, .006) for x in .015 + np.arange(7) * .3
                          for y in .015 + np.arange(7) * .3])
        second = first + [.12, .12, 0.]
        first_keys = set(map(tuple, np.floor(first / .1).tolist()))
        second_keys = set(map(tuple, np.floor(second / .1).tolist()))
        self.assertFalse(first_keys.intersection(second_keys))
        self.assertFalse(filt.mask(first, stamp=10).any())
        self.assertGreaterEqual(filt.last_counts["plane_supported"], 45)
        self.assertEqual(filt.last_counts["temporal_confirmed"], 0)
        selected = filt.mask(second, stamp=10.1)
        self.assertGreaterEqual(selected.sum(), 45)
        self.assertEqual(filt.last_counts["support_scans"], 2)
        self.assertEqual(filt.last_counts["temporal_confirmed"], selected.sum())
        np.testing.assert_array_equal(second[selected], second[np.flatnonzero(selected)])

    def test_confirmation_needs_distinct_nearby_scans_and_same_local_plane(self):
        first = np.array([(x, y, 0.) for x in np.arange(7) * .1
                          for y in np.arange(7) * .1])
        filt = GroundSurfaceFilter(min_observations=2)
        self.assertFalse(filt.mask(first).any())
        self.assertFalse(filt.mask(first, stamp=10).any())
        self.assertFalse(filt.mask(first, stamp=10).any())
        self.assertFalse(filt.mask(first, stamp=9).any())
        self.assertFalse(filt.mask(first + [5, 0, 0], stamp=10.1).any())
        self.assertTrue(filt.mask(first, stamp=10.2).all())
        filt.clear()
        self.assertFalse(filt.mask(first + [0, 0, .1], stamp=10).any())
        self.assertFalse(filt.mask(first, stamp=10.1).any())

    def test_three_observations_are_counted_per_scan_not_neighbour_count(self):
        points = np.array([(x, y, 0.) for x in np.arange(7) * .1
                           for y in np.arange(7) * .1])
        filt = GroundSurfaceFilter(min_observations=3)
        self.assertFalse(filt.mask(np.repeat(points, 10, axis=0), stamp=10).any())
        self.assertFalse(filt.mask(points, stamp=10.1).any())
        self.assertTrue(filt.mask(points, stamp=10.2).all())

    def test_thin_nearly_collinear_strip_is_rejected(self):
        points = np.array([(x, y, 0.) for x in np.linspace(-.4, .4, 15)
                           for y in (-.01, .01)])
        self.assertFalse(GroundSurfaceFilter().mask(points).any())

    def test_wall_root_and_line_are_rejected(self):
        wall = np.array([(0, y, z) for y in np.linspace(-.5, .5, 12)
                          for z in np.linspace(-.1, .7, 12)])
        line = np.column_stack((np.linspace(-.5, .5, 12), np.zeros(12),
                                np.zeros(12)))
        self.assertFalse(GroundSurfaceFilter().mask(wall).any())
        self.assertFalse(GroundSurfaceFilter().mask(line).any())

    def test_isolated_and_identical_returns_cannot_provide_support(self):
        points = np.array([[0, 0, 0], [4, 5, 0], [10, 10, 0]], float)
        self.assertFalse(GroundSurfaceFilter().mask(points).any())
        self.assertFalse(GroundSurfaceFilter().mask(np.repeat(points[:1], 100, axis=0)).any())

    def test_tilted_plane_boundary(self):
        xy = np.array([(x, y) for x in np.linspace(-.4, .4, 5)
                       for y in np.linspace(-.4, .4, 5)])
        gentle = np.column_stack((xy, .1 * xy[:, 0]))
        steep = np.column_stack((xy, .5 * xy[:, 0]))
        self.assertGreater(GroundSurfaceFilter(max_slope_deg=15).mask(gentle).sum(), 10)
        self.assertEqual(GroundSurfaceFilter(max_slope_deg=15).mask(steep).sum(), 0)

    def test_outliers_and_duplicates_do_not_create_points(self):
        xy = np.array([(x, y) for x in (-.4, -.2, 0, .2, .4)
                       for y in (-.4, -.2, 0, .2, .4)], float)
        ground = np.column_stack((xy, np.zeros(len(xy))))
        points = np.vstack((ground, [[0, 0, 1], [10, 10, 0], [0, 0, .10]]))
        original = points.copy()
        selected = GroundSurfaceFilter().mask(points)
        self.assertFalse(selected[-3:].any())
        self.assertTrue(np.all(points[selected, 2] == 0))
        np.testing.assert_array_equal(points, original)

    def test_duplicates_do_not_hide_other_support_in_dense_patch(self):
        ground = np.array([(x, y, 0) for x in (-.2, 0, .2)
                           for y in (-.2, 0, .2)])
        points = np.repeat(ground, 100, axis=0)
        self.assertTrue(GroundSurfaceFilter().mask(points).all())

    def test_ground_height_is_explicit_and_no_height_projection_occurs(self):
        points = np.array([(x, y, .5 + x * .01) for x in (-.2, 0, .2)
                           for y in (-.2, 0, .2)])
        self.assertFalse(GroundSurfaceFilter().mask(points).any())
        selected = GroundSurfaceFilter(ground_z=.5).mask(points)
        self.assertTrue(selected.all())
        self.assertGreater(len(np.unique(points[selected, 2])), 1)

    def test_rough_volume_is_rejected(self):
        points = np.array([(x, y, z) for x in (-.3, -.15, 0, .15, .3)
                           for y in (-.3, -.15, 0, .15, .3) for z in (-.1, .1)])
        self.assertFalse(GroundSurfaceFilter(max_neighbors=64).mask(points).any())

    def test_dense_wall_contamination_is_conservative(self):
        floor = np.array([(x, y, 0) for x in np.linspace(0, .2, 5)
                          for y in np.linspace(-.2, .2, 5)])
        wall = np.array([(0, y, z) for y in np.linspace(-.3, .3, 9)
                         for z in np.linspace(0, .5, 11)])
        points = np.vstack((floor, wall))
        selected = GroundSurfaceFilter(max_neighbors=128).mask(points)
        self.assertFalse(selected.any())

    def test_empty_invalid_and_parameter_validation(self):
        filt = GroundSurfaceFilter()
        self.assertEqual(filt.mask(np.empty((0, 3))).shape, (0,))
        with self.assertRaises(ValueError):
            filt.mask([[0, 0, np.nan]])
        with self.assertRaises(ValueError):
            GroundSurfaceFilter(min_support=2)
        for invalid in ([[0, 0]], [1, 2, 3], [[0, float("inf"), 0]]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                filt.mask(invalid)
        for kwargs in ({"band": -1}, {"radius": 0}, {"ground_z": float("nan")},
                       {"max_slope_deg": 90}, {"max_residual": -1}, {"min_planar_spread": 0},
                       {"min_support": True}, {"max_neighbors": 5}, {"max_neighbors": 7.5},
                       {"support_age": -1}, {"support_age": float("nan")},
                       {"support_frames": 0}, {"support_frames": 1.5}, {"support_frames": True},
                       {"min_observations": 0}, {"min_observations": 7},
                       {"min_observations": 1.5}, {"min_observations": True}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                GroundSurfaceFilter(**kwargs)
        with self.assertRaises(ValueError):
            filt.mask([[0, 0, 0]], stamp=float("nan"))


if __name__ == "__main__":
    unittest.main()
