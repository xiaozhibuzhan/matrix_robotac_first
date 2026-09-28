"""Cross-scan confidence and geometry tests for the ROS-independent voxel map."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from voxel_fusion import VoxelFusion


class VoxelFusionTests(unittest.TestCase):
    def test_one_scan_wall_and_many_duplicates_are_not_confirmed(self):
        fusion = VoxelFusion()
        wall = np.array([[1.01, index * 0.1, 1.01] for index in range(30)])
        repeated = np.repeat(wall, 20, axis=0)
        self.assertEqual(fusion.integrate(repeated, 10.0), [])
        self.assertEqual(fusion.pending_count, 30)
        self.assertEqual(fusion.points, {})

    def test_distinct_scans_confirm_identical_coordinates(self):
        fusion = VoxelFusion()
        point = [[1.01, 2.01, 3.01]]
        fusion.integrate(point, 10.0)
        self.assertEqual(fusion.integrate(point, 10.1), [(20, 40, 60)])
        np.testing.assert_allclose(fusion.points[(20, 40, 60)], point[0])
        self.assertEqual(fusion.pending_count, 0)
        self.assertEqual(fusion.integrate(point, 10.2), [])

    def test_duplicate_and_backwards_stamps_cannot_confirm_or_shift_points(self):
        fusion = VoxelFusion()
        fusion.integrate([[1.01, 2.01, 3.01]], 10.0)
        for stamp in (10.0, 9.0, float("nan"), float("inf")):
            self.assertEqual(fusion.integrate([[1.04, 2.04, 3.04]], stamp), [])
        self.assertEqual(fusion.points, {})
        fusion.integrate([[1.01, 2.01, 3.01]], 10.1)
        self.assertEqual(fusion.integrate([[1.04, 2.04, 3.04]], 10.1), [])
        np.testing.assert_allclose(fusion.points[(20, 40, 60)], (1.01, 2.01, 3.01))

    def test_scan_centroids_have_equal_weight_despite_different_densities(self):
        fusion = VoxelFusion(voxel_size=1.0)
        first = np.array([[0.1, 0.2, 0.3], [0.3, 0.4, 0.5]])
        original = first.copy()
        fusion.integrate(first, 1.0)
        fusion.integrate(np.repeat([[0.8, 0.8, 0.8]], 100, axis=0), 1.1)
        np.testing.assert_allclose(fusion.points[(0, 0, 0)], (0.5, 0.55, 0.6))
        np.testing.assert_array_equal(first, original)
        changed = fusion.integrate([[0.2, 0.1, 0.3]], 1.2)
        self.assertEqual(changed, [(0, 0, 0)])
        np.testing.assert_allclose(fusion.points[(0, 0, 0)], (0.4, 0.4, 0.5))

    def test_repeated_thin_wall_survives_while_one_scan_ghost_expires(self):
        fusion = VoxelFusion()
        wall = np.array([[2.01, 0.01, index * 0.1 + 0.01] for index in range(21)])
        ghost = wall + np.array([0.5, 0.2, 0.0])
        fusion.integrate(np.vstack((wall, ghost)), 1.0)
        fusion.integrate(wall, 1.1)
        self.assertEqual(len(fusion.points), 21)
        self.assertEqual(fusion.pending_count, 21)
        fusion.integrate([], 2.1)
        self.assertEqual(fusion.pending_count, 0)
        self.assertEqual(len(fusion.points), 21)
        np.testing.assert_allclose(sorted(fusion.points.values()), sorted(map(tuple, wall)))

    def test_expired_candidate_starts_new_confirmation_even_if_seen_again(self):
        fusion = VoxelFusion(max_pending_age=1.0)
        fusion.integrate([[0.01, 0.01, 0.01]], 1.0)
        self.assertEqual(fusion.integrate([[0.02, 0.02, 0.02]], 2.01), [])
        self.assertFalse(fusion.points)
        fusion.integrate([[0.03, 0.03, 0.03]], 2.1)
        np.testing.assert_allclose(fusion.points[(0, 0, 0)], (0.025, 0.025, 0.025))

    def test_expiry_boundary_is_inclusive_and_last_seen_refreshes_candidate(self):
        fusion = VoxelFusion(min_observations=3)
        point = [[0.01, 0.01, 0.01]]
        fusion.integrate(point, 1.0)
        fusion.integrate(point, 2.0)
        fusion.integrate([], 2.5)
        self.assertEqual(fusion.pending_count, 1)
        fusion.integrate(point, 3.0)
        self.assertEqual(len(fusion.points), 1)

    def test_negative_coordinates_use_floor_at_voxel_boundaries(self):
        fusion = VoxelFusion(voxel_size=0.5, min_observations=1)
        changed = fusion.integrate([[-0.001, 0, 0], [-0.5, 0, 0], [-0.501, 0, 0], [0, 0, 0]], 1.0)
        self.assertEqual(set(changed), {(-1, 0, 0), (-2, 0, 0), (0, 0, 0)})
        self.assertAlmostEqual(fusion.points[(-1, 0, 0)][0], -0.2505)

    def test_max_weight_caps_averaging_without_stopping_updates(self):
        fusion = VoxelFusion(voxel_size=1.0, max_weight=2)
        fusion.integrate([[0.1, 0.1, 0.1]], 1.0)
        fusion.integrate([[0.3, 0.3, 0.3]], 1.1)
        fusion.integrate([[0.8, 0.8, 0.8]], 1.2)
        np.testing.assert_allclose(fusion.points[(0, 0, 0)], (0.5, 0.5, 0.5))
        fusion.integrate([[0.9, 0.9, 0.9]], 1.3)
        np.testing.assert_allclose(fusion.points[(0, 0, 0)], (0.7, 0.7, 0.7))
        self.assertEqual(fusion._weights[(0, 0, 0)], 2)

    def test_confirmation_count_is_independent_of_weight_cap(self):
        fusion = VoxelFusion(voxel_size=1, min_observations=3, max_weight=1)
        fusion.integrate([[0.1, 0.1, 0.1]], 1.0)
        fusion.integrate([[0.2, 0.2, 0.2]], 1.1)
        self.assertFalse(fusion.points)
        fusion.integrate([[0.3, 0.3, 0.3]], 1.2)
        np.testing.assert_allclose(fusion.points[(0, 0, 0)], (0.3, 0.3, 0.3))

    def test_clear_resets_map_candidates_and_stamp_without_replacing_points_dict(self):
        fusion = VoxelFusion()
        visible_points = fusion.points
        fusion.integrate([[0.01, 0.01, 0.01]], 10.0)
        fusion.integrate([[0.01, 0.01, 0.01], [1.01, 1.01, 1.01]], 10.1)
        fusion.clear()
        self.assertIs(fusion.points, visible_points)
        self.assertEqual(fusion.pending_count, 0)
        self.assertFalse(visible_points)
        fusion.integrate([[2.01, 2.01, 2.01]], 1.0)
        fusion.integrate([[2.01, 2.01, 2.01]], 1.1)
        self.assertEqual(len(visible_points), 1)

    def test_invalid_arrays_do_not_consume_stamp_or_change_state(self):
        fusion = VoxelFusion()
        for points in ([[float("nan"), 0, 0]], [[1, 2]], [1, 2, 3], [[1e100, 0, 0]]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                fusion.integrate(points, 1.0)
        fusion.integrate([[0.01, 0.01, 0.01]], 1.0)
        fusion.integrate([[0.01, 0.01, 0.01]], 1.1)
        self.assertEqual(len(fusion.points), 1)

    def test_invalid_parameters(self):
        for kwargs in ({"voxel_size": 0}, {"voxel_size": float("nan")}, {"max_pending_age": -1},
                       {"max_pending_age": float("inf")}, {"min_observations": 0},
                       {"min_observations": 1.5}, {"min_observations": True}, {"max_weight": 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                VoxelFusion(**kwargs)

    def test_dense_mixed_voxels_match_equal_scan_centroids_after_weight_cap(self):
        fusion = VoxelFusion(voxel_size=1.0, min_observations=3, max_weight=2)
        rng = np.random.default_rng(481)
        expected = {}
        for frame in range(6):
            scan = []
            for key in ((-2, 4, 1), (0, -3, 2), (0, -3, -1), (0, 0, 0)):
                samples = np.asarray(key) + rng.uniform(0.05, 0.95, size=(1 + frame * 9, 3))
                scan.extend(samples)
            rng.shuffle(scan)
            groups = {}
            for sample in scan:
                groups.setdefault(tuple(np.floor(sample).astype(int)), []).append(sample)
            for key, samples in groups.items():
                centroid = np.mean(samples, axis=0)
                expected[key] = centroid if frame == 0 else expected[key] + (centroid - expected[key]) / 2
            changed = fusion.integrate(scan, 1.0 + frame * 0.1)
            if frame < 2:
                self.assertFalse(changed)
            else:
                self.assertEqual(changed, sorted(expected))
                for key, mean in expected.items():
                    np.testing.assert_allclose(fusion.points[key], mean, atol=1e-15)

    def test_snapshot_updates_only_confirmed_geometry_and_preserves_previous_array(self):
        fusion = VoxelFusion(voxel_size=1.0)
        empty = fusion.snapshot()
        self.assertEqual(empty.shape, (0, 3))
        self.assertEqual(empty.dtype, np.dtype("<f4"))
        version = fusion.version
        fusion.integrate([[0.1, 0.2, 0.3]], 1.0)
        self.assertEqual(fusion.version, version)
        self.assertIs(fusion.snapshot(), empty)
        fusion.integrate([[0.3, 0.4, 0.5]], 1.1)
        first = fusion.snapshot()
        self.assertEqual(fusion.version, version + 1)
        self.assertFalse(first.flags.writeable)
        with self.assertRaises(ValueError):
            first[0, 0] = 100
        np.testing.assert_allclose(first, [[0.2, 0.3, 0.4]])
        self.assertIs(fusion.snapshot(), first)
        fusion.integrate([[0.8, 0.9, 0.8], [2.1, 2.2, 2.3]], 1.2)
        second = fusion.snapshot()
        np.testing.assert_allclose(second, [[0.4, 0.5, 0.533333333]])
        np.testing.assert_allclose(first, [[0.2, 0.3, 0.4]])
        fusion.integrate([[2.3, 2.4, 2.5]], 1.3)
        third = fusion.snapshot()
        np.testing.assert_allclose(third, list(fusion.points.values()))
        self.assertEqual(len(third), 2)
        self.assertEqual(len(second), 1)

    def test_snapshot_batches_multiple_updates_in_map_insertion_order(self):
        fusion = VoxelFusion(voxel_size=1.0, min_observations=1)
        fusion.snapshot()
        fusion.integrate([[2.1, 0, 0], [-2.1, 0, 0]], 1.0)
        fusion.integrate([[0.1, 0, 0], [2.3, 0, 0]], 1.1)
        fusion.integrate([[-2.3, 0, 0], [0.3, 0, 0]], 1.2)
        np.testing.assert_allclose(fusion.snapshot(), list(fusion.points.values()))
        self.assertEqual(list(fusion.points), [(-3, 0, 0), (2, 0, 0), (0, 0, 0)])

    def test_pending_expiry_and_invalid_stamps_do_not_invalidate_snapshot(self):
        fusion = VoxelFusion()
        fusion.integrate([[0.01, 0.01, 0.01]], 1.0)
        fusion.integrate([[0.01, 0.01, 0.01], [1.01, 1.01, 1.01]], 1.1)
        snapshot = fusion.snapshot()
        version = fusion.version
        fusion.integrate([], 2.2)
        self.assertEqual(fusion.pending_count, 0)
        for stamp in (2.2, 1.0, float("nan")):
            fusion.integrate([[0.04, 0.04, 0.04]], stamp)
        self.assertEqual(fusion.version, version)
        self.assertIs(fusion.snapshot(), snapshot)

    def test_snapshot_clear_and_explicit_external_invalidation(self):
        fusion = VoxelFusion(min_observations=1)
        fusion.integrate([[0.01, 0.01, 0.01]], 1.0)
        first = fusion.snapshot()
        fusion.points[(0, 0, 0)] = (0.02, 0.03, 0.04)
        fusion.invalidate_snapshot()
        np.testing.assert_allclose(fusion.snapshot(), [[0.02, 0.03, 0.04]])
        fusion.points[(1, 1, 1)] = (0.06, 0.07, 0.08)
        np.testing.assert_allclose(fusion.snapshot(), list(fusion.points.values()))
        fusion.clear()
        self.assertEqual(fusion.snapshot().shape, (0, 3))
        np.testing.assert_allclose(first, [[0.01, 0.01, 0.01]])
        fusion.integrate([[2.01, 2.01, 2.01]], 0.1)
        np.testing.assert_allclose(fusion.snapshot(), [[2.01, 2.01, 2.01]])

    def test_last_changed_points_align_with_changed_keys(self):
        fusion = VoxelFusion(voxel_size=1.0, min_observations=1)
        self.assertEqual(fusion.integrate([[2.1, 0, 0], [0.1, 0, 0]], 1.0), [(0, 0, 0), (2, 0, 0)])
        self.assertEqual(fusion.last_changed_keys, ((0, 0, 0), (2, 0, 0)))
        np.testing.assert_allclose(fusion.last_changed_points,
                                   [fusion.points[key] for key in fusion.last_changed_keys])
        self.assertFalse(fusion.last_changed_points.flags.writeable)
        fusion.integrate([], 1.1)
        self.assertEqual(fusion.last_changed_keys, ())
        self.assertEqual(fusion.last_changed_points.shape, (0, 3))

    def test_dense_storage_growth_pending_expiry_and_new_confirmations(self):
        fusion = VoxelFusion(voxel_size=1.0, min_observations=3, max_pending_age=0.5)
        stable = np.column_stack((np.arange(1500) + .1, np.full(1500, .2), np.full(1500, .3)))
        # The first 1500 confirmations exceed initial dense storage capacity.
        for stamp in (1.0, 1.1, 1.2):
            fusion.integrate(stable, stamp)
        first = fusion.snapshot()
        self.assertEqual(len(first), 1500)
        self.assertEqual(len(fusion.last_added_keys), 1500)
        pending = stable + [5000, 0, 0]
        fusion.integrate(np.vstack((stable + [.1, 0, 0], pending)), 1.3)
        self.assertEqual(fusion.pending_count, 1500)
        fusion.integrate([], 2.0)
        self.assertEqual(fusion.pending_count, 0)
        self.assertEqual(fusion.last_added_keys, ())
        # A disconnected scan cannot reuse expired confidence. A new set of
        # three observations confirms these keys and forces another expansion.
        for i in range(3):
            fusion.integrate(pending, 2.1 + .1 * i)
            self.assertEqual(len(fusion.points), 3000 if i == 2 else 1500)
        self.assertEqual(fusion.pending_count, 0)
        np.testing.assert_array_equal(fusion.snapshot(), np.asarray(list(fusion.points.values()), dtype="<f4"))
        np.testing.assert_allclose(first, stable, atol=1e-4)
        self.assertEqual(len(fusion.last_added_keys), 1500)
        fusion.clear()
        self.assertEqual(fusion.last_added_keys, ())
        self.assertEqual(fusion.snapshot().shape, (0, 3))


if __name__ == "__main__":
    unittest.main()
