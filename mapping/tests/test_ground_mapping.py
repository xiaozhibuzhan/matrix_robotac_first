"""End-to-end measured-ground regression through the real mapper with fake ROS."""

import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_field_mapper import mapper, Odometry, PointCloud2
from clean_pcd import read_pcd


class GroundMappingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "map.pcd"
        self.node = self.make_node()

    def make_node(self, *options):
        return mapper.FieldMapperNode(mapper.build_parser().parse_args([
            "--pcd-file", str(self.path), "--sensor-z", "0.3", *options]))

    @staticmethod
    def floor(shift=0.0, z=0.004):
        # 30 cm beam spacing is wider than the old 20 cm isolation radius.
        x, y = np.meshgrid(2.015 + np.arange(7) * 0.3 + shift,
                           1.015 + np.arange(7) * 0.3 + shift)
        return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))

    @staticmethod
    def wall():
        y, z = np.meshgrid(np.arange(20) * 0.05, 0.4 + np.arange(20) * 0.05)
        return np.column_stack((np.full(y.size, 6.0), y.ravel(), z.ravel()))

    def scan(self, world, seconds=10.0, node=None):
        node = node or self.node
        for offset in range(-8, 4):
            pose = Odometry(seconds + offset * 0.05)
            pose.header.frame_id = "world"
            node.odom_callback(pose)
        local = np.asarray(world, dtype=float).reshape(-1, 3).copy()
        local[:, 2] -= node.sensor_z
        node.cloud_callback(PointCloud2(local, seconds))
        node.process_pending()

    def test_sparse_ground_reaches_rviz_and_pcd_without_changing_wall_map(self):
        baseline = self.make_node("--disable-ground")
        truth = self.floor()
        for i, shift in enumerate((0, 0.032, 0.063)):
            points = np.vstack((self.floor(shift), self.wall()))
            self.scan(points, 10 + i * 0.1)
            self.scan(points, 10 + i * 0.1, baseline)
        self.assertEqual(self.node.voxels, baseline.voxels)
        # Sparse boundary corners may lack six neighbors; the interior must survive.
        self.assertGreaterEqual(len(self.node.ground_fusion.points), math.ceil(0.9 * len(truth)))
        self.assertEqual(len(baseline.ground_fusion.points), 0)
        self.assertGreater(self.node.ground_scan_counts["band_input"], 0)
        self.assertEqual(self.node.ground_scan_counts["band_after_radius"], 0)
        self.assertGreaterEqual(self.node.ground_scan_counts["plane_supported"], math.ceil(0.9 * len(truth)))
        self.node.publish_map()
        ground_msg = self.node.ground_publisher.messages[-1]
        ground = np.frombuffer(ground_msg.data, dtype="<f4").reshape(-1, 3)
        # The average stays on measured ground, not forced to a z=0 plane.
        # The first scan establishes support, the latter two provide retained measurements.
        expected = self.floor((0.032 + 0.063) / 2)
        nearest_expected = np.min(np.linalg.norm(ground[:, None, :] - expected[None, :, :], axis=2), axis=1)
        self.assertTrue(np.all(nearest_expected < 1e-6))
        self.assertEqual(ground_msg.header.frame_id, "world")
        ok, message = self.node.save_pcd()
        self.assertTrue(ok, message)
        saved = read_pcd(self.path)
        published = np.frombuffer(self.node.map_publisher.messages[-1].data, dtype="<f4").reshape(-1, 3)
        self.assertLess(len(published), len(saved))
        saved_rows = set(map(tuple, saved.tolist()))
        self.assertTrue(all(tuple(point) in saved_rows for point in published.tolist()))
        self.assertEqual(len(saved), len(self.node.voxels) + len(ground))
        self.assertTrue(np.all(saved[-len(ground):, 2] > 0))

    def test_duplicate_scan_and_single_frame_do_not_confirm_ground(self):
        points = self.floor()
        self.scan(points, 10)
        self.scan(points, 10)
        self.assertEqual(self.node.rejection_reasons["nonmonotonic_stamp"], 1)
        self.assertFalse(self.node.ground_fusion.points)
        self.scan(points, 10.1)
        self.assertTrue(self.node.ground_fusion.points)
        self.scan(points, 10.2)
        self.assertTrue(self.node.ground_fusion.points)

    def test_ground_candidates_expire_between_disconnected_observations(self):
        self.scan(self.floor(), 10)
        self.scan(self.floor(), 13)
        self.scan(self.floor(), 16)
        self.assertFalse(self.node.ground_fusion.points)

    def test_wall_bases_do_not_create_a_ground_plane(self):
        y, z = np.meshgrid(np.arange(40) * 0.05, np.arange(20) * 0.05)
        points = np.column_stack((np.full(y.size, 2), y.ravel(), z.ravel()))
        for i in range(3):
            self.scan(points, 10 + i * 0.1)
        self.assertTrue(self.node.voxels)
        self.assertGreater(self.node.ground_totals["band_input"], 0)
        self.assertFalse(self.node.ground_fusion.points)
        self.assertEqual(self.node.ground_totals["plane_supported"], 0)

    def test_empty_missing_or_out_of_range_ground_does_not_fill_a_floor(self):
        for i, points in enumerate(([], self.wall(), self.floor() + [100, 0, 0])):
            self.scan(points, 10 + i * 0.1)
        self.assertFalse(self.node.ground_fusion.points)
        self.assertEqual(self.node.ground_totals["band_input"], 0)
        self.assertGreater(self.node.rejected_range, 0)

    def test_motion_gate_applies_to_ground_and_stationary_capture_recovers(self):
        self.node = self.make_node("--motion-policy", "strict", "--motion-window", "0.35", "--motion-lookahead", "0.1")
        for tick in range(192, 219):
            seconds = tick * 0.05
            angle = math.radians(max(0.0, min(30.0, (seconds - 10.0) * 150.0)))
            self.node.odom_callback(Odometry(seconds, quaternion=(0, 0, math.sin(angle / 2), math.cos(angle / 2))))
        local = self.floor() - [0, 0, self.node.sensor_z]
        for seconds in (10.1, 10.3):
            self.node.cloud_callback(PointCloud2(local, seconds))
            self.node.process_pending()
        self.assertEqual(self.node.rejection_reasons["turning_or_settling"], 2)
        self.assertEqual(self.node.ground_fusion.pending_count, 0)
        self.assertFalse(self.node.ground_fusion.points)
        # Independent stable pose window after the turn, with corresponding local XYZ.
        for i in range(3):
            self.scan(self.floor(), 12 + i * 0.1)
        self.assertTrue(self.node.ground_fusion.points)

    def test_clear_and_exit_save_include_ground(self):
        for i in range(3):
            self.scan(self.floor(), 10 + i * 0.1)
        self.assertTrue(self.node.dirty)
        self.node.close()
        self.assertEqual(len(read_pcd(self.path)), len(self.node.ground_fusion.points))
        response = self.node.clear_callback(None, SimpleNamespace())
        self.assertTrue(response.success, response.message)
        self.assertEqual(len(read_pcd(self.path)), 0)
        self.assertFalse(self.node.ground_fusion.points)
        self.assertEqual(self.node.ground_fusion.pending_count, 0)
        self.assertEqual(self.node.ground_publisher.messages[-1].width, 0)
        self.assertFalse(self.node.ground_totals)

    def test_ground_parameters_are_validated(self):
        for option, value in (("--ground-z", "nan"), ("--ground-band", "-1"),
                              ("--ground-radius", "0"), ("--ground-voxel", "inf"),
                              ("--ground-voxel", "0")):
            with self.subTest(option=option), self.assertRaises(ValueError):
                self.make_node(option, value)

    def test_dense_ground_display_is_a_subset_of_combined_pcd_without_duplicate_voxels(self):
        x, y = np.meshgrid(2.001 + np.arange(24) * 0.03, 1.001 + np.arange(24) * 0.03)
        points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.006)))
        for i in range(3):
            self.scan(points, 10 + i * 0.1)
        self.assertTrue(self.node.ground_fusion.points)
        self.node.publish_map()
        self.assertTrue(self.node.save_pcd()[0])
        saved = read_pcd(self.path)
        ground = np.frombuffer(self.node.ground_publisher.messages[-1].data, dtype="<f4").reshape(-1, 3)
        saved_rows = set(map(tuple, saved.tolist()))
        self.assertTrue(all(tuple(p) in saved_rows for p in ground.tolist()))
        keys = np.floor(self.node.map_points().astype(float) / self.node.voxel_size)
        self.assertEqual(len(np.unique(keys, axis=0)), len(saved))

    def test_status_distinguishes_missing_returns_from_unconfirmed_patches(self):
        with mock.patch.object(mapper.time, "monotonic", return_value=100):
            for i in range(10):
                self.scan(self.wall(), 10 + i * 0.1)
            self.node.report_status()
        self.assertTrue(any("no range-valid returns" in msg for _, msg in self.node.logger.entries))
        self.assertTrue(any("Ground stages" in msg and "band_after_radius" in msg for _, msg in self.node.logger.entries))


if __name__ == "__main__":
    unittest.main()
