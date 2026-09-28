"""Exercise motion uncertainty against real pose interpolation and fake ROS IO.

These tests bound displacement under the supplied scan timing/pose model; they
do not establish a real sensor's stamp convention or absolute map accuracy.
"""

from pathlib import Path
import math
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_field_mapper import mapper, Odometry, PointCloud2


class AdaptiveMappingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.created = 0

    def make_node(self, *options):
        self.created += 1
        return mapper.FieldMapperNode(mapper.build_parser().parse_args([
            "--pcd-file", str(Path(self.directory.name) / f"map{self.created}.pcd"),
            "--outlier-radius", "0", "--min-observations", "1",
            "--disable-ground", "--live-display", "scan", *options]))

    @staticmethod
    def trajectory(node, centre=10., angle=None, position=None, axis="z"):
        angle = angle or (lambda t: 0.)
        position = position or (lambda t: (0., 0., 0.))
        for tick in range(-100, 101):
            seconds = centre + tick * .005
            quaternion = Rotation.from_euler(axis, angle(seconds), degrees=True).as_quat()
            pose = Odometry(seconds, position(seconds), tuple(quaternion))
            pose.header.frame_id = "world"
            node.odom_callback(pose)

    @staticmethod
    def send(node, points, seconds=10.):
        node.cloud_callback(PointCloud2(np.asarray(points, dtype=float), seconds))
        node.process_pending()

    @staticmethod
    def match(node, seconds=10.):
        before = (node.motion_window if node.motion_policy == "strict"
                  else max(node.motion_window, node.scan_duration))
        after = (node.motion_lookahead if node.motion_policy == "strict"
                 else max(node.motion_lookahead, node.scan_duration))
        result, reason = node.pose_buffer.lookup(seconds, node.max_pose_gap,
                                                 before, after)
        if result is None:
            raise AssertionError(reason)
        return result

    def test_small_fast_vibration_keeps_geometry_with_small_excursion(self):
        adaptive = self.make_node()
        strict = self.make_node("--motion-policy", "strict")
        angle = lambda t: .1 * math.sin(2 * math.pi * 20 * (t - 10))
        points = [(1., 0., 0.), (10., 0., 0.)]
        for node in (adaptive, strict):
            self.trajectory(node, angle=angle)
            self.send(node, points)
        match = self.match(adaptive)
        self.assertGreater(math.degrees(match.window_angular_speed), 5.)
        self.assertLess(match.angular_excursion, math.radians(.11))
        self.assertEqual(adaptive.frames, 1)
        self.assertEqual(len(adaptive.voxels), 2)
        self.assertEqual(strict.rejection_reasons["turning_or_settling"], 1)
        self.assertFalse(strict.current_publisher.messages)

    def test_moderate_turn_accepts_near_points_and_withholds_far_points(self):
        node = self.make_node()
        self.trajectory(node, angle=lambda t: 20 * (t - 10))
        self.send(node, [(1., 0., 0.), (5., 0., 0.)])
        self.assertEqual(node.frames, 1)
        self.assertEqual(node.motion_rejected_points, 1)
        self.assertEqual(len(node.voxels), 1)
        np.testing.assert_allclose(next(iter(node.voxels.values())), [1., 0., .3], atol=1e-8)
        preview = np.frombuffer(node.current_publisher.messages[-1].data, dtype="<f4").reshape(-1, 3)
        np.testing.assert_allclose(preview, [[1., 0., .3]], atol=1e-8)

    def test_fast_translation_never_fuses_or_previews_a_new_scan(self):
        node = self.make_node()
        self.trajectory(node, position=lambda t: (1.5 * (t - 10), 0., 0.))
        self.send(node, [(1., 0., 0.), (5., 0., 0.)])
        self.assertEqual(node.rejection_reasons["motion_uncertain"], 1)
        self.assertEqual(node.motion_rejected_points, 2)
        self.assertFalse(node.voxels)
        self.assertEqual(node.fusion.pending_count, 0)
        self.assertFalse(node.current_publisher.messages)

    def test_after_stopping_scan_window_recovers_before_legacy_settle_window(self):
        adaptive = self.make_node()
        legacy = self.make_node("--motion-policy", "strict", "--motion-window", ".35",
                                "--motion-lookahead", ".1")
        angle = lambda t: 60. * min(0., t - 10.)
        for node in (adaptive, legacy):
            self.trajectory(node, angle=angle)
            self.send(node, [(2., 0., 0.)], seconds=10.04)
            self.assertFalse(node.voxels)
            self.send(node, [(2., 0., 0.)], seconds=10.12)
        self.assertEqual(adaptive.frames, 1)
        self.assertEqual(len(adaptive.voxels), 1)
        self.assertFalse(legacy.voxels)
        self.assertEqual(legacy.rejection_reasons["turning_or_settling"], 2)

    def test_stationary_default_confirmation_preserves_strict_geometry(self):
        # Independent actual floor and vertical wall samples, all below a 3-D
        # radius-filter threshold. The ground selector stays enabled here.
        nodes = []
        for policy in ("adaptive", "strict"):
            args = mapper.build_parser().parse_args([
                "--pcd-file", str(Path(self.directory.name) / (policy + ".pcd")),
                "--motion-policy", policy])
            nodes.append(mapper.FieldMapperNode(args))
        floor = np.array([(x, y, .006) for x in 1.013 + np.arange(12) * .05
                          for y in 1.017 + np.arange(12) * .05])
        wall = np.array([(3.013, y, z) for y in .017 + np.arange(12) * .05
                         for z in .513 + np.arange(12) * .05])
        world = np.vstack((floor, wall))
        for node in nodes:
            self.trajectory(node)
            for i in range(3):
                self.send(node, world - [0., 0., node.sensor_z], 10. + .1 * i)
        adaptive, strict = nodes
        self.assertEqual(adaptive.frames, 3)
        self.assertEqual(len(adaptive.voxels), len(world))
        self.assertEqual(adaptive.voxels, strict.voxels)
        self.assertEqual(adaptive.ground_fusion.points, strict.ground_fusion.points)
        self.assertTrue(adaptive.ground_fusion.points)
        np.testing.assert_array_equal(adaptive.map_points(), strict.map_points())

    def test_known_acquisition_motion_on_a_wall_obeys_pointwise_error_bound(self):
        node = self.make_node()
        angle = lambda t: 8. * (t - 10.)
        position = lambda t: (.05 * (t - 10.), 0., 0.)
        self.trajectory(node, angle=angle, position=position)
        world = np.array([(2.013, y, .713) for y in np.linspace(-1., 1., 25)])
        acquired = np.linspace(9.9, 10.0, len(world))
        local = []
        for point, seconds in zip(world, acquired):
            rotation = Rotation.from_euler("z", angle(seconds), degrees=True)
            local.append(rotation.inv().apply(point - position(seconds)) - [0., 0., node.sensor_z])
        local = np.asarray(local)
        match = self.match(node)
        accepted = node.motion_mask(local, match)
        self.assertTrue(accepted.all())
        assumed = Rotation.from_quat(match.quaternion).apply(local + [0., 0., node.sensor_z]) + match.position
        error = np.linalg.norm(assumed - world, axis=1)
        bound = match.translation_excursion + 2 * np.linalg.norm(local + [0., 0., node.sensor_z], axis=1) * math.sin(match.angular_excursion / 2)
        self.assertTrue(np.all(error <= bound + 1e-12))
        self.assertTrue(np.all(error[accepted] <= node.max_motion_error))
        self.send(node, local)
        self.assertEqual(len(node.voxels), len(world))
        np.testing.assert_allclose(sorted(node.voxels.values()), sorted(map(tuple, assumed)), atol=1e-6)

    def test_body_rotation_error_includes_lidar_mount_offset(self):
        node = self.make_node()
        self.trajectory(node, angle=lambda t: 100. * (t - 10.), axis="y")
        local = np.array([[0., 0., .15]])
        match = self.match(node)
        actual_error = 2 * (.15 + node.sensor_z) * math.sin(match.angular_excursion / 2)
        self.assertGreater(actual_error, node.max_motion_error)
        self.assertFalse(node.motion_mask(local, match).any())
        self.send(node, local)
        self.assertFalse(node.voxels)
        self.assertFalse(node.current_publisher.messages)

    def test_future_motion_is_checked_for_a_scan_with_start_timestamp(self):
        node = self.make_node()
        self.trajectory(node, angle=lambda t: 40. * max(0., t - 10.))
        match = self.match(node)
        self.assertGreaterEqual(match.angular_excursion, math.radians(3.9))
        self.send(node, [(2., 0., 0.)])
        self.assertEqual(node.rejection_reasons["motion_uncertain"], 1)
        self.assertFalse(node.voxels)
        self.assertFalse(node.current_publisher.messages)

    def test_invalid_timestamp_never_publishes_current_preview(self):
        node = self.make_node()
        self.trajectory(node)
        self.send(node, [(1., 0., 0.)], seconds=0.)
        self.assertEqual(node.rejection_reasons["pose_invalid_stamp"], 1)
        self.assertFalse(node.current_publisher.messages)
        self.assertFalse(node.voxels)
        self.send(node, [(1., 0., 0.)], seconds=10.)
        self.assertEqual(len(node.current_publisher.messages), 1)
        saved_preview = node.current_publisher.messages[-1].data
        self.send(node, [(20., 0., 0.)], seconds=10.)
        self.assertEqual(node.rejection_reasons["nonmonotonic_stamp"], 1)
        self.assertEqual(len(node.current_publisher.messages), 1)
        self.assertEqual(node.current_publisher.messages[-1].data, saved_preview)


if __name__ == "__main__":
    unittest.main()
