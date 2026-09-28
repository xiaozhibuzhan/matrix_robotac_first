"""Offline capture replay with real binary clouds and measured world geometry."""

from contextlib import redirect_stdout, redirect_stderr
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import finalize_mapping as finalize
from test_cloud_io import cloud, field
from test_field_mapper import Odometry, header


def binary_scan(points, stamp, times=None):
    fields = [field("x", 0), field("y", 4), field("z", 8)]
    if times is not None:
        fields.append(field("timestamp", 18, 8))
    result = cloud(points, fields=fields, point_step=26)
    result.header = header(stamp)
    if times is not None:
        data = bytearray(result.data)
        for i, timestamp in enumerate(times):
            struct.pack_into("<d", data, i * 26 + 18, timestamp)
        result.data = bytes(data)
    return result


def embedded_scan(local_points, stamp, position=(0., 0., .3), rpy=(0., 0., 0.)):
    """Build the UE simulator's two pose records plus measured rows."""
    encoded_pose = np.asarray([position[0] * 100., -position[1] * 100., position[2] * 100.])
    encoded_rpy = np.asarray([rpy[0], -rpy[1], -rpy[2]])
    all_points = np.vstack((encoded_pose, encoded_rpy, np.asarray(local_points, dtype=float)))
    fields = [field("x", 0), field("y", 4), field("z", 8), field("intensity", 12)]
    result = cloud(all_points, fields=fields, point_step=16)
    result.header = header(stamp)
    data = bytearray(result.data)
    for i, value in enumerate([111., 111.] + [255.] * len(local_points)):
        struct.pack_into("<f", data, i * 16 + 12, value)
    result.data = bytes(data)
    return result


class FinalizeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name)
        self.output = self.run / "field_map.pcd"
        self.metadata = {"mode": "capture", "bag_path": str(self.run / "raw_bag"),
                         "pcd_path": str(self.output), "capture_complete": True,
                         "mapping_args": {"outlier_radius": 0, "min_observations": 1,
                                          "disable_ground": True}}
        (self.run / "run.json").write_text(json.dumps(self.metadata), encoding="utf-8")
        (self.run / "raw_bag").mkdir()
        (self.run / "raw_bag/metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")

    def args(self, *extra):
        return finalize.parse_arguments([str(self.run), *extra])[0]

    def poses(self, velocity=.2, angular=0):
        t = np.arange(9.5, 10.51, .005)
        xyz = np.column_stack(((t - 10) * velocity, np.zeros_like(t), np.zeros_like(t)))
        q = Rotation.from_euler("z", (t - 10) * angular, degrees=True).as_quat()
        return t, xyz, q

    def test_recorded_defaults_cli_overrides_and_moved_run(self):
        self.metadata["pcd_path"] = "/old/project/maps/run_a/field_map.pcd"
        self.metadata["bag_path"] = "/old/project/maps/run_a/raw_bag"
        (self.run / "run.json").write_text(json.dumps(self.metadata), encoding="utf-8")
        args = self.args("--voxel=.07", "--min-observations=2")
        self.assertEqual(args.voxel, .07)
        self.assertEqual(args.min_observations, 2)
        self.assertEqual(args.outlier_radius, 0)
        self.assertEqual(args.pcd_file, str(self.output.resolve()))
        self.assertEqual((args.live_display, args.save_policy), ("none", "final"))

    def test_pose_loading_sorts_normalizes_and_rejects_frame_mismatch(self):
        poses = [Odometry(10.1), Odometry(9.9), Odometry(10, (1, 0, 0)),
                 Odometry(10, (2, 0, 0), (0, 0, 0, 2)), Odometry(0),
                 Odometry(10.2, quaternion=(1e308, 1e308, 1e308, 1e308))]
        times, xyz, q, counts = finalize.load_poses(poses, "odom")
        np.testing.assert_array_equal(times, [9.9, 10, 10.1])
        np.testing.assert_array_equal(xyz[1], [2, 0, 0])
        np.testing.assert_array_equal(q[1], [0, 0, 0, 1])
        self.assertEqual(counts["invalid"], 2)
        self.assertEqual(counts["duplicate_stamp"], 1)
        with self.assertRaisesRegex(ValueError, "differs"):
            finalize.load_poses(poses, "world")

    def test_offline_backend_never_creates_ros_publishers_or_timers(self):
        with mock.patch.object(finalize.mapper.FieldMapperNode, "create_publisher",
                               side_effect=AssertionError("must not create ROS publishers"), create=True), \
                mock.patch.object(finalize.mapper.FieldMapperNode, "create_subscription",
                                  side_effect=AssertionError("must not create ROS subscriptions"), create=True), \
                mock.patch.object(finalize.mapper.FieldMapperNode, "create_timer",
                                  side_effect=AssertionError("must not create ROS timers"), create=True):
            node = finalize.OfflineMapper(self.args())
            node.process_aligned(np.array([[2, 0, .4]]), np.array([[2, 0, .7]]), np.array([[2, 0, .7]]), 10)
        self.assertEqual(node.frames, 1)
        self.assertFalse(hasattr(node, "map_publisher"))
        self.assertEqual(node.live_display, "none")

    def test_offline_continuous_motion_has_no_wall_clock_expiry(self):
        args = self.args("--deskew", "off", "--max-cloud-wait", "0.000001")
        node = finalize.OfflineMapper(args)
        times, xyz, q = self.poses()
        world = np.array([[2.013, 0, .713], [2.013, .4, .713]])
        scans = [binary_scan(world - [(.2 * (stamp - 10)), 0, .3], stamp)
                 for stamp in (9.9, 10, 10.1)]
        result = finalize.process_scans(node, (times, xyz, q), scans, args)
        self.assertEqual(result["fused_scans"], 3)
        self.assertNotIn("queue_expired", result["rejected_scans"])
        np.testing.assert_allclose(node.map_points(), world, atol=1e-6)

    def test_high_speed_deskew_recovers_world_wall_without_stationary_frames(self):
        args = self.args("--deskew", "required")
        node = finalize.OfflineMapper(args)
        times, xyz, q = self.poses(velocity=1.2, angular=60)
        world = np.array([[3.013, y, .813] for y in np.linspace(-1, 1, 12)])
        scans = []
        for stamp in (9.9, 10, 10.1):
            point_times = np.linspace(stamp - .08, stamp, len(world))
            positions = np.column_stack(((point_times - 10) * 1.2, np.zeros(len(world)), np.zeros(len(world))))
            rotations = Rotation.from_euler("z", (point_times - 10) * 60, degrees=True)
            local = rotations.inv().apply(world - positions) - [0, 0, .3]
            scans.append(binary_scan(local, stamp, point_times))
        result = finalize.process_scans(node, (times, xyz, q), scans, args)
        self.assertEqual(result["deskewed_scans"], 3)
        self.assertEqual(result["fused_scans"], 3)
        self.assertFalse(result["rejected_scans"])
        actual = node.map_points()
        self.assertEqual(len(actual), len(world))
        np.testing.assert_allclose(actual[np.argsort(actual[:, 1])], world, atol=2e-6)

    def test_embedded_lidar_pose_is_preferred_and_removes_metadata_points(self):
        args = self.args("--pose-source", "auto", "--min-observations", "1")
        node = finalize.OfflineMapper(args)
        local = np.array([[1.2, .1, 0.], [1.2, .2, 0.], [1.2, .3, 0.]])
        scans = [embedded_scan(local, stamp, position=(0., 0., .3)) for stamp in (9.9, 10., 10.1)]
        result = finalize.process_scans(node, self.poses(), scans, args)
        self.assertEqual(result["embedded_pose_scans"], 3)
        self.assertEqual(result["fused_scans"], 3)
        self.assertEqual(len(node.voxels), 3)
        self.assertEqual(node.rejected_duplicates, 0)
        self.assertTrue(np.all(node.map_points()[:, 2] < .31))

    def test_continuous_motion_preserves_confirmed_sparse_ground_and_pcd_without_filling(self):
        # Exercise the real defaults rather than this suite's fast one-hit,
        # ground-disabled fixture. Sparse floor beams fail the ordinary 20 cm
        # radius filter and therefore must survive via measured ground support.
        self.metadata["mapping_args"] = {}
        (self.run / "run.json").write_text(json.dumps(self.metadata), encoding="utf-8")
        args = self.args("--deskew", "required")
        self.assertEqual(args.min_observations, 3)
        self.assertFalse(args.disable_ground)
        self.assertEqual(args.ground_min_observations, 2)
        self.assertEqual(args.outlier_radius, .2)
        node = finalize.OfflineMapper(args)
        times, xyz, q = self.poses(velocity=1.2, angular=60)

        ix, iy = np.meshgrid(np.arange(7), np.arange(7))
        observed = ~((ix == 3) & (iy == 3))  # A deliberately unmeasured hole.
        floor = np.column_stack((2.015 + ix[observed] * .3,
                                 1.015 + iy[observed] * .3,
                                 np.full(np.count_nonzero(observed), .004)))
        y, z = np.meshgrid(.013 + np.arange(20) * .05, .413 + np.arange(20) * .05)
        wall = np.column_stack((np.full(y.size, 6.013), y.ravel(), z.ravel()))
        world = np.vstack((floor, wall))
        scans = []
        for stamp in (9.8, 9.9, 10., 10.1, 10.2, 10.3):
            point_times = np.linspace(stamp - .08, stamp, len(world))
            positions = np.column_stack(((point_times - 10) * 1.2,
                                          np.zeros(len(world)), np.zeros(len(world))))
            rotations = Rotation.from_euler("z", (point_times - 10) * 60, degrees=True)
            local = rotations.inv().apply(world - positions) - [0, 0, .3]
            scans.append(binary_scan(local, stamp, point_times))

        first = finalize.process_scans(node, (times, xyz, q), scans[:1], args)
        self.assertEqual(first["deskewed_scans"], 1)
        self.assertFalse(node.voxels)
        self.assertFalse(node.ground_fusion.points)
        second = finalize.process_scans(node, (times, xyz, q), scans[1:2], args)
        self.assertFalse(second["rejected_scans"])
        self.assertFalse(node.voxels)  # The wall still needs its third scan.
        self.assertGreater(len(node.ground_fusion.points), 0)
        final = finalize.process_scans(node, (times, xyz, q), scans[2:], args)
        self.assertEqual(final["deskewed_scans"], 4)
        self.assertEqual(final["fused_scans"], 6)
        self.assertFalse(final["rejected_scans"])
        self.assertEqual(len(node.voxels), len(wall))
        self.assertEqual(node.ground_scan_counts["band_after_radius"], 0)

        ground = np.array(list(node.ground_fusion.points.values()))
        self.assertGreaterEqual(len(ground), int(.85 * len(floor)))
        self.assertLessEqual(len(ground), len(floor))
        distance_to_floor = np.linalg.norm(ground[:, None] - floor[None], axis=2).min(axis=1)
        self.assertTrue(np.all(distance_to_floor < 2e-6))
        # Retain the measured 4 mm elevation; never project onto an invented
        # zero-height plane or manufacture the deliberately absent centre.
        np.testing.assert_allclose(ground[:, 2], .004, atol=1e-6)
        hole = np.array([2.915, 1.915, .004])
        self.assertTrue(np.all(np.linalg.norm(ground-hole, axis=1) > .25))

        success, message = node.save_pcd()
        self.assertTrue(success, message)
        pcd_header, payload = self.output.read_bytes().split(b"DATA binary\n", 1)
        saved = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
        self.assertIn(f"POINTS {len(saved)}\n".encode(), pcd_header)
        self.assertEqual(len(saved), len(wall) + len(ground))
        distance_to_observed = np.linalg.norm(saved[:, None] - world[None], axis=2).min(axis=1)
        self.assertTrue(np.all(distance_to_observed < 2e-6))
        saved_floor = saved[saved[:, 2] < .1]
        self.assertEqual(len(saved_floor), len(ground))
        self.assertTrue(np.all(np.linalg.norm(saved_floor-hole, axis=1) > .25))

    def test_unsupported_timing_does_not_silently_accept_fast_motion(self):
        args = self.args()
        node = finalize.OfflineMapper(args)
        result = finalize.process_scans(node, self.poses(velocity=1.2),
                                        [binary_scan([[3, 0, .5]], 10)], args)
        self.assertEqual(result["rejected_scans"]["motion_uncertain"], 1)
        self.assertEqual(result["fused_scans"], 0)
        self.assertTrue(result["timing_reasons"])

    def test_required_timing_rejects_instead_of_fallback(self):
        args = self.args("--deskew", "required")
        node = finalize.OfflineMapper(args)
        result = finalize.process_scans(node, self.poses(velocity=0),
                                        [binary_scan([[3, 0, .5]], 10)], args)
        self.assertEqual(result["fused_scans"], 0)
        self.assertTrue(any(k.startswith("deskew_") for k in result["rejected_scans"]))

    def test_progress_remains_visible_when_every_scan_is_rejected(self):
        args = self.args()
        node = finalize.OfflineMapper(args)
        with mock.patch.object(finalize.time, "monotonic", side_effect=[0, 6, 6, 12, 12]), \
                redirect_stdout(io.StringIO()) as output:
            result = finalize.process_scans(node, self.poses(),
                                            [binary_scan([[3, 0, .5]], 0) for _ in range(2)], args)
        self.assertEqual(result["fused_scans"], 0)
        self.assertEqual(result["rejected_scans"]["pose_invalid_stamp"], 2)
        self.assertIn("Finalizing: read=2", output.getvalue())

    def test_main_replays_two_topics_writes_pcd_report_and_log_without_ros_node(self):
        t, xyz, q = self.poses(0)
        odometry = [Odometry(stamp, position, quat) for stamp, position, quat in zip(t, xyz, q)]
        for message in odometry:
            message.header.frame_id = "world"
        scans = [binary_scan([[2.013, 0, .713]], stamp) for stamp in (9.9, 10, 10.1)]
        calls = []

        def source(bag, topic, kind):
            calls.append((topic, kind))
            return iter(odometry if kind == "nav_msgs/msg/Odometry" else scans)

        with mock.patch.object(finalize, "iter_bag", side_effect=source), redirect_stdout(io.StringIO()):
            code = finalize.main([str(self.run)])
        self.assertEqual(code, 0)
        self.assertEqual(len(calls), 2)
        report = json.loads((self.run / "finalize_report.json").read_text())
        self.assertTrue(report["success"])
        self.assertEqual(report["processing"]["fused_scans"], 3)
        self.assertEqual(report["points"], 1)
        self.assertIn(b"POINTS 1\n", self.output.read_bytes())
        self.assertIn("Reading capture", (self.run / "finalize.log").read_text())

    def test_no_poses_or_incomplete_capture_never_replace_existing_pcd(self):
        self.output.write_bytes(b"previous good map")
        with mock.patch.object(finalize, "iter_bag", return_value=iter(())), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(finalize.main([str(self.run)]), 1)
        self.assertEqual(self.output.read_bytes(), b"previous good map")
        self.metadata["capture_complete"] = False
        (self.run / "run.json").write_text(json.dumps(self.metadata), encoding="utf-8")
        with mock.patch.object(finalize, "iter_bag") as reader, redirect_stderr(io.StringIO()):
            self.assertEqual(finalize.main([str(self.run)]), 1)
        reader.assert_not_called()

    def test_malformed_run_metadata_reports_failure_without_starting_replay(self):
        self.output.write_bytes(b"previous good map")
        for content in ([], {"mode": "capture", "mapping_args": []},
                        {"mode": "capture", "pcd_path": None}):
            with self.subTest(content=content):
                (self.run / "run.json").write_text(json.dumps(content), encoding="utf-8")
                with mock.patch.object(finalize, "iter_bag") as reader, redirect_stderr(io.StringIO()) as errors:
                    self.assertEqual(finalize.main([str(self.run)]), 1)
                reader.assert_not_called()
                self.assertIn("Cannot finalize", errors.getvalue())
        self.assertEqual(self.output.read_bytes(), b"previous good map")

    def test_log_open_failure_is_reported_before_replay(self):
        (self.run / "finalize.log").mkdir()
        with mock.patch.object(finalize, "iter_bag") as reader, redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(finalize.main([str(self.run)]), 1)
        reader.assert_not_called()
        self.assertIn("Cannot open finalization log", errors.getvalue())

    def test_report_write_failure_is_reported_without_replacing_old_pcd(self):
        self.output.write_bytes(b"previous good map")
        (self.run / "finalize_report.json.tmp").mkdir()
        with mock.patch.object(finalize, "iter_bag", return_value=iter(())), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(finalize.main([str(self.run)]), 1)
        self.assertIn("Cannot write finalization report", errors.getvalue())
        self.assertEqual(self.output.read_bytes(), b"previous good map")

    def test_failed_log_write_keeps_terminal_diagnostics_and_reports_failure(self):
        class FullDisk(io.StringIO):
            def write(self, text):
                raise OSError("no space left on device")

        real_open = Path.open
        failed_log = FullDisk()

        def open_file(path, *args, **kwargs):
            return failed_log if path.name == "finalize.log" else real_open(path, *args, **kwargs)

        with mock.patch.object(Path, "open", new=open_file), \
                mock.patch.object(finalize, "iter_bag", return_value=iter(())), \
                redirect_stdout(io.StringIO()) as output, redirect_stderr(io.StringIO()):
            self.assertEqual(finalize.main([str(self.run)]), 1)
        self.assertIn("no space left on device", output.getvalue())
        report = json.loads((self.run / "finalize_report.json").read_text())
        self.assertFalse(report["success"])
        self.assertEqual(report["log_error"], "no space left on device")

    def test_missing_rosbag_python_support_produces_an_actionable_error(self):
        with mock.patch.dict(sys.modules, {"rosbag2_py": None}):
            messages = finalize.iter_bag(self.run / "raw_bag", "/cloud", "sensor_msgs/msg/PointCloud2")
            with self.assertRaisesRegex(RuntimeError, "ros-humble-rosbag2-py"):
                next(messages)

    def test_bag_topic_iterator_uses_recorded_types_and_topic_filter(self):
        # Check the Humble reader contract; this does not simulate real CDR IO.
        from types import SimpleNamespace
        reader = mock.Mock()
        reader.get_all_topics_and_types.return_value = [SimpleNamespace(name="/cloud", type="sensor_msgs/msg/PointCloud2")]
        reader.has_next.side_effect = [True, False]
        reader.read_next.return_value = ("/cloud", b"cdr", 123)
        bag_module = SimpleNamespace(SequentialReader=lambda: reader, StorageOptions=lambda **k: k,
                                     ConverterOptions=lambda *a: a, StorageFilter=lambda **k: k)
        message_type = object()
        serialization = SimpleNamespace(deserialize_message=lambda data, cls: (data, cls))
        utility = SimpleNamespace(get_message=lambda name: message_type)
        with mock.patch.dict(sys.modules, {"rosbag2_py": bag_module, "rclpy.serialization": serialization,
                                          "rosidl_runtime_py.utilities": utility}):
            messages = list(finalize.iter_bag(self.run / "raw_bag", "/cloud", "sensor_msgs/msg/PointCloud2"))
        self.assertEqual(messages, [(b"cdr", message_type)])
        reader.set_filter.assert_called_once_with({"topics": ["/cloud"]})

    def test_bag_topic_reader_remains_lazy_and_rejects_wrong_recorded_type(self):
        from types import SimpleNamespace
        reader = mock.Mock()
        reader.get_all_topics_and_types.return_value = [SimpleNamespace(name="/cloud", type="sensor_msgs/msg/PointCloud2")]
        reader.has_next.return_value = True
        reader.read_next.side_effect = [("/cloud", b"first", 1), ("/cloud", b"second", 2)]
        bag_module = SimpleNamespace(SequentialReader=mock.Mock(return_value=reader), StorageOptions=lambda **k: k,
                                     ConverterOptions=lambda *a: a, StorageFilter=lambda **k: k)
        deserialize = mock.Mock(side_effect=lambda data, cls: data)
        modules = {"rosbag2_py": bag_module,
                   "rclpy.serialization": SimpleNamespace(deserialize_message=deserialize),
                   "rosidl_runtime_py.utilities": SimpleNamespace(get_message=lambda name: object)}
        with mock.patch.dict(sys.modules, modules):
            messages = finalize.iter_bag(self.run / "raw_bag", "/cloud", "sensor_msgs/msg/PointCloud2")
            bag_module.SequentialReader.assert_not_called()
            self.assertEqual(next(messages), b"first")
            self.assertEqual(reader.read_next.call_count, 1)
            messages.close()
            self.assertEqual(deserialize.call_count, 1)
            with self.assertRaisesRegex(ValueError, "must have type"):
                next(finalize.iter_bag(self.run / "raw_bag", "/cloud", "nav_msgs/msg/Odometry"))
            self.assertEqual(deserialize.call_count, 1)


if __name__ == "__main__":
    unittest.main()
