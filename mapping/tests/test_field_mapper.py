"""Exercise mapping logic with lightweight ROS messages on a non-ROS machine."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

import numpy as np


SOURCE = Path(__file__).resolve().parents[1] / "field_mapper_node.py"
sys.path.insert(0, str(SOURCE.parent))


def stamp(seconds=0.0):
    whole = math.floor(seconds)
    return SimpleNamespace(sec=whole, nanosec=round((seconds - whole) * 1e9))


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def header(seconds=0.0, frame="lidar"):
    return SimpleNamespace(stamp=stamp(seconds), frame_id=frame)


class PointCloud2:
    def __init__(self, points=(), seconds=0.0):
        self.header = header(seconds)
        self.points = points
        self.width = len(points) if hasattr(points, "__len__") else 0
        self.height = 1
        self.fields = []
        self.data = b""
        self.point_step = 0
        self.row_step = 0
        self.is_bigendian = False
        self.is_dense = False


class PointField:
    FLOAT32 = 7

    def __init__(self, name="", offset=0, datatype=7, count=1):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


class Odometry:
    def __init__(self, seconds=0.0, position=(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 0.0, 1.0)):
        self.header = header(seconds, "odom")
        self.child_frame_id = "base_link"
        self.pose = SimpleNamespace(pose=SimpleNamespace(
            position=vector(*position),
            orientation=SimpleNamespace(**dict(zip(("x", "y", "z", "w"), quaternion))),
        ))


class Marker:
    ADD = 0
    DELETEALL = 3
    CUBE = 1
    CUBE_LIST = 6

    def __init__(self):
        self.header = header(frame="")
        self.pose = SimpleNamespace(position=vector(), orientation=SimpleNamespace(w=0.0))
        self.scale = vector()
        self.color = SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
        self.action = self.ADD
        self.points = []


class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class MarkerArray:
    def __init__(self):
        self.markers = []


class Logger:
    def __init__(self):
        self.entries = []

    def info(self, message):
        self.entries.append(("info", message))

    def warning(self, message):
        self.entries.append(("warning", message))

    def error(self, message):
        self.entries.append(("error", message))


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self, name):
        self.name = name
        self.logger = Logger()
        self.timers = []

    def create_publisher(self, *args, **kwargs):
        return Publisher()

    def create_subscription(self, *args, **kwargs):
        return args

    def create_service(self, *args, **kwargs):
        return args

    def create_timer(self, interval, callback, **kwargs):
        self.timers.append((interval, callback))
        return self.timers[-1]

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp(100.0)))


def read_points(message, field_names, skip_nans):
    if field_names != ("x", "y", "z"):
        raise AssertionError("The mapper must request XYZ fields")
    return message.points


def create_cloud_xyz32(cloud_header, points):
    cloud = PointCloud2(list(points))
    cloud.header = cloud_header
    return cloud


def load_mapper():
    """Keep fake ROS imports scoped so discovery does not affect other tests."""
    # Keep real numeric-library imports outside the temporary ROS module table.
    import point_filters
    import pose_buffer
    import voxel_fusion

    modules = {}
    for name in (
        "rclpy", "rclpy.node", "rclpy.executors", "rclpy.callback_groups", "rclpy.qos", "nav_msgs", "nav_msgs.msg",
        "sensor_msgs", "sensor_msgs.msg", "sensor_msgs_py", "sensor_msgs_py.point_cloud2",
        "std_srvs", "std_srvs.srv", "visualization_msgs", "visualization_msgs.msg",
        "geometry_msgs", "geometry_msgs.msg",
    ):
        modules[name] = ModuleType(name)
    for name, module in modules.items():
        parent, _, child = name.rpartition(".")
        if parent:
            setattr(modules[parent], child, module)
    modules["rclpy.node"].Node = Node
    modules["rclpy.executors"].ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
    modules["rclpy.executors"].MultiThreadedExecutor = mock.Mock()
    modules["rclpy.callback_groups"].MutuallyExclusiveCallbackGroup = type("MutuallyExclusiveCallbackGroup", (), {})
    modules["rclpy.callback_groups"].ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})
    qos = modules["rclpy.qos"]
    qos.QoSProfile = lambda **kwargs: SimpleNamespace(**kwargs)
    qos.HistoryPolicy = SimpleNamespace(KEEP_LAST="keep_last")
    qos.ReliabilityPolicy = SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort")
    qos.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL="transient_local", VOLATILE="volatile")
    qos.qos_profile_sensor_data = object()
    modules["nav_msgs.msg"].Odometry = Odometry
    modules["sensor_msgs.msg"].PointCloud2 = PointCloud2
    modules["sensor_msgs.msg"].PointField = PointField
    modules["sensor_msgs_py.point_cloud2"].read_points = read_points
    modules["sensor_msgs_py.point_cloud2"].create_cloud_xyz32 = create_cloud_xyz32
    modules["std_srvs.srv"].Trigger = SimpleNamespace(Request=SimpleNamespace, Response=SimpleNamespace)
    modules["visualization_msgs.msg"].Marker = Marker
    modules["visualization_msgs.msg"].MarkerArray = MarkerArray
    modules["geometry_msgs.msg"].Point = Point
    modules["rclpy"].init = mock.Mock()
    modules["rclpy"].spin = mock.Mock()
    modules["rclpy"].try_shutdown = mock.Mock()
    spec = importlib.util.spec_from_file_location("field_mapper_test_subject", SOURCE)
    mapper = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(mapper)
    return mapper


mapper = load_mapper()


class StructuredPoint:
    """Model an iterable structured-array record without requiring NumPy."""

    def __init__(self, *values):
        self.values = values

    def __iter__(self):
        return iter(self.values)


class FieldMapperTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "map.pcd"
        args = mapper.build_parser().parse_args([
            "--pcd-file", str(self.path), "--global-frame", "odom",
            "--min-range", "0", "--max-range", "0", "--outlier-radius", "0",
            "--min-observations", "1",
            "--motion-policy", "strict", "--motion-window", "0.35", "--motion-lookahead", "0.10",
        ])
        self.node = mapper.FieldMapperNode(args)
        self.next_stamp = 10.0

    def add_stationary_poses(self, seconds, position=(0, 0, 0), quaternion=(0, 0, 0, 1), frame="odom"):
        for offset in range(-8, 4):
            pose = Odometry(seconds + offset * 0.05, position, quaternion)
            pose.header.frame_id = frame
            self.node.odom_callback(pose)

    def add_cloud(self, points, seconds=None, position=(0, 0, 0), quaternion=(0, 0, 0, 1), frame="odom"):
        if seconds is None:
            seconds = self.next_stamp
            self.next_stamp += 0.1
        self.add_stationary_poses(seconds, position, quaternion, frame)
        self.node.cloud_callback(PointCloud2(points, seconds))
        self.node.process_pending()

    def read_pcd(self):
        return self.path.read_bytes().split(b"DATA binary\n", 1)

    def test_finite_xyz_accepts_iterables_and_rejects_bad_points(self):
        self.assertEqual(mapper.finite_xyz(iter((1, "2", 3))), (1.0, 2.0, 3.0))
        self.assertEqual(mapper.finite_xyz(StructuredPoint(1, 2, 3)), (1.0, 2.0, 3.0))
        for value in (None, (), (1, 2), (1, 2, 3, 4), (1, "bad", 3),
                      (1, math.inf, 3), (math.nan, 2, 3)):
            with self.subTest(value=value):
                self.assertIsNone(mapper.finite_xyz(value))

    def test_transform_applies_sensor_height_before_body_rotation(self):
        half = math.sqrt(0.5)
        self.add_cloud(iter([(1, 2, 3), (1, 2, 3)]), position=(10, 20, 30), quaternion=(half, 0, 0, half))
        self.assertEqual(len(self.node.voxels), 1)
        for actual, expected in zip(next(iter(self.node.voxels.values())), (11, 16.7, 32)):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.node.frames, 1)
        self.assertTrue(self.node.dirty)

    def test_cloud_accepts_structured_records_and_ignores_invalid_points(self):
        self.add_cloud([StructuredPoint(1, 2, 3), StructuredPoint(1, 2, 3),
                        StructuredPoint(math.nan, 0, 0), StructuredPoint(0, math.inf, 0)])
        self.assertEqual(list(self.node.voxels.values()), [(1.0, 2.0, 3.3)])

    def test_cloud_uses_interpolated_position_with_complete_motion_window(self):
        for tick in range(192, 205):
            seconds = tick * 0.05
            self.node.odom_callback(Odometry(seconds, (0.1 * (seconds - 10), 0, 0)))
        self.node.cloud_callback(PointCloud2([(1, 0, 0)], 10.025))
        self.node.process_pending()
        self.assertEqual(len(self.node.voxels), 1)
        point = next(iter(self.node.voxels.values()))
        self.assertAlmostEqual(point[0], 1.0025)
        self.assertAlmostEqual(self.node.last_pose_span, 0.05)

    def test_unsynchronized_cloud_does_not_modify_map(self):
        with mock.patch.object(mapper.time, "monotonic", return_value=100.0):
            self.node.odom_callback(Odometry(1.0))
            self.node.cloud_callback(PointCloud2([(1, 2, 3)], 10.0))
            self.node.process_pending()
        self.assertEqual(len(self.node.cloud_queue), 1)
        with mock.patch.object(mapper.time, "monotonic", return_value=101.0):
            self.node.process_pending()
        self.assertEqual(self.node.skipped_frames, 1)
        self.assertFalse(self.node.voxels)
        self.assertFalse(self.node.dirty)

    def test_publish_uses_global_frame_and_height_classification(self):
        self.add_cloud([(0.02, 0.02, 0), (0.03, 0.03, 1), (1.02, 1.02, 0)])
        self.node.publish_timer()
        cloud = self.node.map_publisher.messages[-1]
        self.assertEqual(cloud.header.frame_id, "odom")
        self.assertEqual(cloud.width, 3)
        self.assertEqual(cloud.height, 1)
        self.assertEqual(cloud.point_step, 12)
        self.assertEqual(cloud.row_step, 36)
        self.assertEqual([(field.name, field.offset, field.datatype, field.count) for field in cloud.fields],
                         [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)])
        published = np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3)
        np.testing.assert_allclose(published, list(self.node.voxels.values()), rtol=1e-6)
        for publisher in (self.node.passable_publisher, self.node.structure_publisher):
            markers = publisher.messages[-1].markers
            self.assertEqual(len(markers), 2)
            self.assertEqual(markers[0].action, Marker.DELETEALL)
            self.assertTrue(all(item.header.frame_id == "odom" for item in markers))
            self.assertEqual(markers[1].type, Marker.CUBE_LIST)
            self.assertGreater(markers[1].scale.x, 0.0)
            self.assertLessEqual(markers[1].scale.x, 0.1)
            self.assertEqual(markers[1].scale.x, markers[1].scale.y)
            self.assertEqual(markers[1].scale.y, markers[1].scale.z)
        structure = self.node.structure_publisher.messages[-1].markers[1]
        passable = self.node.passable_publisher.messages[-1].markers[1]
        self.assertEqual(len(structure.points), 2)
        self.assertEqual(len(passable.points), 1)
        for actual, expected in zip(sorted(point.z for point in structure.points), (0.3, 1.3)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(passable.points[0].z, 0.3)

    def test_markers_do_not_fill_empty_height_gap_with_a_tall_column(self):
        self.add_cloud([(2, 2, 0), (2, 2, 100)])
        self.node.publish_map()
        markers = self.node.make_markers("structure").markers
        self.assertEqual(len(markers), 2)
        batch = markers[1]
        self.assertEqual(batch.type, Marker.CUBE_LIST)
        self.assertLessEqual(batch.scale.z, 0.1)
        self.assertEqual(len(batch.points), 2)
        self.assertEqual(sorted(round(point.z, 5) for point in batch.points), [0.3, 100.3])
        self.assertTrue(all(point.z < 0.4 or point.z > 100.2 for point in batch.points))

    def test_default_filters_remove_bad_points_before_map_cells_and_pcd(self):
        args = mapper.build_parser().parse_args([
            "--pcd-file", str(self.path), "--sensor-z", "0", "--voxel", "0.01",
            "--min-observations", "1",
        ])
        self.node = mapper.FieldMapperNode(args)
        wall = [(2.0, index * 0.05, height * 0.05) for index in range(5) for height in range(5)]
        isolated = [(20, 20, 20), (1000, 0, 0), (0, 0, 0)]
        repeated_bad = [(10, 10, 10)] * 10
        self.add_cloud(wall + isolated + repeated_bad, position=(100, 100, 0), frame="world")
        self.assertEqual(len(self.node.voxels), len(wall))
        self.assertEqual(self.node.frames, 1)
        for point in self.node.voxels.values():
            self.assertAlmostEqual(point[0], 102.0)
            self.assertGreaterEqual(point[1], 100.0)
            self.assertLessEqual(point[1], 100.21)
            self.assertGreaterEqual(point[2], 0.0)
            self.assertLessEqual(point[2], 0.21)
        self.node.publish_map()
        for (ix, iy), cell in self.node.cells.items():
            self.assertGreaterEqual(ix * self.node.cell_size, 101.8)
            self.assertLessEqual(iy * self.node.cell_size, 100.21)
            self.assertLessEqual(cell["max_z"], 0.21)
        self.assertTrue(self.node.save_pcd()[0])
        pcd_header, payload = self.read_pcd()
        self.assertIn(f"POINTS {len(wall)}\n".encode("ascii"), pcd_header)
        points = list(struct.iter_unpack("<fff", payload))
        self.assertEqual(len(points), len(wall))
        self.assertTrue(all(abs(point[0] - 102.0) < 1e-5 and point[2] <= 0.21 for point in points))
        self.node.publish_timer()
        self.assertEqual(self.node.map_publisher.messages[-1].header.frame_id, "world")

    def test_default_filters_preserve_observed_thin_pole(self):
        args = mapper.build_parser().parse_args([
            "--pcd-file", str(self.path), "--sensor-z", "0", "--voxel", "0.01",
            "--min-observations", "1",
        ])
        self.node = mapper.FieldMapperNode(args)
        pole = [(2.0, 0.0, index * 0.05) for index in range(21)]
        self.add_cloud(pole)
        self.assertEqual(len(self.node.voxels), len(pole))
        observed = sorted(point[2] for point in self.node.voxels.values())
        for actual, expected in zip(observed, (point[2] for point in pole)):
            self.assertAlmostEqual(actual, expected)
        self.node.publish_map()
        structure = self.node.make_markers("structure").markers[1]
        self.assertEqual(structure.type, Marker.CUBE_LIST)
        self.assertLessEqual(structure.scale.z, 0.1)
        self.assertGreaterEqual(len(structure.points), 5)
        for point in structure.points:
            self.assertAlmostEqual(point.x, 2.0)
            self.assertAlmostEqual(point.y, 0.0)
            self.assertTrue(any(abs(point.z - observed_z) < 1e-6 for observed_z in observed))

    def test_invalid_pose_values_are_rejected_without_poisoning_history(self):
        invalid = (
            Odometry(10.0, position=(math.nan, 0, 0)),
            Odometry(10.0, position=(0, math.inf, 0)),
            Odometry(10.0, quaternion=(0, 0, math.nan, 1)),
            Odometry(10.0, quaternion=(0, 0, 0, 0)),
        )
        for message in invalid:
            with self.subTest(pose=message.pose):
                self.node.odom_callback(message)
                self.assertEqual(self.node.pose_buffer.sample_count, 0)
                self.assertIsNone(self.node.pose_buffer.lookup(10.0)[0])
        self.node.cloud_callback(PointCloud2([(1, 0, 0)], 10.0))
        self.assertFalse(self.node.voxels)
        self.assertFalse(self.node.dirty)
        self.node.odom_callback(Odometry(10.0))
        self.assertEqual(self.node.pose_buffer.sample_count, 1)

    def test_pose_quaternion_is_normalized_before_rotating_points(self):
        twice_half = 2 * math.sqrt(0.5)
        self.add_cloud([(1, 0, 0)], quaternion=(0, 0, twice_half, twice_half))
        pose, _ = self.node.pose_buffer.lookup(10.0)
        self.assertAlmostEqual(sum(value * value for value in pose.quaternion), 1.0)
        point = next(iter(self.node.voxels.values()))
        for actual, expected in zip(point, (0.0, 1.0, 0.3)):
            self.assertAlmostEqual(actual, expected)

    def test_default_frame_and_filter_parameters_match_simulator_mapping(self):
        args = mapper.build_parser().parse_args([])
        self.assertEqual(args.global_frame, "world")
        self.assertEqual(args.min_range, 0.1)
        self.assertEqual(args.max_range, 60.0)
        self.assertEqual(args.outlier_radius, 0.2)
        self.assertEqual(args.min_neighbors, 2)
        self.assertEqual(args.voxel, 0.03)
        self.assertEqual(args.max_pose_gap, 0.10)
        self.assertEqual(args.motion_window, 0.10)
        self.assertEqual(args.motion_lookahead, 0.10)
        self.assertEqual(args.motion_policy, "adaptive")
        self.assertEqual(args.max_motion_error, 0.05)
        self.assertEqual(args.max_angular_speed, 5.0)
        self.assertEqual(args.max_linear_speed, 0.5)
        self.assertEqual(args.min_observations, 3)

    def test_callback_groups_keep_input_separate_from_heavy_map_work(self):
        self.assertEqual(len({id(self.node.odom_group), id(self.node.cloud_group), id(self.node.work_group)}), 3)
        self.add_stationary_poses(10.0)
        with mock.patch.object(self.node, "process_cloud", wraps=self.node.process_cloud) as process:
            self.node.cloud_callback(PointCloud2([(1, 0, 0)], 10.0))
            process.assert_not_called()
            self.assertFalse(self.node.voxels)
            self.node.process_pending()
            process.assert_called_once()

    def test_cloud_waits_for_future_odometry_then_fuses(self):
        for tick in range(192, 201):
            self.node.odom_callback(Odometry(tick * 0.05))
        with mock.patch.object(mapper.time, "monotonic", return_value=100.0):
            self.node.cloud_callback(PointCloud2([(1, 0, 0)], 10.0))
            self.node.process_pending()
        self.assertEqual(len(self.node.cloud_queue), 1)
        self.assertEqual(self.node.last_reason, "waiting_need_future")
        self.assertFalse(self.node.voxels)
        self.assertEqual(self.node.skipped_frames, 0)
        self.node.odom_callback(Odometry(10.05))
        self.node.odom_callback(Odometry(10.10))
        with mock.patch.object(mapper.time, "monotonic", return_value=100.1):
            self.node.process_pending()
        self.assertFalse(self.node.cloud_queue)
        self.assertEqual(self.node.frames, 1)
        self.assertEqual(len(self.node.voxels), 1)

    def test_motion_window_odometry_gap_rejects_entire_cloud(self):
        for tick in range(190, 205):
            if tick not in (195, 196):
                self.node.odom_callback(Odometry(tick * 0.05))
        self.node.cloud_callback(PointCloud2([(1, 0, 0)], 10.0))
        self.node.process_pending()
        self.assertEqual(self.node.rejection_reasons["pose_gap"], 1)
        self.assertFalse(self.node.cloud_queue)
        self.assertFalse(self.node.voxels)

    def test_fast_turn_and_settling_clouds_are_skipped_then_static_scan_recovers(self):
        for tick in range(192, 219):
            seconds = tick * 0.05
            angle = math.radians(max(0.0, min(30.0, (seconds - 10.0) * 150.0)))
            self.node.odom_callback(Odometry(seconds, quaternion=(0, 0, math.sin(angle / 2), math.cos(angle / 2))))
        for seconds in (10.1, 10.3):
            self.node.cloud_callback(PointCloud2([(1, 0, 0)], seconds))
            self.node.process_pending()
            self.assertFalse(self.node.voxels)
        self.assertEqual(self.node.rejection_reasons["turning_or_settling"], 2)
        self.node.cloud_callback(PointCloud2([(1, 0, 0)], 10.7))
        self.node.process_pending()
        self.assertEqual(self.node.frames, 1)
        self.assertEqual(self.node.last_reason, "accepted")
        np.testing.assert_allclose(next(iter(self.node.voxels.values())), [math.cos(math.pi / 6), 0.5, 0.3], atol=1e-7)

    def test_fast_translation_is_skipped_until_motion_window_is_stable(self):
        for tick in range(192, 219):
            seconds = tick * 0.05
            position = max(0.0, min(0.2, seconds - 10.0))
            self.node.odom_callback(Odometry(seconds, position=(position, 0, 0)))
        for seconds in (10.1, 10.3):
            self.node.cloud_callback(PointCloud2([(1, 0, 0)], seconds))
            self.node.process_pending()
        self.assertEqual(self.node.rejection_reasons["moving_fast_or_settling"], 2)
        self.assertFalse(self.node.voxels)
        self.node.cloud_callback(PointCloud2([(1, 0, 0)], 10.7))
        self.node.process_pending()
        self.assertEqual(len(self.node.voxels), 1)

    def test_three_distinct_scans_confirm_voxels_without_counting_duplicate_stamps(self):
        args = mapper.build_parser().parse_args([
            "--pcd-file", str(self.path), "--global-frame", "odom",
            "--min-range", "0", "--max-range", "0", "--outlier-radius", "0",
        ])
        self.node = mapper.FieldMapperNode(args)
        self.add_cloud([(1, 2, 3)] * 20, seconds=10.0)
        self.assertFalse(self.node.voxels)
        self.assertEqual(self.node.fusion.pending_count, 1)
        self.add_cloud([(1, 2, 3)] * 20, seconds=10.0)
        self.assertEqual(self.node.rejection_reasons["nonmonotonic_stamp"], 1)
        self.assertFalse(self.node.voxels)
        self.add_cloud([(1, 2, 3)], seconds=10.1)
        self.assertFalse(self.node.voxels)
        self.assertFalse(self.node.dirty)
        self.add_cloud([(1, 2, 3)], seconds=10.2)
        self.assertEqual(len(self.node.voxels), 1)
        self.assertEqual(self.node.frames, 3)
        self.assertEqual(self.node.fusion.pending_count, 0)
        self.assertTrue(self.node.dirty)

    def test_surface_sample_tracks_fused_mean_without_duplicate_at_coarse_boundary(self):
        args = mapper.build_parser().parse_args([
            "--pcd-file", str(self.path), "--global-frame", "odom", "--sensor-z", "0",
            "--min-range", "0", "--max-range", "0", "--outlier-radius", "0",
        ])
        self.node = mapper.FieldMapperNode(args)
        for index, x in enumerate((0.199, 0.199, 0.199, 0.209, 0.209)):
            self.add_cloud([(x, 0.10, 0.30)], seconds=10.0 + index * 0.1)
        self.assertEqual(len(self.node.voxels), 1)
        fine_key, fused = next(iter(self.node.voxels.items()))
        self.assertEqual(fine_key, (6, 3, 10))
        self.assertAlmostEqual(fused[0], 0.203, places=7)
        self.assertEqual(list(self.node.surface_samples.values()), [fused])
        self.assertEqual(list(self.node.surface_owners.values()), [fine_key])
        self.node.publish_map()
        visible_points = [point for publisher in (self.node.passable_publisher, self.node.structure_publisher)
                          for marker in publisher.messages[-1].markers for point in marker.points]
        self.assertEqual(len(visible_points), 1)
        self.assertEqual((visible_points[0].x, visible_points[0].y, visible_points[0].z), fused)
        response = self.node.clear_callback(None, SimpleNamespace())
        self.assertTrue(response.success, response.message)
        self.assertFalse(self.node.surface_owners)
        self.assertFalse(self.node.surface_samples)

    def test_older_cloud_cannot_add_points_after_newer_cloud_was_processed(self):
        self.add_cloud([(1, 2, 3)], seconds=10.0)
        before = dict(self.node.voxels)
        self.add_cloud([(7, 8, 9)], seconds=9.9)
        self.assertEqual(self.node.voxels, before)
        self.assertEqual(self.node.rejection_reasons["nonmonotonic_stamp"], 1)

    def test_cloud_queue_overflow_drops_oldest_and_processes_at_most_one_per_tick(self):
        self.node.max_queued_clouds = 2
        self.add_stationary_poses(10.0)
        self.add_stationary_poses(10.2)
        for index, seconds in enumerate((10.0, 10.1, 10.2)):
            self.node.cloud_callback(PointCloud2([(1 + index, 0, 0)], seconds))
        self.assertEqual(len(self.node.cloud_queue), 2)
        self.assertEqual(self.node.rejection_reasons["queue_overflow"], 1)
        self.node.process_pending()
        self.assertEqual(len(self.node.cloud_queue), 1)
        self.assertEqual(self.node.frames, 1)
        self.node.process_pending()
        self.assertFalse(self.node.cloud_queue)
        self.assertEqual(self.node.frames, 2)
        self.assertEqual(sorted(point[0] for point in self.node.voxels.values()), [2, 3])

    def test_expired_queue_item_is_rejected_even_if_odometry_is_now_available(self):
        with mock.patch.object(mapper.time, "monotonic", return_value=100.0):
            self.node.cloud_callback(PointCloud2([(1, 0, 0)], 10.0))
        self.add_stationary_poses(10.0)
        with mock.patch.object(mapper.time, "monotonic", return_value=101.0):
            self.node.process_pending()
        self.assertFalse(self.node.voxels)
        self.assertEqual(self.node.rejection_reasons["queue_expired"], 1)

    def test_backlog_uses_latest_pose_ready_scan_without_waiting_on_old_scans(self):
        self.node.queue_policy = "latest"
        for seconds in (10.0, 10.1, 10.2):
            self.add_stationary_poses(seconds)
            self.node.cloud_callback(PointCloud2([(seconds, 0, 0)], seconds))
        self.node.process_pending()
        self.assertEqual(self.node.rejection_reasons["queue_superseded"], 2)
        self.assertEqual(self.node.frames, 1)
        self.assertAlmostEqual(next(iter(self.node.voxels.values()))[0], 10.2, places=5)
        self.assertFalse(self.node.cloud_queue)

    def test_expired_backlog_is_drained_in_one_tick_then_fresh_scan_is_processed(self):
        with mock.patch.object(mapper.time, "monotonic", return_value=100):
            self.node.cloud_callback(PointCloud2([(1, 0, 0)], 9.0))
            self.node.cloud_callback(PointCloud2([(2, 0, 0)], 9.1))
        self.add_stationary_poses(10.0)
        with mock.patch.object(mapper.time, "monotonic", return_value=101):
            self.node.cloud_callback(PointCloud2([(3, 0, 0)], 10.0))
            self.node.process_pending()
        self.assertEqual(self.node.rejection_reasons["queue_expired"], 2)
        self.assertEqual(self.node.frames, 1)
        self.assertEqual(next(iter(self.node.voxels.values()))[0], 3)

    def test_structured_numpy_cloud_with_extra_fields_uses_only_xyz(self):
        points = np.array([(1, 2, 3, 50), (4, 5, 6, 75)],
                          dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
        self.add_cloud(points)
        np.testing.assert_allclose(list(self.node.voxels.values()), [(1, 2, 3.3), (4, 5, 6.3)], atol=1e-7)

    def test_malformed_cloud_payload_is_rejected_and_next_scan_recovers(self):
        self.add_cloud([(1, 2, 3)], seconds=10.0)
        failures = (struct.error("PointCloud2 payload is truncated"),
                    AssertionError("Requested XYZ fields are not present"))
        for index, failure in enumerate(failures):
            with self.subTest(failure=type(failure).__name__):
                seconds = 10.1 + index * 0.2
                self.add_stationary_poses(seconds)
                before = dict(self.node.voxels)
                self.node.cloud_callback(PointCloud2([(100, 100, 100)], seconds))
                with mock.patch.object(mapper.point_cloud2, "read_points", side_effect=failure):
                    self.node.process_pending()
                self.assertEqual(self.node.voxels, before)
                self.assertEqual(self.node.rejection_reasons["invalid_cloud"], index + 1)
                self.add_cloud([(4 + index, 5, 6)], seconds=seconds + 0.1)
                self.assertEqual(len(self.node.voxels), len(before) + 1)
                self.assertEqual(self.node.last_reason, "accepted")

    def test_out_of_range_world_pose_rejects_frame_without_losing_map_or_future_scans(self):
        self.add_cloud([(1, 2, 3)], seconds=10.0)
        before = dict(self.node.voxels)
        self.add_cloud([(4, 5, 6)], seconds=20.0, position=(1e20, 0, 0))
        self.assertEqual(self.node.voxels, before)
        self.assertEqual(self.node.rejection_reasons["invalid_cloud"], 1)
        self.add_cloud([(4, 5, 6)], seconds=21.0)
        self.assertEqual(len(self.node.voxels), 2)
        self.assertEqual(self.node.last_reason, "accepted")

    def test_pcd_has_matching_xyz_float32_payload(self):
        self.add_cloud([(1, 2, 3), (4, 5, 6)])
        success, message = self.node.save_pcd()
        self.assertTrue(success, message)
        pcd_header, payload = self.read_pcd()
        self.assertIn(b"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n", pcd_header)
        self.assertIn(b"WIDTH 2\n", pcd_header)
        self.assertIn(b"POINTS 2\n", pcd_header)
        self.assertEqual(len(payload), 2 * 3 * 4)
        expected = [(1, 2, 3.3), (4, 5, 6.3)]
        for actual, point in zip(struct.iter_unpack("<fff", payload), expected):
            for value, wanted in zip(actual, point):
                self.assertAlmostEqual(value, wanted, places=5)
        self.assertFalse(self.node.dirty)

    def test_failed_replace_preserves_existing_pcd_and_cleans_temporary_file(self):
        self.add_cloud([(1, 2, 3)])
        self.assertTrue(self.node.save_pcd()[0])
        original = self.path.read_bytes()
        self.add_cloud([(4, 5, 6)])
        with mock.patch.object(mapper.os, "replace", side_effect=PermissionError("replace denied")):
            response = self.node.save_callback(None, SimpleNamespace())
        self.assertFalse(response.success)
        self.assertIn("replace denied", response.message)
        self.assertEqual(self.path.read_bytes(), original)
        self.assertEqual(list(self.path.parent.iterdir()), [self.path])
        self.assertTrue(self.node.dirty)

    def test_failed_parent_creation_is_reported_without_raising(self):
        self.node.dirty = True
        with mock.patch.object(Path, "mkdir", side_effect=PermissionError("directory denied")):
            success, message = self.node.save_pcd()
        self.assertFalse(success)
        self.assertIn("directory denied", message)
        self.assertTrue(self.node.dirty)

    def test_clear_publishes_empty_map_and_deletes_markers(self):
        self.add_cloud([(1, 2, 3)])
        self.node.publish_timer()
        response = self.node.clear_callback(None, SimpleNamespace())
        self.assertTrue(response.success, response.message)
        self.assertFalse(self.node.voxels)
        self.assertEqual(self.node.map_publisher.messages[-1].width, 0)
        self.assertEqual(bytes(self.node.map_publisher.messages[-1].data), b"")
        for publisher in (self.node.passable_publisher, self.node.structure_publisher):
            markers = publisher.messages[-1].markers
            self.assertEqual(len(markers), 1)
            self.assertEqual(markers[0].action, Marker.DELETEALL)
            self.assertEqual(markers[0].header.frame_id, "odom")
        pcd_header, payload = self.read_pcd()
        self.assertIn(b"POINTS 0\n", pcd_header)
        self.assertEqual(payload, b"")

    def test_close_saves_dirty_map_once(self):
        self.add_cloud([(1, 2, 3)])
        with mock.patch.object(self.node, "save_pcd", wraps=self.node.save_pcd) as save:
            self.node.close()
            self.node.close()
        self.assertEqual(save.call_count, 1)

    def test_none_display_keeps_fusion_without_constructing_output(self):
        self.node.live_display = "none"
        self.node.save_policy = "final"
        with mock.patch.object(self.node, "update_display", side_effect=AssertionError("unexpected display work")), \
             mock.patch.object(self.node, "cloud_message", side_effect=AssertionError("unexpected serialization")):
            self.add_cloud([(1, 2, 3)])
            self.node.publish_timer()
            self.node.publish_map()
            self.node.autosave_timer()
            self.assertTrue(self.node.voxels)
            self.assertFalse(self.node.cells)
            self.assertFalse(self.path.exists())
            response = self.node.clear_callback(None, SimpleNamespace())
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.path.exists())
        for publisher in (self.node.map_publisher, self.node.ground_publisher,
                          self.node.passable_publisher, self.node.structure_publisher,
                          self.node.current_publisher):
            self.assertFalse(publisher.messages)

    def test_scan_display_only_publishes_current_measurements_and_clear(self):
        self.node.live_display = "scan"
        self.add_cloud([(1, 2, 3)])
        self.add_cloud([(4, 5, 6)])
        self.node.publish_timer()
        self.assertEqual(len(self.node.voxels), 2)
        self.assertEqual(self.node.current_publisher.messages[-1].width, 1)
        self.assertFalse(self.node.map_publisher.messages)
        self.assertFalse(self.node.cells)
        response = self.node.clear_callback(None, SimpleNamespace())
        self.assertTrue(response.success, response.message)
        self.assertEqual(self.node.current_publisher.messages[-1].width, 0)

    def test_final_save_policy_waits_for_explicit_save_or_close(self):
        self.node.save_policy = "final"
        self.add_cloud([(1, 2, 3)])
        self.node.autosave_timer()
        self.assertFalse(self.path.exists())
        response = self.node.save_callback(None, SimpleNamespace())
        self.assertTrue(response.success, response.message)
        self.add_cloud([(4, 5, 6)])
        self.node.close()
        pcd_header, _ = self.read_pcd()
        self.assertIn(b"POINTS 2\n", pcd_header)

    def test_close_drains_pose_ready_scans_without_ros_publication(self):
        self.node.live_display = "scan"
        self.add_stationary_poses(10.0)
        self.node.cloud_callback(PointCloud2([(1, 2, 3)], 10.0))
        with mock.patch.object(self.node, "cloud_message", side_effect=AssertionError("ROS unavailable")):
            self.node.close()
        self.assertFalse(self.node.cloud_queue)
        self.assertEqual(self.node.frames, 1)
        self.assertTrue(self.path.exists())

    def test_offline_initialization_and_aligned_fusion_need_no_ros_entities(self):
        args = mapper.build_parser().parse_args([
            "--pcd-file", str(self.path), "--min-observations", "1", "--disable-ground"])
        with mock.patch.object(Node, "__init__", side_effect=AssertionError("ROS unavailable")), \
             mock.patch.object(Node, "create_publisher", side_effect=AssertionError("ROS unavailable")):
            offline = mapper.FieldMapperNode(args, offline=True)
        with mock.patch.object(offline, "get_logger", return_value=Logger()):
            offline.process_aligned(np.array([[1, 2, 3]]), np.array([[4, 5, 6]]),
                                    np.array([[4, 5, 6]]), 10.0)
        self.assertEqual(offline.live_display, "none")
        self.assertEqual(offline.frames, 1)
        self.assertEqual(list(offline.voxels.values()), [(4, 5, 6)])

    def test_autosave_reports_success_and_failure(self):
        self.add_cloud([(1, 2, 3)])
        self.node.autosave_timer()
        self.assertTrue(self.path.exists())
        self.assertTrue(any(level == "info" and "saved 1 points" in message
                            for level, message in self.node.logger.entries))
        self.add_cloud([(4, 5, 6)])
        self.node.last_save = 0.0
        with mock.patch.object(mapper.os, "replace", side_effect=PermissionError("replace denied")):
            self.node.autosave_timer()
        self.assertTrue(self.node.dirty)
        self.assertTrue(any(level == "error" and "replace denied" in message
                            for level, message in self.node.logger.entries))

    def test_status_reports_missing_inputs_without_claiming_invalid_points(self):
        self.node.logger.entries.clear()
        self.node.report_status()
        warnings = "\n".join(message for level, message in self.node.logger.entries if level == "warning")
        self.assertIn("No lidar received on /front_lidar", warnings)
        self.assertIn("No odometry received on /odom/mujoco_odom", warnings)
        self.assertNotIn("No finite XYZ", warnings)
        self.assertIn((5.0, self.node.report_status), self.node.timers)

    def test_status_distinguishes_stale_inputs_and_pose_timeout(self):
        with mock.patch.object(mapper.time, "monotonic", return_value=100.0):
            self.node.odom_callback(Odometry(1.0))
            self.node.cloud_callback(PointCloud2([(1, 2, 3)], 10.0))
        self.assertEqual(self.node.received_odometry, 1)
        self.assertEqual(self.node.received_clouds, 1)
        self.node.logger.entries.clear()
        with mock.patch.object(mapper.time, "monotonic", return_value=106.0):
            self.node.process_pending()
            self.node.report_status()
        warnings = "\n".join(message for level, message in self.node.logger.entries if level == "warning")
        self.assertIn("Lidar stopped", warnings)
        self.assertIn("Odometry stopped", warnings)
        report = "\n".join(message for level, message in self.node.logger.entries if level == "info")
        self.assertIn("queue_expired", report)
        self.assertNotIn("No finite XYZ", warnings)

    def test_status_reports_received_but_invalid_xyz(self):
        self.add_cloud([(math.nan, 2, 3), (1, math.inf, 3)])
        self.node.logger.entries.clear()
        self.node.report_status()
        warnings = "\n".join(message for level, message in self.node.logger.entries if level == "warning")
        self.assertIn("No finite XYZ", warnings)
        self.assertNotIn("No lidar received", warnings)
        self.assertNotIn("No odometry received", warnings)

    def test_clear_discards_historical_empty_cloud_and_timestamp_diagnostics(self):
        self.add_cloud([(1, 2, 3)])
        self.add_cloud([(math.nan, 2, 3)])
        self.node.cloud_callback(PointCloud2([(4, 5, 6)], 20.0))
        self.node.process_pending()
        self.node.logger.entries.clear()
        self.node.report_status()
        self.assertTrue(any("waiting_need_future" in message
                            for level, message in self.node.logger.entries if level == "info"))
        response = self.node.clear_callback(None, SimpleNamespace())
        self.assertTrue(response.success, response.message)
        self.node.logger.entries.clear()
        self.node.report_status()
        warnings = "\n".join(message for level, message in self.node.logger.entries if level == "warning")
        self.assertNotIn("No finite XYZ", warnings)
        self.assertNotIn("timestamp difference", warnings)
        self.assertFalse(self.node.cloud_queue)
        self.assertEqual(self.node.fusion.pending_count, 0)
        self.assertEqual(self.node.last_reason, "cleared")

    def test_shutdown_closes_node_and_uses_idempotent_ros_shutdown(self):
        for interruption in (KeyboardInterrupt, mapper.ExternalShutdownException):
            with self.subTest(interruption=interruption.__name__):
                node = mock.Mock()
                executor = mock.Mock()
                executor.spin.side_effect = interruption
                events = []
                executor.shutdown.side_effect = lambda: events.append("executor_shutdown")
                node.close.side_effect = lambda: events.append("final_save")
                with mock.patch.object(sys, "argv", [str(SOURCE)]), \
                     mock.patch.object(mapper, "FieldMapperNode", return_value=node), \
                     mock.patch.object(mapper, "MultiThreadedExecutor", return_value=executor) as create_executor, \
                     mock.patch.object(mapper.rclpy, "try_shutdown") as shutdown:
                    mapper.main()
                create_executor.assert_called_once_with(num_threads=3)
                executor.add_node.assert_called_once_with(node)
                executor.shutdown.assert_called_once_with()
                self.assertEqual(events, ["executor_shutdown", "final_save"])
                node.close.assert_called_once_with()
                node.destroy_node.assert_called_once_with()
                shutdown.assert_called_once_with()

    def test_help_can_run_without_ros_dependencies(self):
        result = subprocess.run([sys.executable, "-B", str(SOURCE), "--help"],
                                text=True, capture_output=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--pcd-file", result.stdout)


if __name__ == "__main__":
    unittest.main()
