#!/usr/bin/env python3
"""Accumulate the official lidar topic into a global RViz map and PCD file.

This node uses /odom/mujoco_odom directly and publishes every generated object in the
same global frame (normally ``world``).  RViz only has to display the published
topics; it does not need a TF display or a TF tree for the generated map.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import tempfile
import threading
import time
from array import array
from collections import Counter, deque
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from point_filters import filter_xyz
from pose_buffer import PoseBuffer, PoseMatch
from voxel_fusion import VoxelFusion
from ground_surface import GroundSurfaceFilter
from cloud_io import read_xyz
from runtime_gc import ScheduledGC

try:
    import rclpy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError as exc:  # Allows syntax/help checks on a non-ROS machine.
    rclpy = None  # type: ignore[assignment]
    Odometry = PointCloud2 = PointField = Marker = MarkerArray = Trigger = Point = Any  # type: ignore[misc,assignment]
    Node = object  # type: ignore[assignment,misc]
    ExternalShutdownException = KeyboardInterrupt  # type: ignore[misc,assignment]
    point_cloud2 = None  # type: ignore[assignment]
    qos_profile_sensor_data = None  # type: ignore[assignment]
    DurabilityPolicy = HistoryPolicy = QoSProfile = ReliabilityPolicy = None  # type: ignore[assignment,misc]
    ROS_IMPORT_ERROR = exc
else:
    ROS_IMPORT_ERROR = None


def rotate_vector(point: tuple[float, float, float], quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Rotate a vector by an x/y/z/w quaternion."""
    x, y, z = point
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def finite_xyz(values: Iterable[Any]) -> tuple[float, float, float] | None:
    try:
        point = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    if len(point) != 3 or not all(math.isfinite(value) for value in point):
        return None
    return point  # type: ignore[return-value]


class FieldMapperNode(Node):
    MAP_TOPIC = "/field_map"
    PASSABLE_TOPIC = "/field_map/passable"
    STRUCTURE_TOPIC = "/field_map/structure"
    GROUND_TOPIC = "/field_map/ground"
    CURRENT_TOPIC = "/field_map/current_scan"

    def __init__(self, args: argparse.Namespace, *, offline: bool = False) -> None:
        if not offline:
            super().__init__("field_mapper")
        self.input_topic = args.topic
        self.odom_topic = args.odom_topic
        self.global_frame = args.global_frame
        self.pcd_path = Path(args.pcd_file).expanduser().resolve()
        self.voxel_size = max(0.01, args.voxel)
        self.cell_size = max(self.voxel_size, args.cell_size)
        self.max_step = max(0.0, args.max_step)
        self.max_pose_gap = args.max_pose_gap
        self.motion_window = args.motion_window
        self.motion_lookahead = args.motion_lookahead
        self.max_angular_speed = math.radians(args.max_angular_speed)
        self.max_linear_speed = args.max_linear_speed
        self.max_cloud_wait = args.max_cloud_wait
        self.max_queued_clouds = args.cloud_queue_size
        self.queue_policy = args.queue_policy
        self.sensor_z = args.sensor_z
        self.autosave_seconds = max(1.0, args.autosave) if args.autosave > 0 else math.inf
        self.min_range = args.min_range
        self.max_range = args.max_range
        self.outlier_radius = args.outlier_radius
        self.min_neighbors = args.min_neighbors
        self.motion_policy = args.motion_policy
        self.max_motion_error = args.max_motion_error
        self.scan_duration = args.scan_duration
        self.display_voxel = args.display_voxel
        self.preview_points = args.preview_points
        self.live_display = args.live_display
        self.save_policy = args.save_policy
        self.last_stage_ms: dict[str, float] = {}
        self.gc_maintenance = None
        self.last_publish_ms = 0.0
        self.last_save_ms = 0.0
        self.motion_rejected_points = 0
        self._output_version = None
        self._output_points = np.empty((0, 3), dtype="<f4")
        self._display_points: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        self._display_owners: dict[tuple[int, int, int], tuple[int, int, int]] = {}
        self._display_buckets_by_owner = {}
        self._surface_buckets_by_owner = {}
        self._registered_points = 0
        filter_xyz([], self.min_range, self.max_range, self.outlier_radius, self.min_neighbors)
        for name in ("max_pose_gap", "max_angular_speed", "max_linear_speed", "max_cloud_wait", "publish_period",
                     "max_motion_error", "scan_duration", "display_voxel"):
            if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
                raise ValueError(f"--{name.replace('_', '-')} must be finite and positive")
        for name in ("motion_window", "motion_lookahead"):
            if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0:
                raise ValueError(f"--{name.replace('_', '-')} must be finite and nonnegative")
        if self.max_queued_clouds < 1:
            raise ValueError("--cloud-queue-size must be positive")
        if args.preview_points < 1:
            raise ValueError("--preview-points must be positive")
        if not math.isfinite(self.sensor_z):
            raise ValueError("--sensor-z must be finite")

        self.fusion = VoxelFusion(self.voxel_size, args.min_observations, args.pending_ttl)
        self.voxels = self.fusion.points
        self.ground_filter = GroundSurfaceFilter(args.ground_z, args.ground_band, args.ground_radius,
                                                 min_observations=args.ground_min_observations)
        self.ground_enabled = not args.disable_ground
        # Ground confirmation is spatial support across different scans in the
        # selector, not repeated hits on exactly the same 10cm lattice cell.
        self.ground_fusion = VoxelFusion(args.ground_voxel, 1, args.pending_ttl)
        self.ground_scan_counts: dict[str, int] = {}
        self.ground_totals: Counter[str] = Counter()
        self.ground_scans = 0
        self.last_ground_warning = -math.inf
        self.cells: dict[tuple[int, int], dict[str, float | int]] = {}
        self.surface_samples: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        self.surface_owners: dict[tuple[int, int, int], tuple[int, int, int]] = {}
        self.pose_buffer = PoseBuffer()
        self.cloud_queue: deque[tuple[Any, float]] = deque()
        self.queue_lock = threading.Lock()
        self.rejection_reasons: Counter[str] = Counter()
        self.frames = 0
        self.skipped_frames = 0
        self.received_clouds = 0
        self.received_odometry = 0
        self.empty_clouds = 0
        self.rejected_duplicates = 0
        self.rejected_nonfinite = 0
        self.rejected_range = 0
        self.rejected_isolated = 0
        self.rejected_duplicates = 0
        self.invalid_poses = 0
        self.last_cloud_received: float | None = None
        self.last_odom_received: float | None = None
        self.last_pose_span: float | None = None
        self.last_angular_speed = 0.0
        self.last_linear_speed = 0.0
        self.last_processing_ms = 0.0
        self.last_queue_wait_ms = 0.0
        self.last_reason = "waiting_inputs"
        self.last_processed_stamp = 0.0
        self.dirty = False
        self.display_dirty = False
        self.last_save = 0.0
        self.last_invalid_cloud_warning = -math.inf
        if offline:
            # The recorded-data finalizer reuses filters/fusion/PCD output
            # without constructing ROS entities or requiring a live context.
            self.live_display = "none"
            self.save_policy = "final"
            return
        # The map, services and output timers share one group. Only the short
        # input callbacks run concurrently, so saving never races map updates.
        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.cloud_group = MutuallyExclusiveCallbackGroup()
        self.work_group = MutuallyExclusiveCallbackGroup()

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(PointCloud2, self.MAP_TOPIC, map_qos)
        self.ground_publisher = self.create_publisher(PointCloud2, self.GROUND_TOPIC, map_qos)
        self.current_publisher = self.create_publisher(PointCloud2, self.CURRENT_TOPIC, qos_profile_sensor_data)
        self.passable_publisher = self.create_publisher(MarkerArray, self.PASSABLE_TOPIC, map_qos)
        self.structure_publisher = self.create_publisher(MarkerArray, self.STRUCTURE_TOPIC, map_qos)
        odom_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.cloud_subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos_profile_sensor_data,
            callback_group=self.cloud_group)
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, odom_qos, callback_group=self.odom_group)
        self.save_service = self.create_service(Trigger, "~/save", self.save_callback, callback_group=self.work_group)
        self.clear_service = self.create_service(Trigger, "~/clear", self.clear_callback, callback_group=self.work_group)
        self.worker_timer = self.create_timer(0.02, self.process_pending, callback_group=self.work_group)
        self.timer = self.create_timer(args.publish_period, self.publish_timer, callback_group=self.work_group)
        self.save_timer = self.create_timer(1.0, self.autosave_timer, callback_group=self.work_group)
        self.status_timer = self.create_timer(5.0, self.report_status, callback_group=self.work_group)

        self.get_logger().info(f"Lidar: {self.input_topic}; odometry: {self.odom_topic}")
        self.get_logger().info(f"Global frame: {self.global_frame}; output topic: {self.MAP_TOPIC}")
        self.get_logger().info(f"PCD output: {self.pcd_path}")
        self.get_logger().info("No TF is required: odometry is used as the global pose source")
        self.get_logger().info(
            f"Lidar filter: range=[{self.min_range:g}, {self.max_range:g}]m (max=0: unlimited), "
            f"radius={self.outlier_radius:g}m, other distinct neighbors>={self.min_neighbors}"
        )
        self.get_logger().info(
            f"Pose interpolation: max_gap={self.max_pose_gap:g}s, "
            f"motion_window=[-{self.motion_window:g}, +{self.motion_lookahead:g}]s; "
            f"strict-policy limits={args.max_angular_speed:g}deg/s or {self.max_linear_speed:g}m/s"
        )
        self.get_logger().info(
            f"Fusion: voxel={self.voxel_size:g}m, observations={args.min_observations}, "
            f"pending_ttl={args.pending_ttl:g}s, publish_period={args.publish_period:g}s; "
            "confirmed measurements are saved; current scan preview expires without confirmation"
        )
        self.get_logger().info(
            f"Ground observations: enabled={self.ground_enabled}, world_z={args.ground_z:g} "
            f"+/-{args.ground_band:g}m, radius={args.ground_radius:g}m, distinct_support>=6, "
            f"slope<=15deg, plane_residual<=0.03m; voxel={args.ground_voxel:g}m, "
            f"patch_observations={args.ground_min_observations}; topic={self.GROUND_TOPIC}; "
            "measured samples only, included in the combined PCD; no plane filling"
        )
        self.get_logger().info(
            f"Motion policy: {self.motion_policy}; scan_duration={self.scan_duration:g}s; "
            f"max_point_motion_error={self.max_motion_error:g}m; display_voxel={self.display_voxel:g}m; "
            f"current_scan={self.CURRENT_TOPIC} (temporary measured preview, not saved until confirmed)"
        )

    def odom_callback(self, message: Odometry) -> None:
        self.received_odometry += 1
        self.last_odom_received = time.monotonic()
        if self.received_odometry == 1:
            self.get_logger().info(
                f"First odometry: frame={message.header.frame_id!r}, child={message.child_frame_id!r}, "
                f"stamp={stamp_seconds(message.header.stamp):.6f}"
            )
            if message.header.frame_id != self.global_frame:
                self.get_logger().warning(
                    f"Odometry parent {message.header.frame_id!r} differs from output {self.global_frame!r}. "
                    "Verify --global-frame and RViz Fixed Frame; this node does not transform between global frames."
                )
        orientation = message.pose.pose.orientation
        quaternion = (float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w))
        position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )
        if not self.pose_buffer.append(stamp_seconds(message.header.stamp), position, quaternion):
            self.invalid_poses += 1
            if self.invalid_poses == 1:
                self.get_logger().warning("Rejecting invalid/expired odometry stamp, position or quaternion.")

    def transform_point(self, point: tuple[float, float, float], pose: tuple[float, float, float, float, float, float, float]) -> tuple[float, float, float]:
        px, py, pz, qx, qy, qz, qw = pose
        point = (point[0], point[1], point[2] + self.sensor_z)
        rotated = rotate_vector(point, (qx, qy, qz, qw))
        return (rotated[0] + px, rotated[1] + py, rotated[2] + pz)

    def cloud_callback(self, message: PointCloud2) -> None:
        """Enqueue only: point parsing/filtering must not delay odometry intake."""
        self.received_clouds += 1
        self.last_cloud_received = time.monotonic()
        if self.received_clouds == 1:
            self.get_logger().info(
                f"First lidar: frame={message.header.frame_id!r}, "
                f"stamp={stamp_seconds(message.header.stamp):.6f}, points={message.width * message.height}, "
                f"point_step={message.point_step}, fields={[(f.name, f.offset, f.datatype, f.count) for f in message.fields]}"
            )
        with self.queue_lock:
            if len(self.cloud_queue) >= self.max_queued_clouds:
                self.cloud_queue.popleft()
                self.reject_frame("queue_overflow")
            self.cloud_queue.append((message, self.last_cloud_received))

    def reject_frame(self, reason: str) -> None:
        # Called under queue_lock for inbox mutations; processing also acquires
        # it when updating counters, so overflow cannot lose another increment.
        self.skipped_frames += 1
        self.rejection_reasons[reason] += 1
        self.last_reason = reason

    def process_pending(self) -> None:
        with self.queue_lock:
            if not self.cloud_queue:
                return
            # Throw away stale inbox entries together. Previously each expired
            # scan consumed a separate timer turn and another full pose lookup.
            now = time.monotonic()
            while self.cloud_queue and now - self.cloud_queue[0][1] >= self.max_cloud_wait:
                self.last_queue_wait_ms = (now - self.cloud_queue[0][1]) * 1000.0
                self.cloud_queue.popleft()
                self.reject_frame("queue_expired")
            if not self.cloud_queue:
                return
            if self.queue_policy == "latest" and len(self.cloud_queue) > 2:
                latest_pose = self.pose_buffer.latest_stamp
                if latest_pose is not None:
                    eligible = [index for index, (cloud, _) in enumerate(self.cloud_queue)
                                if stamp_seconds(cloud.header.stamp) + (self.motion_lookahead if self.motion_policy == "strict"
                                                                      else max(self.motion_lookahead, self.scan_duration)) <= latest_pose]
                    for _ in range(eligible[-1] if eligible else 0):
                        self.cloud_queue.popleft()
                        self.reject_frame("queue_superseded")
            message, received = self.cloud_queue[0]
        started = time.monotonic()
        cloud_stamp = stamp_seconds(message.header.stamp)
        before = self.motion_window if self.motion_policy == "strict" else max(self.motion_window, self.scan_duration)
        after = self.motion_lookahead if self.motion_policy == "strict" else max(self.motion_lookahead, self.scan_duration)
        match, reason = self.pose_buffer.lookup(cloud_stamp, self.max_pose_gap, before, after)
        age = started - received
        if age < self.max_cloud_wait and reason in ("need_future", "insufficient_samples"):
            self.last_reason = "waiting_" + reason
            return
        with self.queue_lock:
            # A subscription may have dropped this item while lookup ran.
            if not self.cloud_queue or self.cloud_queue[0][0] is not message:
                return
            self.cloud_queue.popleft()
            self.last_queue_wait_ms = age * 1000.0
            self.last_pose_span = match.span if match else None
            if age >= self.max_cloud_wait:
                self.reject_frame("queue_expired" if match else "pose_timeout_" + reason)
                return
            if match is None:
                self.reject_frame("pose_" + reason)
                return
            if cloud_stamp <= self.last_processed_stamp:
                self.reject_frame("nonmonotonic_stamp")
                return
            self.last_processed_stamp = cloud_stamp
            self.last_angular_speed = math.degrees(match.window_angular_speed)
            self.last_linear_speed = match.window_linear_speed
            if self.motion_policy == "strict" and match.window_angular_speed > self.max_angular_speed:
                self.reject_frame("turning_or_settling")
                return
            if self.motion_policy == "strict" and match.window_linear_speed > self.max_linear_speed:
                self.reject_frame("moving_fast_or_settling")
                return
        try:
            self.process_cloud(message, match)
        except (AssertionError, KeyError, IndexError, TypeError, ValueError, OverflowError, struct.error) as exc:
            # Missing XYZ fields, a truncated payload or unsupported coordinates
            # must not terminate the node and discard the remainder of a run.
            # Parsing and fusion validation happen before map mutation.
            with self.queue_lock:
                self.reject_frame("invalid_cloud")
            if started - self.last_invalid_cloud_warning >= 5.0:
                self.get_logger().warning(f"Rejecting invalid lidar geometry: {type(exc).__name__}: {exc}")
                self.last_invalid_cloud_warning = started
        self.last_processing_ms = (time.monotonic() - started) * 1000.0

    def motion_mask(self, points: np.ndarray, match: PoseMatch) -> np.ndarray:
        if self.motion_policy == "strict":
            return np.ones(len(points), dtype=bool)
        # Approximate worst displacement within the covered scan interval.
        # No fabricated per-point timing: nearer points tolerate gentle motion
        # while distant geometry with excessive uncertainty is withheld.
        local = points.astype(float).copy()
        local[:, 2] += self.sensor_z
        error = match.translation_excursion + 2 * np.linalg.norm(local, axis=1) * math.sin(min(math.pi, match.angular_excursion) / 2)
        return error <= self.max_motion_error

    def process_cloud(self, message: PointCloud2, match: PoseMatch) -> None:
        """Fuse a time-aligned scan after the motion gate has accepted it."""
        pose = (*match.position, *match.quaternion)
        stage_start = time.perf_counter()
        points = (read_xyz(message) if message.fields
                  else point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=False))
        if isinstance(points, np.ndarray) and points.dtype.names:
            # Newer sensor_msgs_py returns a structured array; Humble normally
            # returns an iterator. Request and copy only the XYZ fields.
            local_points = np.column_stack([points[name].reshape(-1) for name in ("x", "y", "z")])
        elif isinstance(points, np.ndarray):
            local_points = points
        else:
            local_points = [tuple(row) for row in points]
        decoded = time.perf_counter()
        ranged, counts = filter_xyz(local_points, self.min_range, self.max_range, radius=0)
        self.rejected_nonfinite += counts["nonfinite"]
        self.rejected_range += counts["range"]
        motion_valid = self.motion_mask(ranged, match)
        self.motion_rejected_points += int(len(ranged) - np.count_nonzero(motion_valid))
        if len(ranged) and not np.any(motion_valid):
            self.last_stage_ms = {"decode": (decoded-stage_start)*1000,
                                  "motion_filter": (time.perf_counter()-decoded)*1000}
            with self.queue_lock:
                self.reject_frame("motion_uncertain")
            return
        ranged = ranged[motion_valid]
        filtered, radius_counts = filter_xyz(ranged, 0, 0, self.outlier_radius, self.min_neighbors)
        filtered_at = time.perf_counter()
        self.rejected_isolated += radius_counts["isolated"]
        self.rejected_duplicates += radius_counts.get("duplicate_points", 0)

        # Transform a scan in one batch. Filtering happens in the lidar frame,
        # so range is measured from the sensor, not from the world origin.
        rotation = np.asarray([rotate_vector(axis, pose[3:]) for axis in (
            (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
        )])
        shifted = filtered.astype(np.float64)
        shifted[:, 2] += self.sensor_z
        global_points = shifted @ rotation + np.asarray(pose[:3])
        ranged_shifted = ranged.astype(np.float64)
        ranged_shifted[:, 2] += self.sensor_z
        ranged_global = ranged_shifted @ rotation + np.asarray(pose[:3])
        self.process_aligned(ranged, ranged_global, global_points, match.stamp)
        self.last_stage_ms.update({"decode": (decoded-stage_start)*1000,
                                   "filter": (filtered_at-decoded)*1000})

    def process_aligned(self, ranged: np.ndarray, ranged_global: np.ndarray,
                        global_points: np.ndarray, stamp: float) -> None:
        """Fuse already aligned measurements from live or recorded scans.

        ``ranged`` retains the sensor-frame rows corresponding to
        ``ranged_global``; ``global_points`` is its radius-filtered world
        subset. Recorded scans may use a different interpolated pose per row.
        """
        stage_start = time.perf_counter()
        ranged = np.asarray(ranged, dtype=np.float64).reshape(-1, 3)
        ranged_global = np.asarray(ranged_global, dtype=np.float64).reshape(-1, 3)
        global_points = np.asarray(global_points, dtype=np.float64).reshape(-1, 3)
        if len(ranged) != len(ranged_global):
            raise ValueError("local/world ranged points must have matching rows")
        # Validate all coordinates before either fusion mutates. Both branches
        # see the same interpolated pose and have already passed the motion gate.
        grid_limit = min(self.voxel_size, self.ground_fusion.voxel_size) * (2.0 ** 63)
        if (not np.isfinite(ranged).all() or not np.isfinite(ranged_global).all()
                or not np.isfinite(global_points).all()
                or np.any(np.abs(ranged_global) >= grid_limit)
                or np.any(np.abs(global_points) >= grid_limit)):
            raise ValueError("points exceed the supported voxel coordinate range")
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError("scan timestamp must be finite and positive")
        selected_ground = (ranged_global[self.ground_filter.mask(ranged_global, stamp)]
                           if self.ground_enabled else np.empty((0, 3)))
        ground_at = time.perf_counter()
        changed = self.fusion.integrate(global_points, stamp)
        ground_changed = self.ground_fusion.integrate(selected_ground, stamp)
        fused_at = time.perf_counter()
        if changed or ground_changed:
            self.dirty = True
            self.display_dirty = self.live_display == "map"
        band_input = int(np.count_nonzero(np.abs(ranged_global[:, 2] - self.ground_filter.ground_z)
                                         <= self.ground_filter.band))
        self.ground_scan_counts = {
            "range_valid": len(ranged),
            "downward_local": int(np.count_nonzero(ranged[:, 2] < 0)),
            "band_input": band_input,
            "band_after_radius": int(np.count_nonzero(np.abs(global_points[:, 2] - self.ground_filter.ground_z)
                                                      <= self.ground_filter.band)),
            "plane_supported": self.ground_filter.last_counts.get("plane_supported", 0) if self.ground_enabled else 0,
            "temporal_confirmed": len(selected_ground),
        }
        self.ground_totals.update(self.ground_scan_counts)
        self.ground_scans += 1
        # Assign each fine voxel to a display bucket using its fixed centre.
        # Its running mean can cross a 20cm boundary, but must not leave stale
        # representatives in two buckets. Refresh the selected owner's sample
        # as it moves. Classification retains observed height bounds; the
        # bucket alignment is approximate to half a fine voxel at boundaries.
        if self.live_display == "map":
            self.update_display(changed)
        display_at = time.perf_counter()
        if self.live_display == "scan":
            preview = global_points if not len(selected_ground) else np.vstack((global_points, selected_ground))
            stride = max(1, math.ceil(len(preview) / self.preview_points))
            self.current_publisher.publish(self.cloud_message(np.asarray(preview[::stride], dtype="<f4")))
        end = time.perf_counter()
        self.last_stage_ms = {"ground": (ground_at-stage_start)*1000, "fusion": (fused_at-ground_at)*1000,
                              "display_update": (display_at-fused_at)*1000 if self.live_display == "map" else 0.0,
                              "preview_publish": (end-display_at)*1000 if self.live_display == "scan" else 0.0}

        if len(global_points) or len(selected_ground):
            self.frames += 1
            self.last_reason = "accepted"
            if self.frames == 1:
                self.get_logger().info(
                    f"Map started: {self.map_point_count()} confirmed points in {self.global_frame!r}; "
                    f"pending={self.fusion.pending_count}; live_display={self.live_display}")
        else:
            self.empty_clouds += 1
            self.last_reason = "empty_after_filter"

    def update_display(self, changed: list[tuple[int, int, int]]) -> None:
        """Reduce fine voxels in NumPy before updating display dictionaries.

        Owners are fixed fine keys; a representative is refreshed even when
        another changed voxel is first in the same coarse bucket. Historical
        height extrema still include every changed measured mean.
        """
        if not changed:
            return
        keys = np.asarray(changed, dtype=np.int64)
        from_fusion = tuple(changed) == self.fusion.last_changed_keys
        values = (self.fusion.last_changed_points if from_fusion
                  else np.asarray([self.voxels[key] for key in changed], dtype=np.float64))
        centres = (keys.astype(np.float64) + 0.5) * self.voxel_size
        surface_grid = np.floor(centres / self.cell_size).astype(np.int64)
        # A fine voxel never changes its fixed display bucket. Register newly
        # confirmed keys once; repeated scans only refresh selected owners.
        added = self.fusion.last_added_keys
        incremental = from_fusion and self._registered_points + len(added) == len(self.voxels)
        register_keys = added if incremental else changed
        register_centres = ((np.asarray(register_keys, dtype=np.int64).reshape(-1, 3) + 0.5)
                            * self.voxel_size)
        for scale, owners, samples, reverse in (
                (self.cell_size, self.surface_owners, self.surface_samples, self._surface_buckets_by_owner),
                (self.display_voxel, self._display_owners, self._display_points, self._display_buckets_by_owner)):
            grid = np.floor(register_centres / scale).astype(np.int64)
            # Stable sorting preserves which fine key first owned a bucket.
            # Coarsening can make separate fine-X runs share a coarse bucket.
            order = np.lexsort((grid[:, 2], grid[:, 1], grid[:, 0]))
            ordered = grid[order]
            first = (order[np.r_[True, np.any(ordered[1:] != ordered[:-1], axis=1)]]
                     if len(grid) else np.empty(0, dtype=np.intp))
            for row, index in zip(grid[first].tolist(), first):
                bucket = tuple(row)
                owner = owners.get(bucket)
                if owner is None:
                    owner = register_keys[index]
                    owners[bucket] = owner
                reverse[owner] = bucket
                samples[bucket] = self.voxels[owner]
            samples.update((reverse[key], self.voxels[key]) for key in changed if key in reverse)
        self._registered_points = len(self.voxels)
        xy = surface_grid[:, :2]
        order = np.lexsort((xy[:, 1], xy[:, 0]))
        ordered = xy[order]
        first = np.flatnonzero(np.r_[True, np.any(ordered[1:] != ordered[:-1], axis=1)])
        minima = np.minimum.reduceat(values[order, 2], first)
        maxima = np.maximum.reduceat(values[order, 2], first)
        for row, low, high in zip(ordered[first].tolist(), minima.tolist(), maxima.tolist()):
            key = tuple(row)
            cell = self.cells.get(key)
            if cell is None:
                self.cells[key] = {"min_z": low, "max_z": high}
            else:
                cell["min_z"] = min(cell["min_z"], low)
                cell["max_z"] = max(cell["max_z"], high)

    def report_status(self) -> None:
        now = time.monotonic()
        if self.last_cloud_received is None:
            self.get_logger().warning(
                f"No lidar received on {self.input_topic}. Check simulator, topic and ROS/RMW environment."
            )
        elif now - self.last_cloud_received > 5.0:
            self.get_logger().warning(f"Lidar stopped: no {self.input_topic} data for {now - self.last_cloud_received:.1f}s")
        if self.last_odom_received is None:
            self.get_logger().warning(f"No odometry received on {self.odom_topic}; global mapping is waiting for a pose.")
        elif now - self.last_odom_received > 5.0:
            self.get_logger().warning(f"Odometry stopped: no {self.odom_topic} data for {now - self.last_odom_received:.1f}s")
        if self.empty_clouds and self.frames == 0:
            self.get_logger().warning("No finite XYZ points passed the lidar filters; check input and filter settings.")
        if self.ground_enabled and self.ground_scans >= 10 and not self.ground_fusion.points and now - self.last_ground_warning >= 30.0:
            if self.ground_totals["band_input"] == 0:
                detail = "no range-valid returns in the expected world-Z band; check sensor coverage and --ground-z"
            elif self.ground_totals["plane_supported"] == 0:
                detail = "low returns exist but have no supported horizontal patches; wall bases alone are not ground"
            else:
                detail = "horizontal returns exist but need support from another scan on the same nearby surface"
            self.get_logger().warning(f"Ground not confirmed: {detail}. No unobserved floor will be generated.")
            self.last_ground_warning = now
        with self.queue_lock:
            queue_length = len(self.cloud_queue)
            reasons = dict(self.rejection_reasons)
        combined_count = self.map_point_count()
        self.get_logger().info(
            f"lidar_received={self.received_clouds}, odom_received={self.received_odometry}, "
            f"frames={self.frames}, unique_points={combined_count}, cells={len(self.cells)}, skipped={self.skipped_frames}, "
            f"rejected_nonfinite={self.rejected_nonfinite}, rejected_range={self.rejected_range}, "
            f"rejected_isolated={self.rejected_isolated}, duplicate_points={self.rejected_duplicates}, invalid_poses={self.invalid_poses}, "
            f"pending_voxels={self.fusion.pending_count}, queue={queue_length}, "
            f"state={self.last_reason}, pose_span={self.last_pose_span}, "
            f"window_angular_deg_s={self.last_angular_speed:.2f}, window_linear_m_s={self.last_linear_speed:.3f}, "
            f"processing_ms={self.last_processing_ms:.1f}, queue_wait_ms={self.last_queue_wait_ms:.1f}, "
            f"rejections={reasons}"
        )
        self.get_logger().info(
            f"Ground stages (pose/motion-accepted scans only): scans={self.ground_scans}, "
            f"last={self.ground_scan_counts}, totals={dict(self.ground_totals)}, "
            f"ground_confirmed={len(self.ground_fusion.points)}, ground_support_scans={self.ground_filter.last_counts.get('support_scans', 0)}, "
            f"combined_pcd_points={combined_count}"
        )
        self.get_logger().info(f"Pipeline: stages_ms={self.last_stage_ms}, motion_rejected_points={self.motion_rejected_points}, "
                               f"display_points={len(self._display_points)}, "
                               f"publish_ms={self.last_publish_ms:.1f}, save_ms={self.last_save_ms:.1f}, "
                               f"live_display={self.live_display}, save_policy={self.save_policy}, "
                               f"motion_policy={self.motion_policy}")
        if self.gc_maintenance is not None:
            gc_state = self.gc_maintenance
            self.get_logger().info(f"GC maintenance: generation={gc_state.last_generation}, "
                                   f"last_ms={gc_state.last_ms:.1f}, max_ms={gc_state.max_ms:.1f}, "
                                   f"collections={gc_state.collections}")

    def ground_points_by_key(self) -> dict[tuple[int, int, int], tuple[float, float, float]]:
        result: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        for point in self.ground_fusion.points.values():
            key = tuple(math.floor(value / self.voxel_size) for value in point)
            result.setdefault(key, self.voxels.get(key, point))
        return result

    def extra_ground_points(self) -> list[tuple[float, float, float]]:
        """Keep fine-map samples in overlapping fine voxels; append new ground."""
        return [point for key, point in self.ground_points_by_key().items() if key not in self.voxels]

    def map_point_count(self) -> int:
        return len(self.voxels) + len(self.extra_ground_points())

    def map_points(self) -> np.ndarray:
        version = (self.fusion.version, self.ground_fusion.version)
        if version != self._output_version:
            extra = np.asarray(self.extra_ground_points(), dtype="<f4").reshape(-1, 3)
            base = self.fusion.snapshot()
            self._output_points = np.vstack((base, extra)) if len(extra) else base
            self._output_version = version
        return self._output_points

    def classified_cells(self, state: str) -> list[tuple[int, dict[str, float | int]]]:
        result: list[tuple[int, dict[str, float | int]]] = []
        marker_id = 0
        for cell_key, cell in sorted(self.cells.items()):
            clearance = float(cell["max_z"]) - float(cell["min_z"])
            current_state = "passable" if clearance <= self.max_step else "structure"
            if current_state == state:
                result.append((marker_id, {**cell, "ix": cell_key[0], "iy": cell_key[1]}))
                marker_id += 1
        return result

    def make_markers(self, state: str) -> MarkerArray:
        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.global_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        color = (0.20, 0.85, 0.30, 0.42) if state == "passable" else (1.0, 0.42, 0.08, 0.62)
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"field_{state}"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = min(0.10, self.voxel_size)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        # Draw only sampled observed surfaces. Never fill the unobserved vertical
        # gap between a cell's minimum and maximum height with a solid column.
        selected = {(int(cell["ix"]), int(cell["iy"])) for _, cell in self.classified_cells(state)}
        for key, sample in self.surface_samples.items():
            if key[:2] in selected:
                point = Point()
                point.x, point.y, point.z = sample
                marker.points.append(point)
        if marker.points:
            array.markers.append(marker)
        return array

    def publish_timer(self) -> None:
        if self.live_display != "map" or not self.display_dirty:
            return
        self.publish_map()

    def cloud_message(self, points: np.ndarray) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header.frame_id = self.global_frame
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                        for index, name in enumerate(("x", "y", "z"))]
        cloud.is_bigendian, cloud.is_dense = False, True
        cloud.point_step, cloud.row_step = 12, len(points) * 12
        # The generated ROS uint8[] setter accepts array('B') directly; bytes
        # otherwise incur validation and conversion of each byte on Humble.
        payload = array("B")
        payload.frombytes(points.tobytes())
        cloud.data = payload
        return cloud

    def publish_map(self) -> None:
        if self.live_display != "map":
            self.display_dirty = False
            return
        started = time.perf_counter()
        # RViz receives a 6cm measured-surface sample. Full 3cm data is retained
        # for PCD; serializing/rendering all accumulated tuples blocked intake.
        display = np.asarray(list(self._display_points.values()) + self.extra_ground_points(), dtype="<f4").reshape(-1, 3)
        self.map_publisher.publish(self.cloud_message(display))
        ground = np.asarray(list(self.ground_points_by_key().values()), dtype="<f4").reshape(-1, 3)
        self.ground_publisher.publish(self.cloud_message(ground))
        self.passable_publisher.publish(self.make_markers("passable"))
        self.structure_publisher.publish(self.make_markers("structure"))
        self.display_dirty = False
        self.last_publish_ms = (time.perf_counter() - started) * 1000

    def autosave_timer(self) -> None:
        if self.save_policy == "periodic" and self.dirty and time.time() - self.last_save >= self.autosave_seconds:
            success, message = self.save_pcd()
            if success:
                self.get_logger().info(message)
            else:
                self.get_logger().error(f"PCD autosave failed: {message}")

    def save_pcd(self) -> tuple[bool, str]:
        started = time.perf_counter()
        points = self.map_points()
        header = ("# .PCD v0.7 - Point Cloud Data file format\n"
                  "VERSION 0.7\n"
                  "FIELDS x y z\n"
                  "SIZE 4 4 4\n"
                  "TYPE F F F\n"
                  "COUNT 1 1 1\n"
                  f"WIDTH {len(points)}\n"
                  "HEIGHT 1\n"
                  "VIEWPOINT 0 0 0 1 0 0 0\n"
                  f"POINTS {len(points)}\n"
                  "DATA binary\n").encode("ascii")
        temporary_name = ""
        try:
            self.pcd_path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile("wb", dir=self.pcd_path.parent, prefix=f".{self.pcd_path.name}.", delete=False) as output:
                temporary_name = output.name
                output.write(header)
                output.write(points.tobytes())
            os.replace(temporary_name, self.pcd_path)
            self.last_save = time.time()
            self.dirty = False
            self.last_save_ms = (time.perf_counter() - started) * 1000
            return True, f"saved {len(points)} points to {self.pcd_path}"
        except OSError as exc:
            if temporary_name:
                try:
                    os.unlink(temporary_name)
                except OSError:
                    pass
            return False, str(exc)

    def save_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, message = self.save_pcd()
        response.success = success
        response.message = message
        return response

    def clear_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.fusion.clear()
        self.ground_fusion.clear()
        self.ground_filter.clear()
        self._display_points.clear()
        self._display_owners.clear()
        self._display_buckets_by_owner.clear()
        self._surface_buckets_by_owner.clear()
        self._registered_points = 0
        self.ground_scan_counts.clear()
        self.ground_totals.clear()
        self.ground_scans = 0
        self.last_ground_warning = -math.inf
        self.cells.clear()
        self.surface_samples.clear()
        self.surface_owners.clear()
        self.frames = 0
        with self.queue_lock:
            self.cloud_queue.clear()
            self.skipped_frames = 0
            self.rejection_reasons.clear()
        self.empty_clouds = 0
        self.last_pose_span = None
        self.last_processed_stamp = 0.0
        self.last_reason = "cleared"
        self.dirty = True
        self.publish_map()
        if self.live_display == "scan":
            self.current_publisher.publish(self.cloud_message(np.empty((0, 3), dtype="<f4")))
        success, message = self.save_pcd()
        response.success = success
        response.message = "map cleared; " + message
        return response

    def close(self) -> None:
        # Drain scans whose pose window is already available before the final
        # PCD write; no further ROS callbacks run after executor shutdown.
        # ROS may already be shut down; final draining only updates geometry.
        display_mode = self.live_display
        self.live_display = "none"
        try:
            for _ in range(self.max_queued_clouds + 1):
                with self.queue_lock:
                    if not self.cloud_queue:
                        break
                self.process_pending()
        finally:
            self.live_display = display_mode
        if self.dirty:
            success, message = self.save_pcd()
            # SIGINT may already have shut down the ROS logging publisher.
            print(f"[{'INFO' if success else 'ERROR'}] [field_mapper] {message}", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Accumulate /front_lidar with /odom/mujoco_odom and save a 3D PCD")
    parser.add_argument("--topic", default="/front_lidar")
    parser.add_argument("--odom-topic", default="/odom/mujoco_odom")
    parser.add_argument("--global-frame", default="world")
    parser.add_argument("--pcd-file", default="field_map.pcd")
    parser.add_argument("--voxel", type=float, default=0.03, help="confirmed voxel size in metres; not an accuracy guarantee")
    parser.add_argument("--cell-size", type=float, default=0.20, help="XY classification cell in metres")
    parser.add_argument("--max-step", type=float, default=0.22, help="height range threshold for passable cells")
    parser.add_argument("--max-pose-gap", "--max-pose-age", dest="max_pose_gap", type=float, default=0.10,
                        help="maximum adjacent odometry gap for interpolation; old --max-pose-age is an alias")
    parser.add_argument("--motion-window", type=float, default=0.10, help="seconds of odometry inspected before scan")
    parser.add_argument("--motion-lookahead", type=float, default=0.10, help="seconds of odometry inspected after scan")
    parser.add_argument("--motion-policy", choices=("adaptive", "strict"), default="adaptive", help="adaptive: bound per-point motion displacement; strict: legacy velocity gate")
    parser.add_argument("--scan-duration", type=float, default=0.10, help="assumed scan span (s); adaptive mode covers this interval on both sides of the stamp")
    parser.add_argument("--max-motion-error", type=float, default=0.05, help="maximum estimated within-scan point displacement (m)")
    parser.add_argument("--max-angular-speed", type=float, default=5.0, help="pause fusion above this body rotation speed (deg/s)")
    parser.add_argument("--max-linear-speed", type=float, default=0.5, help="pause fusion above this translation speed (m/s)")
    parser.add_argument("--max-cloud-wait", type=float, default=0.5, help="maximum wall-clock seconds in the lidar inbox")
    parser.add_argument("--cloud-queue-size", type=int, default=8, help="bounded pending lidar inbox; discard oldest on overflow")
    parser.add_argument("--queue-policy", choices=("ordered", "latest"), default="ordered",
                        help="ordered: retain recoverable queued scans; latest: skip older pose-ready scans on backlog")
    parser.add_argument("--min-observations", type=int, default=3, help="distinct accepted scans needed to confirm a voxel")
    parser.add_argument("--pending-ttl", type=float, default=2.0, help="expire unconfirmed voxels not observed for this many scan seconds")
    parser.add_argument("--publish-period", type=float, default=1.0, help="seconds between changed map publications")
    parser.add_argument("--display-voxel", type=float, default=0.06, help="RViz map sampling (m); full --voxel resolution is saved to PCD")
    parser.add_argument("--preview-points", type=int, default=12000, help="maximum points in temporary current scan preview")
    parser.add_argument("--sensor-z", type=float, default=0.30, help="lidar height relative to the odometry body pose")
    parser.add_argument("--autosave", type=float, default=15.0)
    parser.add_argument("--save-policy", choices=("periodic", "final"), default="periodic",
                        help="periodic: save while running; final: save on service/exit only")
    parser.add_argument("--live-display", choices=("scan", "map", "none"), default="map",
                        help="scan: temporary transformed scan; map: accumulated map; none: no mapper display output")
    parser.add_argument("--gc-policy", choices=("scheduled", "default"), default="scheduled",
                        help="schedule cyclic GC maintenance; default restores Python automatic collection for comparison")
    parser.add_argument("--min-range", type=float, default=0.1, help="minimum local lidar range in metres")
    parser.add_argument("--max-range", type=float, default=60.0, help="maximum local lidar range in metres; 0 disables")
    parser.add_argument("--outlier-radius", type=float, default=0.2, help="scan-local neighbor radius in metres; 0 disables")
    parser.add_argument("--min-neighbors", type=int, default=2, help="minimum OTHER distinct scan points in the radius")
    parser.add_argument("--ground-z", type=float, default=0.0, help="expected floor height in the world frame (m); not sensor height")
    parser.add_argument("--ground-band", type=float, default=0.12, help="ground candidate height tolerance above/below --ground-z (m)")
    parser.add_argument("--ground-radius", type=float, default=0.6, help="local plane support radius for sparse measured ground (m)")
    parser.add_argument("--ground-voxel", type=float, default=0.10, help="confirmed ground voxel size (m); retain measured scan-centroid means")
    parser.add_argument("--ground-min-observations", type=int, default=2, help="distinct scans supporting a local ground patch; does not require identical voxel hits")
    parser.add_argument("--disable-ground", action="store_true", help="disable supplemental ground mapping for comparison with the validated baseline")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if rclpy is None:
        raise SystemExit(f"ROS2 Python dependencies are unavailable: {ROS_IMPORT_ERROR}")
    rclpy.init()
    node = FieldMapperNode(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    if args.gc_policy == "scheduled":
        node.gc_maintenance = ScheduledGC()
        node.create_timer(1.0, node.gc_maintenance.tick, callback_group=node.work_group)
        node.get_logger().info("Cyclic GC: scheduled generations 0/1/2 every 1/15/120s; reference counting remains active")
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Drain running work before final file save; no concurrent map mutation.
        executor.shutdown()
        try:
            node.close()
            node.destroy_node()
            rclpy.try_shutdown()
        finally:
            if node.gc_maintenance is not None:
                node.gc_maintenance.close()


if __name__ == "__main__":
    main()
