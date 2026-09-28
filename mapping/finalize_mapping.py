#!/usr/bin/env python3
"""Build a final PCD from a completed capture, with no ROS playback or RViz."""

from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
from contextlib import redirect_stdout, redirect_stderr
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Iterable, Sequence

import numpy as np

import field_mapper_node as mapper
from cloud_io import read_xyz, read_field
from embedded_pose import decode_embedded_pose
from occupancy_map import build_occupancy
from point_filters import filter_xyz
from pose_buffer import PoseBuffer
from runtime_gc import ScheduledGC


class ConsoleLogger:
    def info(self, message):
        print(f"[INFO] [finalize] {message}", flush=True)

    def warning(self, message):
        print(f"[WARN] [finalize] {message}", flush=True)

    def error(self, message):
        print(f"[ERROR] [finalize] {message}", flush=True)


class OfflineMapper(mapper.FieldMapperNode):
    """Reuse the tested fusion and atomic PCD writer without ROS entities."""

    def __init__(self, args):
        self.console_logger = ConsoleLogger()
        super().__init__(args, offline=True)

    def get_logger(self):
        return self.console_logger


class Tee:
    def __init__(self, console, output):
        self.console, self.output = console, output
        self.error = None

    def disable_output(self, exc):
        self.error = str(exc)
        # A full disk can fail both write and close/flush. Keep diagnostics on
        # the original terminal without recursively writing the failed log.
        try:
            self.output.close()
        except OSError:
            pass
        self.console.write(f"Cannot append finalization log: {exc}\n")

    def write(self, text):
        if self.error is None:
            try:
                self.output.write(text)
            except (OSError, ValueError) as exc:
                self.disable_output(exc)
        self.console.write(text)
        return len(text)

    def flush(self):
        if self.error is None:
            try:
                self.output.flush()
            except (OSError, ValueError) as exc:
                self.disable_output(exc)
        self.console.flush()


def iter_bag(bag: Path, topic: str, message_type: str):
    """Stream one topic. Each pass deserializes only the requested input."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("Source ROS2 Humble and install ros-humble-rosbag2-py, "
                           "ros-humble-rosbag2-storage-default-plugins first") from exc
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if types.get(topic) != message_type:
        raise ValueError(f"Bag topic {topic!r} must have type {message_type!r}; found {types.get(topic)!r}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    cls = get_message(message_type)
    while reader.has_next():
        name, data, _received_ns = reader.read_next()
        if name == topic:
            yield deserialize_message(data, cls)


def load_poses(messages: Iterable, global_frame: str):
    """Sort recorded poses once; duplicate stamps use their last valid sample."""
    poses = {}
    counts = Counter()
    for message in messages:
        counts["received"] += 1
        if message.header.frame_id != global_frame:
            raise ValueError(f"Odometry frame {message.header.frame_id!r} differs from --global-frame {global_frame!r}")
        stamp = mapper.stamp_seconds(message.header.stamp)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        position, quaternion = (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
        norm = math.hypot(*quaternion)
        if (stamp <= 0 or not all(math.isfinite(x) for x in (stamp, *position, *quaternion, norm))
                or norm <= 1e-12):
            counts["invalid"] += 1
            continue
        if stamp in poses:
            counts["duplicate_stamp"] += 1
        poses[stamp] = (position, tuple(x / norm for x in quaternion))
    if len(poses) < 2:
        raise ValueError("Capture contains fewer than two valid odometry timestamps")
    times = np.array(sorted(poses), dtype=np.float64)
    positions = np.array([poses[t][0] for t in times], dtype=np.float64)
    quaternions = np.array([poses[t][1] for t in times], dtype=np.float64)
    counts["valid_unique"] = len(times)
    return times, positions, quaternions, dict(counts)


def pose_match(times, positions, quaternions, stamp, args):
    """Use a tiny recorded time window, never a wall-clock queue deadline."""
    before = args.motion_window if args.motion_policy == "strict" else max(args.motion_window, args.scan_duration)
    after = args.motion_lookahead if args.motion_policy == "strict" else max(args.motion_lookahead, args.scan_duration)
    start = max(0, bisect_left(times, stamp - before) - 1)
    end = min(len(times), bisect_right(times, stamp + after) + 1)
    buffer = PoseBuffer(horizon=max(10., before + after + args.max_pose_gap * 4), max_samples=max(2, end - start + 1))
    for index in range(start, end):
        buffer.append(times[index], positions[index], quaternions[index])
    return buffer.lookup(stamp, args.max_pose_gap, before, after)


def process_scans(node, poses, messages, args):
    """Stream scans into the same measured-point fusion used by online mode."""
    from deskew import Trajectory, deskew_cloud

    times, positions, quaternions = poses
    trajectory = Trajectory(times, positions, quaternions)
    counts, timing_reasons = Counter(), Counter()
    last_stamp = 0.0
    started, next_progress = time.perf_counter(), time.monotonic() + 5
    for message in messages:
        if node.gc_maintenance is not None:
            node.gc_maintenance.tick()
        counts["scans_read"] += 1
        if time.monotonic() >= next_progress:
            print(f"Finalizing: read={counts['scans_read']}, fused={node.frames}, "
                  f"confirmed={node.map_point_count()}, rejected={dict(node.rejection_reasons)}", flush=True)
            next_progress = time.monotonic() + 5
        stamp = mapper.stamp_seconds(message.header.stamp)
        if not math.isfinite(stamp) or stamp <= 0:
            node.reject_frame("pose_invalid_stamp")
            continue
        if stamp <= last_stamp:
            node.reject_frame("nonmonotonic_stamp")
            continue
        last_stamp = stamp
        try:
            # UE lidar embeds the simulator pose in two leading intensity=111
            # records.  When valid, this pose is authoritative and avoids the
            # large odometry timing gap that previously rejected whole scans.
            local_raw = read_xyz(message)
            embedded = None
            try:
                embedded = decode_embedded_pose(local_raw, read_field(message, "intensity"))
            except (ValueError, TypeError):
                embedded = None
            if args.pose_source != "odom" and embedded is not None and embedded.pose is not None:
                from scipy.spatial.transform import Rotation
                local = embedded.points
                finite = np.isfinite(local).all(axis=1)
                node.rejected_nonfinite += int(np.count_nonzero(~finite))
                valid = finite.copy()
                distance = np.linalg.norm(local, axis=1)
                in_range = distance >= node.min_range
                if node.max_range > 0:
                    in_range &= distance <= node.max_range
                node.rejected_range += int(np.count_nonzero(finite & ~in_range))
                valid &= in_range
                if not valid.any():
                    node.reject_frame("embedded_no_valid_points")
                    continue
                ranged = local[valid]
                filtered, filter_counts = filter_xyz(ranged, 0, 0, node.outlier_radius, node.min_neighbors)
                node.rejected_duplicates += filter_counts.get("duplicate_points", 0)
                rotation = Rotation.from_quat(embedded.pose[3:]).as_matrix()
                pos = np.asarray(embedded.pose[:3], dtype=np.float64)
                ranged_global = ranged @ rotation.T + pos
                global_points = filtered @ rotation.T + pos
                node.process_aligned(ranged, ranged_global, global_points, stamp)
                counts["embedded_pose_scans"] += 1
                continue
            if embedded is not None and embedded.reason != "missing_embedded_pose":
                counts["embedded_pose_invalid"] += 1
            result = None
            if args.deskew != "off":
                result = deskew_cloud(message, trajectory, node.sensor_z,
                                      args.scan_duration, args.max_pose_gap)
                timing_reasons[result.reason] += 1
            if result is not None and result.reason == "ok":
                local = local_raw
                valid = result.valid_mask & np.isfinite(local).all(axis=1)
                distance = np.linalg.norm(local, axis=1)
                valid &= distance >= node.min_range
                if node.max_range > 0:
                    valid &= distance <= node.max_range
                counts["deskewed_scans"] += 1
                counts["point_alignment_rejected"] += int(np.count_nonzero(~result.valid_mask))
                if not valid.any():
                    node.reject_frame("deskew_no_valid_points")
                    continue
                ranged, world = local[valid], result.points[valid]
                filtered, filter_counts = filter_xyz(world, 0, 0, node.outlier_radius, node.min_neighbors)
                node.rejected_duplicates += filter_counts.get("duplicate_points", 0)
                node.process_aligned(ranged, world, filtered, stamp)
            else:
                if args.deskew == "required":
                    node.reject_frame("deskew_" + result.reason)
                    continue
                counts["single_pose_scans"] += 1
                match, reason = pose_match(times, positions, quaternions, stamp, args)
                if match is None:
                    node.reject_frame("pose_" + reason)
                    continue
                if node.motion_policy == "strict" and match.window_angular_speed > node.max_angular_speed:
                    node.reject_frame("turning_or_settling")
                    continue
                if node.motion_policy == "strict" and match.window_linear_speed > node.max_linear_speed:
                    node.reject_frame("moving_fast_or_settling")
                    continue
                node.process_cloud(message, match)
        except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
            node.reject_frame("invalid_cloud")
            if counts["invalid_cloud_examples"] < 3:
                print(f"[WARN] Invalid cloud at {stamp:.6f}: {exc}", flush=True)
                counts["invalid_cloud_examples"] += 1
    return {**dict(counts), "timing_reasons": dict(timing_reasons),
            "fused_scans": node.frames, "rejected_scans": dict(node.rejection_reasons),
            "motion_rejected_points": node.motion_rejected_points,
            "duplicate_points": node.rejected_duplicates,
            "elapsed_fusion_seconds": time.perf_counter() - started}


def parse_arguments(argv):
    parser = mapper.build_parser()
    parser.description = __doc__
    parser.add_argument("run_directory", type=Path, help="completed run directory containing raw_bag and run.json")
    parser.add_argument("--deskew", choices=("auto", "off", "required"), default="auto",
                        help="use validated absolute per-point timestamps; auto falls back to adaptive filtering")
    parser.add_argument("--pose-source", choices=("auto", "embedded", "odom"), default="auto",
                        help="auto/embedded: use validated UE lidar pose metadata; odom: force header-time odometry")
    parser.add_argument("--pgm-file", default=None,
                        help="2-D occupancy image path; defaults to field_map.pgm beside the PCD")
    parser.add_argument("--map-yaml", default=None,
                        help="map_server YAML path; defaults beside --pgm-file")
    parser.add_argument("--pgm-resolution", type=float, default=0.05,
                        help="occupancy image metres per pixel")
    parser.add_argument("--pgm-padding", type=float, default=0.10,
                        help="unobserved border added around the projected point cloud")
    parser.add_argument("--pgm-obstacle-height", type=float, default=0.15,
                        help="height above ground used to mark an occupied cell")
    initial = parser.parse_args(argv)
    run = initial.run_directory.expanduser().resolve()
    metadata = json.loads((run / "run.json").read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError("run.json must contain a metadata object")
    if metadata.get("mode") != "capture":
        raise ValueError("This command requires a capture run with raw_bag; historical PCD/logs cannot reconstruct scans")
    recorded_args = metadata.get("mapping_args", {})
    if not isinstance(recorded_args, dict):
        raise ValueError("run.json mapping_args must contain an option object")
    names = {action.dest for action in parser._actions}
    defaults = {key: value for key, value in recorded_args.items() if key in names}
    old_pcd = Path(metadata.get("pcd_path", str(run / "field_map.pcd")))
    old_bag = Path(metadata.get("bag_path", str(run / "raw_bag")))
    defaults["pcd_file"] = str(run / old_pcd.name) if old_pcd.parent == old_bag.parent else str(old_pcd)
    parser.set_defaults(**defaults)
    args = parser.parse_args(argv)
    args.run_directory = run
    args.live_display, args.save_policy = "none", "final"
    args.pcd_file = str(Path(args.pcd_file).expanduser().resolve())
    if args.pgm_file is None:
        args.pgm_file = str(Path(args.pcd_file).with_suffix(".pgm"))
    else:
        args.pgm_file = str(Path(args.pgm_file).expanduser().resolve())
    if args.map_yaml is None:
        args.map_yaml = str(Path(args.pgm_file).with_suffix(".yaml"))
    else:
        args.map_yaml = str(Path(args.map_yaml).expanduser().resolve())
    return args, metadata


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args, metadata = parse_arguments(list(sys.argv[1:] if argv is None else argv))
        bag = args.run_directory / "raw_bag"
        if not (bag / "metadata.yaml").is_file():
            raise ValueError("raw_bag/metadata.yaml is missing. Stop capture cleanly before finalizing.")
        if metadata.get("capture_complete") is False:
            raise ValueError("Recorder did not close successfully; inspect mapper.log before using this capture")
    except (ValueError, TypeError, OSError) as exc:
        print(f"Cannot finalize: {exc}", file=sys.stderr)
        return 1
    log_path = args.run_directory / "finalize.log"
    report_path = args.run_directory / "finalize_report.json"
    try:
        sources = ("finalize_mapping.py", "deskew.py", "embedded_pose.py", "occupancy_map.py",
                   "field_mapper_node.py", "cloud_io.py", "point_filters.py", "pose_buffer.py",
                   "voxel_fusion.py", "ground_surface.py", "runtime_gc.py")
        source_root = Path(__file__).resolve().parent
        report = {"started_at": datetime.now(timezone.utc).isoformat(), "parameters": vars(args),
                  "files_sha256": {"mapping/" + name: hashlib.sha256((source_root / name).read_bytes()).hexdigest()
                                   for name in sources},
                  "bag_metadata_sha256": hashlib.sha256((bag / "metadata.yaml").read_bytes()).hexdigest(),
                  "success": False}
        output = log_path.open("a", encoding="utf-8")
    except OSError as exc:
        print(f"Cannot open finalization log or metadata: {exc}", file=sys.stderr)
        return 1
    stdout_tee, stderr_tee = Tee(sys.stdout, output), Tee(sys.stderr, output)
    with output, redirect_stdout(stdout_tee), redirect_stderr(stderr_tee):
        maintenance = None
        try:
            print(f"Reading capture: {bag}; final PCD: {args.pcd_file}", flush=True)
            node = OfflineMapper(args)
            if args.gc_policy == "scheduled":
                maintenance = ScheduledGC()
                node.gc_maintenance = maintenance
            times, positions, quaternions, pose_counts = load_poses(
                iter_bag(bag, args.odom_topic, "nav_msgs/msg/Odometry"), args.global_frame)
            print(f"Recorded odometry: {pose_counts}", flush=True)
            report["odometry"] = pose_counts
            report["processing"] = process_scans(
                node, (times, positions, quaternions),
                iter_bag(bag, args.topic, "sensor_msgs/msg/PointCloud2"), args)
            report["ground"] = {"scans_evaluated": node.ground_scans,
                                "stage_point_totals": dict(node.ground_totals),
                                "last_scan": dict(node.ground_scan_counts)}
            if not node.frames or not node.map_point_count():
                raise ValueError("No confirmed points after filtering; final PCD was not replaced. See rejection counts.")
            report["points"] = node.map_point_count()
            report["ground_points"] = len(node.ground_fusion.points)
            success, message = node.save_pcd()
            print(message, flush=True)
            if not success:
                raise OSError(message)
            occupancy = build_occupancy(
                node.map_points(), image_path=args.pgm_file, yaml_path=args.map_yaml,
                resolution=args.pgm_resolution, ground_z=args.ground_z,
                ground_band=args.ground_band, obstacle_height=args.pgm_obstacle_height,
                padding=args.pgm_padding,
            )
            report["occupancy_map"] = {
                "image": str(occupancy.image_path), "yaml": str(occupancy.yaml_path),
                "width": occupancy.width, "height": occupancy.height,
                "resolution": occupancy.resolution, "origin": occupancy.origin,
                "free_cells": occupancy.free_cells,
                "occupied_cells": occupancy.occupied_cells,
                "unknown_cells": occupancy.unknown_cells,
            }
            print(f"saved occupancy map {occupancy.width}x{occupancy.height} to "
                  f"{occupancy.image_path} and {occupancy.yaml_path}", flush=True)
            report["success"] = True
        except (ValueError, TypeError, RuntimeError, OSError, ImportError) as exc:
            report["error"] = str(exc)
            print(f"Finalization failed: {exc}", file=sys.stderr, flush=True)
        except KeyboardInterrupt:
            report["error"] = "interrupted; capture retained for retry"
            print("Finalization interrupted. Original capture is retained.", file=sys.stderr, flush=True)
        finally:
            if maintenance is not None:
                maintenance.close()
            report["finished_at"] = datetime.now(timezone.utc).isoformat()
            log_errors = [stream.error for stream in (stdout_tee, stderr_tee) if stream.error is not None]
            if log_errors:
                report["success"] = False
                report["log_error"] = log_errors[0]
            temporary = report_path.with_suffix(".json.tmp")
            try:
                temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str) + "\n", encoding="utf-8")
                temporary.replace(report_path)
            except OSError as exc:
                # Keep the original capture and any atomically saved PCD, but
                # do not claim a completed run when its report could not land.
                report["success"] = False
                print(f"Cannot write finalization report: {exc}", file=sys.stderr, flush=True)
                return 1
        print(f"Finalization report: {report_path}", flush=True)
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
