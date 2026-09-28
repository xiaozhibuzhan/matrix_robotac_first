#!/usr/bin/env python3
"""Synthetic turning regression; its injected delays are NOT field measurements.

Run from the project root with ``python3 -B mapping/tests/turning_regression.py``.
The fixed seed and event stream compare the former immediate nearest-pose,
first-point-per-voxel path against the new real PoseBuffer and VoxelFusion
helpers. No ROS, sensor recording, existing PCD or output file is used.
Both methods use the current 3 cm resolution to isolate timing and fusion
behavior. The baseline reproduces the former algorithm, not its old 5 cm
resolution. New fusion/window settings match the current node defaults.

The simulator emits instantaneous scans of two finite perpendicular wall
rectangles. It exercises cross-stream delivery delay and a single corrupted
scan, not within-scan deskew, SLAM drift or real-world occlusion.
"""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_buffer import PoseBuffer
from voxel_fusion import VoxelFusion


SEED = 260908
BASE_STAMP = 100.0
ODOM_DELAY = 0.080
SCAN_ARRIVAL_DELAY = 0.005
SCAN_PERIOD = 0.1
ODOM_PERIOD = 0.01
SCAN_NOISE = 0.002
VOXEL_SIZE = 0.03
ANGULAR_LIMIT = math.radians(5.0)
WINDOW_BEFORE = 0.35
WINDOW_AFTER = 0.10
PENDING_TTL = 2.0
MAX_POSE_GAP = 0.10
CORRUPTED_SCAN = 5
CORRUPTION_ANGLE = 5.0
POSITION = np.array([2.0, 3.0, 0.3])


def truth_yaw(seconds: float) -> float:
    """Four 90-degree turns, each two seconds, separated by two-second stops."""
    if seconds <= 0:
        return 0.0
    completed = min(4, int(seconds // 4))
    if completed == 4:
        return 2 * math.pi
    phase = seconds - completed * 4
    turn_fraction = min(1.0, max(0.0, (phase - 2.0) / 2.0))
    return (completed + turn_fraction) * math.pi / 2.0


def truth_rotation(seconds: float) -> Rotation:
    return Rotation.from_euler("z", truth_yaw(seconds))


def wall_samples() -> np.ndarray:
    """Two 8 m horizontal segments extruded over z=[0.1, 2.9] m."""
    along = np.linspace(0.0, 8.0, 101)
    height = np.linspace(0.1, 2.9, 8)
    a, z = np.meshgrid(along, height, indexing="ij")
    wall_x = np.column_stack((np.full(a.size, 8.0), a.ravel(), z.ravel()))
    wall_y = np.column_stack((a.ravel(), np.full(a.size, 8.0), z.ravel()))
    return np.unique(np.vstack((wall_x, wall_y)), axis=0)


def distance_to_walls(points: np.ndarray) -> np.ndarray:
    """Exact Euclidean distance to either finite wall rectangle, not infinite planes."""
    if not len(points):
        return np.empty(0)
    q_x = points.copy()
    q_x[:, 0] = 8.0
    q_x[:, 1] = np.clip(q_x[:, 1], 0.0, 8.0)
    q_x[:, 2] = np.clip(q_x[:, 2], 0.1, 2.9)
    q_y = points.copy()
    q_y[:, 0] = np.clip(q_y[:, 0], 0.0, 8.0)
    q_y[:, 1] = 8.0
    q_y[:, 2] = np.clip(q_y[:, 2], 0.1, 2.9)
    return np.minimum(np.linalg.norm(points - q_x, axis=1), np.linalg.norm(points - q_y, axis=1))


def map_metrics(points: np.ndarray, truth: np.ndarray) -> dict[str, int | float]:
    errors = distance_to_walls(points)
    nearest_truth, _ = cKDTree(points).query(truth, k=1)
    return {
        "confirmed_points": len(points),
        "rms_wall_distance_m": float(np.sqrt(np.mean(errors ** 2))),
        "p99_wall_distance_m": float(np.percentile(errors, 99)),
        "max_wall_distance_m": float(errors.max()),
        "points_farther_than_10cm": int(np.count_nonzero(errors > 0.1)),
        "fraction_farther_than_10cm": float(np.mean(errors > 0.1)),
        "truth_sample_coverage_within_5cm": float(np.mean(nearest_truth <= 0.05)),
    }


def run_experiment() -> dict:
    rng = np.random.default_rng(SEED)
    truth = wall_samples()
    buffer = PoseBuffer()
    fusion = VoxelFusion(voxel_size=VOXEL_SIZE, min_observations=3,
                         max_pending_age=PENDING_TTL, max_weight=20)
    old_history = deque(maxlen=200)
    old_points = {}
    pending = deque()
    statistics = {"scans": 0, "old_integrated_scans": 0, "new_integrated_scans": 0,
                  "new_motion_rejected_scans": 0, "new_future_waited_scans": 0,
                  "corrupted_scan_integrated_new": 0, "new_sync_dropped_scans": 0}
    old_pose_ages = []
    waits = []
    spans = []
    interpolation_errors = []
    corrupted_local = None

    def drain(arrival: float) -> None:
        while pending:
            item = pending[0]
            stamp, local, cloud_arrival, corrupted = item
            match, reason = buffer.lookup(stamp, max_gap=MAX_POSE_GAP,
                                          window_before=WINDOW_BEFORE,
                                          window_after=WINDOW_AFTER)
            if reason in ("need_future", "insufficient_samples"):
                return
            pending.popleft()
            if match is None:
                statistics["new_sync_dropped_scans"] += 1
                continue
            waits.append(arrival - cloud_arrival)
            spans.append(match.span)
            truth_at_stamp = truth_rotation(stamp - BASE_STAMP)
            interpolation_errors.append(float((truth_at_stamp.inv() * Rotation.from_quat(match.quaternion)).magnitude()))
            if match.window_angular_speed > ANGULAR_LIMIT:
                statistics["new_motion_rejected_scans"] += 1
                continue
            world = Rotation.from_quat(match.quaternion).apply(local) + np.asarray(match.position)
            fusion.integrate(world, stamp)
            statistics["new_integrated_scans"] += 1
            statistics["corrupted_scan_integrated_new"] += int(corrupted)

    events = []
    for index in range(-100, 1851):
        seconds = index * ODOM_PERIOD
        events.append((seconds + ODOM_DELAY, 0, index))
    # A .003 second offset ensures each scan needs interpolation between odom
    # samples rather than accidentally testing exact timestamp lookup only.
    for index in range(180):
        seconds = index * SCAN_PERIOD + 0.003
        events.append((seconds + SCAN_ARRIVAL_DELAY, 1, index))
    events.sort()

    for arrival, kind, index in events:
        if kind == 0:
            seconds = index * ODOM_PERIOD
            stamp = BASE_STAMP + seconds
            quaternion = truth_rotation(seconds).as_quat()
            assert buffer.append(stamp, POSITION, quaternion)
            old_history.append((stamp, quaternion))
        else:
            seconds = index * SCAN_PERIOD + 0.003
            stamp = BASE_STAMP + seconds
            local = truth_rotation(seconds).inv().apply(truth - POSITION)
            local += rng.normal(0.0, SCAN_NOISE, local.shape)
            corrupted = index == CORRUPTED_SCAN
            if corrupted:
                local = Rotation.from_euler("z", CORRUPTION_ANGLE, degrees=True).apply(local)
                corrupted_local = local.copy()
            statistics["scans"] += 1

            closest_stamp, closest_quaternion = min(old_history, key=lambda pose: abs(pose[0] - stamp))
            if abs(closest_stamp - stamp) <= 0.5:
                old_pose_ages.append(stamp - closest_stamp)
                transformed = Rotation.from_quat(closest_quaternion).apply(local) + POSITION
                for key, point in zip(np.floor(transformed / VOXEL_SIZE).astype(np.int64).tolist(),
                                      transformed.tolist()):
                    old_points.setdefault(tuple(key), tuple(point))
                statistics["old_integrated_scans"] += 1
            match, reason = buffer.lookup(stamp, max_gap=MAX_POSE_GAP,
                                          window_before=WINDOW_BEFORE,
                                          window_after=WINDOW_AFTER)
            statistics["new_future_waited_scans"] += int(reason == "need_future")
            pending.append((stamp, local, arrival, corrupted))
        drain(arrival)

    assert not pending, "Synthetic event stream should drain every buffered scan"
    # No files or stamps are fabricated for field recordings. This empty
    # synthetic scan only advances candidate expiry past the last observation.
    fusion.integrate([], BASE_STAMP + 18.0 + PENDING_TTL)
    old = np.asarray(list(old_points.values()), dtype=float)
    new = np.asarray(list(fusion.points.values()), dtype=float)
    old_metrics = map_metrics(old, truth)
    new_metrics = map_metrics(new, truth)

    # The isolated corrupted scan is stationary, so the gate deliberately lets
    # it through. Its unobserved ghost voxels must fail multi-scan confirmation.
    assert corrupted_local is not None
    corrupted_world = truth_rotation(CORRUPTED_SCAN * SCAN_PERIOD + 0.003).apply(corrupted_local) + POSITION
    corrupted_keys = {tuple(key) for key in np.floor(corrupted_world / VOXEL_SIZE).astype(np.int64).tolist()}
    isolated_bad_keys = {key for key in corrupted_keys if key in old_points
                         and distance_to_walls(np.asarray([old_points[key]]))[0] > 0.1}
    bad_confirmed = len(isolated_bad_keys.intersection(fusion.points))

    assert statistics["new_future_waited_scans"] == statistics["scans"]
    assert statistics["new_sync_dropped_scans"] == 0
    assert statistics["new_motion_rejected_scans"] > 60
    assert statistics["new_integrated_scans"] >= 50
    assert statistics["corrupted_scan_integrated_new"] == 1
    assert len(isolated_bad_keys) > 100 and bad_confirmed == 0
    assert old_metrics["fraction_farther_than_10cm"] > 0.10
    assert new_metrics["fraction_farther_than_10cm"] == 0
    assert new_metrics["rms_wall_distance_m"] < 0.01
    assert new_metrics["truth_sample_coverage_within_5cm"] > 0.99
    assert new_metrics["confirmed_points"] > 500
    assert max(interpolation_errors) < 1e-10

    return {
        "experiment": "synthetic_90_degree_corner_with_delayed_odom",
        "field_measurements": False,
        "scope": "Cross-stream delay, turn gating and one-scan corruption; no scan deskew or SLAM claim",
        "parameters": {
            "seed": SEED, "turns": 4, "turn_angle_deg": 90,
            "turn_rate_deg_s": 45, "injected_odom_delivery_delay_s": ODOM_DELAY,
            "scan_delivery_delay_s": SCAN_ARRIVAL_DELAY, "odom_rate_hz": 1 / ODOM_PERIOD,
            "scan_rate_hz": 1 / SCAN_PERIOD, "noise_std_m": SCAN_NOISE,
            "single_scan_corruption_deg": CORRUPTION_ANGLE, "voxel_size_m": VOXEL_SIZE,
            "min_observations": 3, "max_turn_rate_deg_s": math.degrees(ANGULAR_LIMIT),
            "pending_ttl_s": PENDING_TTL,
            "max_pose_gap_s": MAX_POSE_GAP,
            "baseline_uses_same_resolution_as_new": True,
            "motion_window_before_s": WINDOW_BEFORE, "motion_window_after_s": WINDOW_AFTER,
            "wall_segments_xy": [[[8, 0], [8, 8]], [[0, 8], [8, 8]]],
            "wall_z_extent_m": [0.1, 2.9],
        },
        "scan_counts": statistics,
        "old_nearest_already_received_pose": {
            **old_metrics, "mean_pose_age_s": float(np.mean(old_pose_ages)),
            "max_pose_age_s": float(max(old_pose_ages)),
        },
        "new_interpolation_motion_gate_confirmation": {
            **new_metrics, "mean_buffer_wait_s": float(np.mean(waits)),
            "max_buffer_wait_s": float(max(waits)), "max_pose_bracket_s": float(max(spans)),
            "max_interpolated_orientation_error_deg_all_scans": math.degrees(max(interpolation_errors)),
            "pending_after_expiry": fusion.pending_count,
        },
        "single_corrupted_scan": {"old_ghost_voxels_farther_than_10cm": len(isolated_bad_keys),
                                  "those_voxels_confirmed_new": bad_confirmed},
        "assertions": "passed",
    }


if __name__ == "__main__":
    print(json.dumps(run_experiment(), indent=2, allow_nan=False))
