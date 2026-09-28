"""Deskew recorded PointCloud2 measurements using validated absolute times.

No ROS imports are needed. Missing or ambiguous point timestamps yield a
reason for the caller's conservative single-pose fallback. Valid timestamps
are never extrapolated beyond the recorded odometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from cloud_io import read_xyz


def _point_timestamp_values(message, scan_duration: float, trajectory: "Trajectory"):
    """Decode a PointCloud2 timestamp field into absolute seconds.

    Drivers in the field use all of: absolute ROS seconds, seconds relative to
    the message header, and integer millisecond/microsecond/nanosecond offsets.
    The old decoder accepted only the first form, causing every scan from the
    latter drivers to fall back to single-pose filtering.  We keep strict
    validation, but try the documented relative forms and choose the anchor
    that is covered by the recorded trajectory.
    """
    fields = [field for field in message.fields if field.name == "timestamp"]
    if not fields:
        return None, "missing_timestamp"
    if (len(fields) != 1 or fields[0].datatype != 8 or fields[0].count != 1
            or fields[0].offset < 0 or fields[0].offset + 8 > message.point_step):
        return None, "invalid_timestamp_field"
    field = fields[0]
    endian = ">" if message.is_bigendian else "<"
    try:
        raw = np.ndarray((message.height, message.width), dtype=endian + "f8",
                         buffer=memoryview(message.data), offset=field.offset,
                         strides=(message.row_step, message.point_step)).reshape(-1).astype(np.float64)
    except (TypeError, ValueError, IndexError, BufferError):
        return None, "invalid_timestamp_field"
    if not np.isfinite(raw).all():
        return raw, "nonfinite_timestamp"
    stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
    if not math.isfinite(stamp) or stamp <= 0:
        return raw, "invalid_header_timestamp"
    if len(raw) < 2 or np.max(raw) <= np.min(raw):
        return raw, "constant_timestamp"
    tolerance = max(1e-12, 4 * abs(float(np.spacing(stamp))))

    # Absolute ROS/Unix seconds (the representation used by the original
    # implementation).  Keep this branch first to preserve sub-microsecond
    # epoch timestamps and its precise range checks.
    if np.all(raw > 0) and np.all(np.abs(raw - stamp) <= scan_duration + tolerance):
        return raw, "ok"

    # Values that already look like seconds around the header are malformed
    # absolute stamps, rather than millisecond counters.  Do not silently
    # shrink their scale and turn a bad scan into a plausible relative one.
    if np.all(raw > 0) and abs(float(np.median(raw)) - stamp) <= max(10.0 * scan_duration, 0.5):
        return raw, "timestamp_outside_scan"

    span = float(np.max(raw) - np.min(raw))
    # Infer common integer offset units.  A scale is accepted only when the
    # complete scan fits the configured duration, avoiding arbitrary clocks.
    scale = None
    for candidate in (1.0, 1e-3, 1e-6, 1e-9):
        if span * candidate <= scan_duration + tolerance:
            scale = candidate
            break
    if scale is None:
        return raw, "timestamp_outside_scan"
    offsets = raw * scale
    # Some sensors start their relative counter at a non-zero value.  Remove
    # that constant origin only when it cannot itself be a scan offset.
    if abs(float(np.min(offsets))) > scan_duration * 2:
        offsets = offsets - float(np.min(offsets))
    if float(np.max(offsets) - np.min(offsets)) > scan_duration + tolerance:
        return raw, "timestamp_outside_scan"

    # Header stamps may denote either scan start or scan end.  Score both
    # anchors against trajectory coverage and retain the best-supported one.
    candidates = [stamp + offsets]
    candidates.append(stamp - float(np.max(offsets)) + offsets)
    t0, t1 = float(trajectory.timestamps[0]), float(trajectory.timestamps[-1])
    scores = [int(np.count_nonzero((candidate >= t0 - tolerance) &
                                   (candidate <= t1 + tolerance)))
              for candidate in candidates]
    chosen = candidates[int(np.argmax(scores))]
    if scores[int(np.argmax(scores))] == 0:
        return raw, "timestamp_outside_trajectory"
    if np.max(chosen) <= np.min(chosen):
        return raw, "constant_timestamp"
    return chosen, "ok_relative"


@dataclass(frozen=True)
class DeskewResult:
    # Arrays keep original PointCloud2 row order. Invalid rows contain NaNs.
    points: np.ndarray
    valid_mask: np.ndarray
    reason: str
    timestamps: np.ndarray | None = None


class Trajectory:
    """A validated, sorted odometry trajectory (quaternions use XYZW)."""

    def __init__(self, timestamps, positions, quaternions):
        self.timestamps = np.asarray(timestamps, dtype=np.float64).copy()
        self.positions = np.asarray(positions, dtype=np.float64).copy()
        self.quaternions = np.asarray(quaternions, dtype=np.float64).copy()
        count = len(self.timestamps)
        if (self.timestamps.shape != (count,) or count < 2
                or self.positions.shape != (count, 3)
                or self.quaternions.shape != (count, 4)):
            raise ValueError("trajectory requires at least two timestamp/XYZ/XYZW rows")
        if (not np.isfinite(self.timestamps).all() or not np.isfinite(self.positions).all()
                or not np.isfinite(self.quaternions).all()
                or np.any(self.timestamps <= 0) or np.any(np.diff(self.timestamps) <= 0)):
            raise ValueError("trajectory timestamps must be positive and increase; all pose values must be finite")
        with np.errstate(over="ignore", invalid="ignore"):
            norms = np.linalg.norm(self.quaternions, axis=1)
        if not np.isfinite(norms).all() or np.any(norms <= 1e-12):
            raise ValueError("trajectory contains a zero quaternion")
        self.quaternions /= norms[:, None]

    def interpolate(self, query, max_pose_gap: float):
        """Return positions, rotations and validity for each query timestamp.

        Exact recorded poses remain valid beside a large gap; interpolated
        poses require both bracketing samples and a sufficiently small gap.
        """
        if not math.isfinite(max_pose_gap) or max_pose_gap <= 0:
            raise ValueError("max_pose_gap must be finite and positive")
        query = np.asarray(query, dtype=np.float64)
        if query.ndim != 1:
            raise ValueError("query timestamps must be one dimensional")
        right = np.searchsorted(self.timestamps, query, side="left")
        high = np.minimum(right, len(self.timestamps) - 1)
        exact = query == self.timestamps[high]
        low = np.where(exact, high, np.maximum(right - 1, 0))
        gap = self.timestamps[high] - self.timestamps[low]
        gap_tolerance = 4 * np.maximum(np.abs(np.spacing(self.timestamps[high])),
                                        np.abs(np.spacing(self.timestamps[low])))
        valid = (np.isfinite(query) & (query >= self.timestamps[0])
                 & (query <= self.timestamps[-1]) & (exact | (gap <= max_pose_gap + gap_tolerance)))
        positions = np.full((len(query), 3), np.nan)
        quaternions = np.full((len(query), 4), np.nan)
        if not valid.any():
            return positions, quaternions, valid
        lo, hi = low[valid], high[valid]
        duration = gap[valid]
        fraction = np.divide(query[valid] - self.timestamps[lo], duration,
                             out=np.zeros(len(lo)), where=duration > 0)
        positions[valid] = self.positions[lo] + fraction[:, None] * (self.positions[hi] - self.positions[lo])
        first, second = self.quaternions[lo], self.quaternions[hi].copy()
        cosine = np.sum(first * second, axis=1)
        second[cosine < 0] *= -1
        cosine = np.clip(np.abs(cosine), 0., 1.)
        # Normalized linear interpolation avoids division by sin(0) when
        # orientations are equal. Other rows use shortest-arc spherical lerp.
        close = cosine > 1 - 1e-12
        interpolated = first + fraction[:, None] * (second - first)
        distant = ~close
        if distant.any():
            angle = np.arccos(cosine[distant])
            value = fraction[distant]
            denominator = np.sin(angle)
            interpolated[distant] = (
                first[distant] * (np.sin((1 - value) * angle) / denominator)[:, None]
                + second[distant] * (np.sin(value * angle) / denominator)[:, None])
        interpolated /= np.linalg.norm(interpolated, axis=1)[:, None]
        quaternions[valid] = interpolated
        return positions, quaternions, valid


def deskew_cloud(message, trajectory: Trajectory, sensor_z: float,
                 scan_duration: float, max_pose_gap: float) -> DeskewResult:
    """Transform each measured XYZ row at its own absolute FLOAT64 timestamp.

    ``reason == 'ok'`` means the scan's timestamp field passed validation.
    ``valid_mask`` then identifies finite XYZ rows with a recorded pose;
    missing pose brackets/gaps reject individual points. ``points`` always
    has the original Nx3 shape so callers can apply their range masks.
    """
    if not math.isfinite(sensor_z):
        raise ValueError("sensor_z must be finite")
    if not math.isfinite(scan_duration) or scan_duration <= 0:
        raise ValueError("scan_duration must be finite and positive")
    local = read_xyz(message)
    result = np.full(local.shape, np.nan, dtype=np.float64)
    invalid = np.zeros(len(local), dtype=bool)

    def unavailable(reason, times=None):
        return DeskewResult(result, invalid, reason, times)

    if not len(local):
        return unavailable("empty_cloud")
    times, timestamp_reason = _point_timestamp_values(message, scan_duration, trajectory)
    if times is None:
        return unavailable(timestamp_reason)
    if timestamp_reason not in ("ok", "ok_relative"):
        return unavailable(timestamp_reason, times)
    positions, quaternions, valid = trajectory.interpolate(times, max_pose_gap)
    valid &= np.isfinite(local).all(axis=1)
    if valid.any():
        points = local[valid].astype(np.float64)
        points[:, 2] += sensor_z
        q = quaternions[valid]
        twice_cross = 2 * np.cross(q[:, :3], points)
        result[valid] = points + q[:, 3:4] * twice_cross + np.cross(q[:, :3], twice_cross) + positions[valid]
    return DeskewResult(result, valid, "ok", times)
