"""Bounded, thread-safe odometry interpolation and motion-window inspection."""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math
import threading
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class PoseMatch:
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    stamp: float
    span: float
    linear_speed: float
    angular_speed: float
    window_linear_speed: float
    window_angular_speed: float
    translation_excursion: float = 0.0
    angular_excursion: float = 0.0


@dataclass(frozen=True)
class _Pose:
    stamp: float
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


class PoseBuffer:
    """Keep validated poses in timestamp order without assuming arrival order.

    Quaternions use ROS's x/y/z/w order. All speeds are derived from pose
    differences, in metres/second and radians/second. No twist fields are used.
    Timestamp zero is invalid: interpolation never falls back to the latest pose.
    """

    def __init__(self, horizon: float = 10.0, max_samples: int = 4000) -> None:
        if not math.isfinite(horizon) or horizon <= 0:
            raise ValueError("horizon must be finite and positive")
        if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples < 2:
            raise ValueError("max_samples must be an integer >= 2")
        self.horizon = float(horizon)
        self.max_samples = max_samples
        self._stamps: list[float] = []
        self._poses: list[_Pose] = []
        self._lock = threading.RLock()

    @property
    def sample_count(self) -> int:
        with self._lock:
            return len(self._poses)

    @property
    def latest_stamp(self) -> float | None:
        with self._lock:
            return self._stamps[-1] if self._stamps else None

    @property
    def first_stamp(self) -> float | None:
        with self._lock:
            return self._stamps[0] if self._stamps else None

    def clear(self) -> None:
        with self._lock:
            self._stamps.clear()
            self._poses.clear()

    def append(self, stamp: float, position: Sequence[float], quaternion: Sequence[float]) -> bool:
        """Insert/replace a valid pose; reject data older than the retained range."""
        try:
            timestamp = float(stamp)
            xyz = tuple(float(value) for value in position)
            xyzw = tuple(float(value) for value in quaternion)
        except (TypeError, ValueError, OverflowError):
            return False
        if (
            timestamp <= 0
            or len(xyz) != 3
            or len(xyzw) != 4
            or not all(math.isfinite(value) for value in (timestamp, *xyz, *xyzw))
        ):
            return False
        norm = math.hypot(*xyzw)
        if norm <= 1e-12 or not math.isfinite(norm):
            return False
        pose = _Pose(timestamp, xyz, tuple(value / norm for value in xyzw))
        with self._lock:
            if self._stamps and timestamp < self._stamps[-1] - self.horizon:
                return False
            index = bisect_left(self._stamps, timestamp)
            if index < len(self._stamps) and self._stamps[index] == timestamp:
                self._poses[index] = pose
                return True
            if len(self._poses) >= self.max_samples and index == 0:
                return False
            self._stamps.insert(index, timestamp)
            self._poses.insert(index, pose)
            discard = max(
                bisect_left(self._stamps, self._stamps[-1] - self.horizon),
                len(self._poses) - self.max_samples,
            )
            if discard > 0:
                del self._stamps[:discard]
                del self._poses[:discard]
        return True

    def lookup(
        self,
        stamp: float,
        max_gap: float = 0.10,
        window_before: float = 0.35,
        window_after: float = 0.05,
    ) -> tuple[PoseMatch | None, str]:
        """Interpolate a pose only with complete, gap-free motion-window coverage.

        The full interval [stamp-window_before, stamp+window_after] must be
        covered. Even at an exact sample timestamp, at least two poses are
        required to measure motion. ``max_gap`` limits the spacing of adjacent
        samples, not their distances from the requested timestamp. Motion-window
        speeds are maxima of each intersecting pose interval, including the
        interval ending at the window's final timestamp.

        The reason is one of ``ok``, ``invalid_stamp``, ``insufficient_samples``,
        ``need_past``, ``need_future`` or ``gap``. A missing future sample may be
        retried after more odometry arrives. The returned dataclass is immutable.
        """
        if not math.isfinite(max_gap) or max_gap <= 0:
            raise ValueError("max_gap must be finite and positive")
        if not all(math.isfinite(value) and value >= 0 for value in (window_before, window_after)):
            raise ValueError("Motion windows must be finite and nonnegative")
        try:
            timestamp = float(stamp)
        except (TypeError, ValueError, OverflowError):
            return None, "invalid_stamp"
        if not math.isfinite(timestamp) or timestamp <= 0:
            return None, "invalid_stamp"
        with self._lock:
            poses = tuple(self._poses)
            stamps = tuple(self._stamps)
        if len(poses) < 2:
            return None, "insufficient_samples"

        start = timestamp - window_before
        end = timestamp + window_after
        tolerance = max(1e-9, 4 * math.ulp(max(abs(start), abs(end), abs(stamps[-1]))))

        def snap_to_sample(value: float) -> float:
            index = bisect_left(stamps, value)
            nearest = min(
                (candidate for candidate in (index - 1, index) if 0 <= candidate < len(stamps)),
                key=lambda candidate: abs(stamps[candidate] - value),
            )
            return stamps[nearest] if abs(stamps[nearest] - value) <= tolerance else value

        # Window arithmetic (for example .57-.35) can land just below an
        # existing sample and otherwise include an unrelated earlier interval.
        timestamp, start, end = map(snap_to_sample, (timestamp, start, end))
        if start < stamps[0] - tolerance:
            return None, "need_past"
        if end > stamps[-1] + tolerance:
            return None, "need_future"
        # Clamp sub-ULP arithmetic differences at sampled window boundaries.
        timestamp = min(max(timestamp, stamps[0]), stamps[-1])
        start = max(start, stamps[0])
        end = min(end, stamps[-1])
        exact = bisect_left(stamps, timestamp)
        if exact < len(stamps) and stamps[exact] == timestamp:
            candidates = [(index, index + 1) for index in (exact - 1, exact)
                          if 0 <= index < len(stamps) - 1]

            def bracket_priority(pair: tuple[int, int]) -> tuple[bool, bool]:
                left, right = pair
                span = stamps[right] - stamps[left]
                outside_window = stamps[left] < start - tolerance or stamps[right] > end + tolerance
                return span > max_gap + tolerance, outside_window

            # Even an exact pose needs motion evidence. Prefer a valid bracket
            # inside the requested window instead of blindly taking the left.
            lower, upper = min(candidates, key=bracket_priority)
        else:
            upper = max(1, exact)
            lower = upper - 1
        window_lower = max(0, bisect_right(stamps, start) - 1)
        window_upper = min(len(stamps) - 1, bisect_left(stamps, end))
        first = min(lower, window_lower)
        last = max(upper, window_upper)
        times = np.asarray(stamps[first:last + 1], dtype=np.float64)
        gaps = np.diff(times)
        if np.any(gaps > max_gap + tolerance):
            return None, "gap"

        selected = poses[first:last + 1]
        positions = np.asarray([pose.position for pose in selected], dtype=np.float64)
        rotations = Rotation.from_quat([pose.quaternion for pose in selected])
        linear_rates = np.linalg.norm(np.diff(positions, axis=0), axis=1) / gaps
        angular_rates = (rotations[:-1].inv() * rotations[1:]).magnitude() / gaps
        bracket_index = lower - first
        span = stamps[upper] - stamps[lower]
        fraction = (timestamp - stamps[lower]) / span
        position = (1 - fraction) * np.asarray(poses[lower].position) + fraction * np.asarray(poses[upper].position)
        quaternion = Slerp(
            [stamps[lower], stamps[upper]],
            Rotation.from_quat([poses[lower].quaternion, poses[upper].quaternion]),
        )([timestamp]).as_quat()[0]
        # A bound on motion across the scan, rather than a derivative of two
        # millisecond samples. Small body vibration must not look like a large
        # motion merely because adjacent pose timestamps are very close.
        translation_excursion = float(np.max(np.linalg.norm(positions - position, axis=1)))
        angular_excursion = float(np.max((Rotation.from_quat(quaternion).inv() * rotations).magnitude()))
        return PoseMatch(
            position=tuple(float(value) for value in position),
            quaternion=tuple(float(value) for value in quaternion),
            stamp=timestamp,
            span=span,
            linear_speed=float(linear_rates[bracket_index]),
            angular_speed=float(angular_rates[bracket_index]),
            window_linear_speed=float(np.max(linear_rates)),
            window_angular_speed=float(np.max(angular_rates)),
            translation_excursion=translation_excursion,
            angular_excursion=angular_excursion,
        ), "ok"
