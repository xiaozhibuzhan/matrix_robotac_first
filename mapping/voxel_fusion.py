"""Fuse repeated, timestamped XYZ scans into a confirmed voxel map without ROS."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


VoxelKey = tuple[int, int, int]
XYZ = tuple[float, float, float]


@dataclass
class _Candidate:
    mean: XYZ
    weight: int
    observations: int
    last_seen: float
    samples: list[XYZ] | None = None


class VoxelFusion:
    """Confirm voxels across distinct scans and average their scan centroids.

    A scan contributes at most one observation to each voxel, regardless of its
    point density. Only ``points`` contains confirmed map samples. Unconfirmed
    candidates expire when not observed for more than ``max_pending_age`` scan
    seconds; confirmed voxels are retained. ``max_weight`` bounds the averaging
    weight, giving an exponential moving average after the cap is reached.

    Scan stamps must increase strictly. Equal, older or nonfinite stamps return
    an empty changed-key list without changing confidence, means or expiration.
    Invalid point arrays raise ValueError before any map state changes.
    """

    def __init__(self, voxel_size: float = 0.05, min_observations: int = 2,
                 max_pending_age: float = 1.0, max_weight: int = 20,
                 sample_capacity: int = 1) -> None:
        if not math.isfinite(voxel_size) or voxel_size <= 0:
            raise ValueError("voxel_size must be finite and positive")
        if not math.isfinite(max_pending_age) or max_pending_age < 0:
            raise ValueError("max_pending_age must be finite and nonnegative")
        for name, value in (("min_observations", min_observations), ("max_weight", max_weight),
                            ("sample_capacity", sample_capacity)):
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        self.voxel_size = float(voxel_size)
        self.min_observations = int(min_observations)
        self.max_pending_age = float(max_pending_age)
        self.max_weight = int(max_weight)
        self.sample_capacity = int(sample_capacity)
        self.points: dict[VoxelKey, XYZ] = {}
        self._weights: dict[VoxelKey, int] = {}
        self._pending: dict[VoxelKey, _Candidate] = {}
        self._samples: dict[VoxelKey, list[XYZ]] = {}
        self._expiry: deque[tuple[float, VoxelKey]] = deque()
        self._last_stamp: float | None = None
        self._version = 0
        self._snapshot_data: np.ndarray | None = None
        self._rows: dict[VoxelKey, int] = {}
        self._means = np.empty((0, 3), dtype=np.float64)
        self._weight_values = np.empty(0, dtype=np.int64)
        self._storage_valid = True
        # Geometry changed by the most recent accepted scan, aligned with the
        # keys returned by integrate().  Consumers that maintain a secondary
        # display index can use this array instead of looking up each tuple in
        # ``points``.  It is replaced (never mutated) per scan and is read-only.
        self.last_changed_points = np.empty((0, 3), dtype=np.float64)
        self.last_changed_keys: tuple[VoxelKey, ...] = ()
        self.last_added_keys: tuple[VoxelKey, ...] = ()

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    @property
    def version(self) -> int:
        """Revision of confirmed geometry; pending observations do not change it."""
        return self._version

    def invalidate_snapshot(self) -> None:
        """Invalidate cached output after a caller directly edits ``points``.

        Normal map writes use integrate/clear and invalidate automatically.
        A change in dictionary size is also detected by snapshot(), but a
        same-size external edit needs this explicit call.
        """
        self._version += 1
        self._snapshot_data = None
        self._storage_valid = False

    def _ensure_storage(self) -> None:
        """Synchronize explicit external dict edits before array access."""
        if self._storage_valid and len(self._rows) == len(self.points):
            return
        count = len(self.points)
        capacity = max(1024, count)
        self._rows = {key: index for index, key in enumerate(self.points)}
        self._means = np.empty((capacity, 3), dtype=np.float64)
        self._weight_values = np.empty(capacity, dtype=np.int64)
        if count:
            self._means[:count] = list(self.points.values())
            self._weight_values[:count] = [self._weights.get(key, 1) for key in self.points]
        self._snapshot_data = None
        self._storage_valid = True

    def _reserve(self, count: int) -> None:
        if count <= len(self._means):
            return
        capacity = max(1024, count, 2 * len(self._means))
        means = np.empty((capacity, 3), dtype=np.float64)
        weights = np.empty(capacity, dtype=np.int64)
        used = len(self._rows)
        means[:used] = self._means[:used]
        weights[:used] = self._weight_values[:used]
        self._means, self._weight_values = means, weights

    def snapshot(self) -> np.ndarray:
        """Return an immutable, insertion-ordered float32 XYZ map snapshot.

        Confirmed means live in contiguous float64 storage.  Output needs one
        conversion/copy, independent of how many map tuples changed. Previous
        snapshots remain valid because the returned float32 array is separate.
        """
        if self._storage_valid and len(self._rows) != len(self.points):
            self._version += 1
        self._ensure_storage()
        if self._snapshot_data is None:
            data = self._means[:len(self.points)].astype("<f4")
            data.flags.writeable = False
            self._snapshot_data = data
        return self._snapshot_data

    def clear(self) -> None:
        self.points.clear()
        self._weights.clear()
        self._pending.clear()
        self._samples.clear()
        self._expiry.clear()
        self._last_stamp = None
        self._rows.clear()
        self._means = np.empty((0, 3), dtype=np.float64)
        self._weight_values = np.empty(0, dtype=np.int64)
        self.last_changed_keys = ()
        self.last_added_keys = ()
        self.last_changed_points = np.empty((0, 3), dtype=np.float64)
        self.invalidate_snapshot()

    def _prune(self, stamp: float) -> None:
        cutoff = stamp - self.max_pending_age
        while self._expiry and self._expiry[0][0] < cutoff:
            seen, key = self._expiry.popleft()
            candidate = self._pending.get(key)
            if candidate is not None and candidate.last_seen == seen:
                del self._pending[key]

    def _average(self, previous: XYZ, sample: XYZ, weight: int) -> tuple[XYZ, int]:
        if weight < self.max_weight:
            weight += 1
        # A generator/zip/min for every occupied voxel dominates the Python
        # loop on dense scans. These are the same three running-mean updates.
        x, y, z = previous
        sx, sy, sz = sample
        mean = (x + (sx - x) / weight, y + (sy - y) / weight, z + (sz - z) / weight)
        return mean, weight

    def _record_sample(self, key: VoxelKey, sample: XYZ) -> None:
        if self.sample_capacity <= 1:
            return
        values = self._samples.setdefault(key, [])
        values.append(sample)
        if len(values) > self.sample_capacity:
            del values[0:len(values) - self.sample_capacity]

    def sampled_snapshot(self) -> np.ndarray:
        """Return retained measured representatives for export.

        Multiple representatives preserve real within-voxel variation; no
        synthetic jitter or grid filling is introduced.
        """
        if self.sample_capacity <= 1 or not self._samples:
            return self.snapshot()
        rows = []
        for key in self.points:
            samples = self._samples.get(key)
            rows.extend(samples if samples else [self.points[key]])
        data = np.asarray(rows, dtype="<f4").reshape(-1, 3)
        data.flags.writeable = False
        return data

    def integrate(self, points: np.ndarray, stamp: float) -> list[VoxelKey]:
        stamp = float(stamp)
        if not math.isfinite(stamp) or (self._last_stamp is not None and stamp <= self._last_stamp):
            return []

        xyz = np.asarray(points, dtype=np.float64)
        if xyz.ndim == 1 and xyz.size == 0:
            xyz = xyz.reshape(0, 3)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
            raise ValueError("points must be a finite array with shape (N, 3)")
        with np.errstate(over="ignore", invalid="ignore"):
            grid = np.floor(xyz / self.voxel_size)
        # Avoid undefined float-to-int conversion for corrupted world positions.
        if np.any(grid < -(2.0 ** 63)) or np.any(grid >= 2.0 ** 63):
            raise ValueError("points exceed the supported voxel coordinate range")

        grid = grid.astype(np.int64)
        # Lexsort avoids the structured-array comparison in unique(axis=0).
        # Bincount preserves input-order sums just as add.at does, while its
        # separate coordinate reductions avoid repeated indexed writes.
        order = np.lexsort((grid[:, 2], grid[:, 1], grid[:, 0]))
        ordered_grid = grid[order]
        starts = np.empty(len(grid), dtype=bool)
        if len(grid):
            starts[0] = True
            starts[1:] = np.any(ordered_grid[1:] != ordered_grid[:-1], axis=1)
        keys = ordered_grid[starts]
        inverse = np.empty(len(grid), dtype=np.intp)
        inverse[order] = np.cumsum(starts) - 1
        counts = np.bincount(inverse, minlength=len(keys))
        centroids = np.empty((len(keys), 3), dtype=np.float64)
        for axis in range(3):
            centroids[:, axis] = np.bincount(inverse, weights=xyz[:, axis], minlength=len(keys)) / counts

        self._last_stamp = stamp
        # A valid scan with only pending/expired candidates has no changed
        # confirmed geometry.  Invalid/repeated stamps intentionally retain
        # the previous result so callers can diagnose why no scan was accepted.
        self.last_changed_keys = ()
        self.last_added_keys = ()
        self.last_changed_points = np.empty((0, 3), dtype=np.float64)
        self._prune(stamp)
        changed = []
        self._ensure_storage()
        key_tuples = list(map(tuple, keys.tolist()))
        rows = np.fromiter((self._rows.get(key, -1) for key in key_tuples),
                           dtype=np.intp, count=len(key_tuples))
        confirmed = rows >= 0
        confirmed_indices = np.flatnonzero(confirmed)
        changed_mask = np.zeros(len(keys), dtype=bool)
        # Existing confirmed voxels dominate a mature map.  Update their EMA
        # together; Python dictionaries are synchronized in bulk only after the
        # arithmetic.  Centroids, confidence and weight-cap semantics match
        # the scalar path exactly, including no change for identical geometry.
        if len(confirmed_indices):
            target_rows = rows[confirmed_indices]
            previous = self._means[target_rows]
            previous_weights = self._weight_values[target_rows]
            weights = np.minimum(previous_weights + 1, self.max_weight)
            means = previous + (centroids[confirmed_indices] - previous) / weights[:, None]
            self._weight_values[target_rows] = weights
            growing_weights = weights != previous_weights
            self._weights.update(zip((key_tuples[index] for index in confirmed_indices[growing_weights].tolist()),
                                     weights[growing_weights].tolist()))
            changed_existing = np.any(means != previous, axis=1)
            modified_indices = confirmed_indices[changed_existing]
            modified_means = means[changed_existing]
            self._means[target_rows[changed_existing]] = modified_means
            changed_mask[modified_indices] = True
            self.points.update(zip((key_tuples[index] for index in modified_indices.tolist()),
                                   map(tuple, modified_means.tolist())))

        promoted_keys = []
        promoted_means = []
        promoted_weights = []
        unconfirmed_indices = np.flatnonzero(~confirmed)
        # Candidate entries remain sparse dictionaries: they need per-key
        # timestamps for exact expiry, and only a small fraction survive.
        for index, centroid in zip(unconfirmed_indices.tolist(), centroids[unconfirmed_indices].tolist()):
            key = key_tuples[index]
            sample = tuple(centroid)
            candidate = self._pending.get(key)
            if candidate is None:
                candidate = _Candidate(sample, 1, 1, stamp,
                                       [sample] if self.sample_capacity > 1 else None)
            else:
                candidate.mean, candidate.weight = self._average(candidate.mean, sample, candidate.weight)
                candidate.observations += 1
                candidate.last_seen = stamp
                if candidate.samples is not None:
                    candidate.samples.append(sample)
                    if len(candidate.samples) > self.sample_capacity:
                        del candidate.samples[0:len(candidate.samples) - self.sample_capacity]
            if candidate.observations >= self.min_observations:
                self.points[key] = candidate.mean
                self._weights[key] = candidate.weight
                if candidate.samples is not None:
                    self._samples[key] = list(candidate.samples)
                self._pending.pop(key, None)
                promoted_keys.append(key)
                promoted_means.append(candidate.mean)
                promoted_weights.append(candidate.weight)
                rows[index] = len(self._rows) + len(promoted_keys) - 1
                changed_mask[index] = True
            else:
                self._pending[key] = candidate
                self._expiry.append((stamp, key))
        if promoted_keys:
            self.last_added_keys = tuple(promoted_keys)
            first = len(self._rows)
            end = first + len(promoted_keys)
            self._reserve(end)
            self._rows.update(zip(promoted_keys, range(first, end)))
            self._means[first:end] = promoted_means
            self._weight_values[first:end] = promoted_weights
        if np.any(changed_mask):
            changed_indices = np.flatnonzero(changed_mask)
            changed = [key_tuples[index] for index in changed_indices.tolist()]
            self._version += 1
            self._snapshot_data = None
            self.last_changed_keys = tuple(changed)
            # Both existing and newly confirmed rows now share the same dense
            # storage; gathering avoids a second Python points lookup per key.
            changed_points = self._means[rows[changed_indices]]
            changed_points.flags.writeable = False
            self.last_changed_points = changed_points
        return changed
