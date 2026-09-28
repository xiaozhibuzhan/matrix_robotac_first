"""ROS-independent range and radius-outlier filtering for XYZ point clouds."""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree


def filter_xyz(
    points: np.ndarray,
    min_range: float = 0.1,
    max_range: float = 60.0,
    radius: float = 0.2,
    min_neighbors: int = 2,
) -> tuple[np.ndarray, dict[str, int]]:
    """Return float32 XYZ rows plus counts of input and discarded rows.

    Apply this to a lidar-local scan before transforming it into the map frame.
    Range and neighbor-radius boundaries are inclusive. ``max_range=0`` disables
    the upper range limit; ``radius=0`` disables radius-outlier removal. With
    ``min_range=max_range=0``, offline maps can be filtered without assuming that
    their coordinate origin is the sensor position.

    A point needs ``min_neighbors`` OTHER distinct XYZ coordinates inside the
    radius. Repeated identical rows provide no extra support. Exact duplicate
    rows are collapsed before neighborhood testing and one representative is
    retained. Statistics also contain ``duplicate_points``; removal stages do
    not overlap, so their sum plus output length equals the input count.
    """
    if not all(math.isfinite(value) for value in (min_range, max_range, radius)):
        raise ValueError("Range limits and radius must be finite")
    if min_range < 0 or max_range < 0 or (max_range > 0 and max_range < min_range):
        raise ValueError("Range limits must be nonnegative, with max_range >= min_range or max_range=0")
    if radius < 0:
        raise ValueError("Radius must be nonnegative")
    if radius > 0 and (
        isinstance(min_neighbors, (bool, np.bool_))
        or not isinstance(min_neighbors, (int, np.integer))
        or min_neighbors < 1
    ):
        raise ValueError("Active radius filtering requires an integer min_neighbors >= 1")

    with np.errstate(over="ignore", invalid="ignore"):
        xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim == 1 and xyz.size == 0:
        xyz = xyz.reshape(0, 3)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("Points must have shape (N, 3)")

    stats = {"input": len(xyz), "nonfinite": 0, "range": 0, "isolated": 0}
    finite = np.isfinite(xyz).all(axis=1)
    stats["nonfinite"] = int(len(xyz) - np.count_nonzero(finite))
    xyz = xyz[finite]
    if not len(xyz):
        return xyz, stats

    # Float64 products avoid overflow when finite but corrupted float32 ranges
    # are much larger than a real scan. No square root is needed for bounds.
    distance_squared = np.einsum("ij,ij->i", xyz, xyz, dtype=np.float64)
    in_range = distance_squared >= min_range * min_range
    if max_range > 0:
        in_range &= distance_squared <= max_range * max_range
    stats["range"] = int(len(xyz) - np.count_nonzero(in_range))
    xyz = xyz[in_range]
    if not len(xyz):
        return xyz, stats

    # Same lexicographic distinct-coordinate grouping as unique(axis=0),
    # without structured-record comparisons for every XYZ row.
    order = np.lexsort((xyz[:, 2], xyz[:, 1], xyz[:, 0]))
    ordered = xyz[order]
    starts = np.r_[True, np.any(ordered[1:] != ordered[:-1], axis=1)]
    unique = ordered[starts]
    duplicate_points = int(len(xyz) - len(unique))
    if duplicate_points:
        stats["duplicate_points"] = duplicate_points
    xyz = unique
    if radius == 0:
        return xyz, stats
    if len(unique) <= min_neighbors:
        stats["isolated"] = len(xyz)
        return xyz[:0], stats
    tree = cKDTree(unique)
    # The first neighbor is self. Only the requested support threshold matters;
    # finding its distance avoids enumerating every neighbor on dense walls.
    distances, _ = tree.query(unique, k=[min_neighbors + 1],
                              distance_upper_bound=np.nextafter(radius, math.inf))
    supported = distances[:, 0] <= radius
    stats["isolated"] = int(len(unique) - np.count_nonzero(supported))
    return unique[supported], stats
