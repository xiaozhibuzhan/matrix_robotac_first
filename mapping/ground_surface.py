"""Conservative selection of measured points belonging to a ground surface.

The filter deliberately returns a mask over the input points.  It never creates
or projects points, so callers can merge the selected measurements into their
normal map without changing the geometry of the scan.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
from scipy.spatial import cKDTree


class GroundSurfaceFilter:
    """Identify measured points near a known ground height and a flat plane.

    Neighbourhood support includes the query itself and must contain at least
    ``min_support`` distinct coordinates.  Neighbours come from the complete
    range-valid scan, including points outside the candidate height band, so
    cutting a vertical wall down to its lowest row cannot manufacture a floor.
    The plane must have two-dimensional RMS spread of ``min_planar_spread``
    and its smaller in-plane variance must be at least 10% of the larger one;
    both the neighbourhood RMS and query-to-plane distance are limited by
    ``max_residual``.  Dense mixed wall/floor corners can be rejected; this
    supplementary path favours reliable measured surfaces over full coverage.

    Support is sampled spatially before the bounded neighbour query. Otherwise
    32 returns on one dense lidar ring can crowd out an adjacent ring even when
    both are within ``radius``. Sampling retains original coordinates; the mask
    still applies to all original measurements, not only the support samples.
    When scan stamps are supplied, recent range-valid scans provide additional
    support. Callers must apply their pose/motion checks before calling ``mask``.
    """

    def __init__(self, ground_z: float = 0.0, band: float = 0.12,
                 radius: float = 0.6, min_support: int = 6,
                 max_slope_deg: float = 15.0, max_residual: float = 0.03,
                 min_planar_spread: float = 0.005,
                 max_neighbors: int = 32, support_age: float = 1.0,
                 support_frames: int = 6, min_observations: int = 1) -> None:
        values = (ground_z, band, radius, max_slope_deg, max_residual,
                  min_planar_spread, support_age)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("ground filter parameters must be finite")
        if band < 0 or radius <= 0 or max_slope_deg < 0 or max_slope_deg >= 90:
            raise ValueError("invalid ground band, radius or slope")
        if max_residual < 0 or min_planar_spread <= 0:
            raise ValueError("residual must be nonnegative and planar spread positive")
        if (isinstance(min_support, (bool, np.bool_)) or
                not isinstance(min_support, (int, np.integer)) or min_support < 3):
            raise ValueError("min_support must be an integer >= 3")
        if (isinstance(max_neighbors, (bool, np.bool_)) or
                not isinstance(max_neighbors, (int, np.integer)) or max_neighbors < min_support):
            raise ValueError("max_neighbors must be an integer >= min_support")
        if support_age < 0:
            raise ValueError("support_age must be nonnegative")
        if (isinstance(support_frames, (bool, np.bool_)) or
                not isinstance(support_frames, (int, np.integer)) or support_frames < 1):
            raise ValueError("support_frames must be a positive integer")
        if (isinstance(min_observations, (bool, np.bool_)) or
                not isinstance(min_observations, (int, np.integer)) or
                not 1 <= min_observations <= support_frames):
            raise ValueError("min_observations must be an integer from 1 to support_frames")
        self.ground_z = float(ground_z)
        self.band = float(band)
        self.radius = float(radius)
        self.min_support = int(min_support)
        self.max_slope_deg = float(max_slope_deg)
        self.max_residual = float(max_residual)
        self.min_planar_spread = float(min_planar_spread)
        self.max_neighbors = int(max_neighbors)
        self.support_age = float(support_age)
        self.support_frames = int(support_frames)
        self.min_observations = int(min_observations)
        self._support: deque[tuple[float, np.ndarray]] = deque(maxlen=self.support_frames)
        self._last_stamp: float | None = None
        self.last_counts: dict[str, int] = {}

    def clear(self) -> None:
        """Discard temporal evidence when the map is cleared or restarted."""
        self._support.clear()
        self._last_stamp = None
        self.last_counts.clear()

    def _sample_support(self, xyz: np.ndarray) -> np.ndarray:
        """Keep one real measurement in each 10 cm (or smaller) support cell."""
        if not len(xyz):
            return xyz
        spacing = min(0.1, self.radius / 4.0)
        with np.errstate(over="ignore", invalid="ignore"):
            cells = np.floor(xyz / spacing)
        if not np.isfinite(cells).all():
            raise ValueError("ground support exceeds the supported coordinate range")
        _, first = np.unique(cells, axis=0, return_index=True)
        return xyz[first]

    def mask(self, points: np.ndarray, stamp: float | None = None) -> np.ndarray:
        """Select current real returns; optional stamps enable recent support.

        Repeated/older timestamps never add evidence and return an empty mask.
        A call without a stamp is stateless and ignores buffered scans.
        If ``min_observations > 1``, stamps are required, and each accepted
        return needs measured support from that many different scans within
        30 cm and within ``max_residual`` of the fitted plane. Confidence is
        local to the observed surface, avoiding repeated hits in one 10 cm
        voxel as a prerequisite for mapping a moving, sparse lidar pattern.
        """
        xyz = np.asarray(points, dtype=np.float64)
        if xyz.ndim == 1 and xyz.size == 0:
            xyz = xyz.reshape(0, 3)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        if not np.isfinite(xyz).all():
            raise ValueError("points must contain only finite XYZ values")
        result = np.zeros(len(xyz), dtype=bool)
        self.last_counts = {"candidates": 0, "support_points": 0, "support_scans": 0,
                            "plane_supported": 0, "temporal_confirmed": 0}
        if stamp is not None:
            stamp = float(stamp)
            if not math.isfinite(stamp):
                raise ValueError("ground support stamp must be finite")
            if self._last_stamp is not None and stamp <= self._last_stamp:
                return result
            self._last_stamp = stamp
            while self._support and self._support[0][0] < stamp - self.support_age:
                self._support.popleft()
        heights = np.abs(xyz[:, 2] - self.ground_z)
        candidate_indices = np.flatnonzero(heights <= self.band)
        self.last_counts["candidates"] = len(candidate_indices)
        if not len(candidate_indices):
            return result
        # Every possible 3-D neighbour is inside band + radius. Retain wall
        # points in this slab, rather than cutting a wall down to its bottom row.
        context = self._sample_support(xyz[heights <= self.band + self.radius])
        if stamp is not None:
            self._support.append((stamp, context))
        support_points = (self._sample_support(np.concatenate([p for _, p in self._support]))
                          if stamp is not None else context)
        self.last_counts["support_points"] = len(support_points)
        self.last_counts["support_scans"] = len(self._support) if stamp is not None else 1
        if len(support_points) < self.min_support:
            return result
        tree = cKDTree(support_points)
        prior_trees = ([cKDTree(p) for _, p in list(self._support)[:-1] if len(p)]
                       if stamp is not None and self.min_observations > 1 else [])
        # A bounded nearest-neighbour query keeps work and memory predictable
        # for dense scans while retaining enough support for the PCA test.
        k = min(self.max_neighbors, len(support_points))
        cosine_limit = math.cos(math.radians(self.max_slope_deg))
        spread2_limit = max(self.min_planar_spread * self.min_planar_spread, 1e-10)
        for start in range(0, len(candidate_indices), 1024):
            selected = candidate_indices[start:start + 1024]
            queries = xyz[selected]
            distances, indices = tree.query(queries, k=k,
                                             distance_upper_bound=np.nextafter(self.radius, math.inf))
            valid = np.isfinite(distances) & (indices < len(support_points))
            support = valid.sum(axis=1)
            # Deduplication before the tree query prevents repeated returns
            # from filling all k slots or inflating the distinct support count.
            offsets = support_points[np.minimum(indices, len(support_points) - 1)] - queries[:, None, :]
            offsets[~valid] = 0
            centres = offsets.sum(axis=1) / np.maximum(support, 1)[:, None]
            centred = offsets - centres[:, None, :]
            centred[~valid] = 0
            covariance = np.einsum("bki,bkj->bij", centred, centred)
            covariance /= np.maximum(support, 1)[:, None, None]
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            normals = eigenvectors[:, :, 0]
            # Require two-dimensional spread: a wall edge or a line must not
            # be accepted merely because one possible normal points upward.
            # The ratio is independent of scan density; a 5cm absolute floor
            # would incorrectly reject dense ground when k is capped at 32.
            rms_residual = np.sqrt(np.maximum(eigenvalues[:, 0], 0))
            query_residual = np.abs(np.einsum("bi,bi->b", centres, normals))
            result[selected] = (
                (support >= self.min_support)
                & (np.abs(normals[:, 2]) + 1e-12 >= cosine_limit)
                & (eigenvalues[:, 1] >= spread2_limit)
                & (eigenvalues[:, 1] >= 0.1 * eigenvalues[:, 2])
                & (rms_residual <= self.max_residual)
                & (query_residual <= self.max_residual)
            )
            self.last_counts["plane_supported"] += int(np.count_nonzero(result[selected]))
            if self.min_observations > 1:
                if stamp is None or len(self._support) < self.min_observations:
                    result[selected] = False
                else:
                    keep = np.flatnonzero(result[selected])
                    if not len(keep):
                        continue
                    confirmed = self._confirmed_by_scans(queries[keep], centres[keep], normals[keep],
                                                        prior_trees)
                    result[selected[keep]] &= confirmed
        self.last_counts["temporal_confirmed"] = int(np.count_nonzero(result))
        return result

    def _confirmed_by_scans(self, queries: np.ndarray, centres: np.ndarray,
                            normals: np.ndarray, prior_trees: list[cKDTree]) -> np.ndarray:
        """Count real neighbouring plane support once per historical scan."""
        observations = np.ones(len(queries), dtype=np.int32)  # current scan
        limit = np.nextafter(min(0.3, self.radius), math.inf)
        for previous_tree in reversed(prior_trees):
            needed = np.flatnonzero(observations < self.min_observations)
            if not len(needed):
                break
            previous = previous_tree.data
            # Only the nearest bounded set is examined. Missing valid support
            # may conservatively delay confirmation, never create a new point.
            k = min(self.max_neighbors, len(previous))
            distances, indices = previous_tree.query(
                queries[needed], k=list(range(1, k + 1)), distance_upper_bound=limit)
            valid = np.isfinite(distances) & (indices < len(previous))
            offsets = previous[np.minimum(indices, len(previous) - 1)] - queries[needed, None, :]
            offsets -= centres[needed, None, :]
            residual = np.abs(np.einsum("bki,bi->bk", offsets, normals[needed]))
            observations[needed] += np.any(valid & (residual <= self.max_residual), axis=1)
        return observations >= self.min_observations


__all__ = ["GroundSurfaceFilter"]
