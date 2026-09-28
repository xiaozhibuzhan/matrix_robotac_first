#!/usr/bin/env python3
"""Build a standard ROS map_server PGM/YAML pair from measured 3-D points."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class OccupancyResult:
    image_path: Path
    yaml_path: Path
    width: int
    height: int
    resolution: float
    origin: tuple[float, float, float]
    free_cells: int
    occupied_cells: int
    unknown_cells: int


def _validate_points(points: Iterable, ground_z: float) -> np.ndarray:
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if not np.isfinite(xyz).all():
        raise ValueError("points must contain only finite coordinates")
    if not math.isfinite(ground_z):
        raise ValueError("ground_z must be finite")
    return xyz


def build_occupancy(points: Iterable, *, image_path: str | Path,
                    yaml_path: str | Path | None = None,
                    resolution: float = 0.05,
                    ground_z: float = 0.0,
                    ground_band: float = 0.12,
                    obstacle_height: float = 0.15,
                    padding: float = 0.10) -> OccupancyResult:
    """Write PGM/YAML using measured ground and obstacle evidence.

    PGM values follow ``nav2_map_server`` conventions: 0 is occupied, 254 is
    free and 205 is unknown. The first image row represents the largest Y;
    YAML origin remains the lower-left ``(min_x, min_y, 0)`` map coordinate.
    Cells are never filled from neighboring observations.
    """
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("resolution must be finite and positive")
    if not math.isfinite(ground_band) or ground_band < 0:
        raise ValueError("ground_band must be finite and nonnegative")
    if not math.isfinite(obstacle_height) or obstacle_height <= 0:
        raise ValueError("obstacle_height must be finite and positive")
    if not math.isfinite(padding) or padding < 0:
        raise ValueError("padding must be finite and nonnegative")
    xyz = _validate_points(points, ground_z)
    if not len(xyz):
        raise ValueError("cannot create an occupancy map from zero points")

    minimum = xyz[:, :2].min(axis=0) - padding
    maximum = xyz[:, :2].max(axis=0) + padding
    width = max(1, int(math.ceil((maximum[0] - minimum[0]) / resolution)))
    height = max(1, int(math.ceil((maximum[1] - minimum[1]) / resolution)))
    # Recompute the upper edge from integer dimensions so every point maps to
    # a valid cell and the origin is stable across repeated finalization runs.
    origin = (float(minimum[0]), float(minimum[1]), 0.0)
    image = np.full((height, width), 205, dtype=np.uint8)
    indices = np.floor((xyz[:, :2] - minimum) / resolution).astype(np.int64)
    valid = ((indices[:, 0] >= 0) & (indices[:, 0] < width) &
             (indices[:, 1] >= 0) & (indices[:, 1] < height))
    indices, values = indices[valid], xyz[valid, 2]
    if len(indices):
        ground = np.abs(values - ground_z) <= ground_band
        obstacle = values >= ground_z + obstacle_height
        # Ground evidence wins only when no obstacle was observed in that cell.
        free_keys = {tuple(key) for key in indices[ground].tolist()}
        occupied_keys = {tuple(key) for key in indices[obstacle].tolist()}
        for ix, iy in free_keys - occupied_keys:
            image[height - 1 - iy, ix] = 254
        for ix, iy in occupied_keys:
            image[height - 1 - iy, ix] = 0

    image_path = Path(image_path).expanduser().resolve()
    yaml_path = (Path(yaml_path).expanduser().resolve() if yaml_path is not None
                 else image_path.with_suffix(".yaml"))
    image_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_image = image_path.with_name(f".{image_path.name}.tmp")
    temporary_yaml = yaml_path.with_name(f".{yaml_path.name}.tmp")
    try:
        with temporary_image.open("wb") as output:
            output.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            output.write(image.tobytes())
        yaml = (f"image: {image_path.name}\n"
                f"resolution: {resolution:.12g}\n"
                f"origin: [{origin[0]:.12g}, {origin[1]:.12g}, 0.0]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.196\n")
        temporary_yaml.write_text(yaml, encoding="utf-8")
        temporary_image.replace(image_path)
        temporary_yaml.replace(yaml_path)
    except OSError:
        for temporary in (temporary_image, temporary_yaml):
            try:
                temporary.unlink()
            except OSError:
                pass
        raise
    free = int(np.count_nonzero(image == 254))
    occupied = int(np.count_nonzero(image == 0))
    unknown = int(np.count_nonzero(image == 205))
    return OccupancyResult(image_path, yaml_path, width, height, resolution, origin,
                           free, occupied, unknown)


__all__ = ["OccupancyResult", "build_occupancy"]
